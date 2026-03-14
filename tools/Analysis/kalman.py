import numpy as np

from utils import *

# ---------------- ESKF usage notes ----------------
# - this ESKF assumes acc_meas and gyro_meas are in the BODY frame.
# - rot_from_quat returns rotation matrix to map body->nav (Rbn). adjust sign/convention
#   if your quaternion convention is different.
# - tune: sigma_a, sigma_g, sigma_ba, sigma_bg, initial P0, and magnetometer/gps/baro R values.
# - if GPS timestamps are delayed, buffer IMU predictions and perform update at the correct time.
# - for testing: simulate IMU with known biases, run the filter and check convergence of b_a and b_g.
class ESKF:
    def __init__(self, acc_noise, gyro_noise, baro_noise, gps_noise, s0: np.ndarray, v0: np.ndarray, q0: np.ndarray):
        """
        accelerometer, barometer, gps objects should have .noise (std) fields
        s0: initial position (3,)
        v0: initial velocity (3,)
        q0: initial quaternion (4,) (x,y,z,w)
        """

        # Nominal state:
        # p(3), v(3), q(4), b_a(3), b_g(3) => total 16 (q stored explicitly)
        self.p = s0.astype(float).copy()
        self.v = v0.astype(float).copy()
        self.q = quat_normalize(q0.astype(float).copy())
        self.b_a = np.zeros(3, dtype=float) # accel bias init
        self.b_g = np.zeros(3, dtype=float) # gyro bias init

        # Error-state covariance (15x15): [dp, dv, dtheta, dba, dbg]
        P0 = np.zeros((15,15), dtype=float)
        # sensible initial uncertainties (tune as needed)
        P0[0:3,0:3] = np.eye(3) * 10.0 # pos (m^2)
        P0[3:6,3:6] = np.eye(3) * 1.0  # vel
        P0[6:9,6:9] = np.eye(3) * (np.deg2rad(5.0)**2)     # attitude error ~5 deg
        P0[9:12,9:12] = np.eye(3) * 0.5                    # accel bias (m/s^2)^2
        P0[12:15,12:15] = np.eye(3) * (np.deg2rad(1.0)**2) # gyro bias (rad/s)^2
        self.P = P0

        # sensor noise characteristics
        # accelerometer.noise is (m/s^2), gyro.noise is (rad/s)
        self.sigma_a = acc_noise
        self.sigma_g = gyro_noise
        self.sigma_baro = baro_noise
        self.sigma_gps = gps_noise
        # process noise (continuous) for bias random walks (tune)
        self.sigma_ba = 1e-2            # accel bias random walk (m/s^2 / sqrt(s))
        self.sigma_bg = np.deg2rad(0.1) # gyro bias random walk (rad/s / sqrt(s))

        # gravity in navigation frame (z up)
        self.g = np.array([0.0, 0.0, -9.81], dtype=float)

    # predict + updates
    def __call__(self, dt: float, acc_meas: np.ndarray, gyro_meas: np.ndarray, baro_meas: float, mag_meas: np.ndarray = None, gps_meas: np.ndarray = None):
        """
        dt: time step
        acc_meas: accelerometer measurement (body frame) (3,) [m/s^2]
        gyro_meas: gyro measurement (body frame) (3,) [rad/s]
        baro_meas: barometer altitude measurement (scalar) (m)
        mag_meas: magnetometer measurement (3,) in body frame (optional) - unit or raw
        gps_meas: gps position measurement in navigation frame (3,) (optional)
        """

        # ---------------- PREDICTION (nominal state) ----------------
        # 1) remove bias
        omega_unbiased = gyro_meas - self.b_g # angular rate unbiased (body)
        acc_unbiased = acc_meas - self.b_a    # accel unbiased (body)

        # 2) integrate quaternion: q_{k+1} = q_k * q(omega*dt)
        delta_q = quat_from_small_angle(omega_unbiased * dt) # small-angle quaternion
        self.q = quat_normalize(quat_mul(self.q, delta_q))

        # 3) rotate acceleration to navigation frame and add gravity
        Rbn = rot_from_quat(self.q) # body -> nav (depending on convention used above)
        a_nav = Rbn @ acc_unbiased + self.g

        # 4) integrate v and p (simple Euler / semi-implicit)
        self.v = self.v + a_nav * dt
        self.p = self.p + self.v * dt + 0.5 * a_nav * dt**2

        # ---------------- PREDICTION (covariance) ----------------
        # Build continuous-time linearized system matrix F_c (15x15)
        F = np.zeros((15,15), dtype=float)
        # states ordering: dp(0:3), dv(3:6), dtheta(6:9), dba(9:12), dbg(12:15)

        # dp_dot = dv --> dp/dv
        F[0:3, 3:6] = np.eye(3)

        # dv_dot depends on attitude error and accel bias:
        # dv_dot ≈ - R * skew(a_unbiased) * dtheta  + (-R) * dba
        F[3:6, 6:9] = - Rbn @ skew(acc_unbiased)
        F[3:6, 9:12] = - Rbn

        # dtheta_dot ≈ -skew(omega) * dtheta - dbg
        F[6:9, 6:9] = - skew(omega_unbiased)
        F[6:9, 12:15] = - np.eye(3)

        # bias random walk: dba_dot = 0, dbg_dot = 0 (so zeros)

        # Discretize: Phi ≈ I + F*dt  (first-order)
        Phi = np.eye(15) + F * dt

        # process noise mapping Gc (continuous) -> map noise [a_noise, g_noise, ba_noise, bg_noise]
        # we'll build discrete Qd approx:
        # continuous noise vector: [n_a(3), n_g(3), n_ba(3), n_bg(3)]
        # effect on dv: R * n_a
        # effect on dtheta: n_g
        G = np.zeros((15, 12), dtype=float)
        # dv affected by accel noise
        G[3:6, 0:3] = Rbn
        # dtheta affected by gyro noise
        G[6:9, 3:6] = np.eye(3)
        # bias random walks
        G[9:12, 6:9] = np.eye(3)   # ba influenced by ba noise
        G[12:15, 9:12] = np.eye(3) # bg influenced by bg noise

        # continuous noise covariance
        Qc = np.zeros((12,12), dtype=float)
        Qc[0:3,0:3] = (self.sigma_a**2) * np.eye(3)
        Qc[3:6,3:6] = (self.sigma_g**2) * np.eye(3)
        Qc[6:9,6:9] = (self.sigma_ba**2) * np.eye(3)
        Qc[9:12,9:12] = (self.sigma_bg**2) * np.eye(3)

        # discrete process noise approx: Qd = G * Qc * G^T * dt
        Qd = G @ Qc @ G.T * dt

        # propagate covariance
        self.P = Phi @ self.P @ Phi.T + Qd

        # ---------------- UPDATE: BAROMETER (z = p_z + r) ----------------
        # measurement on true position: z_baro = p_z + noise
        H_baro = np.zeros((1, 15))
        H_baro[0, 2] = 1.0   # maps δp_z
        R_baro = np.array([[self.sigma_baro**2]])

        # innovation
        z_pred_baro = self.p[2]
        y_baro = baro_meas - z_pred_baro
        S = H_baro @ self.P @ H_baro.T + R_baro
        K = self.P @ H_baro.T @ np.linalg.inv(S)
        dx = (K * y_baro).reshape(-1)  # 15x1

        # apply error-state correction to nominal
        self._inject_error(dx)

        # update covariance
        I15 = np.eye(15)
        self.P = (I15 - K @ H_baro) @ self.P @ (I15 - K @ H_baro).T + K @ R_baro @ K.T

        # ---------------- UPDATE: GPS (position) ----------------
        if gps_meas is not None:
            H_gps = np.zeros((3, 15))
            H_gps[0:3, 0:3] = np.eye(3)
            R_gps = np.eye(3) * (self.sigma_gps**2)

            y_gps = gps_meas - self.p
            Sg = H_gps @ self.P @ H_gps.T + R_gps
            Kg = self.P @ H_gps.T @ np.linalg.inv(Sg)
            dxg = (Kg @ y_gps).reshape(-1)

            self._inject_error(dxg)
            self.P = (I15 - Kg @ H_gps) @ self.P @ (I15 - Kg @ H_gps).T + Kg @ R_gps @ Kg.T

        # ---------------- UPDATE: MAGNETOMETER (optional) ----------------
        if mag_meas is not None:
            # TODO: set correct m_ref for your location / inclination
            m_ref = np.array([1.0, 0.0, 0.0])
            # predicted mag in body frame:
            z_pred_mag = (rot_from_quat(self.q).T @ m_ref)
            y_mag = mag_meas - z_pred_mag

            # H_mag maps dtheta -> predicted mag error (approx)
            H_mag = np.zeros((3,15))
            H_mag[:, 6:9] = - rot_from_quat(self.q).T @ skew(m_ref)

            R_mag = np.eye(3) * (0.05**2) # tune magnetometer noise
            S = H_mag @ self.P @ H_mag.T + R_mag
            K = self.P @ H_mag.T @ np.linalg.inv(S)
            dxmag = (K @ y_mag).reshape(-1)
            self._inject_error(dxmag)
            self.P = (I15 - K @ H_mag) @ self.P @ (I15 - K @ H_mag).T + K @ R_mag @ K.T

        return self.get_nominal_state_vector()

    # ---------------- helper to inject an error-state dx into nominal ----------------
    def _inject_error(self, dx: np.ndarray):
        """
        dx: 15-vector [dp, dv, dtheta, dba, dbg]
        apply to nominal:
          p <- p + dp
          v <- v + dv
          q <- q * quat(dtheta)  (multiplicative update)
          b_a <- b_a + dba
          b_g <- b_g + dbg
        Note: after injection, the error state should be zeroed (we already
        apply correction directly to nominal and keep P updated).
        """
        dp = dx[0:3]
        dv = dx[3:6]
        dtheta = dx[6:9]
        dba = dx[9:12]
        dbg = dx[12:15]

        self.p = self.p + dp
        self.v = self.v + dv

        dq = quat_from_small_angle(dtheta)
        self.q = quat_normalize(quat_mul(dq, self.q)) # apply small rotation

        self.b_a = self.b_a + dba
        self.b_g = self.b_g + dbg

    def get_nominal_state_vector(self):
        """Return stacked nominal state for convenience (p,v,q,b_a,b_g)."""
        return np.hstack((self.p, self.v, self.q, self.b_a, self.b_g))

if __name__ == "__main__":
    import argparse
    from pathlib import Path
    import matplotlib.pyplot as plt

    # argument parsing
    parser = argparse.ArgumentParser(description="Data Analysis")

    parser.add_argument(
        "file",
        type=Path,
        nargs='?',
        default=Path("./datalog.txt"),
        help="Path to datalog file (ex: ~/DATALOG.txt)"
    )

    args = parser.parse_args()

    # solve path
    file_path = args.file.expanduser()

    # data
    times = []
    accel = []
    rot = []
    pressure = []
    temperature = []

    altitude = []

    t0 = None
    pressure_0 = None
    altitude_baro = 0.0

    # state, ut, accel.x, accel.y, accel.z, rotation.x, rotation.y, rotation.z, pressure, temperature

    # read file
    try:
        with open(file_path, 'r') as f:
            for line in f:
                line = line.strip()

                if not line: continue

                part = line.split()

                if t0 is None: t0 = int(part[1])

                t = (int(part[1])-t0)/1000

                # if t < 30: continue
                if t < 50: continue
                elif t > 60: break

                times.append(t)

                accel.append(np.array([float(part[3]), float(part[4]), float(part[2])]))
                # accel.append(np.array([float(part[4]), float(part[3]), float(part[2])]))
                # accel.append(np.array([float(part[2]), float(part[3]), float(part[4])]))

                rot.append(np.array([float(part[6]), float(part[7]), float(part[5])]))
                # rot.append(np.array([float(part[7]), float(part[6]), float(part[5])]))
                # rot.append(np.array([float(part[5]), float(part[6]), float(part[7])]))

                pressure.append(float(part[8]))
                temperature.append(float(part[9]))

                if pressure_0 is None: pressure_0 = pressure[-1]

                altitude_baro = 44330.0 * (1.0 - np.pow(pressure[-1] / pressure_0, 0.1903))
                # altitude_baro = 0.9 * altitude_baro + 0.1 * (44330.0 * (1.0 - np.pow(pressure[-1] / pressure_0, 0.1903)))

                altitude.append(altitude_baro)
    except Exception as e:
        print(f"Error reading file: {e}")
        exit()

    times = [t-times[0] for t in times]

    # plot data
    I0 = 550

    plt.title("Accelerometer")
    plt.plot(times, [a[0] for a in accel], label="X")
    plt.plot(times, [a[1] for a in accel], label="Y")
    plt.plot(times, [a[2] for a in accel], label="Z")
    plt.axvline(times[I0], label="init animation")
    plt.legend()
    plt.show()

    plt.title("Gyroscope")
    plt.plot(times, [g[0] for g in rot], label="X")
    plt.plot(times, [g[1] for g in rot], label="Y")
    plt.plot(times, [g[2] for g in rot], label="Z")
    plt.axvline(times[I0], label="init animation")
    plt.legend()
    plt.show()

    # Kalman
    import numpy as  np
    s0 = np.array([0.0, 0.0, 0.0])
    v0 = np.array([0.0, 0.0, 0.0])
    q0 = np.array([0.0, 0.0, 0.0, 1.0])

    # GPS for fix position to (0, 0, 0)
    kalman = ESKF(0.1, 0.1, 2.0, 0.0, s0, v0, q0)

    estimated_pos = []
    estimated_vel = []
    estimated_q = []
    estimated_bias = []

    t0 = -times[1] # fix fist dt reading
    # t0 = 0.0

    for i, t in enumerate(times):
        dt = t - t0
        t0 = t

        # print(accel[i])
        # print(rot[i])

        # if t < 26.5:
        if t < 7.2: # fixed position
            kalman(dt, accel[i]*9.81, rot[i]*(np.pi/180), altitude[i], None, np.array([0.0, 0.0, 0.0]))
        else:
            kalman(dt, accel[i]*9.81, rot[i]*(np.pi/180), altitude[i])

        estimated_pos.append(kalman.p)
        estimated_vel.append(kalman.v)
        estimated_q.append(kalman.q)
        estimated_bias.append((kalman.b_a, kalman.b_g))

    # PLOT BIAS
    plt.title("Biases Accelerometer")
    plt.plot(times, [a[0][0] for a in estimated_bias], label="X")
    plt.plot(times, [a[0][1] for a in estimated_bias], label="Y")
    plt.plot(times, [a[0][2] for a in estimated_bias], label="Z")
    plt.legend()
    plt.show()

    plt.title("Biases Gyroscope")
    plt.plot(times, [a[1][0] for a in estimated_bias], label="X")
    plt.plot(times, [a[1][1] for a in estimated_bias], label="Y")
    plt.plot(times, [a[1][2] for a in estimated_bias], label="Z")
    plt.legend()
    plt.show()

    # ANIMATE
    from scipy.spatial.transform import Rotation as R
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    from matplotlib.animation import FuncAnimation

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # max_range = np.max(np.linalg.norm(self.s, axis=1))
    # max_range = np.max(np.linalg.norm([_ for _ in self.s if _[2] > 0], axis=1))
    max_range = 5
    ax.set_xlim(-max_range, max_range)
    ax.set_ylim(-max_range, max_range)
    ax.set_zlim(0, max_range)

    global rocket_dir
    rocket_pos, = ax.plot([], [], [], 'o', color='red', markersize=2)
    rocket_dir = ax.quiver(0, 0, 0, 0, 0, 0, length=10, color='blue')

    traj_line, = ax.plot([], [], [], '--', color='black', linewidth=1)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Rocket Launch Simulation')


    def update(frame):
        global rocket_dir

        frame += I0

        pos = estimated_pos[frame]
        q = R.from_quat(estimated_q[frame])

        rocket_pos.set_data([pos[0]], [pos[1]])
        rocket_pos.set_3d_properties([pos[2]])

        dir_vec = q.apply(np.array([0, 0, 1]))

        rocket_dir.remove()
        rocket_dir = ax.quiver(pos[0], pos[1], pos[2], dir_vec[0], dir_vec[1], dir_vec[2], length=0.5, color='red')

        traj_line.set_data([p[0] for p in estimated_pos[:frame+1]], [p[1] for p in estimated_pos[:frame+1]])
        traj_line.set_3d_properties([p[2] for p in estimated_pos[:frame+1]])

        return rocket_pos, rocket_dir, traj_line

    anim = FuncAnimation(fig, update, frames=len(estimated_pos)-I0, interval=dt, blit=False, repeat=False)
    # anim = FuncAnimation(fig, update, frames=len(estimated_pos), interval=30, blit=False)
    plt.show()

    
    # PLOT GRAPHS
    plt.title("X-position")
    plt.plot(times, [s[0] for s in estimated_pos], label="Estimated")
    # plt.plot(times, altitude, label="Altitude")
    plt.legend()
    plt.show()

    plt.title("Y-position")
    plt.plot(times, [s[1] for s in estimated_pos], label="Estimated")
    # plt.plot(times, altitude, label="Altitude")
    plt.legend()
    plt.show()

    plt.title("Z-position")
    plt.plot(times, [s[2] for s in estimated_pos], label="Estimated")
    plt.plot(times, altitude, label="Altitude barometer")
    plt.legend()
    plt.show()
    exit()

    
    # plot data
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True, figsize=(8, 6))

    ax1.plot(times, altitude)
    ax1.grid(True)
    ax1.set_title("Altitude")

    # ax3.plot(times, [sqrt(accel_x[i]**2 + accel_y[i]**2 + accel_z[i]**2) for i in range(len(times))])
    ax2.plot(times, [accel[i][0] for i in range(len(times))])
    ax2.grid(True)
    ax2.set_title("Accel")

    # ax3.plot(times, [sqrt(rot[i]**2 + rot_y[i]**2 + rot_z[i]**2) for i in range(len(times))])
    ax3.grid(True)
    ax3.set_title("Rotation")

    plt.xlabel(r"Time $\left[ms\right]$")
    plt.tight_layout()
    plt.show()