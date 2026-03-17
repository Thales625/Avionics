from celestial_body import CelestialBody
from vessel import Vessel
from engine import Engine

from avionics_wrapper import AvionicsSim

import matplotlib.pyplot as plt

if __name__ == "__main__":
    # avionics
    avionics = AvionicsSim(lib_path="./libavionics.so")

    # simulation
    earth = CelestialBody(5.9723651e+24, 6371000)
    vessel = Vessel(3.0, earth)

    engine = Engine("./thrust.txt")
    vessel.add_engine(engine)

    true_altitude_arr = []
    true_velocity_arr = []
    true_acceleration_arr = []
    true_pressure_arr = []

    time_arr = []
    altitude_arr = []
    state_arr = []

    acc_arr = []
    baro_arr = []

    dt = 10 * 1e-3
    t = 0.
    while t < 8:
        vessel.update(dt, t)

        baro_sensor = vessel.baro(t)
        acc_sensor = vessel.acc(t)

        # print("SENSOR:", acc_sensor)

        avionics.set_sensors(
            ut=t*1000, # ms
            ax=acc_sensor+9.81, ay=0, az=0, 
            rx=0, ry=0, rz=0, 
            press=baro_sensor, temp=25.0
        )
        avionics.update()

        time_arr.append(t)

        altitude_arr.append(avionics.altitude)
        state_arr.append(avionics.state)

        baro_arr.append(baro_sensor)
        acc_arr.append(acc_sensor)

        true_altitude_arr.append(vessel.altitude)
        true_velocity_arr.append(vessel.velocity)
        true_acceleration_arr.append(vessel.acceleration)
        true_pressure_arr.append(Vessel.pressure(vessel.altitude))

        t += dt

    # plotting
    fig, axs = plt.subplots(5, 1, sharex=True, figsize=(10, 15))

    axs[0].plot(time_arr, true_altitude_arr, label="True Altitude", color="green")
    axs[0].plot(time_arr, altitude_arr, label="Altitude", color="blue")
    axs[0].set_ylabel("Altitude (m)")
    axs[0].legend(loc="upper right")
    axs[0].grid()

    axs[1].plot(time_arr, true_velocity_arr, label="Velocity", color="blue")
    axs[1].set_ylabel("Velocity (m/s)")
    axs[1].legend(loc="upper right")
    axs[1].grid()

    axs[2].plot(time_arr, true_acceleration_arr, label="True Acceleration", color="green")
    axs[2].plot(time_arr, acc_arr, label="Acceleration", color="red")
    axs[2].set_ylabel("Acceleration (m/s²)")
    axs[2].legend(loc="upper right")
    axs[2].grid()

    axs[3].plot(time_arr, true_pressure_arr, label="True Pressure", color="green")
    axs[3].plot(time_arr, baro_arr, label="Pressure", color="red")
    axs[3].set_ylabel("Pressure")
    axs[3].legend(loc="upper right")
    axs[3].grid()

    axs[4].plot(time_arr, state_arr, label="State", color="red")
    axs[4].set_xlabel("Time (s)")
    axs[4].set_ylabel("State")
    axs[4].legend(loc="upper right")
    axs[4].grid()

    plt.tight_layout()
    plt.show()
