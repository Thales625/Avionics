from vessel import Vessel
from celestial_body import CelestialBody
from engine import Engine

import matplotlib.pyplot as plt

import ctypes

if __name__ == "__main__":
    # avionics
    avionics = ctypes.CDLL("./libavionics.so")
    avionics.loop.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_uint32]
    avionics.loop.restype = ctypes.c_voidp

    avionics.setup.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_uint32]
    avionics.setup.restype = ctypes.c_voidp

    # simulation
    earth = CelestialBody(5.972365131085893e+24, 6371000)
    vessel = Vessel(3.0, earth)
    engine = Engine("./thrust.txt")

    vessel.add_engine(engine)

    time_arr = []
    altitude_arr = []
    velocity_arr = []
    acc_arr = []

    dt = 10 * 1e-3
    t = 0.
    alt_sensor = 0.
    while t < 8:
        vessel.update(dt, t)

        alt_sensor = alt_sensor * 0.9 + vessel.baro(t) * 0.1

        # alt_sensor = vessel.baro(t)
        acc_sensor = vessel.acc(t)

        time_arr.append(t)
        altitude_arr.append(alt_sensor)
        velocity_arr.append(vessel.velocity)
        acc_arr.append(acc_sensor)

        avionics.loop(alt_sensor, acc_sensor, int(t*1000))

        # print("Avionics: ", avionics.loop(vessel.baro(), vessel.acc(), int(t)).decode("utf-8"))

        t += dt

    # plotting
    fig, axs = plt.subplots(3, 1, sharex=True, figsize=(10, 8))

    axs[0].plot(time_arr, altitude_arr, label="Altitude", color="blue")
    axs[0].set_ylabel("Altitude (m)")
    axs[0].legend(loc="upper right")
    axs[0].grid()

    axs[1].plot(time_arr, velocity_arr, label="Velocity", color="green")
    axs[1].set_ylabel("Velocity (m/s)")
    axs[1].legend(loc="upper right")
    axs[1].grid()

    axs[2].plot(time_arr, acc_arr, label="Acceleration", color="red")
    axs[2].set_xlabel("Time (s)")
    axs[2].set_ylabel("Acceleration (m/s²)")
    axs[2].legend(loc="upper right")
    axs[2].grid()

    plt.tight_layout()
    plt.show()
