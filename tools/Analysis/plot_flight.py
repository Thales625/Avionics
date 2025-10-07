from math import sqrt, pow

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
    state = []
    times = []

    accel_x = []
    accel_y = []
    accel_z = []

    rot_x = []
    rot_y = []
    rot_z = []

    pressure = []
    temperature = []

    altitude = []

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

                state.append(int(part[0]))
                times.append(int(part[1]))
                # times.append(int(part[1])/1000)

                accel_x.append(float(part[2]))
                accel_y.append(float(part[3]))
                accel_z.append(float(part[4]))

                rot_x.append(float(part[5]))
                rot_y.append(float(part[6]))
                rot_z.append(float(part[7]))

                pressure.append(float(part[8]))
                temperature.append(float(part[9]))

                if pressure_0 is None: pressure_0 = pressure[-1]

                altitude_baro = 0.9 * altitude_baro + 0.1 * (44330.0 * (1.0 - pow(pressure[-1] / pressure_0, 0.1903)))

                altitude.append(altitude_baro)
    except Exception as e:
        print(f"Error reading file: {e}")
        exit()

    # time analysis
    '''
    import numpy as np
    times = np.array(times) - times[0]
    plt.plot(times[:-1], np.diff(times))
    plt.show()
    exit()
    '''

    # 2.266 (video) => 57.9191 (graph)
    times = [t - times[0] - 57.9191 + 2.266 for t in times]

    print(f"Altitude variation: {max(altitude)-min(altitude):.2f} m")
    
    # plot data
    fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, sharex=True, figsize=(8, 6))

    ax1.plot(times, state)
    ax1.grid(True)
    ax1.set_title("State")

    ax2.plot(times, altitude)
    # ax2.plot(times, [44330.0 * (1.0 - pow(pressure[i] / pressure_0, 0.1903)) for i in range(len(times))]) # raw
    ax2.grid(True)
    ax2.set_title("Altitude")

    # ax3.plot(times, [sqrt(accel_x[i]**2 + accel_y[i]**2 + accel_z[i]**2) for i in range(len(times))])
    ax3.plot(times, [accel_x[i] for i in range(len(times))])
    ax3.grid(True)
    ax3.set_title("Accel")

    ax4.plot(times, [sqrt(rot_x[i]**2 + rot_y[i]**2 + rot_z[i]**2) for i in range(len(times))])
    ax4.grid(True)
    ax4.set_title("Rotation")

    plt.xlabel(r"Time $\left[s\right]$")
    plt.tight_layout()
    plt.show()
