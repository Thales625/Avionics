from math import log

def pressure_to_altitude(pressure):
    P0 = 1013.25
    T0 = 288.15
    L = 0.0065
    R = 287.05
    g = 9.80665

    return (T0 / L) * (1 - (pressure / P0) ** (R * L / g))

def pressure_to_altitude_temp(pressure, temperature):
    P0 = 101325.0
    R = 8.314462618
    g = 9.80665
    M = 0.0289644
    temp_k = temperature + 273.15

    return (R * temp_k) / (g * M) * log(P0 / (100*pressure))

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

    accel_x = []
    accel_y = []
    accel_z = []

    rot_x = []
    rot_y = []
    rot_z = []

    pressure = []
    temperature = []

    altitude = []

    # ut, accel.x, accel.y, accel.z, rotation.x, rotation.y, rotation.z, pressure, temperature

    # read file
    try:
        with open(file_path, 'r') as f:
            for line in f:
                line = line.strip()

                if not line: continue

                part = line.split()
                
                if int(part[0]) > 40000: break # stop at 40 sec

                times.append(int(part[0]))

                accel_x.append(float(part[1]))
                accel_y.append(float(part[2]))
                accel_z.append(float(part[3]))

                rot_x.append(float(part[4]))
                rot_y.append(float(part[5]))
                rot_z.append(float(part[6]))

                pressure.append(float(part[7]))
                temperature.append(float(part[8]))

                altitude.append(pressure_to_altitude_temp(pressure[-1], temperature[-1]))
                # altitude.append(pressure_to_altitude(pressure[-1]))
    except Exception as e:
        print(f"Error reading file: {e}")
        exit()

    print(f"Altitude variation: {max(altitude)-min(altitude):.2f} m")
    
    # plot data
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True, figsize=(8, 6))

    ax1.plot(times, pressure)
    ax1.grid(True)
    ax1.set_title("Pressure")

    ax2.plot(times, temperature)
    ax2.grid(True)
    ax2.set_title("Temperature")

    ax3.plot(times, altitude)
    ax3.grid(True)
    ax3.set_title("Altitude")

    plt.xlabel(r"Time $\left[ms\right]$")
    plt.tight_layout()
    plt.show()
