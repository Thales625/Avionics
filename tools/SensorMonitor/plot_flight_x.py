if __name__ == "__main__":
    from serial import Serial

    # PORT = "/dev/ttyUSB0"
    PORT = "COM5"
    BAUD = 115200

	# initialize serial
    ser = Serial(PORT, BAUD)

    # plot
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    from collections import deque

    MAX_POINTS = 100

    # data timeline
    data_acc_x = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)

    data_pressure = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)
    data_temperature = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)

    # matplotlib - 3 subplots (stacked)
    fig, (ax_acc, ax_press, ax_temp) = plt.subplots(3, 1, sharex=True, figsize=(8, 8))
    fig.suptitle("Flight")

    # Accel plot (three lines)
    line_acc_x, = ax_acc.plot(range(MAX_POINTS), list(data_acc_x), label='acc x', color='r')
    ax_acc.set_ylim(-3., 3.)
    ax_acc.set_ylabel("acc")
    ax_acc.legend(loc='upper right')
    ax_acc.grid(True)

    # Pressure plot
    line_pressure, = ax_press.plot(range(MAX_POINTS), list(data_pressure), label='pressure', color='tab:orange')
    ax_press.set_ylabel("pressure")
    ax_press.legend(loc='upper right')
    ax_press.grid(True)

    # Temperature plot
    line_temperature, = ax_temp.plot(range(MAX_POINTS), list(data_temperature), label='temperature', color='tab:purple')
    ax_temp.set_ylabel("temperature")
    ax_temp.legend(loc='upper right')
    ax_temp.grid(True)

    # fix x-limits to show moving window
    ax_acc.set_xlim(0, MAX_POINTS-1)

    # run
    def update(frame):
        while ser.in_waiting:
            try:
                reading = ser.readline().decode().strip()
            except UnicodeDecodeError:
                continue

            if not reading: 
                continue
            if reading[0] != 'd': 
                continue

            # accel.x, accel.y, accel.z, rotation.x, rotation.y, rotation.z, pressure, temperature
            try:
                values = reading[1:].split(" ")

                data_acc_x.append(float(values[0]))
                data_pressure.append(float(values[1]))
                data_temperature.append(float(values[2]))
            except (ValueError, IndexError):
                print("bad:", reading)
                continue

        # update line data (use x = range to keep window length stable)
        x = range(MAX_POINTS)
        line_acc_x.set_data(x, list(data_acc_x))

        line_pressure.set_data(x, list(data_pressure))
        line_temperature.set_data(x, list(data_temperature))

        # optionally adjust y-limits dynamically (uncomment if desired)
        ax_press.set_ylim(min(data_pressure) - 1, max(data_pressure) + 1)
        ax_temp.set_ylim(min(data_temperature) - 1, max(data_temperature) + 1)

        return line_acc_x, line_pressure, line_temperature

    ani = animation.FuncAnimation(fig, update, interval=50, blit=False)
    plt.tight_layout()
    plt.show()