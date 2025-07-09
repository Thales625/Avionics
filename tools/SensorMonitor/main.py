if __name__ == "__main__":
	from serial import Serial
	from math import pow

	def pressure_to_altitude(pressure): return 44330 * (1.0 - pow(pressure / 1013.25, 0.1903))

	PORT = "/dev/ttyUSB0"
	BAUD = 115200

	# initialize serial
	ser = Serial(PORT, BAUD)

	# plot
	import matplotlib.pyplot as plt
	import matplotlib.animation as animation
	from collections import deque

	MAX_POINTS = 100

	# data timeline
	data = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)
	# timestamps = deque([0]*MAX_POINTS, maxlen=MAX_POINTS) # TODO: used in line.set_xdata

	# matplotlib
	fig, ax = plt.subplots()
	line, = ax.plot(data)
	ax.set_ylim(-100, 100)

	# Z-zero
	z0 = 0
	while ser.in_waiting:
		try:
			z0 = pressure_to_altitude(float(ser.readline().decode().strip()))
		except ValueError:
			pass

	# run
	def update(frame):
		while ser.in_waiting:
			reading = ser.readline().decode().strip()
			try:
				z = pressure_to_altitude(float(reading)) - z0
				data.append(z)
				print(z)
			except ValueError:
				print(reading)

		line.set_ydata(data)
		line.set_xdata(range(len(data))) # TODO: Replace index with actual measurement timestamps
		return line,

	ani = animation.FuncAnimation(fig, update, interval=80)
	plt.show()