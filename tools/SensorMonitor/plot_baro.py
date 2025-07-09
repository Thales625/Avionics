if __name__ == "__main__":
	from serial import Serial
	import numpy as np

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
	data = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)

	# matplotlib
	fig, ax = plt.subplots()
	ax.set_title("Barometer")
	line, = ax.plot(data, label='altitude')
	ax.set_ylim(-10, 10)
	ax.legend()

	# Z-zero
	z0 = 0
	while ser.in_waiting:
		try:
			reading = ser.readline().decode().strip()
		except UnicodeDecodeError:
			continue

		try:
			values = reading.split(" ")

			if len(values) != 7: continue

			z0 = float(values[6])
		except ValueError:
				print(reading)


	# run
	def update(frame):
		while ser.in_waiting:
			# acc.x acc.y acc.z rot.x rot.y rot.z altitude
			try:
				reading = ser.readline().decode().strip()
			except UnicodeDecodeError:
				continue

			try:
				values = reading.split(" ")

				if len(values) != 7: continue

				data.append(float(values[6]) - z0)
			except ValueError:
				print(reading)

		line.set_ydata(data)
		return line

	ani = animation.FuncAnimation(fig, update, interval=10)
	plt.show()