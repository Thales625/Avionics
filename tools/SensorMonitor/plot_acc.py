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
	data_x = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)
	data_y = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)
	data_z = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)

	# matplotlib
	fig, ax = plt.subplots()
	ax.set_title("Accelerometer")
	line_x, = ax.plot(data_x, label='x')
	line_y, = ax.plot(data_y, label='y')
	line_z, = ax.plot(data_z, label='z')
	ax.set_ylim(-5.0, 5.0)
	ax.legend()

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

				data_x.append(float(values[0]))
				data_y.append(float(values[1]))
				data_z.append(float(values[2]))
			except ValueError:
				print(reading)

		line_x.set_ydata(data_x)
		line_y.set_ydata(data_y)
		line_z.set_ydata(data_z)
		return line_x, line_y, line_z

	ani = animation.FuncAnimation(fig, update, interval=10)
	plt.show()