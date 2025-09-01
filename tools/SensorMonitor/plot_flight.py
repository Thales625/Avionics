if __name__ == "__main__":
	from serial import Serial
	import numpy as np

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
	data_baro = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)
	# data_acc = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)

	# matplotlib
	fig, ax = plt.subplots()
	ax.set_title("Flight")
	line_baro, = ax.plot(data_baro, label='baro')
	# line_acc, = ax.plot(data_acc, label='acc')
	# ax.set_ylim(-5.0, 5.0)
	ax.legend()

	# run
	def update(frame):
		while ser.in_waiting:
			try:
				reading = ser.readline().decode().strip()
			except UnicodeDecodeError:
				continue

			if reading[0] != 'd': continue

			try:
				values = reading[1:].split(" ")

				data_baro.append(float(values[0]))
			except ValueError:
				print(reading)

		line_baro.set_ydata(data_baro)
		return line_baro

	ani = animation.FuncAnimation(fig, update, interval=10)
	plt.show()