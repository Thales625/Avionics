if __name__ == "__main__":
	from serial import Serial
	from time import time, sleep
	from utils import *
	import numpy as np
	import matplotlib.pyplot as plt
	import matplotlib.animation as animation
	from mpl_toolkits.mplot3d import Axes3D

	PORT = "COM5"
	BAUD = 115200

	# initialize serial
	ser = Serial(PORT, BAUD)

	# plot
	MAX_POINTS = 100

	# 3D plot setup
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	ax.set_xlim([-10, 10])
	ax.set_ylim([-10, 10])
	ax.set_zlim([-10, 10])

	axes = np.eye(3)

	quivers = [
		ax.quiver(0, 0, 0, 1, 0, 0, color='r', length=1, normalize=True),
		ax.quiver(0, 0, 0, 0, 1, 0, color='g', length=1, normalize=True),
		ax.quiver(0, 0, 0, 0, 0, 1, color='b', length=1, normalize=True)
	]

	# run
	ut0 = time()
	rotation = np.array([0.0, 0.0, 0.0, 1.0])
	velocity = np.array([0.0, 0.0, 0.0])
	position = np.array([0.0, 0.0, 0.0])

	def update(frame):
		global ut0, position, velocity, rotation

		while ser.in_waiting:
			# acc.x acc.y acc.z rot.x rot.y rot.z altitude
			try:
				reading = ser.readline().decode().strip()
			except UnicodeDecodeError:
				continue

			ut = time()
			dt = ut - ut0
			ut0 = ut

			try:
				values = reading.split(" ")

				if len(values) != 7: continue

				acc = np.array([float(values[0]), float(values[1]), float(values[2])])
				gyro = np.radians(np.array([float(values[3]), float(values[4]), float(values[5])]))

				# gyro integration
				rotation = integrate_quaternion(rotation, gyro, dt)

				if np.linalg.norm(acc) > 0:
					acc_norm = acc / np.linalg.norm(acc)
					q_acc = quaternion_from_vector_alignment(acc_norm, np.array([1, 0, 0]))
					rotation = slerp(rotation, q_acc, 0.02)
				
				g_sensor = rotate_vector_by_quaternion(np.array([-1, 0, 0]), rotation)

				a_linear = (acc + g_sensor) * 9.81

				velocity += a_linear * dt
				position += velocity * dt

			except ValueError:
				print(reading)

		for q in quivers: q.remove()
		quivers.clear()

		colors = ['r', 'g', 'b']
		for i in range(3):
			vec = rotate_vector_by_quaternion(axes[i], rotation)
			quivers.append(ax.quiver(position[0], position[1], position[2], -vec[2], vec[1], -vec[0], color=colors[i], length=1, normalize=True))
	
		ax.set_xlim(position[0] - 3, position[0] + 3)
		ax.set_ylim(position[1] - 3, position[1] + 3)
		ax.set_zlim(position[2] - 3, position[2] + 3)

		return quivers

	ani = animation.FuncAnimation(fig, update, interval=10, blit=False)
	plt.show()