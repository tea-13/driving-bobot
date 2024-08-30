from serial.tools import list_ports
import matplotlib.pyplot as plt
import numpy as np

from LIDAR import Lidar


class Visualization:
	"""Visualization: lidar data visualization class"""
	def __init__(self):
		self.MAX_DISTANCE = 3000    # in mm
		self.MIN_DISTANCE = 100     # in mm
		self.fig = plt.figure(dpi=200)
		self.ax = self.fig.add_subplot(projection='polar')


	def plot_lidar_data(self, _data: list):
		angles = np.deg2rad([it.angle for it in _data])
		distances = np.array([it.distance for it in _data])

		self.ax.clear() # clear current plot
		plt.plot(angles, distances, ".")
		self.ax.set_rmax(self.MAX_DISTANCE)
		plt.draw()
		plt.pause(0.001)


def choice_port() -> str:
	ports = list_ports.comports()

	if len(ports) < 1:
		exit('No ports')

	for i, p in enumerate(ports):
		print(f'{i+1}. {p}')

	choice = input('Enter port name: ')

	return choice



if __name__ == '__main__':
	port = choice_port()

	lidar = Lidar(port, 115200)
	vis = Visualization()

	while True:
		try:
			lidar.read_serial()
			vis.plot_lidar_data(lidar.data.filtering_data)
			# lidar.clear_data()
		except KeyboardInterrupt:
			exit('Exit: ctrl + C')
