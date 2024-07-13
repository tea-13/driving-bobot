from dataclasses import dataclass
from serial.tools import list_ports
import serial
import matplotlib.pyplot as plt
import numpy as np
import time



@dataclass
class LidarData:
	"""LidarDate: The class responsible for storing data samples from the lidar"""
	angle: int
	distance: int
	speed: int

	def __str__(self) -> str:
		return f'angle: {self.angle}, distance: {self.distance}, speed: {self.speed}.'

	def update(self, _distance: int, _speed: int):
		self.distance = _distance
		self.speed = _speed



class LidarDataList:
	"""LidarDataList: ..."""
	def __init__(self):
		self.MAX_DATA_SIZE = 360    # resolution : 1 degree
		self.data_mean = [LidarData(ang, 0, 0) for ang in range(360)]
		self.data = {ang : [] for ang in range(360)}

	def parse(self, _sensor_data: list) -> None:
		try:
			sensor_data = list(map(int, _sensor_data))
			current_angle = sensor_data[0]
			current_speed = sensor_data[1]
		

			for i, dist in enumerate(sensor_data[2:6]):
				ang = (current_angle + i) % self.MAX_DATA_SIZE
				self.data[ang].append([dist, current_speed])
				self.filter(ang)

		except Exception as e:
			print(f'Error during parse data: {e}')
			print(f'Parse data: {_sensor_data}]')


	def filter(self, _ang: int) -> None:
		if len(self.data[_ang]) == 3:
			sample_distances =	[]
			sample_speeds = []

			for it in self.data[_ang]:
				sample_distances.append(it[0])
				sample_speeds.append(it[1])

			self.data_mean[_ang].update(
				int(np.median(sample_distances)), 
				int(np.median(sample_speeds))
			)

			self.clear(_ang)

	def clear(self, _ang: int) -> None:
		self.data[_ang].clear()



class Lidar:
	"""Lidar: The class responsible for connecting to the lidar"""
	def __init__(self, _port : str, _baudrate : int):
		self.DATA_LENGTH = 7        # data length : angle, speed, distance 1 - 4, checksum
		self.port = _port          # SPECIFY serial port 
		self.ser = None
		self.BAUDRATE = _baudrate
		self.data = LidarDataList()

	def connect_serial(self) -> None:
		try:
			if self.ser is None or not self.ser.is_open:
				self.ser = serial.Serial(self.port, self.BAUDRATE, timeout=1.0)
				self.ser.reset_input_buffer()
				print(f'Connect to port: {self.port}')
		
		except Exception as e:
			print(f'Error during connection: {e}')

			if self.ser.is_open:
				self.ser.close()

			print('5 seconds wait')
			time.sleep(5)
			self.connect_serial()

	def read_serial(self) -> None:
		try:
			self.connect_serial()

			if self.ser.in_waiting > 0:

				try: # try read serial inputs; if unsuccessful, ignore the current input data
					line = self.ser.readline().decode().rstrip()
					sensor_data = line.split('\t')  

				except: # ignore if the data is invalid 
					print('Invalid data')

				if len(sensor_data) == self.DATA_LENGTH:
					self.data.parse(sensor_data)

		except KeyboardInterrupt:
			if self.ser.is_open:
				self.ser.close()

			exit('Programs close')

	def clear_data(self):
		self.data.clear()



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
			vis.plot_lidar_data(lidar.data.data_mean)
			# lidar.clear_data()
		except KeyboardInterrupt:
			exit('Exit')
