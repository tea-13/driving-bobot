from dataclasses import dataclass
import time

import serial
import numpy as np


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
		self.filtering_data = [LidarData(ang, 0, 0) for ang in range(360)]
		self.data = {ang : [] for ang in range(360)}

		# Data for filtering
		# Moving average
		self.N = 4
		self.alpha = 2 / (self.N+1)
		# Kalman
		var_data = 0.25
		reaction_speed = 0.05

	
	def parse(self, _sensor_data: list) -> None:
		try:
			sensor_data = list(map(int, _sensor_data))
			current_angle = sensor_data[0]
			current_speed = sensor_data[1]
		

			for i, dist in enumerate(sensor_data[2:6]):
				ang = (current_angle + i) % self.MAX_DATA_SIZE
				self.data[ang].append([dist, current_speed])
				# self.median_filter(ang)
				self.moving_average_filter(ang)
				# self.kalman_filter(ang)
				# self.none_filter(ang)


		except Exception as e:
			print(f'Error during parse data: {e}')
			print(f'Parse data: {_sensor_data}]')


	
	def median_filter(self, _ang: int) -> None:
		if len(self.data[_ang]) == 3:
			sample_distances =	[]
			sample_speeds = []

			for it in self.data[_ang]:
				sample_distances.append(it[0])
				sample_speeds.append(it[1])

			self.filtering_data[_ang].update(
				int(np.median(sample_distances)), 
				int(np.median(sample_speeds))
			)

			self.clear(_ang)


	def moving_average_filter(self, _ang: int) -> None:
		if len(self.data[_ang]) == 2:

			ema_distance = self.alpha*self.data[_ang][1][0] + (1 - self.alpha)*self.filtering_data[_ang].distance
			ema_speed = self.alpha*self.data[_ang][1][1] + (1 - self.alpha)*self.filtering_data[_ang].speed

			self.filtering_data[_ang].update(
				ema_distance,
				ema_speed
			)

			self.clear(_ang)


	def kalman_filter(self, _ang: int) -> None:
		pass


	def none_filter(self, _ang: int) -> None:
		self.filtering_data[_ang].update(
			self.data[_ang][-1][0], 
			self.data[_ang][-1][1]
		)

		if len(self.data[_ang]) >= 20:
			self.clear(_ang)

	
	def clear(self, _ang: int) -> None:
		self.data[_ang].clear()


# Его надо синглтоном сделать!
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


	def __del__(self):
		if not(self.ser is None) and self.ser.is_open:
			self.ser.close()

