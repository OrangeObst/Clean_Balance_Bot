import board
import adafruit_mpu6050
import math
import time
import numpy as np
from codetiming import Timer
from multiprocessing import Manager
import mputest
from smbus2 import SMBus 


_DEBUG = False

class LowPassFilter:
		def __init__(self, alpha):
			self.alpha = alpha
			self.y = 0
		
		def filter(self, x):
			self.y = self.alpha * x + (1 - self.alpha) * self.y
			return self.y


class Speed_Calculator(object):
	# 			(		P, I, D,  min_out, 		max_out, 	 	balance_point)
	def __init__(self, kp, ki, kd, min_out=-100, max_out=100, balance_point=0):
		'''
		-------------------- PID values --------------------
		'''
		self.kp = kp
		self.ki = ki
		self.kd = kd

		self.error = 0
		self.previous_error = 0
		self.sum_error = 0
		self.last_time = time.perf_counter()
		
		self.min_out = min_out
		self.max_out = max_out

		manager = Manager()
		self.pid_data = manager.list()
		self.pterms = manager.list()
		self.iterms = manager.list()
		self.dterms = manager.list()

		'''
		-------------------- IMU init --------------------
		'''
		# MPU with bus 1, digital low pass filter 2 and samplerate divisor 4
		bus = SMBus(1)
		self.mpu = mputest.MyMPU6050(bus)
		
		self.mpu.set_dlpf_cfg(2)
		self.mpu.set_smplrt_div(4)

		# Low pass filter
		lpf_alpha = 0.1		# For low pass filter
		self.lpf_x = LowPassFilter(lpf_alpha)
		self.lpf_y = LowPassFilter(lpf_alpha)
		self.lpf_z = LowPassFilter(lpf_alpha)


		self.pitch_offset = 0.9
		self.balance_point = balance_point
		
		self.imu_data = manager.list()
		
		self.alpha = 0.96		# For complementary filter

		# Set initial values
		accel_readings = np.array([self.mpu.MPU_ReadData() for _ in range(10)])
		accel_avg = np.mean(accel_readings, axis=0)
		self.initial_pitch = math.degrees(math.atan2(-accel_avg[0], math.sqrt(accel_avg[1]**2 + accel_avg[2]**2)))
		# self.initial_pitch = self.initial_pitch - self.pitch_offset
		
		self.previous_error = self.balance_point - self.initial_pitch
		self.previous_pitch = self.initial_pitch


	# @Timer(name="Control Loop", text="Control loop: {milliseconds:.6f}ms")
	def control_loop(self):
		'''
		 --------------- Collect and process IMU data ---------------
		'''

		starting_time = time.perf_counter()
		dt = starting_time - self.last_time
		self.last_time = starting_time

		accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = self.mpu.MPU_ReadData()
		# print(f"Raw IMU Data - Accel: ({accel_x}, {accel_y}, {accel_z}), Gyro: ({gyro_x}, {gyro_y}, {gyro_z})")
		accel_x = self.lpf_x.filter(accel_x)
		accel_y = self.lpf_y.filter(accel_y)
		accel_z = self.lpf_z.filter(accel_z)

		# Pitch from accelerometer
		pitch_from_acceleration = math.degrees(math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2)))
		
		# Pitch from gyroscope
		pitch_gyro_integration = self.previous_pitch + gyro_y * dt
		print(f'Pitch from acc: {pitch_from_acceleration}, Pitch from gyro: {pitch_gyro_integration}')
		
		# Apply complementary filter
		pitch = self.alpha * pitch_gyro_integration + (1 - self.alpha) * pitch_from_acceleration
		current_angle = pitch - self.pitch_offset
		# print(f'Pitch: {pitch} - Corrected pitch: {current_angle}')

		self.imu_data.append(current_angle)
		self.previous_pitch = pitch

		'''
		--------------- Update PID controls ---------------
		'''

		# Get error state
		self.error = self.balance_point - current_angle
		self.sum_error += self.error * dt

		# Calculate PID terms
		pterm = self.kp * self.error
		self.pterms.append(pterm)
		iterm = self.ki * self.sum_error
		self.iterms.append(iterm)
		dterm = self.kd * (self.error - self.previous_error) / dt
		self.dterms.append(dterm)
		output = pterm + iterm + dterm

		output = max(self.min_out, min(self.max_out, output))
		print(f'Error: {self.error} = balance_point {self.balance_point} - corr pitch {current_angle}')

		self.previous_error = self.error

		self.pid_data.append(output)

		return output
