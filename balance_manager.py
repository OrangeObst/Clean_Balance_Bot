import board
import adafruit_mpu6050
import math
import time
import numpy as np
from codetiming import Timer
from multiprocessing import Manager
import mputest
from smbus2 import SMBus 


class LowPassFilter:
		def __init__(self, alpha):
			self.alpha = alpha
			self.y = 0
		
		def filter(self, x):
			self.y = self.alpha * x + (1 - self.alpha) * self.y
			return self.y


class Speed_Calculator(object):
	# 			(		P, I, D, 	target_angle, min_out, 		max_out, 	scaling_factor, 	angle_offset)
	def __init__(self, kp, ki, kd, targetAngle=0, min_out=-100, max_out=100, scaling_factor=1, angle_offset=0):
		self.timer = Timer("test", text="Control loop: {milliseconds:.6f} ms")
		self.timer2 = Timer("test2", text="Control loop2: {milliseconds:.6f} ms")
		self.timer3 = Timer("test3", text="Control loop3: {milliseconds:.6f} ms")
		self.timer4 = Timer("test4", text="Control loop4: {milliseconds:.6f} ms")
		self.timer5 = Timer("test5", text="Control loop5: {milliseconds:.6f} ms")

		'''
		--------------- PID values ---------------
		'''

		self.kp = kp * scaling_factor
		self.ki = ki * scaling_factor
		self.kd = kd * scaling_factor

		self.error = 0
		self.previous_error = 0
		self.sum_error = 0
		self.last_time = time.perf_counter()
		
		self.setpoint = targetAngle
		
		self.min_out = min_out
		self.max_out = max_out

		manager = Manager()
		self.pid_data = manager.list()
		self.pterms = manager.list()
		self.iterms = manager.list()
		self.dterms = manager.list()


		'''
		--------------- IMU init ---------------
		'''

		bus = SMBus(1)
		self.mpu = mputest.MyMPU6050(bus)
		
		self.mpu.set_dlpf_cfg(2)
		self.mpu.set_smplrt_div(4)

		lpf_alpha = 0.1		# For low pass filter
		self.lpf_x = LowPassFilter(lpf_alpha)
		self.lpf_y = LowPassFilter(lpf_alpha)
		self.lpf_z = LowPassFilter(lpf_alpha)

		accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = self.mpu.MPU_ReadData()

		self.gyro = 0
		
		self.angle_offset = angle_offset
		
		self.imu_data = manager.list()
		
		self.alpha = 0.95		# For complementary filter

		accel_readings = np.array([self.mpu.MPU_ReadData() for _ in range(10)])
		accel_avg = np.mean(accel_readings, axis=0)
		self.initial_pitch = math.degrees(math.atan2(accel_avg[0], math.sqrt(accel_avg[1]**2 + accel_avg[2]**2)))
		
		self.previous_pitch = self.initial_pitch


	# @Timer(name="Control Loop", text="Control loop: {milliseconds:.6f}ms")
	def control_loop(self):
		# self.timer.start()
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

		self.gyro = gyro_y * dt
		# print(self.gyro)

		# Pitch stablefrom accelerometer
		pitch_from_acceleration = math.degrees(math.atan2(accel_x, math.sqrt(accel_y**2 + accel_z**2)))
		
		# Integrate gyro data to get pitch change
		pitch_gyro_integration = self.previous_pitch + self.gyro
		# print(f'Pitch from acc: {pitch_from_acceleration}, Pitch from gyro: {pitch_gyro_integration}')
		
		# Apply complementary filter
		pitch = self.alpha * pitch_gyro_integration + (1 - self.alpha) * pitch_from_acceleration
		corrected_pitch = pitch - self.angle_offset
		# print(f'Pitch: {pitch} - Corrected pitch: {corrected_pitch}')

		self.imu_data.append(corrected_pitch)
		self.previous_pitch = pitch
		# self.timer4.stop()
		'''
		--------------- Update PID controls ---------------
		'''
		# self.timer5.start()
		self.error = self.setpoint - corrected_pitch
		self.sum_error += self.error * dt

		pterm = self.kp * self.error
		self.pterms.append(pterm)
		iterm = self.ki * self.sum_error
		self.iterms.append(iterm)
		dterm = self.kd * (self.error - self.previous_error) / dt
		self.dterms.append(dterm)
		output = pterm + iterm + dterm

		output = max(self.min_out, min(self.max_out, output))
		print(f'Error: {self.error} = setpoint {self.setpoint} - corr pitch {corrected_pitch}')
		print(f'pterm: {pterm}, iterm {iterm} dterm {dterm} output {output}')
		self.previous_error = self.error

		self.pid_data.append(output)
		# self.timer5.stop()
		# self.timer.stop()
		return output

	def mean_time(self):
		# print(f'Mean: {self.timer.timers.mean("test")}, stdev: {self.timer.timers.stdev("test")}')
		# print(f'Mean2: {self.timer.timers.mean("test2")}, stdev2: {self.timer.timers.stdev("test2")}')
		# print(f'Mean3 {self.timer.timers.mean("test3")}, stdev3: {self.timer.timers.stdev("test3")}')
		# print(f'Mean4: {self.timer.timers.mean("test4")}, stdev4: {self.timer.timers.stdev("test4")}')
		# print(f'Mean5: {self.timer.timers.mean("test5")}, stdev5: {self.timer.timers.stdev("test5")}')
		pass
