#!/usr/bin/python3.11.2
'''

	Do not forget to source the virtual environment first with
	1.	source ~/Desktop/Clean_Balance_Bot/bin/activate
	
	TODO:
	
'''

import board
import adafruit_mpu6050
import math
import time
import matplotlib.pyplot as plt
import numpy as np
from codetiming import Timer
from multiprocessing import Process, Pipe, Manager
from SunFounder_TB6612 import TB6612
from SunFounder_PCA9685 import PCA9685

class LowPassFilter:
	def __init__(self, alpha):
		self.alpha = alpha
		self.y = 0
	
	def filter(self, x):
		self.y = self.alpha * x + (1 - self.alpha) * self.y
		return self.y


class imu_manager():

	def __init__(self):
		i2c = board.I2C()
		self.mpu = adafruit_mpu6050.MPU6050(i2c)

		self.accel = {0, 0, 0}
		self.gyro = 0
		self.angle_offset = 1.48
		self.last_time = time.time()
		
		self.gyroBias = [-0.055574134215309275, -0.005761568977347489, 0.0062436446482807125]
		manager = Manager()
		self.data = manager.list()

		lpf_alpha = 0.1		# For low pass filter
		self.lpf_x = LowPassFilter(lpf_alpha)
		self.lpf_y = LowPassFilter(lpf_alpha)
		self.lpf_z = LowPassFilter(lpf_alpha)
		
		self.alpha = 0.95		# For complementary filter

		accel_readings = np.array([self.mpu.acceleration for _ in range(10)])
		accel_avg = np.mean(accel_readings, axis=0)
		self.initial_pitch = math.degrees(math.atan2(accel_avg[0], math.sqrt(accel_avg[1]**2 + accel_avg[2]**2)))

		self.previous_pitch = self.initial_pitch
		
	'''
		TODO:
		- Try to reduce time used by getting data from MPU
	'''
	# @Timer(name="IMU Loop", text="IMU: {:.6f}")
	def readLoop(self):
		
		starting_time = time.time()
		dt = starting_time - self.last_time
		self.last_time = starting_time
		# dt = 0.01

		# Get acceleration and apply low pass filter
		raw_accel_x, raw_accel_y, raw_accel_z = self.mpu.acceleration
		filtered_accel_x = self.lpf_x.filter(raw_accel_x)
		filtered_accel_y = self.lpf_y.filter(raw_accel_y)
		filtered_accel_z = self.lpf_z.filter(raw_accel_z)

		# Integrate gyro movement
		self.gyro += self.mpu.gyro[1] * dt

		# Pitch stablefrom accelerometer
		pitch_from_acceleration = math.degrees(math.atan2(filtered_accel_x, math.sqrt(filtered_accel_y**2 + filtered_accel_z**2)))

		# Integrate gyro data to get pitch change
		pitch_gyro_integration = self.previous_pitch + (self.gyro - self.gyroBias[1]) * dt

		# Apply complementary filter
		pitch = self.alpha * pitch_gyro_integration + (1 - self.alpha) * pitch_from_acceleration
		corrected_pitch = pitch - 1.48
		# print(corrected_pitch)

		if pitch > 180:
			pitch -= 360
		elif pitch < -180:
			pitch += 360
			
		self.data.append(corrected_pitch)
		self.previous_pitch = pitch
		
		return pitch


class wheel_manager():
	def __init__(self, motor_pin, pwm_pin):
		self.motor_pin = motor_pin
		self.pwm_pin = pwm_pin

		self.motor = TB6612.Motor(self.motor_pin)

		self.pwm = PCA9685.PWM(1)
		def _set_pwm(value):
			pulse_wide = int(self.pwm.map(value, 0, 100, 0, 4095))
			self.pwm.write(self.pwm_pin, 0, pulse_wide)

		self.motor.pwm  = _set_pwm
		self.pwm.setup()
		self.motor.stop()

	# @Timer(name="Set Speed", text="Speed: {:.6f}")
	def set_speed(self, speed):
		speed = max(-100, min(100, speed))
		if speed >= 0:
			self.motor.backward()
		else:
			self.motor.forward()
		
		self.motor.speed = abs(speed)

	def stop(self):
		self.motor.stop()


class pid_manager():

	def __init__(self, kp, ki, kd, targetAngle=0, angle_offset=0, min_out=-100, max_out=100, scaling_factor=1):
		self.kp = kp * scaling_factor
		self.ki = ki * scaling_factor
		self.kd = kd * scaling_factor
		self.error = 0
		self.previous_error = 0
		self.sum_error = 0
		self.last_time = time.time()
		
		self.setpoint_bias = angle_offset # 2.057713965631952
		self.setpoint = targetAngle
		
		self.min_out = min_out
		self.max_out = max_out

		manager = Manager()
		self.data = manager.list()
		self.pterms = manager.list()
		self.iterms = manager.list()
		self.dterms = manager.list()


	'''
		TODO:
		- Find a way to solve the problem of varying time influencing the derivative term
	'''
	# @Timer(name="PID Loop", text="PID: {:.6f}")	
	def update(self, measured_value):

		# dt = 0.01
		starting_time = time.time()
		dt = starting_time - self.last_time
		# print(dt)
		self.error = (self.setpoint + self.setpoint_bias) - measured_value
		self.sum_error += self.error * dt

		pterm = self.kp * self.error
		self.pterms.append(pterm)
		iterm = self.ki * self.sum_error
		self.iterms.append(iterm)
		dterm = self.kd * (self.error - self.previous_error) / dt
		self.dterms.append(dterm)
		output = pterm + iterm + dterm

		output = max(self.min_out, min(self.max_out, output))

		self.previous_error = self.error
		self.last_time = starting_time

		self.data.append(output)
		return output


def stackplot_pid_values(pid : pid_manager):
	fig, left_ax = plt.subplots(figsize = (10, 10))
	
	p_values = len(pid.pterms)
	i_values = len(pid.iterms)
	d_values = len(pid.dterms)

	smaller_value = min(p_values, i_values, d_values)
	print(f'Datapoints: {smaller_value}')
	time_values = np.linspace(0, 10, smaller_value)

	left_ax.set_xlim(0, 10)
	left_ax.set_ylim(-100, 100)
	
	y = np.vstack([pid.pterms, pid.iterms, pid.dterms])
	left_ax.stackplot(time_values, y)

	#plt.show()
	plt.savefig('stackplot.png')


def plot_angle_speed(imu : imu_manager, pid : pid_manager):
	fig, left_ax = plt.subplots(figsize = (10, 10))
	right_ax = left_ax.twinx()

	time_values = np.linspace(0, 10, len(imu.data))

	p1, = left_ax.plot(time_values, imu.data, "b-")
	p2, = right_ax.plot(time_values, pid.data, "r-")

	left_ax.set_xlim(0, 10)
	left_ax.set_ylim(-5, 5)
	right_ax.set_ylim(-110, 110)

	left_ax.set_xlabel('Time (s)')
	left_ax.set_ylabel('Angle (°)')
	right_ax.set_ylabel('Speed')

	left_ax.yaxis.label.set_color(p1.get_color())
	right_ax.yaxis.label.set_color(p2.get_color())

	#plt.show()
	plt.savefig('speedplot.png')


@Timer(name="Calc Loop", text="Calc loop: {:.6f}")
def process_calculation(imu : imu_manager, pid : pid_manager, conn_right=None, conn_left=None):
	timer = time.time()
	while((time.time() - timer) < 10):
		# loop_time = time.time() + 0.01

		pitch_value = imu.readLoop()
		speed_value = pid.update(pitch_value)
		# print(f'pitch: {pitch_value}, last time: {imu.last_time}, gyro: {imu.gyro}')
		if conn_right and conn_left is not None:
			conn_right.send(speed_value)
			conn_left.send(speed_value)

	if conn_right and conn_left is not None:
		conn_right.send('Done')
		conn_left.send('Done')
		conn_right.close()
		conn_left.close()


@Timer(name="Motor Loop", text="Motor: {:.6f}")
def motor_control(wm : wheel_manager, conn):
	while True:
		speed = conn.recv()
		if(speed == 'Done'):
			break
		wm.set_speed(int(speed))

	conn.close()
	wm.stop()

'''
	Motor_A = 17
	Motor_B = 27

	PWM_A = 4
	PWM_B = 5
'''
if(__name__ == "__main__"):
	# (P, I, D, target_angle, angle_offset, min_out, max_out, scaling_factor)
	pid = pid_manager(15, 0.7, 2.85, 0.0, 1.48, -100, 100, 1)
	imu = imu_manager()

	use_wheels = False
	if use_wheels:
		right_wheel = wheel_manager(17, 4)
		left_wheel = wheel_manager(27, 5)
	
		parent_conn_right, child_conn_right = Pipe()
		parent_conn_left, child_conn_left = Pipe()
		try:
			imu_process = Process(target=process_calculation, args=(imu, pid, parent_conn_right, parent_conn_left))
			right_motor_process = Process(target=motor_control, args=(right_wheel, child_conn_right))
			left_motor_process = Process(target=motor_control, args=(left_wheel, child_conn_left))

		finally:
			right_wheel.stop()
			left_wheel.stop()
	else:
		imu_process = Process(target=process_calculation, args=(imu, pid))

	imu_process.start()
	if use_wheels:
		right_motor_process.start()
		left_motor_process.start()
	
	imu_process.join()
	if use_wheels:
		right_motor_process.join()
		left_motor_process.join()

	stackplot_pid_values(pid)
	plot_angle_speed(imu, pid)
	
