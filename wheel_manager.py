#!/usr/bin/python3.11.2
'''

    This class is derived from the SunFounder Picar-V backwheel class
    and represents a single wheel. This allows threads to take control of
    one wheel each.
	
	TODO:
	
'''

from SunFounder_TB6612 import TB6612
from SunFounder_PCA9685 import PCA9685
from codetiming import Timer
from multiprocessing import Lock

class Wheel_Manager(object):
	def __init__(self, motor_pin, pwm_pin):
		self.lock = Lock()

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

	'''
		set_speed
		- Sets the speed of this motor
		- a speed of 0 does not stop the motors, as it'd cause delay
			when starting them again
	'''
	@Timer(name="Set Speed", text="Speed: {milliseconds:.6f}ms")
	def set_speed(self, speed):
		with self.lock:
			speed = max(-100, min(100, speed))
			if speed <= 0:
				self.motor.backward()
			else:
				self.motor.forward()

			self.motor.speed = abs(speed)

	def stop(self):
		self.motor.stop()


if __name__ == "__main__":
	pass