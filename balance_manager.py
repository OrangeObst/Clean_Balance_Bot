import board
import adafruit_mpu6050
import math
import time
import numpy as np
from codetiming import Timer
from multiprocessing import Manager
from mpu6050 import MyMPU6050
from pid_controller import PID_Controller
from smbus2 import SMBus 


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
        self.last_time = time.perf_counter()

        '''
        -------------------- PID init --------------------
        '''
        self.balance_point = balance_point
        self.pid = PID_Controller(kp, ki, kd, min_out, max_out)

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
        self.mpu = MyMPU6050(bus)
        
        self.mpu.set_dlpf_cfg(2)
        self.mpu.set_smplrt_div(4)

        # Low pass filter
        lpf_alpha = 0.1
        self.lpf_x = LowPassFilter(lpf_alpha)
        self.lpf_y = LowPassFilter(lpf_alpha)
        self.lpf_z = LowPassFilter(lpf_alpha)
        
        self.imu_data = manager.list()
        
        self.alpha = 0.96		# For complementary filter

        # Set initial values
        accel_readings = np.array([self.mpu.MPU_ReadData() for _ in range(10)])
        accel_avg = np.mean(accel_readings, axis=0)
        initial_pitch = math.degrees(math.atan2(-accel_avg[0], math.sqrt(accel_avg[1]**2 + accel_avg[2]**2)))
        
        self.previous_pitch = initial_pitch

        self.pid.update(self.balance_point, initial_pitch, 0.001)


    # @Timer(name="Control Loop", text="Control loop: {milliseconds:.6f}ms")
    def control_loop(self):
        '''
         --------------- Collect and process IMU data ---------------
        '''
        starting_time = time.perf_counter()
        dt = starting_time - self.last_time
        self.last_time = starting_time

        accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = self.mpu.MPU_ReadData()

        # Apply low pass filter
        accel_x = self.lpf_x.filter(accel_x)
        accel_y = self.lpf_y.filter(accel_y)
        accel_z = self.lpf_z.filter(accel_z)

        # Calculate pitch from accelerometer and gyroscope
        pitch_from_acceleration = math.degrees(math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2)))
        pitch_gyro_integration = self.previous_pitch + gyro_y * dt
        # print(f'Pitch from acc: {pitch_from_acceleration}, Pitch from gyro: {pitch_gyro_integration}')
        # TODO: Why is pitch_acc > pitch_gyro
        
        # Apply complementary filter
        pitch = self.alpha * pitch_gyro_integration + (1 - self.alpha) * pitch_from_acceleration

        self.imu_data.append(pitch)
        self.previous_pitch = pitch

        '''
        --------------- Update PID controls ---------------
        '''
        speed, pterm, iterm, dterm = self.pid.update(self.balance_point, pitch, dt)
        self.pid_data.append(speed)
        self.pterms.append(pterm)
        self.iterms.append(iterm)
        self.dterms.append(dterm)

        return speed