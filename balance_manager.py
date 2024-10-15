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
    def __init__(self, balance_point, desired_speed): # self, kp, ki, kd, min_out=-100, max_out=100, balance_point=0
        self.last_time = time.perf_counter()

        '''
        -------------------- PID init --------------------
        '''
        self.balance_point = balance_point
        self.desired_speed = desired_speed

        # Outer loop speed_pid takes desired speed and outputs desired angle
        # self.speed_pid = PID_Controller(23, 0.4, 0.9, -100, 100, self.desired_speed) # 23, 0.4, 0.9, -100, 100, -2.50

        # Inner loop angle_pid takes desired angle and outputs the motor speed
            # actually angle_pid, but currently speed_pid for convenience
        self.speed_pid = PID_Controller(28, 0.4, 0.7, self.balance_point) # 35, 0.4, 0.7
        self.min_speed = -100
        self.max_speed = 100

        '''
        -------------------- IMU init --------------------
        '''
        # MPU with bus 1, digital low pass filter 2 and samplerate divisor 4
        bus = SMBus(1)
        self.mpu = MyMPU6050(bus)
        
        self.mpu.set_dlpf_cfg(2)
        self.mpu.set_smplrt_div(4)

        self.mpu.set_accel_offset(0.031837708, 0.024646467, 1.064992507)
        self.mpu.set_gyro_offset(-0.024843416, -0.002446569, 0.004664117)
        '''
        OFFSET AX 0.03183770817630166, AY 0.024646467493363834, AZ 1.064992507951752
        OFFSET GX -0.024843416633825305, GY -0.002446569556977695, GZ 0.004664117503684159
        '''
        # Low pass filter
        lpf_alpha = 0.1
        self.lpf_x = LowPassFilter(lpf_alpha)
        self.lpf_y = LowPassFilter(lpf_alpha)
        self.lpf_z = LowPassFilter(lpf_alpha)
        
        self.alpha = 0.96		# For complementary filter
        
        '''
        -------------------- Extras --------------------
        '''
        manager = Manager()
        self.speed_pid_data = manager.list()
        self.pterms = manager.list()
        self.iterms = manager.list()
        self.dterms = manager.list()

        self.imu_data = manager.list()

        # Set initial values
        accel_readings = np.array([self.mpu.MPU_ReadData() for _ in range(10)])
        accel_avg = np.mean(accel_readings, axis=0)
        initial_pitch = math.degrees(math.atan2(-accel_avg[0], math.sqrt(accel_avg[1]**2 + accel_avg[2]**2)))

        self.previous_pitch = initial_pitch

        self.speed_pid.update(self.previous_pitch, 0.001)


    # @Timer(name="Control Loop", text="Control loop: {milliseconds:.6f}ms")
    def control_loop(self):
        '''
         --------------- Collect and process IMU data ---------------
        '''
        starting_time = time.perf_counter()
        dt = starting_time - self.last_time
        self.last_time = starting_time
        # print(dt)

        accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = self.mpu.MPU_ReadData()

        # Apply low pass filter
        accel_x = self.lpf_x.filter(accel_x)
        accel_y = self.lpf_y.filter(accel_y)
        accel_z = self.lpf_z.filter(accel_z)

        # Calculate pitch from accelerometer and gyroscope
        pitch_from_acceleration = math.degrees(math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2)))

        pitch_gyro_integration = self.previous_pitch + gyro_y * dt
        # print(f'Pitch from acc: {pitch_from_acceleration}, Pitch from gyro: {pitch_gyro_integration}')
        
        # Apply complementary filter
        pitch = self.alpha * pitch_gyro_integration + (1 - self.alpha) * pitch_from_acceleration
        print(f'{pitch:.3f}')

        self.imu_data.append(pitch)
        self.previous_pitch = pitch

        '''
        --------------- Update PID controls ---------------
        '''
        speed, pterm, iterm, dterm = self.speed_pid.update(pitch, dt)
        speed = max(self.min_speed, min(self.max_speed, speed))

        self.speed_pid_data.append(speed)
        self.pterms.append(pterm)
        self.iterms.append(iterm)
        self.dterms.append(dterm)

        return speed