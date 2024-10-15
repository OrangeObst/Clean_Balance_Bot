import time
import matplotlib.pyplot as plt
import numpy as np
from codetiming import Timer
from multiprocessing import Process, Pipe, Manager
import balance_manager
import wheel_manager

def stackplot_pid_values(bm : balance_manager):
	fig, left_ax = plt.subplots(figsize = (10, 10))
	
	p_values = len(bm.pterms)
	i_values = len(bm.iterms)
	d_values = len(bm.dterms)

    # Need size of smallest dataset to be able to plot properly
	smaller_value = min(p_values, i_values, d_values)
	print(f'Datapoints: {smaller_value}')
	time_values = np.linspace(0, 10, smaller_value)

	left_ax.set_xlim(0, 10)
	left_ax.set_ylim(-100, 100)
	
	color_map = ["#0000FF", "#00FF00", "#FF0000"]
	y = np.vstack([bm.pterms, bm.iterms, bm.dterms])
	left_ax.stackplot(time_values, y, colors = color_map)

	#plt.show()
	plt.savefig('stackplot.png')

def subplot_p_i_d_values(bm : balance_manager):
	fig, axs = plt.subplots(3, figsize = (10, 10))
	
	p_values = len(bm.pterms)
	d_values = len(bm.dterms)

    # Need size of smallest dataset to be able to plot properly
	smaller_value = min(p_values, d_values)
	print(f'Datapoints: {smaller_value}')
	time_values = np.linspace(0, 10, smaller_value)

	axs[0].set_xlim(0, 10)
	axs[0].set_ylim(-100, 100)
	axs[1].set_xlim(0, 10)
	axs[1].set_ylim(-100, 100)
	axs[2].set_xlim(0, 10)
	axs[2].set_ylim(-100, 100)
	
	p = np.vstack([bm.pterms])
	i = np.vstack([bm.iterms])
	d = np.vstack([bm.dterms])
	axs[0].stackplot(time_values, p)
	axs[0].set_title('Proportional')
	axs[1].stackplot(time_values, i)
	axs[1].set_title('Integral')
	axs[2].stackplot(time_values, d)
	axs[2].set_title('Derivativ')
	#plt.show()
	plt.savefig('subplot.png')

def plot_angle_speed(bm : balance_manager):
	fig, left_ax = plt.subplots(figsize = (10, 10))
	right_ax = left_ax.twinx()

	time_values = np.linspace(0, 10, len(bm.imu_data))

	p1, = left_ax.plot(time_values, bm.imu_data, "b-")
	p2, = right_ax.plot(time_values, bm.speed_pid_data, "r-")
	left_ax.axhline(y=bm.balance_point, color='k', linestyle='--', label=f'Balancing point at {bm.balance_point}°')

	# Find high and low points
	high_point = np.max(bm.imu_data)
	low_point = np.min(bm.speed_pid_data)
	high_point_index = np.argmax(bm.imu_data)
	low_point_index = np.argmin(bm.speed_pid_data)

	# Add vertical lines at high and low points
	left_ax.axvline(x=time_values[high_point_index], color='g', linestyle='--', label=f'High point at {high_point}°')
	left_ax.axvline(x=time_values[low_point_index], color='m', linestyle='--', label=f'Low point at {low_point}°')

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

def plot_all_in_one(bm : balance_manager):
	fig, left_ax = plt.subplots(figsize = (10, 10))
	right_ax = left_ax.twinx()

	p_values = len(bm.pterms)
	i_values = len(bm.iterms)
	d_values = len(bm.dterms)

    # Need size of smallest dataset to be able to plot properly
	smaller_value = min(p_values, i_values, d_values)
	print(f'Datapoints: {smaller_value}')
	time_values = np.linspace(0, 10, smaller_value)

	left_ax.set_xlim(0, 10)
	left_ax.set_ylim(-5, 5)
	right_ax.set_ylim(-110, 110)
	
	y = np.vstack([bm.pterms, bm.iterms, bm.dterms])
	right_ax.stackplot(time_values, y)
	p2, = right_ax.plot(time_values, bm.speed_pid_data, "r-")
	p1, = left_ax.plot(time_values, bm.imu_data, "b-")

	left_ax.set_ylabel('Angle (°)')
	right_ax.set_ylabel('Speed')

	left_ax.yaxis.label.set_color(p1.get_color())
	right_ax.yaxis.label.set_color(p2.get_color())

	#plt.show()
	plt.savefig('Stats.png')


@Timer(name="Calc Loop", text="Calc loop: {:.6f}s")
def process_calculation(bm : balance_manager, conn_right=None, conn_left=None):
	timer = time.time()
	while((time.time() - timer) < 5):
		loop_time = time.time() + 0.01

		speed_value = bm.control_loop()

		if conn_right is not None and conn_left is not None:
			conn_right.send(speed_value)
			conn_left.send(speed_value)

		while(time.time() < loop_time):
			# time.sleep(0.0001)
			pass

	if conn_right is not None and conn_left is not None:
		conn_right.send('Done')
		conn_left.send('Done')
		conn_right.close()
		conn_left.close()
		


@Timer(name="Motor Loop", text="Motor: {:.6f}s")
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
if __name__ == "__main__":
	# (P, I, D, target_angle, min_out, max_out, balance_point) 0.83
	# TODO: Fix issue with balance point / pitch offset / setpoint .. 
	bm = balance_manager.Speed_Calculator(-2.76, 0.0) # 23, 0.4, 0.9, -100, 100, -2.50

	use_wheels = False
	if use_wheels:
		right_wheel = wheel_manager.Wheel_Manager(17, 4)
		left_wheel = wheel_manager.Wheel_Manager(27, 5)
	
		parent_conn_right, child_conn_right = Pipe()
		parent_conn_left, child_conn_left = Pipe()
		try:
			bm_process = Process(target=process_calculation, args=(bm, parent_conn_right, parent_conn_left))
			right_motor_process = Process(target=motor_control, args=(right_wheel, child_conn_right))
			left_motor_process = Process(target=motor_control, args=(left_wheel, child_conn_left))

		finally:
			right_wheel.stop()
			left_wheel.stop()
	else:
		bm_process = Process(target=process_calculation, args=(bm,))

	bm_process.start()
	if use_wheels:
		right_motor_process.start()
		left_motor_process.start()

	bm_process.join()
	if use_wheels:
		right_motor_process.join()
		left_motor_process.join()

	subplot_p_i_d_values(bm)
	stackplot_pid_values(bm)
	plot_angle_speed(bm)
	# plot_all_in_one(bm)

