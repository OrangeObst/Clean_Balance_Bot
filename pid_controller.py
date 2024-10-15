import time

class PID_Controller(object):
    def __init__(self, kp, ki, kd, setpoint):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.setpoint = setpoint

        self.previous_error = 0
        self.sum_error = 0

    def update(self, measured_value, dt):
        # Get error state
        error = self.setpoint - measured_value
        self.sum_error += error * dt

        # Calculate PID terms
        pterm = self.kp * error
        iterm = self.ki * self.sum_error
        dterm = self.kd * (error - self.previous_error) / dt

        output = pterm + iterm + dterm

        print(f'Error: {error:f} = {self.setpoint} - {measured_value:f} balance_point - current pitch')

        self.previous_error = error

        # pterm, iterm und dterm sind nur zum plotten
        return output, pterm, iterm, dterm
