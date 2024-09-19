import time

class PID_Controller(object):
    def __init__(self, kp, ki, kd, min_out, max_out):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.min_out = min_out
        self.max_out = max_out

        self.previous_error = 0
        self.sum_error = 0

    def update(self, setpoint, measured_value, dt):
        # Get error state
        error = setpoint - measured_value
        self.sum_error += error * dt

        # Calculate PID terms
        pterm = self.kp * error
        # self.pterms.append(pterm)
        iterm = self.ki * self.sum_error
        # self.iterms.append(iterm)
        dterm = self.kd * (error - self.previous_error) / dt
        # self.dterms.append(dterm)
        output = pterm + iterm + dterm

        output = max(self.min_out, min(self.max_out, output))
        print(f'Error: {error} = balance_point {setpoint} - current pitch {measured_value}')

        self.previous_error = error

        # pterm, iterm und dterm sind nur zum plotten
        return output, pterm, iterm, dterm
