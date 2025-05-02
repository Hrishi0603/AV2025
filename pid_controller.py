import logging

class PIDController:
    def __init__(self, Kp, Ki, Kd, max_output=None, min_output=None):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.max_output = max_output
        self.min_output = min_output
        self.integral = 0
        self.previous_error = 0

        # Set up logger
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.DEBUG)  # Set log level to DEBUG
        handler = logging.StreamHandler()  # Log to console
        formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        handler.setFormatter(formatter)
        self.logger.addHandler(handler)

        self.logger.info(f"PID Controller initialized with Kp={Kp}, Ki={Ki}, Kd={Kd}")
        self.logger.info(f"Output limits: min={min_output}, max={max_output}")

    def calculate(self, error, delta_time):
        self.logger.debug(f"Calculating with error={error:.4f}, delta_time={delta_time:.4f}s")
        proportional = self.Kp * error
        self.logger.debug(f"Proportional component: {proportional:.2f}")
        self.integral += error * delta_time
        integral_term = self.Ki * self.integral
        self.logger.debug(f"Integral component: {integral_term:.2f} (accumulated: {self.integral:.4f})")
        derivative = self.Kd * (error - self.previous_error) / delta_time
        self.logger.debug(f"Derivative component: {derivative:.2f} (error change: {error - self.previous_error:.4f})")
        self.previous_error = error
        output = proportional + integral_term + derivative
        if self.max_output is not None:
            output = min(output, self.max_output)
        if self.min_output is not None:
            output = max(output, self.min_output)
        self.logger.debug(f"Limited output: {output:.2f}")
        return output
    


# # filepath: pid_controller.py
# class PIDController:
#     def __init__(self, Kp, Ki, Kd, max_output=None, min_output=None):
#         self.Kp = Kp
#         self.Ki = Ki
#         self.Kd = Kd
#         self.max_output = max_output
#         self.min_output = min_output
#         self.integral = 0
#         self.previous_error = 0

#     def calculate(self, error, delta_time):
#         proportional = self.Kp * error
#         self.integral += error * delta_time
#         integral_term = self.Ki * self.integral
#         derivative = self.Kd * (error - self.previous_error) / delta_time
#         self.previous_error = error
#         output = proportional + integral_term + derivative

#         if self.max_output is not None:
#             output = min(output, self.max_output)
#         if self.min_output is not None:
#             output = max(output, self.min_output)

#         return output