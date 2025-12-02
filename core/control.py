import numpy as np


# defined PID regulator for the thurst to keep climb at a set angle
class PIDController:
    def __init__(
        self,
        Kp=1,
        Ki=0.5,
        Kd=0.01,
        target_angle=15.0,
        output_limits=(0, 500),
        tau=0.05,
        max_angle=20,
    ):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.target_angle = target_angle
        self.min_thrust, self.max_thrust = output_limits
        self.tau = tau
        self.max_angle = max_angle

        # Internal variables
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_deriv = 0.0
        self.prev_time = 0.0
        self.old_value = 0

    # update loop for the regulator to give thrust needed for set angle
    def update(self, measured_angle, dt):

        measured_angle = (
            measured_angle * 0.1 + self.old_value * 0.9
        )  # fiter meassured thrust through the last iteration

        if dt <= 0:
            dt = 1e-6  # a low dt for a samll error

        # the fault between our setpoint angle and our current angle
        error = self.target_angle - measured_angle

        # proportions term
        P = self.Kp * error

        # integral term
        integral_temp = self.integral + self.Ki * error * dt
        I = integral_temp

        # derive term with a low pass filter for lower disturbance
        deriv_unfiltered = (error - self.prev_error) / dt
        deriv_filtered = (self.tau * self.prev_deriv + dt * deriv_unfiltered) / (
            self.tau + dt
        )
        D = self.Kd * deriv_filtered

        # Total thrust we want to use
        new_thrust = P + I + D

        # Apply thrust limit to not over burden the motor
        if new_thrust > self.max_thrust:
            new_thrust = self.max_thrust
        elif new_thrust < self.min_thrust:
            new_thrust = self.min_thrust
        else:
            self.integral = integral_temp

        # Uppdate the internal values for the next iteration
        self.prev_error = error
        self.prev_deriv = deriv_filtered
        self.old_value = measured_angle
        return new_thrust  # return a new thrust for our desired angle

    def sensor(self, x, y):
        # a simulated sensor that meassures the current angle
        current_angle = np.arctan2(y, x)
        current_angle = np.clip(
            current_angle, -self.max_angle, self.max_angle
        )  # limit our angle between some reasnoable value

        if -0.001 < current_angle < 0.001:  # low error near 0 angle
            sigma = 0
        else:
            sigma = 0.015  # 1.5% fault to simulate a sensor meassurment error

        # to have a standrad diviated angle fault
        noise = np.random.normal(
            -(sigma / 2 * abs(current_angle)), sigma / 2 * abs(current_angle)
        )

        return (
            current_angle + noise
        )  # a small meassure error for our angle meassurement
