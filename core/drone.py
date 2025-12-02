import numpy as np
import time
import os
from scrape import get_cl_cd
from control import PIDController


class Drone:
    def __init__(
        self,
        mass=5.0,
        v_thresh=1e-6,
        wing_area=0.8,
        thrust=150.0,
        battery_capacity_wh=500.0,
        path="../prop/aero",
    ):
        self.mass = mass
        self.v_thresh = v_thresh
        self.wing_area = wing_area
        self.thrust_max = thrust
        self.battery_capacity_wh = battery_capacity_wh
        self.path = path
        self.reset()

    def reset(self):
        self.pos = np.array([0.0, 0.0])
        self.v_body = np.array([0.0, 0.0])
        self.t = 0.0
        self.battery_wh = getattr(self, "battery_capacity_wh", 0.0)
        self.li_vec = np.array([0.0, 0.0])
        self.dr_vec = np.array([0.0, 0.0])
        self.th_vec = np.array([0.0, 0.0])
        self.f_vec = np.array([0.0, 0.0])

        self.climb_pid = PIDController()
        self.cruise_pid = PIDController()

        self.takeoff = False
        self.climbed = False
        self.cruised = False

        return self

    def update_values(self, rho):
        x, y = self.v_norm
        angle = np.arctan2(y, x)
        v_abs = np.linalg.norm(self.v_body)
        cl, cd, _, _ = get_cl_cd(v_abs, angle, self.path)
        self.rho = rho
        self.cl = cl
        self.cd = cd

    def weight_vec(self):
        return np.array([0.0, -self.mass * 9.81])

    @property
    def v_norm(self):
        v_abs = np.linalg.norm(self.v_body)
        if self.v_low:
            # low speed => v norm is [1,0]
            return np.array([1.0, 0])
        else:
            return self.v_body / v_abs

    @property
    def v_low(self):
        return np.linalg.norm(self.v_body) < self.v_thresh

    def lift_vec(self):
        v_abs = np.linalg.norm(self.v_body)
        if self.v_low:
            # low speed => up is [0,1]
            lift_dir = np.array([0.0, 1.0])
        else:
            vx, vy = self.v_norm
            # rotate 90
            lift_dir = np.array([-vy, vx])
        lift_abs = 0.5 * self.rho * v_abs**2 * self.wing_area * self.cl
        return lift_abs * lift_dir

    def drag_vec(self):
        v_abs = np.linalg.norm(self.v_body)
        if self.v_low:
            return np.zeros(2)
        drag_abs = 0.5 * self.rho * v_abs**2 * self.wing_area * self.cd
        return -drag_abs * self.v_norm

    def thrust_vec(self):
        # PID
        dt = 0.1
        if self.takeoff:
            angle = self.climb_pid.sensor(self.pos[0], self.pos[1])
            self.thrust = self.climb_pid.update(angle, dt)
        else:
            self.thrust = self.thrust_max
        return self.thrust * self.v_norm

    def power_required(self):
        if self.v_low:
            return 0.0
        drag_vec = self.drag_vec()
        power_shaft = abs(np.dot(drag_vec, self.v_body))
        return power_shaft

    def consume_energy(self, power_w, dt):
        self.wh_used = (power_w * dt) / 3600.0
        self.battery_wh = max(0.0, self.battery_wh - self.wh_used)
        return self.wh_used

    def compute_forces(self, rho):
        self.update_values(rho)
        self.li_vec = self.lift_vec()
        self.dr_vec = self.drag_vec()
        self.th_vec = self.thrust_vec()
        self.weight = self.weight_vec()
        self.f_vec = self.li_vec + self.dr_vec + self.th_vec + self.weight
        return self.f_vec

    def step(self, env, force_vec, dt):
        self.v_body += force_vec / self.mass * dt
        env.ground_constraint(self)

        self.pos += self.v_body * dt
        self.t += dt
        power = self.power_required()
        self.consume_energy(power, dt)
