import numpy as np
import time
import os

from debug import Debugger
from scrape import get_cl_cd


class Drone:
    def __init__(
        self,
        mass=5.0,
        v_thresh=1e-6,
        wing_area=0.8,
        thrust=150.0,
        battery_capacity_wh=500.0,
        debug = False
    ):
        self.mass = mass
        self.v_thresh = v_thresh
        self.wing_area = wing_area
        self.thrust_max = thrust
        self.battery_capacity_wh = battery_capacity_wh
        self.debug = debug

        self.reset()

    def reset(self):
        self.pos = np.array([0.0, 0.0])
        self.v_body = np.array([0.0, 0.0])
        self.t = 0.0
        self.battery_wh = getattr(self, "battery_capacity_wh", 0.0)

        self.takeoff = False
        self.climbed = False
        self.cruised = False
        
        return self

    def update_values(self, rho):
        x, y = self.v_norm
        angle = np.arctan2(y, x)
        v_abs = np.linalg.norm(self.v_body)
        cl, cd, _, _ = get_cl_cd(v_abs, angle)
        self.rho = rho
        self.cl = cl
        self.cd = cd

    @property
    def weight(self):
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
        # from other (dummy) - simon brask må veta vilken thrust
        return self.thrust_max * self.v_norm

    def power_required(self):
        if self.v_low:
            return 0.0
        drag_vec = self.drag_vec()
        power_shaft = abs(np.dot(drag_vec, self.v_body))
        return power_shaft

    def consume_energy(self, power_w, dt):
        wh_used = (power_w * dt) / 3600.0
        self.battery_wh = max(0.0, self.battery_wh - wh_used)
        return wh_used

    def compute_forces(self, rho):
        self.update_values(rho)
        lift_vec = self.lift_vec()
        drag_vec = self.drag_vec()
        thrust_vec = self.thrust_vec()
        weight_vec = self.weight

        if self.debug:
            time.sleep(0.1)
            print("lift_vec\t", lift_vec)
            print("drag_vec\t", drag_vec)
            print("thrust_vec\t", thrust_vec)
            print("weight_vec\t", weight_vec)
            print("pos\t\t", self.pos)
            print()

        return lift_vec + drag_vec + thrust_vec + weight_vec

    def step(self, env, force_vec, dt):
        self.v_body += force_vec / self.mass * dt
        env.ground_constraint(self)
        self.pos += self.v_body * dt
        self.t += dt
        power = self.power_required()
        self.consume_energy(power, dt)
