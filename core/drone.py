import numpy as np
import time
import os

from prop.scrape import get_cl_cd

class Drone:
    def __init__(
        self,
        mass=5.0,
        wing_area=0.8,
        cd0=0.03,
        k_induced=0.04,
        thrust=150.0,
        battery_capacity_wh=500.0,
        v_thresh=1e-6,
    ):
        self.mass = mass
        self.wing_area = wing_area
        self.cd0 = cd0  # zero-lift drag coefficient
        self.k_induced = k_induced  # induced drag factor
        self.thrust_max = thrust
        self.battery_capacity_wh = battery_capacity_wh
        self.v_thresh = v_thresh

        self.reset()

    def reset(self):
        self.pos = np.array([0.0, 0.0])
        self.v_body = np.array([0.0, 0.0])
        self.t = 0.0
        self.battery_wh = getattr(self, "battery_capacity_wh", 0.0)

        self.takeoff = False
        self.climbed = False
        self.cruised = False

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

    def lift_vec(self, rho, cl):
        v_abs = np.linalg.norm(self.v_body)
        if self.v_low:
            # low speed => up is [0,1]
            lift_dir = np.array([0.0, 1.0])
        else:
            vx, vy = self.v_norm
            # rotate 90
            lift_dir = np.array([-vy, vx])
        lift_abs = 0.5 * rho * v_abs**2 * self.wing_area * cl
        return lift_abs * lift_dir

    def drag_vec(self, rho, cl):
        # from openvsp
        cd = self.cd0 + self.k_induced * cl**2
        v_abs = np.linalg.norm(self.v_body)
        if self.v_low:
            return np.zeros(2)
        drag_abs = 0.5 * rho * v_abs**2 * self.wing_area * cd
        return -drag_abs * self.v_norm

    def thrust_vec(self):
        # from other (dummy) - simon brask må veta vilken thrust
        return self.thrust_max * self.v_norm

    def required_cl_for_level(self):
        x,y = self.v_norm 
        angle = np.atan2(y,x)
        cl, cd, used_speed, used_file = get_cl_cd(self.v_body, angle)

        print("Using file:", used_file)
        print("File speed:", used_speed)
        print("CL =", cl)
        print("CD =", cd)
        
        return cl
    
    def power_required(self, rho, cl):
        if self.v_low:
            return 0.0
        drag_vec = self.drag_vec(rho, cl)
        power_shaft = abs(np.dot(drag_vec, self.v_body))
        return power_shaft

    def consume_energy(self, power_w, dt):
        wh_used = (power_w * dt) / 3600.0
        self.battery_wh = max(0.0, self.battery_wh - wh_used)
        return wh_used

    def compute_forces(self, rho, cl_req):
        lift_vec = self.lift_vec(rho, cl_req)
        drag_vec = self.drag_vec(rho, cl_req)
        thrust_vec = self.thrust_vec()
        weight_vec = self.weight

        debug = True
        if debug:
            print("lift_vec\t", lift_vec)
            print("drag_vec\t", drag_vec)
            print("thrust_vec\t", thrust_vec)
            print("weight_vec\t", weight_vec)
            print()

        return lift_vec + drag_vec + thrust_vec + weight_vec

    def take_physics_step(self, env, force_vec, dt, cl):
        a_acc = env.constrain_acc(self, force_vec / self.mass)
        self.v_body += a_acc * dt
        self.pos += self.v_body * dt
        env.apply_ground_constraint(self)
        self.t += dt
        power = self.power_required(env.rho, cl)
        self.consume_energy(power, dt)
