from drone import Drone
from environment import Env

import numpy as np
import time


class Sim:
    def __init__(self, drone: Drone, env: Env, dt=0.1):
        self.drone = drone
        self.env = env
        self.dt = dt

    def run(self, segment):
        self.drone.reset()
        segment.run(self)
        return None


class Takeoff:
    def __init__(self, runaway_length):
        self.runaway_length = runaway_length

    def run(self, sim: Sim):
        drone = sim.drone
        drone.takeoff = False

        while not drone.takeoff and drone.pos[0] < self.runaway_length:
            time.sleep(0.1)

            cl_req = drone.required_cl_for_level(sim.env.rho)
            force_vec = drone.compute_forces(sim.env.rho, cl_req)
            drone.take_physics_step(sim.env, force_vec, sim.dt, cl_req)

            if force_vec[1] >= 0:
                drone.takeoff = True
                break
        return self
