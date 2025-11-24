import numpy as np
import time

from drone import Drone
from environment import Env


class Sim:
    def __init__(self, drone: Drone, env: Env, dt=0.1):
        self.drone = drone.reset()
        self.env = env
        self.dt = dt

    def run(self, segment):
        segment.run(self)
        return None

class Mission: 
    # TODO make a general mission that takeoff and climb inherit from
    def __init__(self):
        pass

class Takeoff:
    def __init__(self, runaway_length):
        self.runaway_length = runaway_length

    def run(self, sim: Sim):
        drone = sim.drone
        drone.takeoff = False

        while not drone.takeoff and drone.pos[0] < self.runaway_length:
            force_vec = drone.compute_forces(sim.env.rho)
            drone.step(sim.env, force_vec, sim.dt)

            if force_vec[1] >= 0:
                drone.takeoff = True
        return self


class Climb:
    def __init__(self, climb_angle, target_altitude):
        self.climb_angle = climb_angle
        self.target_altitude = target_altitude

    def run(self, sim: Sim):
        drone = sim.drone
        self.climbed = False

        while not drone.climbed:
            force_vec = drone.compute_forces(sim.env.rho)
            drone.step(sim.env, force_vec, sim.dt)
        
            if drone.pos[1] >= self.target_altitude:        
                self.climbed = True
        return self
