import numpy as np
import time

from analyzer import Analyzer
from drone import Drone
from environment import Env


class Sim:
    def __init__(self, drone: Drone, env: Env, dt=0.1):
        self.drone = drone.reset()
        self.env = env
        self.dt = dt

    def run(self, mission):
        mission.run(self)


class Mission:
    def __init__(self, analyzer: Analyzer | None = None):
        self.analyzer = analyzer

    def on_start(self, sim: Sim):
        pass

    def on_step(self, sim: Sim):
        pass

    def on_end(self, sim: Sim):
        pass

    def simulate_condition(self, sim: Sim):
        raise NotImplementedError

    def success_check(self, sim: Sim):
        raise NotImplementedError

    def log(self, drone: Drone):
        if self.analyzer != None:
            self.analyzer.get_data(drone)

    def run(self, sim: Sim):
        self.on_start(sim)
        drone = sim.drone
        self.log(drone)
        while self.simulate_condition(sim):
            force_vec = drone.compute_forces(sim.env.rho)
            drone.step(sim.env, force_vec, sim.dt)
            self.on_step(sim)
            self.log(drone)
            if self.success_check(sim):
                break
        self.on_end(sim)
        return self


class Takeoff(Mission):
    def __init__(self, runway_length, analyzer: Analyzer | None = None):
        super().__init__(analyzer=analyzer)
        self.runway_length = runway_length

    def simulate_condition(self, sim: Sim):
        drone = sim.drone
        return (not drone.takeoff) and (drone.pos[0] < self.runway_length)

    def success_check(self, sim: Sim):
        if sim.drone.compute_forces(sim.env.rho)[1] >= 0:
            sim.drone.takeoff = True
            return True
        return False

    def on_start(self, sim: Sim):
        sim.drone.takeoff = False
        
    def on_end(self, sim: Sim):
        sim.drone.takeoff = True


class Climb(Mission):
    def __init__(self, altitude_goal, analyzer: Analyzer | None = None):
        super().__init__(analyzer=analyzer)
        self.altitude_goal = altitude_goal

    def simulate_condition(self, sim: Sim):
        drone = sim.drone
        return (not drone.climbed) and (drone.pos[1] < self.altitude_goal)

    def success_check(self, sim: Sim):
        if sim.drone.pos[1] >= self.altitude_goal:
            sim.drone.climbed = True
            return True
        return False
    
    def on_step(self, sim: Sim):
        pass
        #print(sim.drone.compute_forces(sim.env.rho)[1])

    def on_start(self, sim: Sim):
        sim.drone.climbed = False
