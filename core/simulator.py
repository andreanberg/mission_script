import numpy as np
import time

from debug import Debugger
from drone import Drone
from environment import Env

# TODO, work through and understand the role of kwargs 
# and if that is worth your time

class Sim:
    def __init__(self, drone: Drone, env: Env, dt=0.1):
        self.drone = drone.reset()
        self.env = env
        self.dt = dt

    def run(self, mission):
        mission.run(self)


class Mission:
    """
    Base Mission class with optional context (kwargs).
    Subclasses override simulate_condition and success_check
    to define their behavior.
    """

    def __init__(self, **kwargs):
        # Mission context storage (all user-defined mission parameters)
        self.ctx = kwargs

    def on_start(self, sim):
        """Hook called before the mission loop begins."""
        pass

    def on_step(self, sim):
        """Hook called after each simulation step."""
        pass

    def on_end(self, sim):
        """Hook called after mission success."""
        pass

    # ---- REQUIRED: Subclasses override these ----
    def simulate_condition(self, sim):
        """
        Returns True if the simulation loop should continue.
        """
        raise NotImplementedError

    def success_check(self, sim):
        """
        Returns True when the mission has succeeded.
        """
        raise NotImplementedError

    # ---- Generic runner ----
    def run(self, sim: Sim):
        self.on_start(sim)

        while self.simulate_condition(sim):
            drone = sim.drone
            force_vec = drone.compute_forces(sim.env.rho)
            drone.step(sim.env, force_vec, sim.dt)
            self.on_step(sim)

            if self.success_check(sim):
                break

        self.on_end(sim)
        return self


class Takeoff(Mission):
    """
    Takeoff mission. Requires:
        runway_length
    """

    def __init__(self, runway_length, **kwargs):
        super().__init__(runway_length=runway_length, **kwargs)

    # Required overrides --------------------------------
    def simulate_condition(self, sim: Sim):
        drone = sim.drone
        return (not drone.takeoff) and (drone.pos[0] < self.ctx["runway_length"])

    def success_check(self, sim: Sim):
        drone = sim.drone
        if drone.compute_forces(sim.env.rho)[1] >= 0:  # lift >= weight
            drone.takeoff = True
            return True
        return False

    # Optional hooks ------------------------------------
    def on_start(self, sim: Sim):
        sim.drone.takeoff = False

    def on_end(self, sim: Sim):
        print("Takeoff successful.")


class Climb(Mission):
    """
    Climb mission. Requires:
        climb_angle =
        target_altitude
    """

    def __init__(self, climb_angle, target_altitude, **kwargs):
        super().__init__(
            climb_angle=climb_angle, target_altitude=target_altitude, **kwargs
        )
        self.climbed = False

    def simulate_condition(self, sim: Sim):
        return not self.climbed

    def success_check(self, sim: Sim):
        drone = sim.drone
        if drone.pos[1] >= self.ctx["target_altitude"]:
            self.climbed = True
            return True
        return False

    def on_end(self, sim):
        print("Climb complete.")
