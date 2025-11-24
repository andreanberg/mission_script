import numpy as np

from drone import Drone

# this one should only be a functionz not two functions



class Env:
    # environment with constant wind and air density.
    def __init__(
        self,
        air_density = 1.225,
    ):
        self.rho = air_density

    def apply_ground_constraint(self, drone : Drone, constrain_lateral=True):
        """Apply ground constraints to drone if it's on the ground"""
        if drone.pos[1] <= 0.0 and not drone.takeoff:
            # Constrain vertical motion
            drone.pos[1] = 0.0
            drone.v_body[1] = 0.0

            # Optionally constrain lateral motion for runway simulation
            if constrain_lateral:
                drone.pos[1] = 0.0
                drone.v_body[1] = 0.0

            return True  # drone is on ground
        return False  # drone is airborne

    def constrain_acc(self, drone, acc):
        """Modify acc if drone is constrained to ground"""
        if drone.pos[1] <= 0.0 and not drone.takeoff:
            acc[1] = 0.0  # No vertical acc on ground
        return acc
