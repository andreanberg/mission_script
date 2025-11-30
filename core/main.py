# Necessary imports
import numpy as np

from drone import Drone
from environment import Env
from simulator import Sim, Takeoff
from analyzer import Analyzer


def main():
    # Init necessary objects for simulation
    drone = Drone(mass=6.0, wing_area=0.9, thrust=50.0, path="prop/aero")
    env = Env(air_density=1.225)
    sim = Sim(drone=drone, env=env, dt=0.01)

    # Take-off simulation with debugger
    args = [("t", "battery_wh"), "pos"]
    an_to = Analyzer(args=args)
    sim.run(Takeoff(runway_length=20, analyzer=an_to))
    an_to.show_data(size=(5, 3), color="Black")


if __name__ == "__main__":
    main()
