# Necessary imports
import numpy as np
from drone import Drone
from environment import Env
from simulator import Sim, Takeoff
from debug import Debugger


def main():
    # Init necessary objects for simulation
    drone = Drone(mass=6.0, wing_area=0.9, thrust=50.0, path="prop/aero")
    env = Env(air_density=1.225)
    sim = Sim(drone=drone, env=env, dt=0.01)

    # Take-off simulation with debugger
    args = [
        ("t", "battery_wh"),
        "pos",
        #("t", "battery_wh"),
        #"pos",
    ]
    debug = Debugger(args=args)
    sim.run(Takeoff(runway_length=20, debugger=debug))
    debug.show_data(size=(5, 3))


if __name__ == "__main__":
    main()
