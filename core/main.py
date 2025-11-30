# Necessary imports
import numpy as np
from drone import Drone
from environment import Env
from simulator import Sim, Takeoff
from debug import Debugger

def main():
    # Init necessary objects for simulation
    drone = Drone(
        mass=6.0,
        wing_area=0.9,
        thrust=50.0,
        path="prop/aero"
    )
    env = Env(air_density=1.225)
    sim = Sim(drone=drone, env=env, dt=0.01)

    # Take-off simulation with debugger
    args = [
        "pos",  # (2,) - plottable
        #("t", "battery_wh"),  # (2,) - plottable
        # ("pos", "v_body") # (2,2)
        # plottable with vector TODO perhaps every 100th
    ]
    debug = Debugger(args=args)
    sim.run(Takeoff(runway_length=20, debugger=debug))
    #debug.show_data()
    
if __name__ == '__main__':
    main()