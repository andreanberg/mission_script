# Necessary imports
import numpy as np

from drone import Drone
from environment import Env
from simulator import Sim, Takeoff, Climb
from analyzer import Analyzer, Point


def main():
    drone = Drone(
        mass=6.0,
        wing_area=0.9,
        thrust=50.0,
        path="prop/aero",
    )
    env = Env(air_density=1.225)
    sim = Sim(drone=drone, env=env, dt=0.01)

    vis_points = [
        Point(key=("t", "battery_wh")),
        Point(key="pos", data=("f_vec",), interpolants=10, normalized=False),
    ]
    
    print_args = [
        "li_vec",
        "dr_vec",
        "th_vec",
        "weight",
        "f_vec",
        "pos",
    ]


    an_to = Analyzer(vis_points=vis_points)
    sim.run(Takeoff(runway_length=20, analyzer=an_to))
    #an_to.show_data(size=(5, 3), color="Black")
    
    #an_cl = Analyzer(print_args=print_args)
    sim.run(Climb(altitude_goal=20, analyzer=an_to))
    an_to.show_data(size=(5, 3), color="Black")


if __name__ == "__main__":
    main()

# TODO get some good scraping from the motors and thrust,
# use that to get good takeoff metrics

# TODO vector plotting, get some settings and make the analyzer
# be able to plot vectors over time

# TODO measure time and iterations, give some sort of
# presentation of this in the show

# TODO other way around of collecting data, recording everything
# and scraping after, good for jupyter

# TODO have a setting that gives all the forces in real-time (also motivates vector plotting)


