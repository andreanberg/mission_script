import numpy as np

from drone import Drone
from environment import Env
from simulator import Sim, Takeoff


if __name__ == "__main__":
    env = Env(air_density=1.225)
    drone = Drone(mass=6.0, wing_area=0.9, thrust=50.0)
    sim = Sim(drone, env, dt=0.01)

    to = sim.run(Takeoff(runaway_length=100))
    
    print(f"Takeoff finished: \n{drone.t:.1f}\n")
    print(f"pos \n{np.round(drone.pos,3)}\n")
    print(f"v \n{np.linalg.norm(drone.v_body):.2f} m/s\n")
    print(f"takeoff \n{drone.takeoff}\n")