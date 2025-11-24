import numpy as np

from drone import Drone
from environment import Env
from simulator import Sim, Takeoff, Climb


if __name__ == "__main__":
    drone = Drone(mass=6.0, wing_area=0.9, thrust=50.0)
    env = Env(air_density=1.225)
    sim = Sim(drone, env, dt=0.01)

    to = sim.run(Takeoff(runaway_length=20))
    
    print(f"Takeoff finished: \n{drone.t:.1f}\n")
    print(f"pos \n{np.round(drone.pos,3)}\n")
    print(f"v \n{np.linalg.norm(drone.v_body):.2f} m/s\n")
    print(f"takeoff \n{drone.takeoff}\n")
    
    drone.debug = True
    cl = sim.run(Climb(climb_angle=15, target_altitude=10))
    
    print(f"Climb finished: \n{drone.t:.1f}\n")
    print(f"pos \n{np.round(drone.pos,3)}\n")
    print(f"v \n{np.linalg.norm(drone.v_body):.2f} m/s\n")
    print(f"climbed \n{drone.climbed}\n")
    
## TODO något script sådant att man kan se live med 
# en plot som dynamiskt växer medan scriptet körs
    