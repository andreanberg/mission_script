import numpy as np
import pandas as pd
from drone import Drone


class Debugger:
    def __init__(self, show=False, args=None):
        self.data = []
        self.show = show
        self.args = args

    def get_data(self, drone: Drone):
        if self.args == None:
            timeframe = drone.__dict__
        else:
            timeframe = [drone.__dict__[key] for key in self.args]
        self.data.append(timeframe)

    def show_data(self):
        if self.show:
            print(self.data)


"""
if self.debug:
    time.sleep(0.01)
    print("lift_vec\t", lift_vec)
    print("drag_vec\t", drag_vec)
    print("thrust_vec\t", thrust_vec)
    print("weight_vec\t", weight_vec)
    print("pos\t\t", self.pos)
    print()
"""
