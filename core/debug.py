import numpy as np
import pandas as pd
from drone import Drone
import matplotlib.pyplot as plt


class Debugger:
    def __init__(self, args=None):
        self.data = []
        self.args = args

    def get_data(self, drone: Drone):
        self.drone = drone
        if self.args == None:
            timedict = drone.__dict__
        else:
            timedict = {}
            for key in self.args:
                if np.shape(key) == (2,):
                    di = drone.__dict__
                    timedict[key] = (di[key[0]], di[key[1]])
                elif np.shape(key) == ():
                    timedict[key] = drone.__dict__[key]
                else:
                    raise ValueError(f'Key "{key}" incorrect shape')
        self.data.append(timedict)

    def format(self):
        self.table = {}
        re = self.drone.__dict__ if self.args == None else self.args
        for key in re:
            self.table[key] = np.array([d[key] for d in self.data])

    def show_data(self):
        self.format()
        if self.args != None:
            plt.figure()
            for key in self.args:
                if np.shape(key) == (2,) or np.shape(key) == ():
                    x, y = self.table[key]
                    label = f"test"
                else:
                    raise ValueError(f'Key "{key}" incorrect shape')
                plt.plot(x, y, label=label)
            plt.legend()
            plt.xlabel("x")
            plt.ylabel("y")
            plt.title("Debugger plots")
            plt.show()
