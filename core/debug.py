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
            frame_dict = drone.__dict__
        else:
            frame_dict = {}
            for key in self.args:
                if np.shape(key) == (2,):
                    x = drone.__dict__[key[0]]
                    y = drone.__dict__[key[1]]
                    value = np.array([x, y])
                elif np.shape(key) == ():
                    value = drone.__dict__[key]
                else:
                    raise ValueError(f'Key "{key}" incorrect shape')
                frame_dict[key] = value
        self.data += [frame_dict]

    def format(self):
        self.table = {}
        for key in self.data[0].keys:
            self.table[key] = np.array([d[key] for d in self.data])

    def show_data(self):
        #print(self.data)
        self.format()
        #print(self.table)
        if self.args != None:
            plt.figure()
            for key in self.args:
                if np.shape(key) == (2,) or np.shape(key) == ():
                    # print(self.table[key])
                    x, y = self.table[key][:, 0], self.table[key][:, 1]
                    # print(f"X : {x, np.shape(x)} \n Y: {y, np.shape(y)} \n")
                    label = f"test"
                else:
                    raise ValueError(f'Key "{key}" incorrect shape')
                plt.plot(x, y, label=label)
            plt.legend()
            plt.xlabel("x")
            plt.ylabel("y")
            plt.title("Debugger plots")
            plt.show()
