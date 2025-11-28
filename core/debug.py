import numpy as np
import pandas as pd
from drone import Drone
import matplotlib.pyplot as plt


class Debugger:
    def __init__(self, args=None):
        self.data = []
        self.args = args
        self.flatten = np.array(args).flatten()

    def get_data(self, drone: Drone):
        self.drone = drone
        if self.args == None:
            timedict = drone.__dict__
        else:
            timedict = {}
            for key in self.flatten:
                timedict[key] = drone.__dict__[key]
        self.data.append(timedict)

    def format(self):
        self.table = {}
        re = self.drone.__dict__ if self.args == None else self.flatten
        for key in re:
            self.table[key] = np.array([d[key] for d in self.data])

    def show_data(self):
        self.format()
        plt.figure()
        if self.args == None:
            pass
        else:
            print(self.args)
            for x_name, y_name in self.args:
                plt.plot(
                    self.table[x_name],
                    self.table[y_name],
                    label=f"{y_name} vs {x_name}",
                )
            plt.legend()
            plt.xlabel("x")
            plt.ylabel("y")
            plt.title("Debugger plots")
            plt.show()
