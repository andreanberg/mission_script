import numpy as np
import pandas as pd
import copy
from drone import Drone
import itertools
import matplotlib.pyplot as plt


class Debugger:
    def __init__(self, args=None):
        self.data = []
        self.args = args

    def get_data(self, drone: Drone):
        data = drone.__dict__
        if self.args == None:
            frame = data
        else:
            frame = {}
            for key in self.args:
                if np.shape(key) == (2,):
                    value = np.array([data[key[0]], data[key[1]]])
                elif np.shape(key) == ():
                    value = data[key]
                else:
                    raise ValueError(f'Key "{key}" incorrect shape')
                frame[key] = value
        self.data.append(copy.deepcopy(frame))

    def format(self):
        table = {}
        for key in self.data[0]:
            table[key] = np.array([d[key] for d in self.data])
        return table

    def mesh(self, figsize):  # TODO
        if self.args == None:
            raise ValueError("No arguments given")
        plots = len(self.args)
        ncols, nrows = 1, 1
        while ncols * nrows < plots:
            ncols += 1
            if ncols * nrows >= plots:
                break
            nrows += 1
        figsize = (figsize[0] * ncols, figsize[1] * nrows)
        fig, axs = plt.subplots(nrows, ncols, figsize=figsize, squeeze=False)
        return fig, axs

    def plot(self, table, axs, color):
        if self.args == None:
            raise ValueError("No arguments given")
        ran = range(max((list(np.shape(axs)) + [1])[0:2]))
        for (x_pos, y_pos), key in zip(itertools.product(ran, repeat=2), self.args):
            x_values = table[key][:, 0]
            y_values = table[key][:, 1]
            if np.shape(key) == ():
                label = f'Drone: "{key}"'
            elif np.shape(key) == (2,):
                label = f'Drone: "{key[0]}" vs "{key[1]}"'
            else:
                raise ValueError(f'Key "{key}" incorrect shape')
            axs[x_pos, y_pos].plot(x_values, y_values, label=label, color=color)
            axs[x_pos, y_pos].legend()

        plt.tight_layout()
        plt.show()
        plt.figure()

    def show_data(self, size, color="Black"):
        table = self.format()
        if self.args != None:
            _, axs = self.mesh(size)
            self.plot(table, axs, color)
