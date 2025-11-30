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

    def figsize(self, figsize):  # TODO
        if self.args == None:
            raise ValueError("No arguments given")
        plots = len(self.args)
        ncols, nrows = 1, 1
        while ncols * nrows < plots:
            ncols += 1
            if ncols * nrows >= plots:
                break
            nrows += 1

        figsize = tuple(np.array(figsize) * ncols)
        fig, axs = plt.subplots(nrows, ncols, figsize=figsize)
        return fig, axs

    def plot(self, table, fig, axs):
        if self.args == None:
            raise ValueError("No arguments given")
        ran = range(max(np.shape(axs)))
        print(len(np.shape(axs)))
        for pos, key in zip(itertools.product(ran, repeat=2), self.args):
            x, y = table[key][:, 0], table[key][:, 1]
            if np.shape(key) == ():
                label = f'Drone: "{key}"'
            elif np.shape(key) == (2,):
                label = f'Drone: "{key[0]}" vs "{key[1]}'
            else:
                raise ValueError(f'Key "{key}" incorrect shape')
            print(pos)
            
            
            axs[pos[0], pos[1]].plot(x, y, label=label)
            axs[pos[0], pos[1]].legend()

        plt.tight_layout()
        plt.show()
        plt.figure()

    def show_data(self, size):
        table = self.format()
        if self.args != None:
            fig, axs = self.figsize(size)
            self.plot(table, fig, axs)
