import matplotlib.pyplot as plt
import numpy as np
import itertools
import platform
import time
import copy
import os

from drone import Drone


class Point:
    def __init__(
        self,
        key,
        data: tuple | None = None,
        interpolants: int | None = None,
        normalized = False,
    ):
        self.key = key
        self.data = data
        self.interpolants = interpolants
        self.normalized = normalized


class Analyzer:
    def __init__(self, vis_args=None, print_args=None):
        self.data = []
        self.vis_args = vis_args
        self.print_args = print_args

    def get_data(self, drone: Drone):
        if self.vis_args == None:
            frame = data = drone.__dict__
        else:
            frame = {}
            data = drone.__dict__
            for key in self.vis_args:
                if np.shape(key) == (2,):
                    value = np.array([data[key[0]], data[key[1]]])
                elif np.shape(key) == ():
                    value = data[key]
                else:
                    raise ValueError(f'Key "{key}" incorrect shape')
                frame[key] = value
        self.data.append(copy.deepcopy(frame))

        if self.print_args != None:
            data = drone.__dict__
            for key in self.print_args:
                if np.shape(key) == ():
                    print(f"{key}:    \t {np.round(data[key], 3)}")
                else:
                    raise ValueError(f'Key "{key}" incorrect shape')
            time.sleep(0.01)
            if platform.system() == "Windows":
                os.system("cls")
            else:
                os.system("clear")

            print()

    def format(self):
        table = {}
        for key in self.data[0]:
            table[key] = np.array([d[key] for d in self.data])
        return table

    def mesh(self, figsize):
        if self.vis_args == None:
            raise ValueError("No arguments given")
        plots = len(self.vis_args)
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
        if self.vis_args == None:
            raise ValueError("No arguments given")
        ran = range(max((np.shape(axs))))
        for (row, col), key in zip(itertools.product(ran, repeat=2), self.vis_args):
            x_values = table[key][:, 0] 
            y_values = table[key][:, 1]
            if np.shape(key) == ():
                label = f'Drone: "{key}"'
            elif np.shape(key) == (2,):
                label = f'Drone: "{key[0]}" vs "{key[1]}"'
            else:
                raise ValueError(f'Key "{key}" incorrect shape')
            axs[row, col].plot(x_values, y_values, label=label, color=color)
            axs[row, col].legend()

        plt.tight_layout()
        plt.show()

    def show_data(self, size, color="Black"):
        table = self.format()
        if self.vis_args != None:
            _, axs = self.mesh(size)
            self.plot(table, axs, color)
