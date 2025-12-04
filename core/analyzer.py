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
        normalized: bool = False,
        color: str | None = None,
        linewidth: float | None = None,
    ):
        """Represents a plotting item.

        - key: the primary variable name (string) or a tuple of two names (for x vs y pairs)
        - data: when present, names of additional fields to use for plotting (e.g. vector field names)
        - interpolants: for vector plotting, number of arrows to draw along the trajectory
        - normalized: whether to normalize vector lengths when plotting
        - color, linewidth: optional appearance controls for plotting
        """
        self.key = key
        self.data = data
        self.interpolants = interpolants
        self.normalized = normalized
        self.color = color
        self.linewidth = linewidth


class Analyzer:
    def __init__(self, vis_points=None, print_args=None):
        self.data = []
        self.vis_points = vis_points
        self.print_args = print_args

    def get_data(self, drone: Drone):
        # If vis_points is None we capture the full drone state; otherwise
        # require that every vis_arg is a Point and collect the referenced fields.
        if self.vis_points is None:
            frame = data = drone.__dict__
        else:
            # validate that only Points are present
            for v in self.vis_points:
                if not isinstance(v, Point):
                    raise ValueError(
                        "All entries in vis_points must be `Point` instances"
                    )

            frame = {}
            data = drone.__dict__
            for k in self.vis_points:
                key = k.key
                # primary key may be a 2-tuple (x,y) or a single field
                if np.shape(key) == (2,):
                    value = np.array([data[key[0]], data[key[1]]])
                elif np.shape(key) == ():
                    value = data[key]
                else:
                    raise ValueError(f'Key "{key}" incorrect shape')
                frame[key] = value
                # store any additional referenced data (e.g. vector fields)
                if k.data is not None:
                    for d in k.data:
                        frame[d] = data[d]

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
        if self.vis_points == None:
            raise ValueError("No arguments given")
        plots = len(self.vis_points)
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
        if self.vis_points == None:
            raise ValueError("No arguments given")
        ran = range(max((np.shape(axs))))
        # validate all are Points (defensive)
        for v in self.vis_points:
            if not isinstance(v, Point):
                raise ValueError(
                    "All entries in vis_points must be `Point` instances for plotting"
                )

        # iterate through subplots and vis_points in order
        for (row, col), vis in zip(itertools.product(ran, repeat=2), self.vis_points):
            ax = axs[row, col]
            key = vis.key
            # construct label
            if isinstance(key, tuple) and len(key) == 2:
                label = f'Drone: "{key[0]}" vs "{key[1]}"'
            else:
                label = f'Drone: "{key}"'

            # Always plot the key (trajectory) as a continuous line/curve
            arr = table[key]
            if hasattr(arr, "ndim") and arr.ndim == 2 and arr.shape[1] >= 2:
                x_values = arr[:, 0]
                y_values = arr[:, 1]
                ax.plot(x_values, y_values, label=label, color=(vis.color or color))
            else:
                # fall back to plotting scalar over index
                ax.plot(
                    np.arange(arr.shape[0]),
                    arr,
                    label=label,
                    color=(vis.color or color),
                )

            # If data is provided, plot vectors only at interpolant sample points
            if vis.data is not None:
                pos_arr = table[key]
                vec_name = vis.data[0]
                vec_arr = table[vec_name]
                T = pos_arr.shape[0]
                n_arrows = vis.interpolants or min(20, T)
                if n_arrows <= 0:
                    n_arrows = min(20, T)
                indices = np.linspace(0, T - 1, n_arrows, dtype=int)

                # project to 2D (x,y)
                xs = pos_arr[indices, 0]
                ys = pos_arr[indices, 1]
                us = vec_arr[indices, 0]
                vs = vec_arr[indices, 1]

                # normalize optionally
                if vis.normalized:
                    norms = np.linalg.norm(np.stack((us, vs), axis=1), axis=1)
                    norms[norms == 0] = 1.0
                    us = us / norms
                    vs = vs / norms
                    scale = 1.0
                else:
                    # choose a scale that keeps arrows visible; using data units
                    scale = 1.0

                ax.scatter(xs, ys, color=(vis.color or color), s=6)
                ax.quiver(
                    xs,
                    ys,
                    us,
                    vs,
                    angles="xy",
                    scale_units="xy",
                    scale=scale,
                    color=(vis.color or color),
                    width=0.004,
                )
                ax.set_aspect("equal", adjustable="datalim")

            ax.legend()

        plt.tight_layout()
        plt.show()

    def show_data(self, size, color="Black"):
        table = self.format()
        if self.vis_points != None:
            _, axs = self.mesh(size)
            self.plot(table, axs, color)
