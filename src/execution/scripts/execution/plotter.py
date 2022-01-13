import matplotlib
matplotlib.use('GTK3Agg')
from matplotlib import pyplot as plt
import numpy as np

class LivePlotter():

    def __init__(self, blit=True, title='Title', xlabel='x', ylabel='y', xlim = (-1, 1), ylim = (-1, 1)):
        self.line = None
        self.blit = blit
        self.xlim = xlim
        self.ylim = ylim
        self.title = title
        self.xlabel = xlabel
        self.ylabel = ylabel

    def set_x(self, x):
        self.x = x

    def set_y(self, y):
        self.y = y

    def append_x(self, x):
        self.x = np.append(self.x, x)

    def append_y(self, y):
        self.y = np.append(self.y, y)

    def _init_plot(self):
        plt.ion()
        self.fig, self.ax = plt.subplots()

        self.line = self.ax.plot([], [], animated=True)[0]
        self.reference = self.ax.plot([], [], animated=True)[0]

        self.line.set_label('Measurements')
        self.reference.set_label('Reference')
        self.ax.legend()

        self.ax.set_xlim(self.xlim[0], self.xlim[1])
        self.ax.set_ylim(self.ylim[0], self.ylim[1])
        self.ax.set_xlabel(self.xlabel)
        self.ax.set_ylabel(self.ylabel)
        self.ax.set_title(self.title)

        plt.show(block=False)
        plt.pause(0.1)

        if self.blit:
            self.background = self.fig.canvas.copy_from_bbox(self.fig.bbox)
            self.ax.draw_artist(self.line)
            self.ax.draw_artist(self.reference)
            self.fig.canvas.blit(self.fig.bbox)

    def plot(self, reference=None):
        if self.line is None:
            self._init_plot()

        self.line.set_data(self.x, self.y)

        if reference is not None:
            self.reference.set_data(self.x, reference)

        redraw = False
        if self.x[-1] >= self.xlim[1]:
            self.xlim = (self.xlim[0], self.xlim[1] * 2)
            self.ax.set_xlim(self.xlim[0], self.xlim[1])
            redraw = True

        if self.blit and not redraw:
            self.fig.canvas.restore_region(self.background)

            self.ax.draw_artist(self.line)
            self.ax.draw_artist(self.reference)
            self.fig.canvas.blit(self.fig.bbox)
            self.fig.canvas.flush_events()
        else:
            self.fig.canvas.draw()

    def close(self):
        plt.close(self.fig)

