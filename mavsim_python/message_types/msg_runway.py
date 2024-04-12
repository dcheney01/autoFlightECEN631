
import numpy as np

class MsgRunway:
    def __init__(self):
        self.start = np.array([[0.0, 0.0, 0.0]]).T
        self.length = 1000
        self.orientation = np.deg2rad(45)
        self.width = 100
        self.plot_updated = False

    def update(self, start, length, orientation):
        self.start = start
        self.length = length
        self.orientation = orientation
        self.plot_updated = False