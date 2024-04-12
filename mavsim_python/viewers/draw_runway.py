import numpy as np
import pyqtgraph.opengl as gl

from message_types.msg_runway import MsgRunway

class DrawRunway:
    def __init__(self, window: gl.GLViewWidget):
        self.window = window
        self.purple = np.array([1., 0., 1., 1.])
        self.mav_meshColors = np.empty((2, 3, 4), dtype=np.float32)
        self.mav_meshColors[0] = self.purple
        self.mav_meshColors[1] = self.purple
        
    def update(self, run_way: MsgRunway):
        self.start = run_way.start
        self.length = run_way.length
        self.orientation = run_way.orientation
        self.width = run_way.width


        # Points in runway
        dir_x = np.array([np.cos(self.orientation), np.sin(self.orientation)]).reshape((2,1))
        dir_y = np.array([-np.sin(self.orientation), np.cos(self.orientation)]).reshape((2,1))

        p1, p2, p3, p4 = np.zeros((3,1)), np.zeros((3,1)), np.zeros((3,1)), np.zeros((3,1))
        # create line in direction of orientation from self.start
        p1[:2] = np.flip(self.start[:2]) + (self.width/2 * dir_x)
        p2[:2] = np.flip(self.start[:2]) - (self.width/2 * dir_x)
        p3[:2] = p1[:2] + self.length * dir_y
        p4[:2] = p2[:2] + self.length * dir_y

        p1[2] = -20
        p2[2] = -20
        p3[2] = -20
        p4[2] = -20
        
        # draw rectangle
        mesh = np.array([[p1, p2, p4],
                         [p1, p3, p4]]).squeeze()
        
        self.runway_plot_object = gl.GLMeshItem(vertexes=mesh,  # defines the triangular mesh (Nx3x3)
                                      vertexColors=self.mav_meshColors,  # defines mesh colors (Nx1)
                                      drawEdges=True,  # draw edges between mesh elements
                                      smooth=False,  # speeds up rendering
                                      computeNormals=False)  # speeds up rendering
        
        self.window.addItem(self.runway_plot_object)  # add body to plot

        print("Drew Runway")