"""
mavsim_python: waypoint viewer (for chapter 11)
    - Beard & McLain, PUP, 2012
    - Update history:
        4/15/2019 - BGM
        7/13/2023 - RWB
        3/25/2024 - Carson Moon
"""
import numpy as np
import pyqtgraph.opengl as gl
from viewers.draw_mav import DrawMav
from viewers.draw_path import DrawPath
from viewers.draw_waypoints import DrawWaypoints
from viewers.draw_runway import DrawRunway
from time import time


class MAVAndWaypointViewer:
    def __init__(self, app, ts_refresh=1./30., runway=False):
        self.runway_flag = runway
        self.scale = 2000
        # initialize Qt gui application and window
        self.app = app  # initialize QT
        self.window = gl.GLViewWidget()  # initialize the view object
        self.window.setWindowTitle('World Viewer')
        grid = gl.GLGridItem() # make a grid to represent the ground
        grid.scale(self.scale/20, self.scale/20, self.scale/20) # set the size of the grid (distance between each line)
        self.window.addItem(grid) # add grid to viewer
        self.window.setCameraPosition(distance=self.scale, elevation=90, azimuth=-90)
        self.window.setBackgroundColor('k')  # set background color to black
        self.window.setGeometry(0, 0, 750, 750)  # args: upper_left_x, upper_right_y, width, height
        center = self.window.cameraPosition()
        center.setX(250)
        center.setY(250)
        center.setZ(0)
        self.window.setCameraPosition(pos=center, distance=self.scale, elevation=50, azimuth=-90)
        self.window.show()  # display configured window
        self.window.raise_()  # bring window to the front
        self.plot_initialized = False  # has the mav been plotted yet?
        self.mav_plot = []
        self.path_plot = []
        self.waypoint_plot = []
        self.ts_refresh = ts_refresh
        self.t = time()
        self.t_next = self.t

    def update(self, state, path, waypoints, runway):
        blue = np.array([[30, 144, 255, 255]])/255.
        red = np.array([[1., 0., 0., 1]])
        # initialize the drawing the first time update() is called
        if not self.plot_initialized:
            self.mav_plot = DrawMav(state, self.window)
            self.waypoint_plot = DrawWaypoints(waypoints, path.orbit_radius, blue, self.window)
            self.path_plot = DrawPath(path, red, self.window)
            self.plot_initialized = True

            if self.runway_flag:
                self.runway_plot = DrawRunway(self.window)
                self.runway_plot.update(runway)
                runway.plot_updated = True
        # else update drawing on all other calls to update()
        else:
            t = time()
            if t-self.t_next > 0.0:
                self.mav_plot.update(state)
                self.t = t
                self.t_next = t + self.ts_refresh
            if not waypoints.plot_updated:
                self.waypoint_plot.update(waypoints)
                waypoints.plot_updated = True
            if not path.plot_updated:  # only plot path when it changes
                self.path_plot.update(path, red)
                path.plot_updated = True
            if not runway.plot_updated:
                self.runway_plot.update(runway)
                runway.plot_updated = True
        # redraw
        self.app.processEvents()
