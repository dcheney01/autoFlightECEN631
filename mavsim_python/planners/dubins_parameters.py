# dubins_parameters
#   - Dubins parameters that define path between two configurations
#
# mavsim_matlab 
#     - Beard & McLain, PUP, 2012
#     - Update history:  
#         3/26/2019 - RWB
#         4/2/2020 - RWB
#         3/30/2022 - RWB
#         7/13/2023 - RWB
#         3/27/2024 - RWB
import numpy as np


class DubinsParameters:
    '''
    Class that contains parameters for a Dubin's car path

    Attributes
    ----------
        p_s : np.ndarray (3x1)
            inertial position of start position, in meters
        chi_s : float
            course of start position in radians, measured from North
        p_e : np.ndarray (3x1)
            inertial position of end position, in meters
        chi_e : float
            course of end position in radians, measured from North
        R : float
            radius of start and end circles, from north
        center_s : np.ndarray (3x1)
            inertial center of start circle
        dir_s : int 
            direction of start circle: +1 CW, -1 CCW
        center_e : np.ndarray (3x1)
            inertial center of end circle
        dir_e : int 
            direction of end circle: +1 CW, -1 CCW
        length : float
            length of straight line segment
        r1 : np.ndarray (3x1)
            position on half plane for transition from start circle to straight-line
        n1 : np.ndarray (3x1)
            unit vector defining half plane for transition from start circle to straight-line, and from straight line to end circle
        r2 : np.ndarray (3x1)
            position on half plane for transition from straight line to end circle
        r3 : np.ndarray (3x1)
            position on half plane for end of dubins path
        n3 : np.ndarray (3x1)
            unit vector defining half plane for end of dubins path

    Methods
    ----------
    update(ps, chis, pe, chie, R)
        : create new Dubins path from start to end poses, with specified radius
    compute_parameters()
        : construct four dubins paths and pick the shortest and define all associated parameters.
    compute_points()
        : find equally spaced points along dubins path - for plotting and collision checking
    '''

    def update(self, 
               ps: np.ndarray, # (3x1) 
               chis: float, 
               pe: np.ndarray, # (3x1)
               chie: float, 
               R: float):
         self.p_s = ps
         self.chi_s = chis
         self.p_e = pe
         self.chi_e = chie
         self.radius = R
         self.compute_parameters()

    def compute_parameters(self):
        ps = self.p_s
        pe = self.p_e
        chis = self.chi_s
        chie = self.chi_e
        R = self.radius
        ell = np.linalg.norm(ps[0:2] - pe[0:2])

        ##### TODO #####
        if ell < 2 * R:
            print('Error in Dubins Parameters: The distance between nodes must be larger than 2R.')
        else:
            # compute start and end circles
            crs = ps + R * rotz(np.pi / 2) @ np.array([[np.cos(chis)], [np.sin(chis)], [0]])
            cls = ps + R * rotz(-np.pi / 2) @ np.array([[np.cos(chis)], [np.sin(chis)], [0]])
            cre = pe + R * rotz(np.pi / 2) @ np.array([[np.cos(chie)], [np.sin(chie)], [0]])
            cle = pe + R * rotz(-np.pi / 2) @ np.array([[np.cos(chie)], [np.sin(chie)], [0]])

            # compute L1
            angle = np.arctan2(cre.item(1) - crs.item(1), cre.item(0) - crs.item(0))
            dist_crs = R * mod(2*np.pi + mod(angle - np.pi/2) - mod(chis - np.pi/2))
            dist_cre = R * mod(2*np.pi + mod(chie - np.pi/2) - mod(angle - np.pi/2))
            L1 = np.linalg.norm(crs - cre) + dist_crs + dist_cre

            # compute L2
            angle = np.arctan2(cle.item(1) - crs.item(1), cle.item(0) - crs.item(0))
            ell = np.linalg.norm(cle - crs)
            angle_2 = angle - np.pi/2 + np.arcsin(2*R/ell)
            dist_crs = R * mod(2*np.pi + mod(angle_2) - mod(chis - np.pi/2))
            dist_cle = R * mod(2*np.pi + mod(angle_2 + np.pi) - mod(chie + np.pi/2)) 
            L2 = np.sqrt(ell**2 - 4*R**2) + dist_crs + dist_cle

            # compute L3
            angle = np.arctan2(cre.item(1) - cls.item(1), cre.item(0) - cls.item(0))
            ell = np.linalg.norm(cre - cls)
            angle_2 = np.arccos(2*R/ell)
            dist_cls = R * mod(2*np.pi + mod(chis + np.pi/2) - mod(angle + angle_2))
            dist_cre = R * mod(2*np.pi + mod(chie - np.pi/2) - mod(angle + angle_2 - np.pi))
            L3 = np.sqrt(ell**2 - 4*R**2) + dist_cls + dist_cre

            # compute L4
            angle = np.arctan2(cle.item(1) - cls.item(1), cle.item(0) - cls.item(0))
            dist_cls = R * mod(2*np.pi + mod(chis + np.pi/2) - mod(angle + np.pi/2))
            dist_cle = R * mod(2*np.pi + mod(angle + np.pi/2) - mod(chie + np.pi/2))
            L4 = np.linalg.norm(cls - cle) + dist_cls + dist_cle

            # L is the minimum distance
            L = np.min([L1, L2, L3, L4])
            min_idx = np.argmin([L1, L2, L3, L4])

            if min_idx == 0:
                cs = crs
                lambda_s = 1
                ce = cre
                lambda_e = 1
                q1 = (ce - cs) / np.linalg.norm(ce - cs)
                z1 = cs + R * rotz(-np.pi/2) @ q1
                z2 = ce + R * rotz(-np.pi/2) @ q1
            elif min_idx == 1:
                cs = crs
                lambda_s = 1
                ce = cle
                lambda_e = -1
                angle = np.arctan2(ce.item(1) - cs.item(1), ce.item(0) - cs.item(0))
                ell = np.linalg.norm(ce - cs)
                angle_2 = angle - np.pi/2 + np.arcsin(2*R/ell)
                q1 = rotz(angle_2 + np.pi/2) @ np.array([[np.cos(chie)], [np.sin(chie)], [0]])
                z1 = cs + R * rotz(angle_2) @ np.array([[np.cos(chie)], [np.sin(chie)], [0]])
                z2 = ce + R * rotz(angle_2 + np.pi) @ np.array([[np.cos(chie)], [np.sin(chie)], [0]])
            elif min_idx == 2:
                cs = cls
                lambda_s = -1
                ce = cre
                lambda_e = 1
                angle = np.arctan2(ce.item(1) - cs.item(1), ce.item(0) - cs.item(0))
                ell = np.linalg.norm(ce - cs)
                angle_2 = np.arccos(2*R/ell)
                q1 = rotz(angle + angle_2 - np.pi/2) @ np.array([[np.cos(chie)], [np.sin(chie)], [0]])
                z1 = cs + R * rotz(angle + angle_2) @ np.array([[np.cos(chie)], [np.sin(chie)], [0]])
                z2 = ce + R * rotz(angle + angle_2 - np.pi) @ np.array([[np.cos(chie)], [np.sin(chie)], [0]])
            elif min_idx == 3:
                cs = cls
                lambda_s = -1
                ce = cle
                lambda_e = -1
                q1 = (ce - cs) / np.linalg.norm(ce - cs)
                z1 = cs + R * rotz(np.pi/2) @ q1
                z2 = ce + R * rotz(np.pi/2) @ q1
        
            self.length = L
            self.center_s = cs
            self.dir_s = lambda_s
            self.center_e = ce
            self.dir_e = lambda_e
            self.r1 = z1
            self.n1 = rotz(lambda_s * np.pi/2) @ q1
            self.r2 = z2
            self.r3 = pe
            self.n3 = rotz(chie) @ np.array([[np.cos(chie)], [np.sin(chie)], [0]])

            # self.length = 0
            # self.center_s = 0
            # self.dir_s = 0
            # self.center_e = 0
            # self.dir_e = 0
            # self.r1 = 0
            # self.n1 = 0
            # self.r2 = 0
            # self.r3 = 0
            # self.n3 = 0

    def compute_points(self):
        ##### TODO ##### - uncomment lines and remove last line
        Del = 0.1  # distance between point

        # points along start circle
        th1 = np.arctan2(self.p_s.item(1) - self.center_s.item(1),
                         self.p_s.item(0) - self.center_s.item(0))
        th1 = mod(th1)
        th2 = np.arctan2(self.r1.item(1) - self.center_s.item(1),
                         self.r1.item(0) - self.center_s.item(0))
        th2 = mod(th2)
        th = th1
        theta_list = [th]
        if self.dir_s > 0:
            if th1 >= th2:
                while th < th2 + 2*np.pi - Del:
                    th += Del
                    theta_list.append(th)
            else:
                while th < th2 - Del:
                    th += Del
                    theta_list.append(th)
        else:
            if th1 <= th2:
                while th > th2 - 2*np.pi + Del:
                    th -= Del
                    theta_list.append(th)
            else:
                while th > th2 + Del:
                    th -= Del
                    theta_list.append(th)

        points = np.array([[self.center_s.item(0) + self.radius * np.cos(theta_list[0]),
                            self.center_s.item(1) + self.radius * np.sin(theta_list[0]),
                            self.center_s.item(2)]])
        for angle in theta_list:
            new_point = np.array([[self.center_s.item(0) + self.radius * np.cos(angle),
                                   self.center_s.item(1) + self.radius * np.sin(angle),
                                   self.center_s.item(2)]])
            points = np.concatenate((points, new_point), axis=0)

        # points along straight line
        sig = 0
        while sig <= 1:
            new_point = np.array([[(1 - sig) * self.r1.item(0) + sig * self.r2.item(0),
                                   (1 - sig) * self.r1.item(1) + sig * self.r2.item(1),
                                   (1 - sig) * self.r1.item(2) + sig * self.r2.item(2)]])
            points = np.concatenate((points, new_point), axis=0)
            sig += Del

        # points along end circle
        th2 = np.arctan2(self.p_e.item(1) - self.center_e.item(1),
                         self.p_e.item(0) - self.center_e.item(0))
        th2 = mod(th2)
        th1 = np.arctan2(self.r2.item(1) - self.center_e.item(1),
                         self.r2.item(0) - self.center_e.item(0))
        th1 = mod(th1)
        th = th1
        theta_list = [th]
        if self.dir_e > 0:
            if th1 >= th2:
                while th < th2 + 2 * np.pi - Del:
                    th += Del
                    theta_list.append(th)
            else:
                while th < th2 - Del:
                    th += Del
                    theta_list.append(th)
        else:
            if th1 <= th2:
                while th > th2 - 2 * np.pi + Del:
                    th -= Del
                    theta_list.append(th)
            else:
                while th > th2 + Del:
                    th -= Del
                    theta_list.append(th)
        for angle in theta_list:
            new_point = np.array([[self.center_e.item(0) + self.radius * np.cos(angle),
                                   self.center_e.item(1) + self.radius * np.sin(angle),
                                   self.center_e.item(2)]])
            points = np.concatenate((points, new_point), axis=0)
        # points = np.zeros((5,3))
        return points


def rotz(theta: float):
    '''
    returns rotation matrix for right handed passive rotation about z-axis
    '''
    return np.array([[np.cos(theta), -np.sin(theta), 0],
                    [np.sin(theta), np.cos(theta), 0],
                    [0, 0, 1]])


def mod(x: float):
    '''
    wrap x to be between 0 and 2*pi
    '''
    while x < 0:
        x += 2*np.pi
    while x > 2*np.pi:
        x -= 2*np.pi
    return x


