import numpy as np
from message_types.msg_autopilot import MsgAutopilot
from tools.wrap import wrap


class PathFollower:
    def __init__(self):
        ##### TODO #####
        self.chi_inf = np.deg2rad(60)  # approach angle for large distance from straight-line path
        self.k_path = 0.03  # path gain for straight-line path following
        self.k_orbit = 1.0  # path gain for orbit following
        self.gravity = 9.81
        self.autopilot_commands = MsgAutopilot()  # message sent to autopilot

    def update(self, path, state):
        if path.type == 'line':
            self._follow_straight_line(path, state)
        elif path.type == 'orbit':
            self._follow_orbit(path, state)
        return self.autopilot_commands

    def _follow_straight_line(self, path, state):
        ##### TODO #####
        e = np.array([state.north - path.line_origin[0], 
                       state.east - path.line_origin[1], 
                       -state.altitude + path.line_origin[2]
                    ])
        s = (e - path.line_direction @ path.line_direction.T @ e)[:,0]
        rn = path.line_origin[0,0]
        re = path.line_origin[1,0]
        rd = path.line_origin[2,0]
        qn = path.line_direction[0,0]
        qe = path.line_direction[1,0]
        qd = path.line_direction[2,0]
        chi = state.chi

        #airspeed command
        self.autopilot_commands.airspeed_command = path.airspeed

        # course command
        chi_q = wrap(np.arctan2(qe,qn), chi)
        ep_y = -np.sin(chi_q) * (state.north - rn) + np.cos(chi_q) * (state.east - re)
        self.autopilot_commands.course_command = chi_q - self.chi_inf * 2 / np.pi * np.arctan(self.k_path * ep_y)

        # altitude command
        self.autopilot_commands.altitude_command = -rd - np.sqrt(s[0]**2 + s[1]**2) * (qd / np.sqrt(qn**2 + qe**2))

        # feedforward roll angle for straight line is zero
        self.autopilot_commands.phi_feedforward = 0

    def _follow_orbit(self, path, state):
        ##### TODO #####

        pn = state.north
        pe = state.east

        cn = path.orbit_center[0,0]
        ce = path.orbit_center[1,0]
        cd = path.orbit_center[2,0]

        d = np.sqrt((pn - cn)**2 + (pe - ce)**2)
        little_phi = wrap(np.arctan2(pe - ce, pn - cn), state.chi)
        lambda_ = 1 if path.orbit_direction == 'CW' else -1 

        # airspeed command
        self.autopilot_commands.airspeed_command = path.airspeed

        # course command
        self.autopilot_commands.course_command = little_phi + lambda_ * \
                                                    (np.pi / 2 + np.arctan(self.k_orbit * (d - path.orbit_radius) / path.orbit_radius))

        # altitude command
        self.autopilot_commands.altitude_command = -cd
        
        # roll feedforward command
        self.autopilot_commands.phi_feedforward = lambda_ * np.arctan(state.Va**2 / (self.gravity * path.orbit_radius))




