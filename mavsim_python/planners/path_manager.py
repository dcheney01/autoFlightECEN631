"""
mavsim_python: drawing tools
    - Beard & McLain, PUP, 2012
    - Update history:
        4/15/2019 - RWB
        3/30/2022 - RWB
        7/13/2023 - RWB
        3/25/2024 - RWB
"""

import numpy as np
from planners.dubins_parameters import DubinsParameters
from message_types.msg_state import MsgState
from message_types.msg_path import MsgPath
from message_types.msg_waypoints import MsgWaypoints


class PathManager:
    '''
        Path manager

        Attributes
        ----------
        path : MsgPath
            path message sent to path follower
        num_waypoints : int
            number of waypoints
        ptr_previous : int
            pointer to previous waypoint
            MAV is traveling from previous to current waypoint
        ptr_current : int
            pointer to current waypoint
        ptr_next : int
            pointer to next waypoint
        halfspace_n : np.nparray (3x1)
            the normal vector that defines the current halfspace plane
        halfspace_r : np.nparray (3x1)
            the inertial vector that defines a point on the current halfspace plane
        manager_state : int
            state of the manager state machine
        manager_requests_waypoints : bool
            a flag for handshaking with the path planner
            True when new waypoints are needed, i.e., at the end of waypoint list.
        dubins_path : DubinsParameters
            A class that defines a dubins path      

        Methods
        -------
        update(waypoints, radius, state)

        _initialize_pointers() :
            initialize the points to 0(previous), 1(current), 2(next)  
        _increment_pointers() :  
            add one to every pointer - currently does it modulo num_waypoints          
        _inHalfSpace(pos):
            checks to see if the position pos is in the halfspace define

        _line_manager(waypoints, state):
            Assumes straight-line paths.  Transition is from one line to the next
            _construct_line(waypoints): 
                used by line manager to construct the next line path

        _fillet_manager(waypoints, radius, state):
            Assumes straight-line waypoints.  Constructs a fillet turn between lines.
            _construct_fillet_line(waypoints, radius):
                used by _fillet_manager to construct the next line path
            _construct_fillet_circle(waypoints, radius):
                used by _fillet_manager to construct the fillet orbit
            
        _dubins_manager(waypoints, radius, state):
            Assumes dubins waypoints.  Constructs Dubin's path between waypoints
            _construct_dubins_circle_start(waypoints, dubins_path):
                used by _dubins_manager to construct the start orbit
            _construct_dubins_line(waypoints, dubins_path):
                used by _dubins_manager to construct the middle line
            _construct_dubins_circle_end(waypoints, dubins_path):
                used by _dubins_manager to construct the end orbit
    '''
    def __init__(self):
        self._path = MsgPath()
        self._num_waypoints = 0
        self._ptr_previous = 0
        self._ptr_current = 1
        self._ptr_next = 2
        self._halfspace_n = np.inf * np.ones((3,1))
        self._halfspace_r = np.inf * np.ones((3,1))
        self._manager_state = 1
        self.manager_requests_waypoints = True
        self.dubins_path = DubinsParameters()


    def update(self, 
               waypoints: MsgWaypoints, 
               radius: float, 
               state: MsgState) -> MsgPath:

        if self._ptr_current > waypoints.num_waypoints - 1:
            return None

        if waypoints.num_waypoints == 0:
            self.manager_requests_waypoints = True
        if self.manager_requests_waypoints is True \
                and waypoints.flag_waypoints_changed is True:
            self.manager_requests_waypoints = False
        if waypoints.type == 'straight_line' or waypoints.type == 'landing':
            flag = self._line_manager(waypoints, state)
            if flag == False:
                return None
        elif waypoints.type == 'fillet':
            flag = self._fillet_manager(waypoints, radius, state)
            if flag == False:
                return None
        elif waypoints.type == 'dubins':
            self._dubins_manager(waypoints, radius, state)
        else:
            print('Error in Path Manager: Undefined waypoint type.')
        
        return self._path

    def _line_manager(self,  
                      waypoints: MsgWaypoints, 
                      state: MsgState):
        mav_pos = np.array([[state.north, state.east, -state.altitude]]).T
        # if the waypoints have changed, update the waypoint pointer
        
        ##### TODO ######
        # Use functions - 
                #@@ self._initialize_pointers(), 
                #@@ self._construct_line()
                #@@ self._inHalfSpace(mav_pos),
                #@@  self._increment_pointers(), 
                #@@ self._construct_line()

        # Use variables - self._ptr_current, self.manager_requests_waypoints,
        # waypoints.__, radius

        if self.manager_requests_waypoints or waypoints.flag_waypoints_changed:
            self.manager_requests_waypoints = False
            waypoints.flag_waypoints_changed = False
            self._num_waypoints = waypoints.num_waypoints
            self._initialize_pointers()

        if self._inHalfSpace(mav_pos):
            self._increment_pointers()
            # waypoints.flag_waypoints_changed = True
            # waypoints.plot_updated = False
            self._path.plot_updated = False
        
        if self._ptr_current > waypoints.num_waypoints - 1:
            return False
        self._construct_line(waypoints)
        return True

    def _fillet_manager(self,  
                        waypoints: MsgWaypoints, 
                        radius: float, 
                        state: MsgState):
        mav_pos = np.array([[state.north, state.east, -state.altitude]]).T
        # if the waypoints have changed, update the waypoint pointer

        ##### TODO ######
        # Use functions - 
        # self._initialize_pointers(), 
        # self._construct_fillet_line(),
        # self._inHalfSpace(), 
        # self._construct_fillet_circle(), 
        # self._increment_pointers()

        # Use variables self._num_waypoints, self._manager_state, self._ptr_current
        # self.manager_requests_waypoints, waypoints.__, radius

        if self.manager_requests_waypoints or waypoints.flag_waypoints_changed:
            self._num_waypoints = waypoints.num_waypoints
            self._initialize_pointers()
            self._manager_state = 1
            self.manager_requests_waypoints = False
            waypoints.flag_waypoints_changed = False

        if self._manager_state == 1:
            self._construct_fillet_line(waypoints, radius)

            if self._inHalfSpace(mav_pos):
                self._manager_state = 2
                self._path.plot_updated = False    

        if self._manager_state == 2:
            flag = self._construct_fillet_circle(waypoints, radius)
            if flag == False:
                return False
            if self._inHalfSpace(mav_pos):
                self._increment_pointers()
                self._manager_state = 1
                self._path.plot_updated = False    

    def _dubins_manager(self,  
                        waypoints: MsgWaypoints, 
                        radius: float, 
                        state: MsgState):
        mav_pos = np.array([[state.north, state.east, -state.altitude]]).T
        # if the waypoints have changed, update the waypoint pointer

        ##### TODO #####
        # Use functions - self._initialize_pointers(), self._dubins_path.update(),
        # self._construct_dubins_circle_start(), self._construct_dubins_line(),
        # self._inHalfSpace(), self._construct_dubins_circle_end(), self._increment_pointers(),

        # Use variables - self._num_waypoints, self._dubins_path, self._ptr_current,
        # self._ptr_previous, self._manager_state, self.manager_requests_waypoints,
        # waypoints.__, radius

        if self.manager_requests_waypoints or waypoints.flag_waypoints_changed:
            self._num_waypoints = waypoints.num_waypoints
            self._initialize_pointers()
            self._manager_state = 1
            self.manager_requests_waypoints = False
            waypoints.flag_waypoints_changed = False
            self._construct_dubins_circle_start(waypoints, radius)
        
        if self._manager_state == 1: # Starting start circle
            if self._inHalfSpace(mav_pos):
                self._halfspace_n = self.dubins_path.q1
                self._halfspace_r = self.dubins_path.z1

                self._manager_state = 2
                self._path.plot_updated = False    

        if self._manager_state == 2: # Finishing Start Circle
            if self._inHalfSpace(mav_pos):
                self._path.line_origin = self.dubins_path.z1
                self._path.line_direction = self.dubins_path.q1
                self._path.airspeed = waypoints.airspeed[self._ptr_current]
                self._path.type = 'line'     

                self._halfspace_n = self.dubins_path.q1
                self._halfspace_r = self.dubins_path.z2           
                self._manager_state = 3
                self._path.plot_updated = False    

        if self._manager_state == 3:
            if self._inHalfSpace(mav_pos):
                self._path.orbit_center = self.dubins_path.center_e
                self._path.orbit_radius = self.dubins_path.radius
                self._path.airspeed = waypoints.airspeed[self._ptr_current]
                self._path.orbit_direction = 'CW' if self.dubins_path.dir_e == 1 else 'CCW'
                self._path.type = "orbit"        

                self._halfspace_n = -self.dubins_path.q3
                self._halfspace_r = self.dubins_path.z3     
                self._manager_state = 4
                self._path.plot_updated = False    

        if self._manager_state == 4:
            if self._inHalfSpace(mav_pos):
                self._halfspace_n = self.dubins_path.q3
                self._halfspace_r = self.dubins_path.z3
                self._manager_state = 5
                self._path.plot_updated = False    

        if self._manager_state == 5:
            if self._inHalfSpace(mav_pos):
                self._increment_pointers()
                self._construct_dubins_circle_start(waypoints, radius)
                self._manager_state = 1
                self._path.plot_updated = False    


    def _initialize_pointers(self):
        if self._num_waypoints >= 3:
            #### TODO #####
            self._ptr_previous = 0
            self._ptr_current = 1
            self._ptr_next = 2 if self._num_waypoints > 2 else 9999
        else:
            print('Error Path Manager: need at least three waypoints')

    def _increment_pointers(self):
        ##### TODO #####
        self._ptr_previous += 1
        self._ptr_current += 1
        self._ptr_next += 1

        if self._ptr_next > self._num_waypoints - 1:
            self._ptr_next = 9999

    def _construct_line(self, 
                        waypoints: MsgWaypoints):
        previous = waypoints.ned[:, self._ptr_previous].reshape((3,1))
        ##### TODO #####

        current = waypoints.ned[:, self._ptr_current].reshape((3,1))
        q_prev = (current - previous) / np.linalg.norm(current - previous)
        
        # Update path variables
        self._path.line_origin = previous
        self._path.line_direction = q_prev
        self._path.airspeed = waypoints.airspeed[self._ptr_current]
        self._path.type = 'line'

        # update halfspace variables
        if self._ptr_next == 9999:
            next = current.reshape((3,1)) + 100*self._path.line_direction
        else:
            if self._ptr_next > self._num_waypoints - 1:
                next = current
            else:
                next = waypoints.ned[:, self._ptr_next].reshape((3,1))
        q_current = (next - current) / np.linalg.norm(next - current)
        self._halfspace_r = current
        self._halfspace_n = (q_prev + q_current) / (2 * np.linalg.norm((q_prev + q_current) / 2))

    def _construct_fillet_line(self, 
                               waypoints: MsgWaypoints, 
                               radius: float):
        previous = waypoints.ned[:, self._ptr_previous].reshape((3,1))
        ##### TODO #####
        current = waypoints.ned[:, self._ptr_current].reshape((3,1))

        q_prev = (current - previous) / np.linalg.norm(current - previous)
        
        # Update path variables
        self._path.line_origin = previous
        self._path.line_direction = q_prev
        self._path.airspeed = waypoints.airspeed[self._ptr_current]
        self._path.type = 'line'

        if self._ptr_next > waypoints.num_waypoints - 1:
            # next = current.reshape((3,1))
            return
        else:
            next = waypoints.ned[:, self._ptr_next].reshape((3,1))

        q_current = (next - current) / np.linalg.norm(next - current)
        
        angle_between_lines = np.arccos(-q_prev.T @ q_current)
        z = current - (radius / np.tan(angle_between_lines/2)) * q_prev

        self._halfspace_r = z
        self._halfspace_n = q_prev

    def _construct_fillet_circle(self, 
                                 waypoints: MsgWaypoints, 
                                 radius: float):
        previous = waypoints.ned[:, self._ptr_previous].reshape((3,1))
        ##### TODO #####
        current = waypoints.ned[:, self._ptr_current].reshape((3,1))
        if self._ptr_next > waypoints.num_waypoints - 1:
            return False
        next = waypoints.ned[:, self._ptr_next].reshape((3,1))

        q_prev = (current - previous) / np.linalg.norm(current - previous)
        q_current = (next - current) / np.linalg.norm(next - current)

        if (q_prev == q_current).all():
            # angle_between_lines = np.pi
            # q_prev_term = 1
            return False
        else:
            q_prev_term = (q_prev - q_current) / np.linalg.norm(q_prev - q_current)
            angle_between_lines = np.arccos(-q_prev.T @ q_current)
        c = current - (radius / np.sin(angle_between_lines/2)) * q_prev_term
        
        _lambda = np.sign(q_prev[0]*q_current[1] - q_prev[1]*q_current[0])
        z = current + (radius / np.tan(angle_between_lines/2)) * q_current

        # update halfspace variables
        self._halfspace_n = z
        self._halfspace_r = q_current
        
        # Update path variables
        self._path.orbit_center = c
        self._path.orbit_radius = radius
        self._path.airspeed = waypoints.airspeed[self._ptr_current]
        self._path.orbit_direction = 'CW' if _lambda == 1 else 'CCW'
        self._path.type = 'orbit'
        return True

    def _construct_dubins_circle_start(self, waypoints, radius):
        self.dubins_path.update(waypoints.ned[:, self._ptr_previous], 
                        waypoints.course[self._ptr_previous],
                        waypoints.ned[:, self._ptr_current],
                        waypoints.course[self._ptr_current],
                        radius)
        
        self._halfspace_n = -self.dubins_path.q1
        self._halfspace_r = self.dubins_path.z1
        
        # Starting circle path
        self._path.orbit_center =  self.dubins_path.center_s
        self._path.orbit_radius = self.dubins_path.radius
        self._path.airspeed = waypoints.airspeed[self._ptr_current]
        self._path.orbit_direction = 'CW' if self.dubins_path.dir_s == 1 else 'CCW'
        self._path.type = "orbit"

    def _inHalfSpace(self, 
                     pos: np.ndarray)->bool:
        '''Is pos in the half space defined by r and n?'''
        if (pos-self._halfspace_r).T @ self._halfspace_n >= 0.0:
            return True
        else:
            return False

