"""
mavDynamics 
    - this file implements the dynamic equations of motion for MAV
    - use unit quaternion for the attitude state
    
mavsim_python
    - Beard & McLain, PUP, 2012
    - Update history:  
        2/24/2020 - RWB
"""
import numpy as np
from models.mav_dynamics import MavDynamics as MavDynamicsForces
# load message types
from message_types.msg_state import MsgState
from message_types.msg_delta import MsgDelta
import parameters.aerosonde_parameters as MAV
from tools.rotations import quaternion_to_rotation, quaternion_to_euler


class MavDynamics(MavDynamicsForces):
    def __init__(self, Ts):
        super().__init__(Ts)
        # store wind data for fast recall since it is used at various points in simulation
        self._wind = np.array([[0.], [0.], [0.]])  # wind in NED frame in meters/sec
        # store forces to avoid recalculation in the sensors function
        self._forces = np.array([[0.], [0.], [0.]])
        self._Va = MAV.u0
        self._alpha = 0
        self._beta = 0
        # update velocity data and forces and moments
        self._update_velocity_data()
        self._forces_moments(delta=MsgDelta())
        # update the message class for the true state
        self._update_true_state()


    ###################################
    # public functions
    def update(self, delta, wind):
        '''
            Integrate the differential equations defining dynamics, update sensors
            delta = (delta_a, delta_e, delta_r, delta_t) are the control inputs
            wind is the wind vector in inertial coordinates
            Ts is the time step between function calls.
        '''
        # get forces and moments acting on rigid bod
        forces_moments = self._forces_moments(delta)
        super()._rk4_step(forces_moments)
        # update the airspeed, angle of attack, and side slip angles using new state
        self._update_velocity_data(wind)
        # update the message class for the true state
        self._update_true_state()

    ###################################
    # private functions
    def _update_velocity_data(self, wind=np.zeros((6,1))):
        steady_state = wind[0:3]
        gust = wind[3:6]

        ##### TODO #####
        # convert wind vector from world to body frame (self._wind = ?)
        R = quaternion_to_rotation(self._state[6:10])
        self._wind = R.T @ steady_state + gust
        
        # velocity vector relative to the airmass ([ur , vr, wr]= ?)
        vel_wrt_airmass = self._state[3:6] - self._wind

        # compute airspeed (self._Va = ?)
        self._Va = np.linalg.norm(vel_wrt_airmass)

        # compute angle of attack (self._alpha = ?)
        if vel_wrt_airmass[0] == 0:
            self._alpha = 0
        else:
            self._alpha = np.arctan(vel_wrt_airmass[2] / vel_wrt_airmass[0])
        
        # compute sideslip angle (self._beta = ?)
        if self._Va == 0:
            self._beta = 0
        else:
            self._beta = np.arcsin(vel_wrt_airmass[1] / self._Va)

    def _forces_moments(self, delta):
        """
        return the forces on the UAV based on the state, wind, and control surfaces
        :param delta: np.matrix(delta_a, delta_e, delta_r, delta_t)
        :return: Forces and Moments on the UAV np.matrix(Fx, Fy, Fz, Ml, Mn, Mm)
        """
        ##### TODO ######
        # extract states (phi, theta, psi, p, q, r)
        e0 = self._state[6]
        ex = self._state[7]
        ey = self._state[8]
        ez = self._state[9]
        p, q, r = self._state[10:13]
        rho_Va2_S_term = 0.5*MAV.rho*(self._Va**2)*MAV.S_wing

        if self._Va == 0.0:
            c_term = 0.0
            b_term = 0.0
        else:
            c_term = MAV.c / (2 * self._Va)
            b_term = MAV.b / (2 * self._Va)

        # compute gravitational forces ([fg_x, fg_y, fg_z])
        fg = MAV.mass * MAV.gravity * np.array([2*(ex*ez - ey*e0),
                                                2*(ey*ez + ex*e0),
                                                (ez**2 + e0**2 - ex**2 - ey**2)])

        # compute Lift and Drag coefficients (CL, CD)
        C_L = MAV.C_L_0 + MAV.C_L_alpha * self._alpha + MAV.C_L_q * c_term * q + MAV.C_L_delta_e * delta.elevator
        C_D_alpha = MAV.C_D_p + (MAV.C_L_0 + MAV.C_L_alpha * self._alpha) ** 2 / (np.pi * MAV.e * MAV.AR)
        C_D = C_D_alpha + MAV.C_D_q * c_term * q + MAV.C_D_delta_e * delta.elevator
        
        C_m0 = MAV.C_m_0 + MAV.C_m_alpha * self._alpha + MAV.C_m_q * c_term * q + MAV.C_m_delta_e * delta.elevator
        C_Y = MAV.C_Y_0 + MAV.C_Y_beta * self._beta + MAV.C_Y_p * b_term * p + MAV.C_Y_r * b_term * r + MAV.C_Y_delta_a * delta.aileron + MAV.C_Y_delta_r * delta.rudder
        C_l0 = MAV.C_ell_0 + MAV.C_ell_beta * self._beta + MAV.C_ell_p * b_term * p + MAV.C_ell_r * b_term * r + MAV.C_ell_delta_a * delta.aileron + MAV.C_ell_delta_r * delta.rudder
        C_n0 = MAV.C_n_0 + MAV.C_n_beta * self._beta + MAV.C_n_p * b_term * p + MAV.C_n_r * b_term * r + MAV.C_n_delta_a * delta.aileron + MAV.C_n_delta_r * delta.rudder

        # # compute Lift and Drag Forces (F_lift, F_drag)
        F_lift = rho_Va2_S_term * C_L
        F_drag = rho_Va2_S_term * C_D

        # compute forces in body frame (fx, fy, fz)
        fx = -np.cos(self._alpha) * F_drag + np.sin(self._alpha) * F_lift
        fy = rho_Va2_S_term * C_Y
        fz = -np.sin(self._alpha) * F_drag - np.cos(self._alpha) * F_lift

        # compute torques in body frame (Mx, My, Mz)
        l = rho_Va2_S_term * MAV.b * C_l0
        m = rho_Va2_S_term * MAV.c * C_m0
        n = rho_Va2_S_term * MAV.b * C_n0

        # propeller thrust and torque
        thrust_prop, torque_prop = self._motor_thrust_torque(self._Va, delta.throttle)
        
        fx_final = fx + thrust_prop + fg[0]
        fy_final = fy + fg[1]
        fz_final = fz + fg[2]
        l_final = l - torque_prop
        m_final = m
        n_final = n
        forces_moments = np.array([[fx_final, fy_final, fz_final, l_final, m_final, n_final]]).T
        return forces_moments

    def _motor_thrust_torque(self, Va, delta_t):
        # compute thrust and torque due to propeller
        ##### TODO #####
        # map delta_t throttle command(0 to 1) into motor input voltage
        v_in = MAV.V_max * delta_t

        # Angular speed of propeller (omega_p = ?)
        a = (MAV.C_Q0 * MAV.rho * (MAV.D_prop**5)) / ((2. * np.pi)**2)
        b = (MAV.C_Q1 * MAV.rho * (MAV.D_prop**4)) / ((2. * np.pi)) * Va + MAV.KQ*MAV.KV / MAV.R_motor
        c = (MAV.C_Q2 * MAV.rho * (MAV.D_prop**3) * (Va**2)) - (MAV.KQ / MAV.R_motor) * v_in + MAV.KQ * MAV.i0

        omega_p = (-b + np.sqrt(b**2 - 4. * a * c)) / (2. * a)
        
        J_op = 2. * np.pi * Va / (omega_p * MAV.D_prop)
        C_T = MAV.C_T2 * J_op**2 + MAV.C_T1 * J_op + MAV.C_T0
        C_Q = MAV.C_Q2 * J_op**2 + MAV.C_Q1 * J_op + MAV.C_Q0
        n = omega_p / (2 * np.pi)

        # thrust and torque due to propeller
        thrust_prop = MAV.rho * (n**2) * (MAV.D_prop**4) * C_T
        torque_prop = MAV.rho * (n**2) * (MAV.D_prop**5) * C_Q

        return thrust_prop, torque_prop
    
    def _update_true_state(self):
        # rewrite this function because we now have more information
        phi, theta, psi = quaternion_to_euler(self._state[6:10])
        pdot = quaternion_to_rotation(self._state[6:10]) @ self._state[3:6]
        self.true_state.north = self._state.item(0)
        self.true_state.east = self._state.item(1)
        self.true_state.altitude = -self._state.item(2)
        self.true_state.Va = self._Va
        self.true_state.alpha = self._alpha
        self.true_state.beta = self._beta
        self.true_state.phi = phi
        self.true_state.theta = theta
        self.true_state.psi = psi
        self.true_state.Vg = np.linalg.norm(pdot)
        self.true_state.gamma = np.arcsin(pdot.item(2) / self.true_state.Vg)
        self.true_state.chi = np.arctan2(pdot.item(1), pdot.item(0))
        self.true_state.p = self._state.item(10)
        self.true_state.q = self._state.item(11)
        self.true_state.r = self._state.item(12)
        self.true_state.wn = self._wind.item(0)
        self.true_state.we = self._wind.item(1)
        self.true_state.bx = 0
        self.true_state.by = 0
        self.true_state.bz = 0
        self.true_state.camera_az = 0
        self.true_state.camera_el = 0
