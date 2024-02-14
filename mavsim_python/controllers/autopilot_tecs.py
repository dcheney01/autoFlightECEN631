"""
autopilot block for mavsim_python - Total Energy Control System
    - Beard & McLain, PUP, 2012
    - Last Update:
        2/14/2020 - RWB
"""
import numpy as np
import parameters.control_parameters as AP
import parameters.aerosonde_parameters as MAV
from tools.transfer_function import TransferFunction
from tools.wrap import wrap
from controllers.pi_control import PIControl
from controllers.pd_control_with_rate import PDControlWithRate
from message_types.msg_state import MsgState
from message_types.msg_delta import MsgDelta


class Autopilot:
    def __init__(self, ts_control):
        # instantiate lateral controllers
        self.roll_from_aileron = PDControlWithRate(
                        kp=AP.roll_kp,
                        kd=AP.roll_kd,
                        limit=np.radians(45))
        self.course_from_roll = PIControl(
                        kp=AP.course_kp,
                        ki=AP.course_ki,
                        Ts=ts_control,
                        limit=np.radians(30))
        self.yaw_damper = TransferFunction(
                        num=np.array([[AP.yaw_damper_kr, 0]]),
                        den=np.array([[1, AP.yaw_damper_p_wo]]),
                        Ts=ts_control)

        # instantiate TECS controllers
        self.pitch_from_elevator = PDControlWithRate(
                        kp=AP.pitch_kp,
                        kd=AP.pitch_kd,
                        limit=np.radians(45))
        
        # Weight between potential and kinetic energies
        self.k = 1
        # throttle gains (unitless)
        self.kp_t = 0
        self.ki_t = 0
        # pitch gains
        self.kp_theta = 0
        self.ki_theta = 0
        # saturated altitude error
        self.h_error_max = 0  # meters
        self.E_integrator = 0
        self.L_integrator = 0
        self.E_error_d1 = 0
        self.L_error_d1 = 0
        self.delta_t_d1 = 0
        self.theta_c_d1 = 0
        self.theta_c_max = 0
        self.Ts = ts_control
        self.commanded_state = MsgState()

    def update(self, cmd, state):
	
	###### TODO ######
        Va_ref = cmd.airspeed_command
        h_ref = cmd.altitude_command
        gamma_ref = cmd.course_command 

        Va = state.Va
        h = state.altitude

        E_T = 0.5 * MAV.rho * Va_ref**2 + MAV.mass * MAV.gravity * h_ref
        self.E_integrator += (E_T - self.E_error_d1) * self.Ts / 2
        # Edot_T = (E_T - self.E_error_d1) / self.Ts

        E_error_k = 0.5 * MAV.mass * (Va_ref**2 - Va**2)
        E_error_p = MAV.mass * MAV.gravity * (h_ref - h)
        E_error_T = E_error_p + E_error_k
        E_error_D = E_error_p + E_error_k
        D = MAV.mass * MAV.gravity + E_error_D

        # lateral autopilot
        Tc = self.kp_t * E_T + self.ki_t * self.E_integrator
        # Tc = D + self.kp_t * (Edot_T / (MAV.mass * MAV.gravity * Va)) + self.ki_t *(E_error_T / (MAV.mass * MAV.gravity * Va)) 
        self.commanded_state.theta = 0 #self.kp_theta / (Va * MAV.gravity) * ((2 - k) * Edot_P - k * Edot_k) + self.ki_theta / (Va * MAV.gravity) * E_error_D


        # longitudinal TECS autopilot

        self.E_error_d1 = E_error_T

        # construct output and commanded states
        delta = MsgDelta(elevator=0,
                         aileron=0,
                         rudder=0,
                         throttle=Tc)
        self.commanded_state.altitude = h_ref                
        self.commanded_state.phi = 0
        self.commanded_state.chi = 0

        return delta, self.commanded_state

    def saturate(self, input, low_limit, up_limit):
        if input <= low_limit:
            output = low_limit
        elif input >= up_limit:
            output = up_limit
        else:
            output = input
        return output
