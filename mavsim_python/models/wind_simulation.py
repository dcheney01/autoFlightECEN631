"""
Class to determine wind velocity at any given moment,
calculates a steady wind speed and uses a stochastic
process to represent wind gusts. (Follows section 4.4 in uav book)
"""
from tools.transfer_function import TransferFunction
import numpy as np
import parameters.aerosonde_parameters as MAV_PARAM


class WindSimulation:
    def __init__(self, Ts, gust_flag = True, steady_state = np.array([[0., 0., 0.]]).T):
        # steady state wind defined in the inertial frame
        self._steady_state = steady_state
        ##### TODO #####

        #   Dryden gust model parameters (pg 56 UAV book)
        self.L_u = 200.0
        self.L_v = 200.0
        self.L_w = 50.0
        self.Va = MAV_PARAM.Va0

        if gust_flag:
            self.sigma_u = 1.06
            self.sigma_v = 1.06
            self.sigma_w = 0.7
        else:
            self.sigma_u = 0.0
            self.sigma_v = 0.0
            self.sigma_w = 0.0

        #   Dryden transfer functions (section 4.4 UAV book) - Fill in proper num and den
        u_coefficient = self.sigma_u*np.sqrt(2*self.Va / (np.pi * self.L_u))
        v_coefficient = self.sigma_v*np.sqrt(2*self.Va / (np.pi*self.L_v))
        w_coefficient = self.sigma_w*np.sqrt(3*self.Va / (np.pi*self.L_w))
        
        self.u_w = TransferFunction(num=np.array([[u_coefficient]]),
                                    den=np.array([[1, self.Va/self.L_u]]), Ts=Ts)
        self.v_w = TransferFunction(num=np.array([[v_coefficient, v_coefficient*self.Va/(np.sqrt(3)*self.L_v)]]),
                                    den=np.array([[1, 2*self.Va/self.L_v, self.Va**2/self.L_v**2]]), Ts=Ts)
        self.w_w = TransferFunction(num=np.array([[w_coefficient, w_coefficient*self.Va/(np.sqrt(3)*self.L_w)]]),
                                    den=np.array([[1, 2*self.Va/self.L_w, self.Va**2/self.L_w**2]]), Ts=Ts)
        


    def update(self):
        # returns a six vector.
        #   The first three elements are the steady state wind in the inertial frame
        #   The second three elements are the gust in the body frame
        gust = np.array([[self.u_w.update(np.random.randn())],
                         [self.v_w.update(np.random.randn())],
                         [self.w_w.update(np.random.randn())]])
        return np.concatenate(( self._steady_state, gust ))

