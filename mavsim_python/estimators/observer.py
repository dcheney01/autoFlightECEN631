"""
observer
    - Beard & McLain, PUP, 2012
    - Last Update:
        3/2/2019 - RWB
        3/4/2024 - RWB
"""
import numpy as np
import parameters.control_parameters as CTRL
import parameters.aerosonde_parameters as MAV
import parameters.sensor_parameters as SENSOR
from tools.wrap import wrap
from message_types.msg_state import MsgState
from message_types.msg_sensors import MsgSensors
from estimators.filters import AlphaFilter, ExtendedKalmanFilterContinuousDiscrete

class Observer:
    def __init__(self, ts: float, initial_measurements: MsgSensors=MsgSensors()):
        self.Ts = ts  # sample rate of observer
        # initialized estimated state message
        self.estimated_state = MsgState()

        ##### TODO #####
        self.lpf_gyro_x = AlphaFilter(alpha=.7, y0=initial_measurements.gyro_x)
        self.lpf_gyro_y = AlphaFilter(alpha=0.9, y0=initial_measurements.gyro_y)
        self.lpf_gyro_z = AlphaFilter(alpha=0.7, y0=initial_measurements.gyro_z)
        self.lpf_accel_x = AlphaFilter(alpha=0.7, y0=initial_measurements.accel_x)
        self.lpf_accel_y = AlphaFilter(alpha=0.7, y0=initial_measurements.accel_y)
        self.lpf_accel_z = AlphaFilter(alpha=0.7, y0=initial_measurements.accel_z)

        self.lpf_north = AlphaFilter(alpha=0.7, y0=initial_measurements.gps_n)
        self.lpf_east = AlphaFilter(alpha=0.7, y0=initial_measurements.gps_e)
        self.lpf_Vg = AlphaFilter(alpha=0.7, y0=initial_measurements.gps_Vg)
        self.lpf_course = AlphaFilter(alpha=0.7, y0=initial_measurements.gps_course)

        # use alpha filters to low pass filter absolute and differential pressure
        self.lpf_abs = AlphaFilter(alpha=0.9, y0=initial_measurements.abs_pressure)
        self.lpf_diff = AlphaFilter(alpha=0.1, y0=initial_measurements.diff_pressure)
        # ekf for phi and theta
        self.attitude_ekf = ExtendedKalmanFilterContinuousDiscrete(
            f=self.f_attitude, 
            Q=np.diag([
                (0)**2, # phi 
                (0)**2, # theta
                ]), 
            P0= np.diag([
                (0)**2, # phi
                (0)**2, # theta
                ]), 
            xhat0=np.array([
                [0.*np.pi/180.], # phi 
                [0.*np.pi/180.], # theta
                ]), 
            Qu=np.diag([
                SENSOR.gyro_sigma**2, 
                SENSOR.gyro_sigma**2, 
                SENSOR.gyro_sigma**2, 
                SENSOR.abs_pres_sigma]), 
            Ts=ts,
            N=5
            )
        # ekf for pn, pe, Vg, chi, wn, we, psi
        self.position_ekf = ExtendedKalmanFilterContinuousDiscrete(
            f=self.f_smooth, 
            Q=np.diag([
                (0.0)**2,  # pn
                (0.0)**2,  # pe
                (0.0)**2,  # Vg
                (0.0)**2, # chi
                (0.0)**2, # wn
                (0.0)**2, # we
                (0.0)**2, # psi
                ]), 
            P0=np.diag([
                (0.)**2, # pn
                (0.0)**2, # pe
                (0.0)**2, # Vg
                (0.*np.pi/180.)**2, # chi
                (0.0)**2, # wn
                (0.0)**2, # we
                (0.*np.pi/180.)**2, # psi
                ]), 
            xhat0=np.array([
                [0.0], # pn 
                [0.0], # pe 
                [0.0], # Vg 
                [0.0], # chi
                [0.0], # wn 
                [0.0], # we 
                [0.0], # psi
                ]), 
            Qu=0.*np.diag([
                SENSOR.gyro_sigma**2, 
                SENSOR.gyro_sigma**2, 
                SENSOR.abs_pres_sigma,
                np.radians(3), # guess for noise on roll
                np.radians(3), # guess for noise on pitch
                ]),
            Ts=ts,
            N=10
            )
        self.R_accel = np.diag([
                SENSOR.accel_sigma**2, 
                SENSOR.accel_sigma**2, 
                SENSOR.accel_sigma**2
                ])
        self.R_pseudo = np.diag([
                0.0,  # pseudo measurement #1 ##### TODO #####
                0.0,  # pseudo measurement #2 ##### TODO #####
                ])
        self.R_gps = np.diag([
                    SENSOR.gps_n_sigma**2,  # y_gps_n
                    SENSOR.gps_e_sigma**2,  # y_gps_e
                    SENSOR.gps_Vg_sigma**2,  # y_gps_Vg
                    SENSOR.gps_course_sigma**2,  # y_gps_course
                    ])
        self.gps_n_old = 9999
        self.gps_e_old = 9999
        self.gps_Vg_old = 9999
        self.gps_course_old = 9999

    def update(self, measurement: MsgSensors) -> MsgState:
        ##### TODO #####

        # estimates for p, q, r are low pass filter of gyro minus bias estimate
        self.estimated_state.p = self.lpf_gyro_x.update(measurement.gyro_x) - SENSOR.gyro_x_bias
        self.estimated_state.q = self.lpf_gyro_y.update(measurement.gyro_y) - SENSOR.gyro_y_bias
        self.estimated_state.r = self.lpf_gyro_z.update(measurement.gyro_z) - SENSOR.gyro_z_bias

        # invert sensor model to get altitude and airspeed
        self.estimated_state.altitude = self.lpf_abs.update(measurement.abs_pressure) / (CTRL.rho * CTRL.gravity)
        self.estimated_state.Va = np.sqrt(2/CTRL.rho * self.lpf_diff.update(measurement.diff_pressure))

        # estimate phi and theta with simple ekf
        self.attitude_ekf.update(measurement, self.estimated_state)

        # estimate pn, pe, Vg, chi, wn, we, psi with ekf
        self.position_ekf.update(measurement, self.estimated_state)

        # not estimating these
        self.estimated_state.alpha = self.estimated_state.theta
        self.estimated_state.beta = 0.0
        self.estimated_state.bx = 0.0
        self.estimated_state.by = 0.0
        self.estimated_state.bz = 0.0
        return self.estimated_state


class AlphaFilter:
    # alpha filter implements a simple low pass filter
    # y[k] = alpha * y[k-1] + (1-alpha) * u[k]
    def __init__(self, alpha=0.5, y0=0.0):
        self.alpha = alpha  # filter parameter
        self.y = y0  # initial condition

    def update(self, u):
        ##### TODO #####
        self.y = self.alpha*self.y + (1-self.alpha)*u
        return self.y


class EkfAttitude:
    # implement continous-discrete EKF to estimate roll and pitch angles
    def __init__(self, ts):
        ##### TODO #####
        self.Q = np.diag([
            1e-6, # phi 
            1e-6, # theta
            ])
        self.P = np.diag([
            np.deg2rad(10)**2, # phi
            np.deg2rad(10)**2, # theta
            ])
        self.xhat = np.array([
            [MAV.phi0], # phi 
            [MAV.theta0], # theta
            ]) # initial state: phi, theta
        self.Q_gyro = SENSOR.gyro_sigma**2 * np.eye(3)
        self.R_accel = SENSOR.accel_sigma**2 * np.eye(3)
        self.N = 5  # number of prediction step per sample
        self.Ts = ts/self.N
        self.gate_threshold = stats.chi2.isf(q=0.01, df=3)

    def update(self, measurement, state):
        self.propagate_model(measurement, state)
        self.measurement_update(measurement, state)
        state.phi = self.xhat.item(0)
        state.theta = self.xhat.item(1)

    def f(self, x, measurement, state):
        # system dynamics for propagation model: xdot = f(x, u)
        ##### TODO #####
        phi = x.item(0)
        theta = x.item(1)

        p = measurement.gyro_x - SENSOR.gyro_x_bias
        q = measurement.gyro_y - SENSOR.gyro_y_bias
        r = measurement.gyro_z - SENSOR.gyro_z_bias

        xdot = np.array([[p + q*np.sin(phi)*np.tan(theta) + r*np.cos(phi)*np.tan(theta)],
                            [q*np.cos(phi) - r*np.sin(phi)]])
        return xdot

    def h_accel(self, x: np.ndarray, u: np.ndarray)->np.ndarray:
        '''
            measurement model y=h(x,u) for accelerometers
                x = [phi, theta].T
                u = [p, q, r, Va].T
        '''
        ##### TODO #####
        phi = x.item(0)
        theta = x.item(1)

        p = u.gyro_x - SENSOR.gyro_x_bias
        q = u.gyro_y - SENSOR.gyro_y_bias
        r = u.gyro_z - SENSOR.gyro_z_bias
        Va = np.sqrt(2/CTRL.rho * u.diff_pressure)
        g = CTRL.gravity

        y = np.array([[q*Va*np.sin(theta) + g*np.sin(theta)],
                      [r*Va*np.cos(theta) - p*Va*np.sin(theta) - g*np.cos(theta)*np.sin(phi)],
                      [-(q*Va*np.cos(theta) - g*np.cos(theta)*np.cos(phi))] ])
        return y

    def propagate_model(self, measurement, state):
        # model propagation
        ##### TODO #####
        for i in range(0, self.N):
            self.xhat = self.xhat + self.Ts*self.f(self.xhat, measurement, state)

            G = np.array([[1, np.sin(self.xhat.item(0))*np.tan(self.xhat.item(1)), np.cos(self.xhat.item(0))*np.tan(self.xhat.item(1))],
                          [0, np.cos(self.xhat.item(0)), -np.sin(self.xhat.item(0))]]) # TODO fix G
            A = jacobian(self.f, self.xhat, measurement, state)
            Ad = np.eye(2) + A*self.Ts + A@A*self.Ts**2
            self.P = Ad @ self.P @ Ad.T + self.Ts**2 * (self.Q + G @ self.Q_gyro @ G.T)

    def measurement_update(self, measurement, state):
        # measurement updates
        yhat = self.h(self.xhat, measurement, state)
        C = jacobian(self.h, self.xhat, measurement, state)
        y = np.array([[measurement.accel_x, measurement.accel_y, measurement.accel_z]]).T

        ##### TODO #####
        S_inv = np.linalg.inv(C @ self.P @ C.T + self.R_accel)
        if (y-yhat).T @ S_inv @ (y-yhat) < self.gate_threshold:
            L = self.P @ C.T @ S_inv
            self.P = (np.eye(2) - L @ C) @ self.P @ (np.eye(2) - L @ C).T + L @ self.R_accel @ L.T
            self.xhat = self.xhat + L @ (y-yhat)


class EkfPosition:
    # implement continous-discrete EKF to estimate pn, pe, Vg, chi, wn, we, psi
    def __init__(self, ts):
        self.Q = np.diag([
                    0.1, # pn
                    0.1, # pe
                    10, # Vg
                    10, # chi
                    0.1, # wn
                    0.1, # we
                    1, # psi
                    ])
        self.P = np.eye(7)
        self.xhat = np.array([
            [MAV.north0], # pn
            [MAV.east0], # pe
            [MAV.Va0], # Vg
            [0], # chi
            [0], # wn
            [0], # we
            [0], # psi
            ])
        self.R_gps = np.diag([
                    SENSOR.gps_n_sigma**2,  # y_gps_n
                    SENSOR.gps_e_sigma**2,  # y_gps_e
                    SENSOR.gps_Vg_sigma**2,  # y_gps_Vg
                    SENSOR.gps_course_sigma**2,  # y_gps_course
                    ])
        self.R_pseudo = np.diag([
                    0.001,  # pseudo measurement #1
                    0.001,  # pseudo measurement #2
                    ])
        self.N = 10  # number of prediction step per sample
        self.Tp = ts / self.N
        self.gps_n_old = 9999
        self.gps_e_old = 9999
        self.gps_Vg_old = 9999
        self.gps_course_old = 9999
        self.pseudo_threshold = stats.chi2.isf(q=0.01, df=2)
        self.gps_threshold = 100000 # don't gate GPS

    def update(self, measurement, state):
        self.propagate_model(measurement, state)
        self.measurement_update(measurement, state)
        state.north = self.xhat.item(0)
        state.east = self.xhat.item(1)
        state.Vg = self.xhat.item(2)
        state.chi = self.xhat.item(3)
        state.wn = self.xhat.item(4)
        state.we = self.xhat.item(5)
        state.psi = self.xhat.item(6)

    def f(self, x, measurement, state):
        # system dynamics for propagation model: xdot = f(x, u)
        Vg = x.item(2)
        chi = x.item(3)
        wn = x.item(4)
        we = x.item(5)
        psi = x.item(6)

        Va = np.sqrt(2/CTRL.rho * measurement.diff_pressure)
        q = measurement.gyro_y - SENSOR.gyro_y_bias
        r = measurement.gyro_z - SENSOR.gyro_z_bias
        phi = state.phi
        theta = state.theta

        psi_dot = q*np.sin(phi)/np.cos(theta) + r*np.cos(phi)/np.cos(theta)

        xdot = np.array([[Vg*np.cos(chi)],
                       [Vg*np.sin(chi)],
                       [((Va*np.cos(psi) + wn)*(-Va*psi_dot*np.sin(psi)) + (Va*np.sin(psi) + we)*(Va*psi_dot*np.cos(psi))) / Vg],
                       [CTRL.gravity/Vg*np.tan(phi)*np.cos(chi-psi)],
                       [0.0],
                       [0.0],
                       [psi_dot],
                       ])
        return xdot

    def h_gps(self, x, measurement, state):
        # measurement model for gps measurements y=h(x,u)

        y = np.array([
            [x.item(0)],  # pn
            [x.item(1)],  # pe
            [x.item(2)],  # Vg
            [x.item(3)],  # chi
        ])
        return y

    def h_pseudo(self, x, measurement, state):
        # measurement model for wind triangale pseudo measurement y=h(x,u)
        Vg = x.item(2)
        chi = x.item(3)
        wn = x.item(4)
        we = x.item(5)
        psi = x.item(6)

        Va = np.sqrt(2/CTRL.rho * measurement.diff_pressure)

        y = np.array([
            [Va*np.cos(psi) + wn - Vg*np.cos(chi)],  # wind triangle x
            [Va*np.sin(psi) + we - Vg*np.sin(chi)],  # wind triangle y
        ])
        return y

    def propagate_model(self, measurement, state):
        # model propagation
        for i in range(0, self.N):
            # propagate model
            self.xhat = self.xhat + self.Tp*self.f(self.xhat, measurement, state)

            # compute Jacobian
            A = jacobian(self.f, self.xhat, measurement, state)
            
            # convert to discrete time models
            Ad = np.eye(7) + A*self.Tp + A@A*self.Tp**2
            
            # update P with discrete time model
            self.P = Ad @ self.P @ Ad.T + self.Tp**2 * self.Q

    def measurement_update(self, measurement, state):
        # always update based on wind triangle pseudo measurement
        yhat = self.h_pseudo(self.xhat, measurement, state)
        C = jacobian(self.h_pseudo, self.xhat, measurement, state)
        y = np.array([[0, 0]]).T
        S_inv = np.linalg.inv(C @ self.P @ C.T + self.R_pseudo)
        if (y-yhat).T @ S_inv @ (y-yhat) < self.pseudo_threshold:
            # print("Used measurement update for pseudo")
            L = self.P @ C.T @ S_inv
            self.P = (np.eye(7) - L @ C) @ self.P @ (np.eye(7) - L @ C).T + L @ self.R_pseudo @ L.T
            self.xhat = self.xhat + L @ (y-yhat)

        # only update GPS when one of the signals changes
        if (measurement.gps_n != self.gps_n_old) \
            or (measurement.gps_e != self.gps_e_old) \
            or (measurement.gps_Vg != self.gps_Vg_old) \
            or (measurement.gps_course != self.gps_course_old):

            yhat = self.h_gps(self.xhat, measurement, state)
            C = jacobian(self.h_gps, self.xhat, measurement, state)
            y_chi = wrap(measurement.gps_course, yhat[3, 0])
            y = np.array([[measurement.gps_n,
                           measurement.gps_e,
                           measurement.gps_Vg,
                           y_chi]]).T
            S_inv = np.linalg.inv(C @ self.P @ C.T + self.R_gps)
            if True: #(y-yhat).T @ S_inv @ (y-yhat) < self.gps_threshold:
                L = self.P @ C.T @ S_inv
                self.P = (np.eye(7) - L @ C) @ self.P @ (np.eye(7) - L @ C).T + L @ self.R_gps @ L.T
                self.xhat = self.xhat + L @ (y-yhat)

            # update stored GPS signals
            self.gps_n_old = measurement.gps_n
            self.gps_e_old = measurement.gps_e
            self.gps_Vg_old = measurement.gps_Vg
            self.gps_course_old = measurement.gps_course


