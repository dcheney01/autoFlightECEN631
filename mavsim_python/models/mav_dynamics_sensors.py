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
from message_types.msg_sensors import MsgSensors
import parameters.aerosonde_parameters as MAV
import parameters.sensor_parameters as SENSOR
from models.mav_dynamics_control import MavDynamics as MavDynamicsNoSensors
from tools.rotations import quaternion_to_rotation, quaternion_to_euler, euler_to_rotation

class MavDynamics(MavDynamicsNoSensors):
    def __init__(self, Ts):
        super().__init__(Ts)
        # initialize the sensors message
        self._sensors = MsgSensors()
        # random walk parameters for GPS
        self._gps_nu_n = 0.
        self._gps_nu_e = 0.
        self._gps_nu_h = 0.
        # timer so that gps only updates every ts_gps seconds
        self._t_gps = 999.  # large value ensures gps updates at initial time.

    def sensors(self):
        "Return value of sensors on MAV: gyros, accels, absolute_pressure, dynamic_pressure, GPS"

        fx = self._forces.item(0)
        fy = self._forces.item(1)
        fz = self._forces.item(2)
        w_n = self._wind.item(0)
        w_e = self._wind.item(1)

        m = MAV.mass
        g = MAV.gravity
        rho = MAV.rho

        p_n = self._state.item(0)
        p_e = self._state.item(1)
        p_d = self._state.item(2)
        phi = self._state.item(6)
        theta = self._state.item(7)
        psi = self._state.item(8)
        p = self._state.item(10)
        q = self._state.item(11)
        r = self._state.item(12)

        # simulate rate gyros(units are rad / sec)
        self._sensors.gyro_x = p * SENSOR.gyro_x_bias + np.random.normal(scale=SENSOR.gyro_sigma)
        self._sensors.gyro_y = q * SENSOR.gyro_y_bias + np.random.normal(scale=SENSOR.gyro_sigma)
        self._sensors.gyro_z = r * SENSOR.gyro_z_bias + np.random.normal(scale=SENSOR.gyro_sigma)

        # simulate accelerometers(units of g)
        self._sensors.accel_x = fx/m + g*np.sin(theta) + np.random.normal(scale=SENSOR.accel_sigma)
        self._sensors.accel_y = fy/m - g*np.cos(theta)*np.sin(phi) + np.random.normal(scale=SENSOR.accel_sigma)
        self._sensors.accel_z = fz/m - g*np.cos(theta)*np.cos(phi) + np.random.normal(scale=SENSOR.accel_sigma)

        # simulate magnetometers
        # magnetic field in provo has magnetic declination of 12.5 degrees
        # and magnetic inclination of 66 degrees
        self._sensors.mag_x = 0
        self._sensors.mag_y = 0
        self._sensors.mag_z = 0

        # simulate pressure sensors
        self._sensors.abs_pressure = rho * g * -p_d + np.random.normal(scale=SENSOR.abs_pres_sigma)
        self._sensors.diff_pressure = (rho*self._Va**2) / 2 + np.random.normal(scale=SENSOR.diff_pres_sigma)
        
        # simulate GPS sensor
        if self._t_gps >= SENSOR.ts_gps:
            self._gps_nu_n = np.exp(-SENSOR.gps_k * SENSOR.ts_gps) * self._gps_nu_n + SENSOR.ts_gps*np.random.normal(scale=SENSOR.gps_n_sigma)
            self._gps_nu_e = np.exp(-SENSOR.gps_k * SENSOR.ts_gps) * self._gps_nu_e + SENSOR.ts_gps*np.random.normal(scale=SENSOR.gps_e_sigma)
            self._gps_nu_h = np.exp(-SENSOR.gps_k * SENSOR.ts_gps) * self._gps_nu_h + SENSOR.ts_gps*np.random.normal(scale=SENSOR.gps_h_sigma)
            self._sensors.gps_n = p_n + self._gps_nu_n
            self._sensors.gps_e = p_e + self._gps_nu_e
            self._sensors.gps_h = -p_d + self._gps_nu_h
            self._sensors.gps_Vg = np.sqrt(((self._Va*np.cos(psi) + w_n)**2) + (self._Va*np.sin(psi) + w_e)**2) + np.random.normal(scale=SENSOR.gps_Vg_sigma)
            self._sensors.gps_course = np.arctan2(self._Va*np.sin(psi) + w_e, self._Va*np.cos(psi) + w_n) + np.random.normal(scale=SENSOR.gps_course_sigma)
            self._t_gps = 0.
        else:
            self._t_gps += self._ts_simulation

        return self._sensors

    def external_set_state(self, new_state):
        self._state = new_state

    def _update_true_state(self):
        # update the class structure for the true state:
        #   [pn, pe, h, Va, alpha, beta, phi, theta, chi, p, q, r, Vg, wn, we, psi, gyro_bx, gyro_by, gyro_bz]
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
        self.true_state.bx = SENSOR.gyro_x_bias
        self.true_state.by = SENSOR.gyro_y_bias
        self.true_state.bz = SENSOR.gyro_z_bias
        self.true_state.camera_az = self._state.item(13)
        self.true_state.camera_el = self._state.item(14)