import numpy as np
import models.model_coef as TF
import parameters.aerosonde_parameters as MAV


#### TODO #####
gravity = MAV.gravity  # gravity constant
Va0 = TF.Va_trim
rho = MAV.rho  # density of air
sigma = 0.005  # low pass filter gain for derivative

#----------roll loop-------------
# get transfer function data for delta_a to phi
wn_roll = 0
zeta_roll = 0
roll_kp = 3.5
roll_kd = 0.3

#----------course loop-------------
wn_course = 0
zeta_course = 0
course_kp = 2.0
course_ki = 0

#----------yaw damper-------------
yaw_damper_p_wo = 0
yaw_damper_kr = 0

#----------pitch loop-------------
wn_pitch = 0
zeta_pitch = 0 
pitch_kp = -3.0
pitch_kd = -0.3
K_theta_DC = 0

#----------altitude loop-------------
wn_altitude = 0
zeta_altitude = 0
altitude_kp = 0.06
altitude_ki = 0.005
altitude_zone = 20

#---------airspeed hold using throttle---------------
wn_airspeed_throttle = 0
zeta_airspeed_throttle = 0
airspeed_throttle_kp = -2.2
airspeed_throttle_ki = -1.0
