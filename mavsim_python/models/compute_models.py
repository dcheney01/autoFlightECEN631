"""
compute_ss_model
    - Chapter 5 assignment for Beard & McLain, PUP, 2012
    - Update history:  
        2/4/2019 - RWB
"""
import numpy as np
from scipy.optimize import minimize
from tools.rotations import euler_to_quaternion, quaternion_to_euler
import parameters.aerosonde_parameters as MAV
from parameters.simulation_parameters import ts_simulation as Ts
from message_types.msg_delta import MsgDelta


def compute_model(mav, trim_state, trim_input):
    # Note: this function alters the mav private variables
    A_lon, B_lon, A_lat, B_lat = compute_ss_model(mav, trim_state, trim_input)
    Va_trim, alpha_trim, theta_trim, a_phi1, a_phi2, a_theta1, a_theta2, a_theta3, \
    a_V1, a_V2, a_V3 = compute_tf_model(mav, trim_state, trim_input)

    # write transfer function gains to file
    file = open('models/model_coef.py', 'w')
    file.write('import numpy as np\n')
    file.write('x_trim = np.array([[%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f]]).T\n' %
               (trim_state.item(0), trim_state.item(1), trim_state.item(2), trim_state.item(3),
                trim_state.item(4), trim_state.item(5), trim_state.item(6), trim_state.item(7),
                trim_state.item(8), trim_state.item(9), trim_state.item(10), trim_state.item(11),
                trim_state.item(12)))
    file.write('u_trim = np.array([[%f, %f, %f, %f]]).T\n' %
               (trim_input.elevator, trim_input.aileron, trim_input.rudder, trim_input.throttle))
    file.write('Va_trim = %f\n' % Va_trim)
    file.write('alpha_trim = %f\n' % alpha_trim)
    file.write('theta_trim = %f\n' % theta_trim)
    file.write('a_phi1 = %f\n' % a_phi1)
    file.write('a_phi2 = %f\n' % a_phi2)
    file.write('a_theta1 = %f\n' % a_theta1)
    file.write('a_theta2 = %f\n' % a_theta2)
    file.write('a_theta3 = %f\n' % a_theta3)
    file.write('a_V1 = %f\n' % a_V1)
    file.write('a_V2 = %f\n' % a_V2)
    file.write('a_V3 = %f\n' % a_V3)
    file.write('A_lon = np.array([\n    [%f, %f, %f, %f, %f],\n    '
               '[%f, %f, %f, %f, %f],\n    '
               '[%f, %f, %f, %f, %f],\n    '
               '[%f, %f, %f, %f, %f],\n    '
               '[%f, %f, %f, %f, %f]])\n' %
    (A_lon[0][0], A_lon[0][1], A_lon[0][2], A_lon[0][3], A_lon[0][4],
     A_lon[1][0], A_lon[1][1], A_lon[1][2], A_lon[1][3], A_lon[1][4],
     A_lon[2][0], A_lon[2][1], A_lon[2][2], A_lon[2][3], A_lon[2][4],
     A_lon[3][0], A_lon[3][1], A_lon[3][2], A_lon[3][3], A_lon[3][4],
     A_lon[4][0], A_lon[4][1], A_lon[4][2], A_lon[4][3], A_lon[4][4]))
    file.write('B_lon = np.array([\n    [%f, %f],\n    '
               '[%f, %f],\n    '
               '[%f, %f],\n    '
               '[%f, %f],\n    '
               '[%f, %f]])\n' %
    (B_lon[0][0], B_lon[0][1],
     B_lon[1][0], B_lon[1][1],
     B_lon[2][0], B_lon[2][1],
     B_lon[3][0], B_lon[3][1],
     B_lon[4][0], B_lon[4][1],))
    file.write('A_lat = np.array([\n    [%f, %f, %f, %f, %f],\n    '
               '[%f, %f, %f, %f, %f],\n    '
               '[%f, %f, %f, %f, %f],\n    '
               '[%f, %f, %f, %f, %f],\n    '
               '[%f, %f, %f, %f, %f]])\n' %
    (A_lat[0][0], A_lat[0][1], A_lat[0][2], A_lat[0][3], A_lat[0][4],
     A_lat[1][0], A_lat[1][1], A_lat[1][2], A_lat[1][3], A_lat[1][4],
     A_lat[2][0], A_lat[2][1], A_lat[2][2], A_lat[2][3], A_lat[2][4],
     A_lat[3][0], A_lat[3][1], A_lat[3][2], A_lat[3][3], A_lat[3][4],
     A_lat[4][0], A_lat[4][1], A_lat[4][2], A_lat[4][3], A_lat[4][4]))
    file.write('B_lat = np.array([\n    [%f, %f],\n    '
               '[%f, %f],\n    '
               '[%f, %f],\n    '
               '[%f, %f],\n    '
               '[%f, %f]])\n' %
    (B_lat[0][0], B_lat[0][1],
     B_lat[1][0], B_lat[1][1],
     B_lat[2][0], B_lat[2][1],
     B_lat[3][0], B_lat[3][1],
     B_lat[4][0], B_lat[4][1],))
    file.write('Ts = %f\n' % Ts)
    file.close()

def compute_tf_model(mav, trim_state, trim_input):
    # trim values
    mav._state = trim_state
    mav._update_velocity_data()
    Va_trim = mav._Va
    alpha_trim = mav._alpha
    phi, theta_trim, psi = quaternion_to_euler(trim_state[6:10])

    ###### TODO ######
    # define transfer function constants
    a_phi1 = -0.5*MAV.rho*Va_trim**2*MAV.S_wing*MAV.b*MAV.C_p_p*MAV.b/(2*Va_trim)
    a_phi2 = 0.5*MAV.rho*Va_trim**2*MAV.S_wing*MAV.b*MAV.C_p_delta_a
    a_theta1 = -MAV.rho*Va_trim**2*MAV.c*MAV.S_wing/(2*MAV.Jy)*MAV.C_m_q*MAV.c/(2*Va_trim)
    a_theta2 = -MAV.rho*Va_trim**2*MAV.c*MAV.S_wing/(2*MAV.Jy)*MAV.C_m_alpha
    a_theta3 = MAV.rho*Va_trim**2*MAV.c*MAV.S_wing/(2*MAV.Jy)*MAV.C_m_delta_e

    # Compute transfer function coefficients using new propulsion model
    a_V1 = MAV.rho*Va_trim*MAV.S_wing / MAV.mass * (MAV.C_D_0 + MAV.C_D_alpha*alpha_trim + MAV.C_D_delta_e*trim_input.elevator)\
            - (1/MAV.mass)*dT_dVa(mav, Va_trim, trim_input.throttle)
    a_V2 = (1/MAV.mass) * dT_ddelta_t(mav, Va_trim, trim_input.throttle)
    a_V3 = MAV.gravity*np.cos(theta_trim - alpha_trim)

    return Va_trim, alpha_trim, theta_trim, a_phi1, a_phi2, a_theta1, a_theta2, a_theta3, a_V1, a_V2, a_V3


def compute_ss_model(mav, trim_state, trim_input):
    x_euler = euler_state(trim_state)
    

    # QUESTIONHERE: do I use E1 and E2 to convert to A_lon/B_lon? Same with E3/E4
    ##### TODO #####
    A = df_dx(mav, x_euler, trim_input)
    B = df_du(mav, x_euler, trim_input)
    # extract longitudinal states (u, w, q, theta, pd)
    E1 = np.zeros((5,12))
    E1[0, 3] = 1
    E1[1, 5] = 1
    E1[2, 10] = 1
    E1[3, 9] = 7
    E1[4, 2] = -1
    E2 = np.array([[0, 1], [1, 0], [0, 0], [0, 0], [0, 0]])

    A_lon = E1 @ A @ E1.T
    B_lon = E1 @ B @ E2.T
    # change pd to h

    # extract lateral states (v, p, r, phi, psi)
    A_lat = np.zeros((5,5))
    B_lat = np.zeros((5,2))
    return A_lon, B_lon, A_lat, B_lat

def euler_state(x_quat):
    # convert state x with attitude represented by quaternion
    # to x_euler with attitude represented by Euler angles
    
    ##### TODO #####
    x_euler = np.zeros((12,1))
    x_euler[:6] = x_quat[:6]
    x_euler[6:9,0] = quaternion_to_euler(x_quat[6:10])
    x_euler[9:] = x_quat[10:]
    return x_euler

def quaternion_state(x_euler):
    # convert state x_euler with attitude represented by Euler angles
    # to x_quat with attitude represented by quaternions

    ##### TODO #####
    x_quat = np.zeros((13,1))
    x_quat[:6] = x_euler[:6]
    x_quat[6:10] = euler_to_quaternion(x_euler.item(6), x_euler.item(7), x_euler.item(8))
    x_quat[10:] = x_euler[9:]
    return x_quat

def f_euler(mav, x_euler, delta):
    # return 12x1 dynamics (as if state were Euler state)
    # compute f at euler_state, f_euler will be f, except for the attitude states

    # need to correct attitude states by multiplying f by
    # partial of quaternion_to_euler(quat) with respect to quat
    # compute partial quaternion_to_euler(quat) with respect to quat
    # dEuler/dt = dEuler/dquat * dquat/dt
    x_quat = quaternion_state(x_euler)
    mav._state = x_quat
    mav._update_velocity_data()
    ##### TODO #####
    f_euler_ = df_dx(x_euler, x_quat, delta) + df_du(mav, x_euler, delta)

    return f_euler_

def df_dx(mav, x_euler, delta):
    # take partial of f_euler with respect to x_euler
    eps = 0.01  # deviation

    ##### TODO #####
    A = np.zeros((12, 12))  # Jacobian of f wrt x
    f_at_x = f_euler(mav, x_euler, delta)
    for i in range(12):
        x_euler_eps = x_euler
        x_euler_eps[i] += eps
        f_at_x_eps = f_euler(mav, x_euler_eps, delta)
        A[:, i] = ((f_at_x_eps - f_at_x) / eps)[:,0]
    return A

def df_du(mav, x_euler, delta):
    # take partial of f_euler with respect to input
    eps = 0.01  # deviation

    # QUESTIONHERE: how to add eps to delta?
    ##### TODO #####
    B = np.zeros((12, 4))  # Jacobian of f wrt u
    f_at_u = f_euler(mav, x_euler, delta)
    for i in range(4):
        delta_eps = delta
        delta_eps[i] += eps
        f_at_u_eps = f_euler(mav, x_euler, delta_eps)
        B[:, i] = ((f_at_u_eps - f_at_u) / eps)[:,0]
    return B


def dT_dVa(mav, Va, delta_t):
    # returns the derivative of motor thrust with respect to Va
    eps = 0.01

    ##### TODO #####
    Va_eps = Va + eps
    T_eps = mav._motor_thrust_torque(Va_eps, delta_t)[0]
    T = mav._motor_thrust_torque(Va, delta_t)[0]
    dT_dVa = (T_eps - T) / eps

    return dT_dVa

def dT_ddelta_t(mav, Va, delta_t):
    # returns the derivative of motor thrust with respect to delta_t
    eps = 0.01

    ##### TODO #####
    delta_t_eps = delta_t + eps
    T_eps = mav._motor_thrust_torque(Va, delta_t_eps)[0]
    T = mav._motor_thrust_torque(Va, delta_t)[0]
    dT_ddelta_t = (T_eps - T) / eps

    return dT_ddelta_t
