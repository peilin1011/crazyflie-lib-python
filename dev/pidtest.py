import math
import numpy as np
from scipy.spatial.transform import Rotation as R


def dslPIDPositionControl(cur_pos, cur_euler, cur_vel):
    '''DSL's CF2.x PID position control.

    Args:
        cur_pos (ndarray): (3,1)-shaped array of floats containing the current position.
        cur_quat (ndarray): (4,1)-shaped array of floats containing the current orientation as a quaternion.
        cur_vel (ndarray): (3,1)-shaped array of floats containing the current velocity.
        target_pos (ndarray): (3,1)-shaped array of floats containing the desired position.
        target_rpy (ndarray): (3,1)-shaped array of floats containing the desired orientation as roll, pitch, yaw.
        target_vel (ndarray): (3,1)-shaped array of floats containing the desired velocity.

    Returns:
        thrust (float): The target thrust along the drone z-axis.
        target_euler (ndarray): (3,1)-shaped array of floats containing the target roll, pitch, and yaw.
        pos_e (float): The current position error.
    '''
    '''new code'''

    target_pos = np.array([0.0, 0.0, 1])
    target_vel = np.array([0.0, 0.0, 0.0])
    target_rpy = np.array([0.0, 0.0, 0.0])
    rot = R.from_euler('zyx', cur_euler, degrees=True)
    cur_rotation = rot.as_matrix()
    pos_e = target_pos - cur_pos
    vel_e = target_vel - cur_vel

    # self.integral_pos_e = self.integral_pos_e + pos_e * self.control_timestep
    # self.integral_pos_e = np.clip(self.integral_pos_e, -2., 2.)
    # self.integral_pos_e[2] = np.clip(self.integral_pos_e[2], -0.15, .15)

    # PID target thrust.
    target_thrust = np.multiply(2.0, pos_e) + np.multiply(
        25.0, vel_e) + np.array([0, 0, 0.26])
    # + np.multiply(self.I_COEFF_FOR, self.integral_pos_e) \
    # + np.multiply(self.D_COEFF_FOR, vel_e) + np.array([0, 0, self.GRAVITY])
    scalar_thrust = max(0., np.dot(target_thrust, cur_rotation[:, 2]))
    thrust = (math.sqrt(scalar_thrust / (4 * 3.16e-10)) - 4070.3) / 0.2685
    target_z_ax = target_thrust / np.linalg.norm(target_thrust)
    target_x_c = np.array(
        [math.cos(target_rpy[2]),
         math.sin(target_rpy[2]), 0])
    target_y_ax = np.cross(target_z_ax, target_x_c) / np.linalg.norm(
        np.cross(target_z_ax, target_x_c))
    target_x_ax = np.cross(target_y_ax, target_z_ax)
    target_rotation = (np.vstack([target_x_ax, target_y_ax,
                                  target_z_ax])).transpose()

    # Target rotation.
    target_euler = (R.from_matrix(target_rotation)).as_euler('XYZ',
                                                             degrees=False)

    # if np.any(np.abs(target_euler) > math.pi):
    #     raise ValueError('\n[ERROR] ctrl it', self.control_counter, 'in Control._dslPIDPositionControl(), values outside range [-pi,pi]')

    return thrust, target_euler