###############  test the PID controller from 'crazyflie_simulator-main'   ###################
import logging
import numpy as np

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
# from cflib.positioning.motion_commander import MotionCommander
# from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils import uri_helper
from cflib.crazyflie.syncLogger import SyncLogger

from position_ctl_m import position_controller_m

# Initialize logging
logging.basicConfig(level=logging.ERROR)

# Set threshold and target height
threshold = 0.001
target_height = 0.5
max_thrust = 39000

x = 0
y = 0
z = 0
roll = 0
pitch = 0
yaw = 0
vx = 0
vy = 0
vz = 0

# Define the connection URI and default height
URI = uri_helper.uri_from_env(default='radio://0/82/2M/E7E7E7E701')
DEFAULT_HEIGHT = 0.8

# DEBUG with high-level control
'''
def log_pos_callback(timestamp, data, logconf):
    global x, y, z, roll, pitch, yaw, vx, vy, vz
    x = data['stateEstimate.x']
    y = data['stateEstimate.y']
    z = data['stateEstimate.z']
    roll = data['stateEstimate.roll']
    pitch = data['stateEstimate.pitch']
    yaw = data['stateEstimate.yaw']
    vx = data['stateEstimate.vx']
    vy = data['stateEstimate.vy']
    vz = data['stateEstimate.vz']
    # print(data)


def move_linear_simple(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(1)
        mc.forward(0.5)
        time.sleep(1)
        mc.turn_left(180)
        time.sleep(1)
        mc.forward(0.5)
        time.sleep(1)



def circle_right(scf, radius_m, velocity=VELOCITY, angle_degrees=360.0):
    """
    Go in circle, clock wise

    :param radius_m: The radius of the circle (meters)
    :param velocity: The velocity along the circle (meters/second)
    :param angle_degrees: How far to go in the circle (degrees)
    :return:
    """
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        distance = 2 * radius_m * math.pi * angle_degrees / 360.0
        flight_time = distance / velocity

        mc.start_circle_right(radius_m, velocity)
        time.sleep(flight_time)
        mc.stop()


def tracking():
    with PositionHlCommander(
            scf, controller=PositionHlCommander.CONTROLLER_PID) as pc:
        # Go to the coordinate (0, 0, 1)
        pc.go_to(0.0, 0.0, 1.0)
        # The Crazyflie lands when leaving this "with" section
    # When leaving this "with" section, the connection is automatically closed
'''


# Calculate the required thrust and target attitude using PID control
def simple_log(scf):

    cur_pos = np.zeros((3, ), dtype=float)
    cur_euler = np.zeros((3, ), dtype=float)
    cur_vel = np.zeros((3, ), dtype=float)
    cur_euler_rate = np.zeros((3, ), dtype=float)

    lg_stab = LogConfig(name='state',
                        period_in_ms=10)  # log synchronously every 2 seconds
    lg_stab.add_variable('stateEstimate.x', 'float')
    lg_stab.add_variable('stateEstimate.y', 'float')
    lg_stab.add_variable('stateEstimate.z', 'float')

    lg_stab.add_variable('stateEstimate.vx', 'float')
    lg_stab.add_variable('stateEstimate.vy', 'float')
    lg_stab.add_variable('stateEstimate.vz', 'float')

    lg_stab_2 = LogConfig(name='angle', period_in_ms=10)
    lg_stab_2.add_variable('stateEstimate.roll', 'float')
    lg_stab_2.add_variable('stateEstimate.pitch', 'float')
    lg_stab_2.add_variable('stateEstimate.yaw', 'float')

    lg_stab_2.add_variable('extrx.rollRate', 'float')
    lg_stab_2.add_variable('extrx.pitchRate', 'float')
    lg_stab_2.add_variable('extrx.yawRate', 'float')

    with SyncLogger(scf, [lg_stab, lg_stab_2]) as logger:
        l = {}

        for log_entry in logger:

            # timestamp = log_entry[0]
            data = log_entry[1]
            logconf_name = log_entry[2]
            # DEBUG:
            # print('[%d][%s]: %s' % (timestamp, logconf_name, data))
            # break
            l.update(data)
            if logconf_name.name == 'angle':

                x = l['stateEstimate.x']
                y = l['stateEstimate.y']
                z = l['stateEstimate.z']
                vx = l['stateEstimate.vx']
                vy = l['stateEstimate.vy']
                vz = l['stateEstimate.vz']
                roll = l['stateEstimate.roll']
                pitch = l['stateEstimate.pitch']
                yaw = l['stateEstimate.yaw']
                roll_rate = l['extrx.rollRate']
                pitch_rate = l['extrx.pitchRate']
                yaw_rate = l['extrx.yawRate']
                cur_pos = np.array([x, y, z])
                cur_euler = np.array([roll, pitch, yaw])
                cur_vel = np.array([vx, vy, vz])
                cur_euler_rate = np.array([roll_rate, pitch_rate, yaw_rate])

                # print(cur_pos, cur_euler, cur_vel, cur_euler_rate)
                print('cur_pos: ', cur_pos)
                # print(type(cur_pos))

                thrust, target_euler = position_controller_m(
                    cur_pos, cur_vel, cur_euler, quad)
                print('calculated thrust:', thrust)
                if thrust >= 60000:
                    thrust = max_thrust
                elif thrust <= 0:
                    thrust = 0
                print('required thrust and euler:', thrust, target_euler)
                return thrust, target_euler


# Send control commands to the drone
def run(scf, target_euler, thrust):
    cf = scf.cf
    cf.commander.send_setpoint(target_euler[0], target_euler[1],
                               -0.001 * target_euler[2], int(thrust))
    # cf.commander.send_position_setpoint(0, 0, 0.5, target_euler[2])


# Get the current position information
def get_position(scf):

    cur_pos = np.zeros((3, ), dtype=float)
    cur_euler = np.zeros((3, ), dtype=float)
    cur_vel = np.zeros((3, ), dtype=float)
    cur_euler_rate = np.zeros((3, ), dtype=float)

    lg_stab = LogConfig(name='stateEstimate',
                        period_in_ms=10)  # log synchronously every 2 seconds
    lg_stab.add_variable('stateEstimate.x', 'float')
    lg_stab.add_variable('stateEstimate.y', 'float')
    lg_stab.add_variable('stateEstimate.z', 'float')

    lg_stab.add_variable('stateEstimate.vx', 'float')
    lg_stab.add_variable('stateEstimate.vy', 'float')
    lg_stab.add_variable('stateEstimate.vz', 'float')

    lg_stab_2 = LogConfig(name='angle', period_in_ms=10)
    lg_stab_2.add_variable('stateEstimate.roll', 'float')
    lg_stab_2.add_variable('stateEstimate.pitch', 'float')
    lg_stab_2.add_variable('stateEstimate.yaw', 'float')

    lg_stab_2.add_variable('extrx.rollRate', 'float')
    lg_stab_2.add_variable('extrx.pitchRate', 'float')
    lg_stab_2.add_variable('extrx.yawRate', 'float')

    with SyncLogger(scf, [lg_stab, lg_stab_2]) as logger:
        l = {}
        for log_entry in logger:

            # timestamp = log_entry[0]
            data = log_entry[1]
            logconf_name = log_entry[2]
            # DEBUG:
            # print('[%d][%s]: %s' % (timestamp, logconf_name, data))
            # break
            l.update(data)

            if logconf_name.name == 'angle':

                x = l['stateEstimate.x']
                y = l['stateEstimate.y']
                z = l['stateEstimate.z']
                vx = l['stateEstimate.vx']
                vy = l['stateEstimate.vy']
                vz = l['stateEstimate.vz']
                roll = l['stateEstimate.roll']
                pitch = l['stateEstimate.pitch']
                yaw = l['stateEstimate.yaw']
                roll_rate = l['extrx.rollRate']
                pitch_rate = l['extrx.pitchRate']
                yaw_rate = l['extrx.yawRate']
                cur_pos = np.array([x, y, z])
                cur_euler = np.array([roll, pitch, yaw])
                cur_vel = np.array([vx, vy, vz])
                cur_euler_rate = np.array([roll_rate, pitch_rate, yaw_rate])
                return cur_pos, cur_euler, cur_vel, cur_euler_rate


if __name__ == '__main__':
    # Initialize the Crazyflie library
    cflib.crtp.init_drivers()
    # Establish a connection with the drone using the SyncCrazyflie class
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        print('Connect Successful')
        # Unlock startup thrust protection
        scf.cf.commander.send_setpoint(0, 0, 0, 0)
        # print(get_position(scf)[0])
        thrust = 0
        target_euler = np.array([0, 0, 0])

        while True:
            # Check if the current position is close to the target height
            if (abs(get_position(scf)[0][2] - target_height) >
                    threshold).all():  # not reach the target postion
                # Get the new thrust and target attitude
                thrust, target_euler = simple_log(
                    scf)  # get new requiredt thrust
            # Send control commands
            run(scf, target_euler, thrust)
