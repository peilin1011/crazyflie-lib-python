import logging
import sys
import time
from threading import Event
import math

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils import uri_helper
from cflib.crazyflie.syncLogger import SyncLogger
from pidtest import dslPIDPositionControl

x = 0
y = 0
z = 0
roll = 0
pitch = 0
yaw = 0
vx = 0
vy = 0
vz = 0

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E704')

DEFAULT_HEIGHT = 0.5
VELOCITY = 1
position_estimate = [0, 0]
logging.basicConfig(level=logging.ERROR)


def log_pos_callback(timestamp, data, logconf):
    global x, y, z, roll, pitch, yaw, vx, vy, vz
    x = data['stateEstimate.x']
    y = data['stateEstimate.y']
    z = data['stateEstimate.z']
    roll = data['stateEstimate.roll']
    pitch = data['stateEstimate.pitch']
    yaw = data['stateEstimate.yaw']
    '''
    vx = data['stateEstimate.vx']
    vy = data['stateEstimate.vy']
    vz = data['stateEstimate.vz']
    '''


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


def simple_log(scf, logconf):

    with SyncLogger(scf, logconf) as logger:

        for log_entry in logger:

            timestamp = log_entry[0]
            data = log_entry[1]
            logconf_name = log_entry[2]

            print('[%d][%s]: %s' % (timestamp, logconf_name, data))
            break


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    # 飞行前的安全性
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

        # 记录飞行日志

        logconf = LogConfig(name='stateEstimate', period_in_ms=10)
        logconf.add_variable('stateEstimate.roll', 'float')
        logconf.add_variable('stateEstimate.pitch', 'float')
        logconf.add_variable('stateEstimate.yaw', 'float')

        logconf.add_variable('stateEstimate.x', 'float')
        logconf.add_variable('stateEstimate.y', 'float')
        logconf.add_variable('stateEstimate.z', 'float')
        '''
        logconf.add_variable('stateEstimate.vx', 'float')
        logconf.add_variable('stateEstimate.vy', 'float')
        logconf.add_variable('stateEstimate.vz', 'float')
        '''

        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_pos_callback)

        # 执行任务
        logconf.start()
        # action
        cf = scf.cf
        height = 1
        cf.commander.send_setpoint(0, 0, 0, 0)

        for i in range(500):

            cf.commander.send_position_setpoint(0, 0, height, 0)
            time.sleep(0.01)
            simple_log(scf, logconf)
        '''
        to do: stabilizazion 
        for i in range(500):
            # print(logconf.data_received_cb.add_callback(log_pos_callback))
            # thrust, target_euler, pos_e = dslPIDPositionControl(cur_pos,cur_euler,cur_vel)
            # cf.commander.send_setpoint(target_euler[0],target_euler[1],0,thrust)
            cf.commander.send_setpoint(0, 0, 0, 35000)
            time.sleep(0.01)
        '''
        # cf.commander.send_stop_setpoint()
        time.sleep(5)
        logconf.stop()