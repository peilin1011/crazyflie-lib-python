import logging
import time
import numpy as np

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
# from cflib.positioning.motion_commander import MotionCommander
# from cflib.positioning.position_hl_commander import PositionHlCommander
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

# DEFAULT_HEIGHT = 0.5
# VELOCITY = 1
# position_estimate = [0, 0]
logging.basicConfig(level=logging.ERROR)


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


# DEBUG with high-level control
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
'''


def simple_log(scf, logconf):

    cur_pos = np.zeros((3, ), dtype=float)
    cur_euler = np.zeros((3, ), dtype=float)
    cur_vel = np.zeros((3, ), dtype=float)
    with SyncLogger(scf, logconf) as logger:

        for log_entry in logger:

            # timestamp = log_entry[0]
            data = log_entry[1]
            # logconf_name = log_entry[2]
            # DEBUG
            # print('[%d][%s]: %s' % (timestamp, logconf_name, data))
            # break
            cur_pos[:] = data[:3]
            cur_euler[:] = data[3:6]
            cur_vel[:] = data[6:9]
            return cur_pos, cur_euler, cur_pos


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
        logconf.add_variable('stateEstimate.vx', 'float')
        logconf.add_variable('stateEstimate.vy', 'float')
        logconf.add_variable('stateEstimate.vz', 'float')

        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_pos_callback)

        # start fly
        logconf.start()  # start record fly log
        cf = scf.cf
        height = 1
        cf.commander.send_setpoint(0, 0, 0, 0)  # uunlock carzyfile

        for i in range(500):
            logconf.data_received_cb.add_callback(log_pos_callback)
            cur_pos, cur_euler, cur_vel = simple_log(
                scf, logconf)  # call back crazy state
            thrust, target_euler, pos_e = dslPIDPositionControl(
                cur_pos, cur_euler, cur_vel)  # get next action
            cf.commander.send_setpoint(target_euler[0], target_euler[1], 0,
                                       thrust)  # send command to board
            time.sleep(0.01)

        cf.commander.send_stop_setpoint()
        # time.sleep(5)
        logconf.stop()
'''
import logging
import time
import numpy as np

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from pidtest import dslPIDPositionControl

class Crazy_Run:
  
    def __init__(self, link_uri):
        """ Initialize crazyflie using passed in link"""
        self._cf = Crazyflie()

        self._cf.open_link(link_uri) #connects to crazyflie and downloads TOC/Params
        self.is_connected = True

        # Logged states - ,
        # log.position, log.velocity and log.attitude are all in the body frame of reference
        self.position = [0.0, 0.0, 0.0]  # [m] in the global frame of reference
        self.velocity = [0.0, 0.0, 0.0]  # [m/s] in the global frame of reference
        self.attitude = [0.0, 0.0, 0.0]  # [rad] Attitude (p,r,y) with inverted roll (r)

        # Controller settings
        self.isEnabled = True

        self._PID = dslPIDPositionControl

    def _run_controller(self):
        """ Main control loop """
        # Wait for feedback
        time.sleep(2)
        with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

          logconf = LogConfig(name='stateEstimate', period_in_ms=10)
          logconf.add_variable('stateEstimate.roll', 'float')
          logconf.add_variable('stateEstimate.pitch', 'float')
          logconf.add_variable('stateEstimate.yaw', 'float')
          logconf.add_variable('stateEstimate.x', 'float')
          logconf.add_variable('stateEstimate.y', 'float')
          logconf.add_variable('stateEstimate.z', 'float')
          logconf.add_variable('stateEstimate.vx', 'float')
          logconf.add_variable('stateEstimate.vy', 'float')
          logconf.add_variable('stateEstimate.vz', 'float')  
          
          scf.cf.log.add_config(logconf)
          logconf.data_received_cb.add_callback(log_pos_callback)      

          # start fly
          logconf.start()  # start record fly log
          cf = scf.cf
          height = 1
          
          # Unlock Crazyflie
          self._cf.commander.send_setpoint(0, 0, 0, 0)

          if self.isEnabled:
            logconf.data_received_cb.add_callback(log_pos_callback)
            cur_pos, cur_euler, cur_vel = self.simple_log(scf, logconf)  # call back crazy state
          
            thrust, target_euler, pos_e= self._PID(cur_pos, cur_euler, cur_vel)  # get next action
          else:
            # If the controller is disabled, send a zero-thrust
            thrust, target_euler[0], target_euler[1] = (0, 0, 0)
            
          self._cf.commander.send_setpoint(target_euler[0], target_euler[1], 0,thrust)  # send command to board

          

          if pos_e[0] == 1.0 and pos_e[1] == 1.5 and pos_e[3] == 2.0:
            sleep(5)
            self._cf.commander.send_stop_setpoint()
            logconf.stop()

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

    def simple_log(scf, logconf):

       cur_pos = np.zeros((3, ), dtype=float)
       cur_euler = np.zeros((3, ), dtype=float)
       cur_vel = np.zeros((3, ), dtype=float)
       with SyncLogger(scf, logconf) as logger:

        for log_entry in logger:

            data = log_entry[1]
            cur_pos[:] = data[:3]
            cur_euler[:] = data[3:6]
            cur_vel[:] = data[6:9]
            return cur_pos, cur_euler, cur_pos


if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    # Scan for Crazyflies and use the first one found
    print("Scanning interfaces for Crazyflies...")
    available = cflib.crtp.scan_interfaces()
    print("Crazyflies found:")
   

    le = Crazy_Run('radio://0/80/2M/E7E7E7E704')
    if le.is_connected:
        time.sleep(1)



    else:
        print("No Crazyflies found, cannot run example")
'''
