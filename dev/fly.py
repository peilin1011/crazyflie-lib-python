# import logging
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
        self.uri = link_uri
        self._cf.open_link(
            link_uri)  # connects to crazyflie and downloads TOC/Params
        self.is_connected = True

        # Logged states - ,
        # log.position, log.velocity and log.attitude are all in the body frame of reference
        self.position = [0.0, 0.0, 0.0]  # [m] in the global frame of reference
        self.velocity = [0.0, 0.0,
                         0.0]  # [m/s] in the global frame of reference
        self.attitude = [0.0, 0.0,
                         0.0]  # [rad] Attitude (p,r,y) with inverted roll (r)

        # Controller settings
        self.isEnabled = True

        self._PID = dslPIDPositionControl

    def log_pos_callback(self, timestamp, data, logconf):
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

    def simple_log(self, scf, logconf):

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

    def _run_controller(self):
        """ Main control loop """
        # Wait for feedback
        time.sleep(2)
        with SyncCrazyflie(self.uri, cf=Crazyflie(rw_cache='./cache')) as scf:

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
            logconf.data_received_cb.add_callback(self.log_pos_callback)

            # start fly
            logconf.start()  # start record fly log
            # cf = scf.cf
            # height = 1

            # Unlock Crazyflie
            self._cf.commander.send_setpoint(0, 0, 0, 0)

            if self.isEnabled:
                logconf.data_received_cb.add_callback(self.log_pos_callback)
                cur_pos, cur_euler, cur_vel = self.simple_log(
                    scf, logconf)  # call back crazy state

                thrust, target_euler, pos_e = self._PID(
                    cur_pos, cur_euler, cur_vel)  # get next action
            else:
                # If the controller is disabled, send a zero-thrust
                thrust, target_euler[0], target_euler[1] = (0, 0, 0)

            self._cf.commander.send_setpoint(target_euler[0], target_euler[1],
                                             0,
                                             thrust)  # send command to board

            if pos_e[0] == 1.0 and pos_e[1] == 1.5 and pos_e[3] == 2.0:
                time.sleep(5)
                self._cf.commander.send_stop_setpoint()
                logconf.stop()


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
