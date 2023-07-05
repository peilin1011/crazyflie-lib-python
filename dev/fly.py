'''
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

                thrust, target_euler, pos_e = self._PID(cur_pos, cur_euler, cur_vel)  # get next action
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
'''
import os
import sys
sys.path.append("../lib")
import time
from threading import Thread, Timer
import threading

import termios
import contextlib
import numpy as np
import logging
logging.basicConfig(level=logging.ERROR)

from crazyflie_optitrack import Sensors

from cflib.crazyflie import Crazyflie
import cflib.crtp
from math import sin, cos, sqrt
from pidtest import dslPIDPositionControl



class Crazy_Auto:
    """ Basic calls and functions to enable autonomous flight """  
    def __init__(self, link_uri):
        """ Initialize crazyflie using passed in link"""
        self.s1 = threading.Semaphore(1)
        self._cf = Crazyflie()
        self.t = Sensors.logs(self)

        self._cf.open_link(link_uri) #connects to crazyflie and downloads TOC/Params
        self.is_connected = True

        # Logged states - ,
        # log.position, log.velocity and log.attitude are all in the body frame of reference
        self.position = [0.0, 0.0, 0.0]  # [m] in the global frame of reference
        self.velocity = [0.0, 0.0, 0.0]  # [m/s] in the global frame of reference
        self.attitude = [0.0, 0.0, 0.0]  # [rad] Attitude (p,r,y) with inverted roll (r)

        # Controller settings
        self.isEnabled = True


    def _run_controller(self):
        """ Main control loop """
        # Wait for feedback
        time.sleep(2)
        # Unlock the controller, BE CAREFLUE!!!
        self._cf.commander.send_setpoint(0, 0, 0, 0)

       

        #self.last_time = time.time()
        while True:

            x, y, z = self.position
            dx, dy, dz = self.velocity
            roll, pitch, yaw = self.attitude
            state = np.array([x, y, z, roll, pitch, yaw, dx, dy, dz])

            print("state: ", state)

            # Compute control signal - map errors to control signals
            if self.isEnabled:
    
                    a, b, c, d, e, f, g = dslPIDPositionControl(state)
                    #mpc
                    #mpc_policy = mpc(state[:6], target, horizon)
                    #roll_r, pitch_r, thrust_r = mpc_policy.solve()

                    # lqr
                    # lqr_policy = lqr(state, target, horizon)
                    # roll_r, pitch_r, thrust_r = lqr_policy.solve()
                    # print("roll_computed: ", roll_r)
                    # print("pitch_computed: ", pitch_r)
                    # print("thrust_computed: ", thrust_r)

                    #roll_r = self.saturate(roll_r/self.pi*180, self.roll_limit)
                    #pitch_r = self.saturate(pitch_r/self.pi*180, self.pitch_limit)
                    #thrust_r = self.saturate((thrust_r + self.m * self.g) * self.thrust2input, self.thrust_limit)  # minus, see notebook
                    # thrust_r = self.saturate(
                    #     (thrust_r + self.m * self.g) / (cos(pitch/180.*self.pi) * cos(roll/180.*self.pi)) * self.thrust2input,
                    #     self.thrust_limit)  # minus, see notebook

            else:
                # If the controller is disabled, send a zero-thrust
                roll_r, pitch_r, thrust_r = (0, 0, 0)

           
            print("thrust: ", int(thrust))
            print("target_euler: ", target_euler)
            print("pos_e: ", int(pos_e))

            cf.commander.send_setpoint(target_euler[0], target_euler[1], 0, thrust)


            '''
            ## PID
            # Compute control errors
            ex = x - x_r
            ey = y - y_r
            ez = z - z_r
            dex = dx - dx_r
            dey = dy - dy_r
            dez = dz - dz_r

            xi = 1.2
            wn = 3.0q
            Kp = - wn * wn
            Kd = - 2 * wn * xi

            Kxp = 1.2 * Kp
            Kxd = 1.2 * Kd
            Kyp = Kp
            Kyd = Kd
            Kzp = 0.8 * Kp
            Kzd = 0.8 * Kd
            # Compute control signal - map errors to control signals
            if self.isEnabled:
                ux = self.saturate(Kxp * ex + Kxd * dex, self.roll_limit)
                uy = self.saturate(Kyp * ey + Kyd * dey, self.pitch_limit)
                pitch_r = uy
                roll_r = ux
                # pitch_r = cos(yaw) * ux - sin(yaw) * uy
                # roll_r = sin(yaw) * ux + cos(yaw) * uy
                thrust_r = self.saturate((Kzp * ez + Kzd * dez + self.g) * self.m * self.thrust2input,
                                         self.thrust_limit)  # / (cos(roll) * cos(pitch))

            else:
                # If the controller is disabled, send a zero-thrust
                roll_r, pitch_r, yaw_r, thrust_r = (0, 0, 0, 0)
            yaw_r = 0
            # self._cf.commander.send_setpoint(roll_r, pitch_r, 0, int(thrust_r))
            # self._cf.commander.send_setpoint(0, 0, 0, int(thrust_r))
            print("Kp: ", Kp)
            print("Kd: ", Kd)
            print("z control: ", (((Kzp * ez + Kzd * dez + self.g) * self.m)))
            print("roll_r: ", roll_r)
            print("pitch_r: ", pitch_r)
            print("thrust_r: ", int(thrust_r))
            control_data.append(np.array([roll_r, pitch_r, yaw_r, int(thrust_r)]))
            '''

    def update_vals(self):
        self.s1.acquire()
        self.position = self.t.position
        self.velocity = self.t.velocity  # [m/s] in the global frame of reference
        self.attitude = self.t.attitude  # [rad] Attitude (p,r,y) with inverted roll (r)
        self.s1.release()
        # print("update_vals")
        Timer(.005, self.update_vals).start()



if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers()
    for i in available:
        print(i[0])

     le = Crazy_Auto('radio://0/82/2M/E7E7E7E701')
     while le.is_connected:
         time.sleep(1)

    else:
        print("No Crazyflies found, cannot run example")
