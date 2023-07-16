import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.motion_commander import MotionCommander

# URI to the Crazyflie to connect to
uri = 'radio://0/82/2M/E7E7E7E701'

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)
DEFAULT_HEIGHT = 0.5


def simple_log(scf, logconf):

    with SyncLogger(scf, lg_stab) as logger:

        for log_entry in logger:

            timestamp = log_entry[0]
            data = log_entry[1]
            logconf_name = log_entry[2]

            print('[%d][%s]: %s' % (timestamp, logconf_name, data))

            break


def move_linear_simple(scf, lg_stab):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(1)
        simple_log(scf, lg_stab)
        mc.forward(0.5)
        time.sleep(1)
        simple_log(scf, lg_stab)
        mc.turn_left(180)
        time.sleep(1)
        simple_log(scf, lg_stab)
        mc.forward(0.5)
        time.sleep(1)
        simple_log(scf, lg_stab)


if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
    lg_stab.add_variable('stabilizer.roll', 'float')
    lg_stab.add_variable('stabilizer.pitch', 'float')
    lg_stab.add_variable('stabilizer.yaw', 'float')

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:

        # simple_connect()

        move_linear_simple(scf, lg_stab)
