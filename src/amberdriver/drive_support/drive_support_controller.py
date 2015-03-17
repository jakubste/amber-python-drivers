import logging
import logging.config
import sys
import threading
import traceback

from amberclient.common.amber_client import AmberClient
from amberclient.hokuyo.hokuyo import HokuyoProxy
from amberclient.ninedof.ninedof import NinedofProxy

import os
import serial

from amberdriver.drive_support.drive_support import DriveSupport
from amberdriver.null.null import NullController
from amberdriver.roboclaw.roboclaw import Roboclaw
from amberdriver.roboclaw.roboclaw_controller import RoboclawController
from amberdriver.tools import serial_port, config


__author__ = 'paoolo'

pwd = os.path.dirname(os.path.abspath(__file__))
logging.config.fileConfig('%s/dummy.ini' % pwd)
config.add_config_ini('%s/dummy.ini' % pwd)

LOGGER_NAME = 'DriverSupportController'

MOTORS_MAX_QPPS = int(config.ROBOCLAW_MAX_QPPS)
MOTORS_P_CONST = int(config.ROBOCLAW_P)
MOTORS_I_CONST = int(config.ROBOCLAW_I)
MOTORS_D_CONST = int(config.ROBOCLAW_D)

REAR_RC_ADDRESS = int(config.ROBOCLAW_REAR_RC_ADDRESS)
FRONT_RC_ADDRESS = int(config.ROBOCLAW_FRONT_RC_ADDRESS)

SERIAL_PORT = config.ROBOCLAW_SERIAL_PORT
BAUD_RATE = config.ROBOCLAW_BAUD_RATE
TIMEOUT = 0.3

if __name__ == '__main__':
    try:
        _serial = serial.Serial(port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=TIMEOUT)
        _serial_port = serial_port.SerialPort(_serial)

        roboclaw_front = Roboclaw(_serial_port, FRONT_RC_ADDRESS)
        roboclaw_rear = Roboclaw(_serial_port, REAR_RC_ADDRESS)

        roboclaw_front.set_m1_pidq(MOTORS_P_CONST, MOTORS_I_CONST, MOTORS_D_CONST, MOTORS_MAX_QPPS)
        roboclaw_front.set_m2_pidq(MOTORS_P_CONST, MOTORS_I_CONST, MOTORS_D_CONST, MOTORS_MAX_QPPS)
        roboclaw_rear.set_m1_pidq(MOTORS_P_CONST, MOTORS_I_CONST, MOTORS_D_CONST, MOTORS_MAX_QPPS)
        roboclaw_rear.set_m2_pidq(MOTORS_P_CONST, MOTORS_I_CONST, MOTORS_D_CONST, MOTORS_MAX_QPPS)

        client_for_hokuyo = AmberClient('127.0.0.1', name='hokuyo')
        client_for_ninedof = AmberClient('127.0.0.1', name='ninedof')

        hokuyo_proxy = HokuyoProxy(client_for_hokuyo, 0)
        ninedof_proxy = NinedofProxy(client_for_ninedof, 0)

        driver_support = DriveSupport(roboclaw_front, roboclaw_rear, hokuyo_proxy, ninedof_proxy)

        measuring_speeds_thread = threading.Thread(target=driver_support.measuring_loop, name='measuring-thread')
        measuring_speeds_thread.start()

        avoiding_thread = threading.Thread(target=driver_support.avoiding_loop, name='avoiding-thread')
        avoiding_thread.start()

        driving_thread = threading.Thread(target=driver_support.driving_loop, name='driving-thread')
        driving_thread.start()

        controller = RoboclawController(sys.stdin, sys.stdout, driver_support)
        controller()

    except BaseException as e:
        sys.stderr.write('Run without DriveSupport.\n')
        traceback.print_exc()

        controller = NullController(sys.stdin, sys.stdout)
        controller()
