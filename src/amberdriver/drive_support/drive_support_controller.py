import logging
import logging.config
import sys
import threading

import traceback
from amberclient.ninedof.ninedof import NinedofProxy
import os
import serial
from amberclient.common.amber_client import AmberClient
from amberclient.hokuyo.hokuyo import HokuyoProxy

from amberdriver.drive_support.drive_support import DriveSupport
from amberdriver.null.null import NullController
from amberdriver.roboclaw.roboclaw import Roboclaw
from amberdriver.roboclaw.roboclaw_controller import RoboclawController, RoboclawDriver
from amberdriver.tools import serial_port, config


__author__ = 'paoolo'

pwd = os.path.dirname(os.path.abspath(__file__))
logging.config.fileConfig('%s/drive_support.ini' % pwd)
config.add_config_ini('%s/drive_support.ini' % pwd)

LOGGER_NAME = 'DriverSupport'

SERIAL_PORT = config.ROBOCLAW_SERIAL_PORT
BAUD_RATE = config.ROBOCLAW_BAUD_RATE

REAR_RC_ADDRESS = int(config.ROBOCLAW_REAR_RC_ADDRESS)
FRONT_RC_ADDRESS = int(config.ROBOCLAW_FRONT_RC_ADDRESS)

MOTORS_MAX_QPPS = int(config.ROBOCLAW_MOTORS_MAX_QPPS)
MOTORS_P_CONST = int(config.ROBOCLAW_P_CONST)
MOTORS_I_CONST = int(config.ROBOCLAW_I_CONST)
MOTORS_D_CONST = int(config.ROBOCLAW_D_CONST)

TIMEOUT = 0.7

if __name__ == '__main__':
    try:
        _serial = serial.Serial(port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=TIMEOUT)
        _serial_port = serial_port.SerialPort(_serial)

        roboclaw_front = Roboclaw(_serial_port, FRONT_RC_ADDRESS)
        roboclaw_rear = Roboclaw(_serial_port, REAR_RC_ADDRESS)
        roboclaw_driver = RoboclawDriver(roboclaw_front, roboclaw_rear,
                                         MOTORS_P_CONST, MOTORS_I_CONST, MOTORS_D_CONST, MOTORS_MAX_QPPS)

        sys.stderr.write('FIRMWARE VERSION, FRONT:\n%s\n' % str(roboclaw_front.read_firmware_version()))
        sys.stderr.write('FIRMWARE VERSION, REAR:\n%s\n' % str(roboclaw_rear.read_firmware_version()))

        sys.stderr.write('BUFFER LENGTH, FRONT:\n%s\n' % str(roboclaw_front.read_buffer_length()))
        sys.stderr.write('BUFFER LENGTH, REAR:\n%s\n' % str(roboclaw_rear.read_buffer_length()))

        sys.stderr.write('PIDQ SETTINGS, FRONT, M1:\n%s\n' % str(roboclaw_front.read_m1_pidq_settings()))
        sys.stderr.write('PIDQ SETTINGS, FRONT, M2:\n%s\n' % str(roboclaw_front.read_m2_pidq_settings()))
        sys.stderr.write('PIDQ SETTINGS, REAR, M1:\n%s\n' % str(roboclaw_rear.read_m1_pidq_settings()))
        sys.stderr.write('PIDQ SETTINGS, REAR, M2:\n%s\n' % str(roboclaw_rear.read_m2_pidq_settings()))

        sys.stderr.write('ENCODER MODE, FRONT:\n%s\n' % str(roboclaw_front.read_encoder_mode()))
        sys.stderr.write('ENCODER MODE, REAR:\n%s\n' % str(roboclaw_rear.read_encoder_mode()))

        client_for_hokuyo = AmberClient('127.0.0.1', name='hokuyo')
        hokuyo_proxy = HokuyoProxy(client_for_hokuyo, 0)
        client_for_ninedof = AmberClient('127.0.0.1', name='ninedof')
        ninedof_proxy = NinedofProxy(client_for_ninedof, 0)

        drive_support = DriveSupport(roboclaw_driver, hokuyo_proxy, ninedof_proxy)

        controller = RoboclawController(sys.stdin, sys.stdout, drive_support)

        timeout_monitor_thread = threading.Thread(target=roboclaw_driver.timeout_monitor_loop,
                                                  name='timeout-monitor-thread')
        error_monitor_thread = threading.Thread(target=roboclaw_driver.error_monitor_loop,
                                                name='error-monitor-thread')
        measuring_thread = threading.Thread(target=drive_support.measure_speeds_loop,
                                            name='measuring-thread')

        timeout_monitor_thread.start()
        error_monitor_thread.start()
        measuring_thread.start()

        controller.run()

    except BaseException as e:
        sys.stderr.write('Run without DriveSupport.\n')
        traceback.print_exc()

        controller = NullController(sys.stdin, sys.stdout)
        controller.run()