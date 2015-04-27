import logging
import logging.config
import time

import os
from amberclient.common.listener import Listener

from ambercommon.common import runtime

from amberdriver.drive_support import drive_support_logic
from amberdriver.tools import config


__author__ = 'paoolo'

pwd = os.path.dirname(os.path.abspath(__file__))
logging.config.fileConfig('%s/drive_support.ini' % pwd)
config.add_config_ini('%s/drive_support.ini' % pwd)

LOGGER_NAME = 'DriveSupport'


class ScanHandler(Listener):
    def __init__(self, driver):
        self.__driver_support = driver

    def handle(self, response):
        self.__driver_support.set_scan(response)


class MotionHandler(Listener):
    def __init__(self, driver):
        self.__drive_support = driver

    def handle(self, response):
        self.__drive_support.set_motion(response)


class DriveSupport(object):
    def __init__(self, roboclaw_driver, hokuyo_proxy, ninedof_proxy):
        self.__scan = None
        self.__motion = None

        self.__roboclaw_driver = roboclaw_driver

        self.__is_active = True

        self.__hokuyo_proxy = hokuyo_proxy
        self.__ninedof_proxy = ninedof_proxy

        self.__hokuyo_listener = ScanHandler(self)
        self.__ninedof_listener = MotionHandler(self)

        hokuyo_proxy.subscribe(self.__hokuyo_listener)
        ninedof_proxy.subscribe(self.__ninedof_listener)

        self.__logger = logging.getLogger(LOGGER_NAME)

        runtime.add_shutdown_hook(self.terminate)

    def terminate(self):
        self.__is_active = False
        self.__hokuyo_proxy.unsubscribe(self.__hokuyo_listener)

    def stop(self):
        self.__roboclaw_driver.stop()

    def set_scan(self, scan):
        self.__scan = scan

    def set_motion(self, motion):
        self.__motion = motion

    def set_speeds(self, front_left, front_right, rear_left, rear_right):
        fl, fr, rl, rr = front_left, front_right, rear_left, rear_right
        fl, fr, rl, rr = DriveSupport.__drive_support(fl, fr, rl, rr, time.time(), self.__scan)
        self.__roboclaw_driver.set_speeds(fl, fr, rl, rr)

    def get_measured_speeds(self):
        return self.__roboclaw_driver.get_measured_speeds()

    @staticmethod
    def __drive_support(fl, fr, rl, rr, speed_ts, scan):
        if scan is None:
            return 0, 0, 0, 0

        scan_ts = scan.get_timestamp()
        scan_points = scan.get_points()

        fl, fr = drive_support_logic.limit_due_to_distance(fl, fr, scan_points)
        rl, rr = drive_support_logic.limit_due_to_distance(rl, rr, scan_points)

        """
        TODO(paoolo): do nice stuff with avoiding obstacles
        * filter speed values
        * filter motion values
        * filter distances values
        * predict acc/gyro values
        * compare predicted values with real data
        * detect robot motion state (?)
        * apply amendment to speed (?)
        * lookahead around robot
        * smooth speed changes
        * apply trust values
        """

        current_timestamp = time.time()
        trust_level = drive_support_logic.scan_trust(scan_ts, current_timestamp) * \
                      drive_support_logic.command_trust(speed_ts, current_timestamp)

        fl *= trust_level
        rl *= trust_level
        fr *= trust_level
        rr *= trust_level

        return int(fl), int(fr), int(rl), int(rr)