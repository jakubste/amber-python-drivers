import logging
import logging.config
import time

import os

from amberclient.common.listener import Listener

from ambercommon.common import runtime

from amberdriver.drive_support.drive_support_logic import MotionAnalyzer, SpeedsAnalyzer, ScanAnalyzer, \
    VoltagesAnalyzer
from amberdriver.tools import config


__author__ = 'paoolo'

pwd = os.path.dirname(os.path.abspath(__file__))
logging.config.fileConfig('%s/drive_support.ini' % pwd)
config.add_config_ini('%s/drive_support.ini' % pwd)

LOGGER_NAME = 'DriveSupport'

ROBO_WIDTH = float(config.ROBO_WIDTH)


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
        self.__roboclaw_driver = roboclaw_driver

        self.__scan_analyzer = ScanAnalyzer()
        self.__motion_analyzer = MotionAnalyzer()
        self.__measured_speeds_analyzer = SpeedsAnalyzer()
        self.__voltages_analyzer = VoltagesAnalyzer()
        self.__user_speeds_analyzer = SpeedsAnalyzer()

        self.__measured_speeds = (0, 0, 0, 0)

        self.__last_motions = []
        self.__last_scans = []
        self.__last_voltages = []
        self.__last_measured_speeds = []
        self.__last_user_speeds = []

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
        self.__ninedof_proxy.unsubscribe(self.__ninedof_listener)

    def stop(self):
        self.__roboclaw_driver.stop()

    def set_scan(self, scan):
        scan = self.__scan_analyzer(scan)
        self.__last_scans.append(scan)

    def set_motion(self, motion):
        motion = self.__motion_analyzer(motion)
        self.__last_motions.append(motion)

    def measure_loop(self):
        while self.__is_active:
            speeds = self.__roboclaw_driver.get_speeds()
            voltages = self.__roboclaw_driver.get_voltages()

            speeds = self.__measured_speeds_analyzer(speeds)
            voltages = self.__voltages_analyzer(voltages)

            self.__last_measured_speeds.append(speeds)
            self.__last_voltages.append(voltages)

            self.__measured_speeds = (speeds.speed_front_left, speeds.speed_front_right,
                                      speeds.speed_rear_left, speeds.speed_rear_right)

            time.sleep(0.1)

    def get_speeds(self):
        return self.__measured_speeds

    def set_speeds(self, front_left, front_right, rear_left, rear_right):
        # detect if oscillation in speeds exists
        # detect if oscillation in measured speed exists
        # reduce/change speed due to environment
        # filter data

        user_speeds = self.__user_speeds_analyzer((front_left, front_right, rear_left, rear_right))

        self.__last_user_speeds.append(user_speeds)

        self.__roboclaw_driver.set_speeds(user_speeds.speed_front_left, user_speeds.speed_front_right,
                                          user_speeds.speed_rear_left, user_speeds.speed_rear_right)