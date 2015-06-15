import logging
import logging.config
import time

import os

from amberclient.common.listener import Listener

from ambercommon.common import runtime

from amberdriver.drive_support import drive_support_logic
from amberdriver.tools import logic, config


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

        self.__scan_analyzer = logic.ScanAnalyzer()
        self.__motion_analyzer = drive_support_logic.MotionAnalyzer()
        self.__speeds_analyzer = drive_support_logic.SpeedsAnalyzer()

        self.__speeds_limiter = drive_support_logic.Limiter()
        self.__measured_speeds = (0, 0, 0, 0)

        self.__is_active = True
        self.__last_timestamp = 0.0

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
        self.__speeds_limiter.update_scan(scan)

    def set_motion(self, motion):
        motion = self.__motion_analyzer(motion)
        self.__speeds_limiter.update_motion(motion)

    def measure_speeds_loop(self):
        while self.__is_active:
            measured_speeds = self.__roboclaw_driver.get_speeds()
            measured_speeds = self.__speeds_analyzer(measured_speeds)
            self.__speeds_limiter.update_measured_speeds(measured_speeds)
            self.__measured_speeds = (measured_speeds.speed_front_left, measured_speeds.speed_front_right,
                                      measured_speeds.speed_rear_left, measured_speeds.speed_rear_right)
            time.sleep(0.2)

    def get_speeds(self):
        return self.__measured_speeds

    def set_speeds(self, front_left, front_right, rear_left, rear_right):
        current_timestamp = time.time()
        if current_timestamp - self.__last_timestamp > 0.07:
            self.__last_timestamp = current_timestamp
            user_speeds = drive_support_logic.Speeds(front_left, front_right, rear_left, rear_right)
            self.__speeds_limiter(user_speeds)
            self.__roboclaw_driver.set_speeds(user_speeds.speed_front_left, user_speeds.speed_front_right,
                                              user_speeds.speed_rear_left, user_speeds.speed_rear_right)