import logging
import logging.config
import time

import os

from amberclient.common.listener import Listener

from ambercommon.common import runtime

from amberdriver.drive_support.drive_support_logic import average, Speed
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
        pass

    def set_motion(self, motion):
        pass

    def measure_loop(self):
        while self.__is_active:
            speeds = self.__roboclaw_driver.get_speeds()
            currents = self.__roboclaw_driver.get_currents()
            voltages = self.__roboclaw_driver.get_voltages()
            time.sleep(0.1)

    def get_speeds(self):
        return self.__roboclaw_driver.get_speeds()

    def set_speeds(self, front_left, front_right, rear_left, rear_right):
        # detect if oscillation in speeds exists
        # detect if oscillation in measured speed exists
        # reduce/change speed due to environment
        # filter data

        speeds = Speed(front_left, front_right, rear_left, rear_right)
        self.__roboclaw_driver.set_speeds(speeds.speed_front_left, speeds.speed_front_right,
                                          speeds.speed_rear_left, speeds.speed_rear_right)


def compute_speed_radius(speeds):
    speed_left = average(speeds.front_left, speeds.rear_left)
    speed_right = average(speeds.front_right, speeds.rear_right)
    speed_front = average(speeds.front_left, speeds.front_right)
    speed_rear = average(speeds.rear_left, speeds.rear_right)

    speed_left_right = average(speed_left, speed_right)
    speed_front_rear = average(speed_front, speed_rear)

    speed = average(speed_left_right, speed_front_rear)
    if abs(speed_left - speed_right) > 0.0:
        radius = speed_right * ROBO_WIDTH / (speed_left - speed_right) + (ROBO_WIDTH / 2.0)
    else:
        radius = 0.0

    return speed, radius


def compute_accel_side_speed_rotational(speed, radius):
    speed_rotational = speed / radius
    acc_side = speed_rotational * speed
    return acc_side, speed_rotational