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
        self.__scan, self.__scan_ts = None, 0.0
        self.__motion, self.__motion_ts = None, 0.0
        self.__measured_speed, self.__measure_ts = None, 0.0

        self.__first_lpf = LowPassFilter([0.0] * 4)
        self.__last_lpf = LowPassFilter([0.0] * 4)

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
        self.__scan = scan.get_points()
        self.__scan_ts = scan.get_timestamp()

    def set_motion(self, motion):
        self.__motion = motion
        self.__motion_ts = time.time()

    def set_speeds(self, front_left, front_right, rear_left, rear_right):
        fl, fr, rl, rr = front_left, front_right, rear_left, rear_right
        fl, fr, rl, rr = self.__drive_support(fl, fr, rl, rr)
        self.__roboclaw_driver.set_speeds(fl, fr, rl, rr)

    def get_measured_speeds(self):
        return self.__measured_speed

    def measure_loop(self):
        while self.__is_active:
            self.__measured_speed = self.__roboclaw_driver.get_measured_speeds()
            self.__measure_ts = time.time()
            time.sleep(0.1)

    def __drive_support(self, fl, fr, rl, rr):
        if self.__scan is None:
            return 0, 0, 0, 0

        """
        TODO(paoolo):
        * filter motion values
        * filter distances values
        """

        fl, fr, rl, rr = self.__first_lpf.compute([fl, fr, rl, rr])

        fl, fr, rl, rr = drive_support_logic.adjust_speed(fl, fr, rl, rr, self.__motion, self.__measured_speed)
        fl, fr, rl, rr = drive_support_logic.limit_speed(fl, fr, rl, rr, self.__scan)

        fl, fr, rl, rr = self.__last_lpf.compute([fl, fr, rl, rr])

        ts = time.time()
        trust_level = drive_support_logic.data_trust(self.__scan_ts, ts) * \
                      drive_support_logic.data_trust(self.__motion_ts, ts) * \
                      drive_support_logic.data_trust(self.__measure_ts, ts)

        fl *= trust_level
        rl *= trust_level
        fr *= trust_level
        rr *= trust_level

        return int(fl), int(fr), int(rl), int(rr)


class LowPassFilter(object):
    def __init__(self, values, alpha=0.3):
        self.__values = values
        self.__alpha = alpha

    def compute(self, values):
        values = map(lambda (prev, curr): (prev + self.__alpha * (curr - prev)), zip(self.__values, values))
        self.__values = values
        return values