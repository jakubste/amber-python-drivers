import logging
import logging.config
import time

import os
from amberclient.common.listener import Listener

from ambercommon.common import runtime

from amberdriver.drive_support import drive_support_logic
from amberdriver.drive_support.drive_support_logic import LowPassFilter, Accumulator, average
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


class Scan(object):
    def __init__(self, points):
        self.points = points
        self.timestamp = time.time() * 1000.0

    def __iter__(self):
        return iter(self.points)


class Motion(object):
    def __init__(self, acc_forward, acc_side, speed_rotational):
        self.accel_forward = acc_forward
        self.accel_side = acc_side
        self.speed_rotational = speed_rotational
        self.timestamp = time.time() * 1000.0


class Power(object):
    def __init__(self, current, voltage):
        self.current = current
        self.voltage = voltage
        self.timestamp = time.time() * 1000.0


class Speed(object):
    def __init__(self, fl, fr, rl, rr):
        self.front_left, self.front_right = fl, fr
        self.rear_left, self.rear_right = rl, rr
        self.timestamp = time.time() * 1000.0

    def __iter__(self):
        return iter((self.front_left, self.front_right, self.rear_left, self.rear_right))


def get_motion_data(motion):
    accel = motion.get_accel()
    gyro = motion.get_gyro()

    # getting data from motion sensor specific for installation
    acc_forward, acc_side, = accel.x_axis, accel.y_axis
    speed_rotational = gyro.z_axis

    return acc_forward, acc_side, speed_rotational


class DriveSupport(object):
    def __init__(self, roboclaw_driver, hokuyo_proxy, ninedof_proxy):
        self.__scan = None
        self.__motion = None
        self.__speeds = None
        self.__power = None
        self.__user_speeds = None

        self.__motion_filter = LowPassFilter(0.3, 0.0, 0.0, 0.0)
        self.__speeds_filter = LowPassFilter(0.45, 0.0, 0.0, 0.0, 0.0)
        self.__power_filter = LowPassFilter(0.2, 0.0, 0.0)
        self.__user_speeds_filter = LowPassFilter(0.3, 0.0, 0.0, 0.0, 0.0)

        self.__current_accumulator = Accumulator()
        self.__voltage_accumulator = Accumulator()

        self.__speed_computed_from_motion = 0.0
        self.__radius_computed_from_motion = 0.0

        self.__speed_computed_from_measurement = 0.0
        self.__radius_computed_from_measurement = 0.0

        self.__speed_limits = None
        self.__environmental_forces = None
        self.__current_environment_map = None

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
        # Filter (?) and create object
        self.__scan = Scan(scan.get_points())

        self.__speed_limits = drive_support_logic.compute_speed_limits(scan)
        self.__environmental_forces = drive_support_logic.compute_environmental_forces(scan)
        self.__current_environment_map = drive_support_logic.convert_map_polar_to_grid(scan)

    def set_motion(self, motion):
        accel_forward, accel_side, speed_rotational = get_motion_data(motion)
        # Filter and create object
        accel_forward, accel_side, speed_rotational = self.__motion_filter(accel_forward, accel_side, speed_rotational)
        self.__motion = Motion(accel_forward, accel_side, speed_rotational)
        # Compute forward speed and turn radius
        speed = self.__motion.accel_side / self.__motion.speed_rotational
        radius = speed / self.__motion.speed_rotational
        self.__speed_computed_from_motion = speed
        self.__radius_computed_from_motion = radius

    def measure_loop(self):
        while self.__is_active:
            speeds = self.__roboclaw_driver.get_measured_speeds()
            # Filter and create object
            speeds = self.__speeds_filter(*speeds)
            self.__speeds = Speed(*speeds)

            current, voltage = self.__roboclaw_driver.get_current_and_voltage()
            # Filter and create object
            current, voltage = self.__power_filter(current, voltage)
            self.__power = Power(current, voltage)

            self.__current_accumulator.append(current)
            self.__voltage_accumulator.append(voltage)

            # detect changes on current and voltage

            # Compute forward speed and turn radius
            speed, radius = compute_speed_radius(speeds)
            self.__speed_computed_from_measurement = speed
            self.__radius_computed_from_measurement = radius

            time.sleep(0.1)

    def get_measured_speeds(self):
        return self.__speeds

    def set_speeds(self, front_left, front_right, rear_left, rear_right):
        current_timestamp = time.time()
        # Filter and create object
        speeds = front_left, front_right, rear_left, rear_right
        speeds = self.__user_speeds_filter(*speeds)
        speeds = Speed(*speeds)

        # detect if oscillation exists
        # reduce speed due to environment
        # filter data
        speeds = Speed(*self.__user_speeds(*speeds))
        speed, radius = compute_speed_radius(speeds)

        sd1 = self.__speed_computed_from_motion - speed
        sd2 = self.__speed_computed_from_measurement - speed
        rd1 = self.__radius_computed_from_motion - radius
        rd2 = self.__radius_computed_from_measurement - radius

        trust_level = drive_support_logic.data_trust(self.__scan.timestamp / 1000.0, current_timestamp) * \
                      drive_support_logic.data_trust(self.__motion.timestamp / 1000.0, current_timestamp) * \
                      drive_support_logic.data_trust(self.__speeds.timestamp / 1000.0, current_timestamp) * \
                      drive_support_logic.data_trust(self.__power.timestamp / 1000.0, current_timestamp)
        speeds = map(lambda val: int(val * trust_level), speeds)

        self.__roboclaw_driver.set_speeds(*speeds)


def compute_speed_radius(speeds):
    speed_left = average(speeds.front_left, speeds.rear_left)
    speed_right = average(speeds.front_right, speeds.rear_right)
    speed_front = average(speeds.front_left, speeds.front_right)
    speed_rear = average(speeds.rear_left, speeds.rear_right)

    speed_left_right = average(speed_left, speed_right)
    speed_front_rear = average(speed_front, speed_rear)

    speed = average(speed_left_right, speed_front_rear)
    radius = speed_left * ROBO_WIDTH / (speed_left - speed_right)

    return speed, radius


def compute_accel_side_speed_rotational(speed, radius):
    speed_rotational = speed / radius
    acc_side = speed_rotational * speed
    return acc_side, speed_rotational