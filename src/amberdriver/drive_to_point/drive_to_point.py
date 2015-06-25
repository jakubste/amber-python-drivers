import logging
import logging.config
import threading
import time
import math

import traceback
import os

from ambercommon.common import runtime

from amberclient.common.listener import Listener

from amberdriver.drive_to_point import drive_to_point_logic
from amberdriver.tools import logic, config, bound_sleep_interval
from amberdriver.tools.logic import sign

__author__ = 'paoolo'

pwd = os.path.dirname(os.path.abspath(__file__))
logging.config.fileConfig('%s/drive_to_point.ini' % pwd)
config.add_config_ini('%s/drive_to_point.ini' % pwd)

LOGGER_NAME = 'DriveToPoint'

MAX_SPEED = float(config.DRIVE_TO_POINT_MAX_SPEED)


def compute_sleep_interval(current_timestamp, last_timestamp, sleep_interval,
                           max_interval=2.0, alpha=0.5):
    """
    compute_sleep_interval(current timestamp in ms, last timestamp in ms, actual sleep interval) -> new sleep interval
    """
    timestamp_interval = current_timestamp - last_timestamp
    timestamp_interval /= 1000.0
    if timestamp_interval < max_interval:
        sleep_interval += alpha * (timestamp_interval - sleep_interval)
        sleep_interval = bound_sleep_interval(sleep_interval)
    return sleep_interval


class ScanHandler(Listener):
    def __init__(self, driver):
        self.__drive_support = driver

    def handle(self, response):
        self.__drive_support.set_scan(response)


class DriveToPoint(object):
    TIMESTAMP_FIELD = 4

    def __init__(self, driver_proxy, location_proxy, hokuyo_proxy):
        self.__driver_proxy = driver_proxy
        self.__location_proxy = location_proxy
        self.__hokuyo_proxy = hokuyo_proxy

        self.__hokuyo_listener = ScanHandler(self)
        hokuyo_proxy.subscribe(self.__hokuyo_listener)
        self.__scan = None

        self.__next_targets, self.__visited_targets = [], []
        self.__current_location, self.__next_targets_timestamp = None, 0.0
        self.__targets_lock = threading.Condition()

        self.__is_active = True
        self.__driving_allowed = False

        self.__locator = drive_to_point_logic.Locator()

        self.__speeds_filter = logic.LowPassFilter(0.5, 0.0, 0.0)
        self.__logger = logging.getLogger(LOGGER_NAME)

        runtime.add_shutdown_hook(self.stop)

    def set_scan(self, scan):
        self.__scan = scan

    def stop(self):
        self.__is_active = False

    def set_targets(self, targets):
        self.__targets_lock.acquire()
        try:
            self.__next_targets = targets
            self.__next_targets_timestamp = time.time()
            self.__driving_allowed = len(targets) > 0
            self.__visited_targets = []
        finally:
            self.__targets_lock.notify_all()
            self.__targets_lock.release()

    def get_next_targets_and_location(self):
        self.__targets_lock.acquire()
        try:
            return self.__next_targets[:], self.__current_location
        finally:
            self.__targets_lock.release()

    def get_next_target_and_location(self):
        self.__targets_lock.acquire()
        try:
            _next_target = self.__next_targets[0] if len(self.__next_targets) > 0 else (0, 0, 0)
            return _next_target, self.__current_location
        finally:
            self.__targets_lock.release()

    def get_visited_targets_and_location(self):
        self.__targets_lock.acquire()
        try:
            return self.__visited_targets[:], self.__current_location
        finally:
            self.__targets_lock.release()

    def get_visited_target_and_location(self):
        self.__targets_lock.acquire()
        try:
            _visited_target = self.__visited_targets[-1] if len(self.__visited_targets) > 0 else (0, 0, 0)
            return _visited_target, self.__current_location
        finally:
            self.__targets_lock.release()

    def location_loop(self):
        sleep_interval = 0.5

        last_location = self.__location_proxy.get_location()
        last_location = last_location.get_location()
        self.__current_location = last_location

        time.sleep(sleep_interval)
        while self.__is_active:
            current_location = self.__location_proxy.get_location()
            current_location = current_location.get_location()
            self.__current_location = current_location
            self.__locator.update_absolute_location(self.__current_location)

            try:
                sleep_interval = compute_sleep_interval(current_location[DriveToPoint.TIMESTAMP_FIELD],
                                                        last_location[DriveToPoint.TIMESTAMP_FIELD],
                                                        sleep_interval)
            except TypeError:
                traceback.print_exc()

            last_location = current_location
            time.sleep(sleep_interval)

    def measuring_loop(self):
        while self.__is_active:
            motors_speed = self.__driver_proxy.get_current_motors_speed()
            speed_left = logic.average(motors_speed.get_front_left_speed(),
                                       motors_speed.get_rear_left_speed())
            speed_right = logic.average(motors_speed.get_front_right_speed(),
                                        motors_speed.get_rear_right_speed())
            self.__locator.calculate_relative_location(speed_left, speed_right)
            time.sleep(0.09)

    def driving_loop(self):
        driving = False
        while self.__is_active:
            target = self.__get_next_target()
            while self.__is_active and target is not None:
                driving = True
                self.__drive_to(target, self.__next_targets_timestamp)
                self.__add_target_to_visited(target)
                target = self.__get_next_target()

            if driving:
                self.__logger.warning('Next targets list is empty, stop driving.')
                self.__stop()
                driving = False

            time.sleep(0.1)

    def __get_next_target(self):
        self.__targets_lock.acquire()
        try:
            return self.__next_targets[0]
        except IndexError:
            return None
        finally:
            self.__targets_lock.release()

    def __drive_to(self, target, next_targets_timestamp):
        self.__logger.info('Drive to %s', str(target))

        location = self.__locator.get_location()
        while location is None:
            time.sleep(0.2)
            location = self.__locator.get_location()

        while not DriveToPoint.target_reached(location, target) and self.__driving_allowed and self.__is_active \
                and not self.__next_targets_timestamp > next_targets_timestamp:
            left, right = self.compute_speed(location, target)

            """
            TODO(paoolo): do nice stuff with avoiding obstacles
            * map scan histogram to cartesian grid
            * update temporary map (remove old/low priority)
            * update location (position/angle)
            * detect obstacles from scan histogram
            * determine position of destination
            * determine distance to destination
            * analyze temporary map to find path to destination
            * analyze probability of local minimum
            * drive to (temporary) destination
            """

            left, right = int(left), int(right)
            self.__driver_proxy.send_motors_command(left, right, left, right)

            time.sleep(0.2)
            location = self.__locator.get_location()

        self.__logger.info('Target %s reached', str(target))

    def __add_target_to_visited(self, target):
        self.__targets_lock.acquire()
        try:
            self.__next_targets.remove(target)
            self.__visited_targets.append(target)
        except ValueError:
            self.__logger.warning('Target %s is not in next targets list, not added to visited targets list.',
                                  str(target))
        finally:
            self.__targets_lock.release()

    def __stop(self):
        self.__driver_proxy.send_motors_command(0, 0, 0, 0)

    @staticmethod
    def target_reached(location, target):
        target_x, target_y, target_radius = target
        location_x, location_y, _, _, _ = location
        try:
            diff_x = location_x - target_x
            diff_y = location_y - target_y
            return math.pow(diff_x, 2) + math.pow(diff_y, 2) < math.pow(target_radius, 2)
        except TypeError:
            traceback.print_exc()
            return False

    def compute_speed(self, location, target):
        target_x, target_y, _ = target

        location_x, location_y, location_angle, = location.x, location.y, location.angle

        if location_x is None or location_y is None or location_angle is None:
            # sth wrong, stop!
            return 0.0, 0.0

        location_angle = drive_to_point_logic.normalize_angle(location_angle)

        diff_y = target_y - location_y
        diff_x = target_x - location_x
        target_angle = math.atan2(target_y - location_y, target_x - location_x)
        drive_angle = target_angle - location_angle
        drive_angle = drive_to_point_logic.normalize_angle(drive_angle)
        drive_angle = -drive_angle  # mirrored map
        drive_distance = math.sqrt(diff_y * diff_y + diff_x * diff_x) * 1000.0

        factor = -math.pow(0.997, drive_distance) + 1.0
        alpha = 0.17453292519943295 + factor * 0.3490658503988659
        beta = 1.0471975511965976 + factor * 0.5235987755982988

        if abs(drive_angle) < alpha:  # 10st
            # drive normal
            left, right = MAX_SPEED, MAX_SPEED
        elif abs(drive_angle) > beta:  # 60st
            # rotate in place
            left = -MAX_SPEED
            right = MAX_SPEED
            if drive_angle < 0:
                left, right = right, left
        else:
            # drive on turn
            left = MAX_SPEED - DriveToPoint.compute_change(drive_angle, math.pi / beta)
            right = MAX_SPEED + DriveToPoint.compute_change(drive_angle, math.pi / beta)

        _left, _right = self.__speeds_filter(abs(left), abs(right))
        left, right = sign(left) * _left, sign(right) * _right

        return left, right

    @staticmethod
    def compute_change(drive_angle, driving_alpha):
        return driving_alpha * drive_angle / math.pi * MAX_SPEED
