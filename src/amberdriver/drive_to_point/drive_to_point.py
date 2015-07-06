import logging
import logging.config
import threading
import time
import math

import traceback
import os

from ambercommon.common import runtime

from amberclient.common.listener import Listener

from amberdriver.drive_support import drive_support_logic
from amberdriver.drive_to_point import drive_to_point_logic
from amberdriver.tools import logic, config

__author__ = 'paoolo'

pwd = os.path.dirname(os.path.abspath(__file__))
logging.config.fileConfig('%s/drive_to_point.ini' % pwd)
config.add_config_ini('%s/drive_to_point.ini' % pwd)

LOGGER_NAME = 'DriveToPoint'

MAX_SPEED = float(config.DRIVE_TO_POINT_MAX_SPEED)
ROBO_WIDTH = float(config.ROBO_WIDTH)
MAXIMUM_TIME_DRIVE_TO = float(config.MAXIMUM_TIME_DRIVE_TO)


class ScanHandler(Listener):
    def __init__(self, driver):
        self.__drive_to_point = driver

    def handle(self, response):
        self.__drive_to_point.set_scan(response)


class DriveToPoint(object):
    TIMESTAMP_FIELD = 4

    def __init__(self, driver_proxy, location_proxy, hokuyo_proxy):
        self.__driver_proxy = driver_proxy
        self.__location_proxy = location_proxy
        self.__hokuyo_proxy = hokuyo_proxy

        self.__scan_analyzer = logic.ScanAnalyzer()

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
        self.__scan = self.__scan_analyzer(scan)

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
        while self.__is_active:
            time.sleep(0.09)
            current_location = self.__location_proxy.get_location()
            current_location = current_location.get_location()
            self.__current_location = current_location
            self.__locator.update_absolute_location(current_location)

    def measuring_loop(self):
        while self.__is_active:
            time.sleep(0.09)
            motors_speed = self.__driver_proxy.get_current_motors_speed()
            speed_left = logic.average(motors_speed.get_front_left_speed(),
                                       motors_speed.get_rear_left_speed())
            speed_right = logic.average(motors_speed.get_front_right_speed(),
                                        motors_speed.get_rear_right_speed())
            self.__locator.calculate_relative_location(speed_left, speed_right)

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

            time.sleep(0.09)

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

        coming_out_from_local_minimum = False
        temporary_target = None
        location = self.__locator.get_absolute_location()

        start_time = time.time()
        while not DriveToPoint.target_reached(location, target) and \
                self.__driving_allowed and self.__is_active and \
                not self.__next_targets_timestamp > next_targets_timestamp and \
                                start_time + MAXIMUM_TIME_DRIVE_TO > time.time():
            if coming_out_from_local_minimum:
                if temporary_target is not None and not DriveToPoint.target_reached(location, temporary_target):
                    drive_angle, drive_distance = DriveToPoint.__compute_drive_angle_distance(location,
                                                                                              temporary_target)
                    if drive_angle is not None and drive_distance is not None:
                        self.__send_commands(drive_angle, drive_distance)
                else:
                    coming_out_from_local_minimum = False
                    temporary_target = None
                    self.__logger.warn('Temporary target reached!')
                    self.__stop()
            else:
                drive_angle, drive_distance = DriveToPoint.__compute_drive_angle_distance(location, target)

                if drive_angle is not None and drive_distance is not None:
                    scan = self.__scan
                    scan_distance = drive_support_logic.get_distance(scan, drive_angle)

                    if drive_distance > scan_distance:
                        coming_out_from_local_minimum = True
                        temporary_target = DriveToPoint.__find_temporary_target(scan, scan_distance, drive_angle)
                        self.__logger.warn('Setup temporary target: %s', str(target))
                        self.__stop()
                    else:
                        self.__send_commands(drive_angle, drive_distance)

            time.sleep(0.07)
            location = self.__locator.get_absolute_location()

        if start_time + MAXIMUM_TIME_DRIVE_TO < time.time():
            self.__logger.warn('Target %s not reachable', str(target))
        else:
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
        location_x, location_y = location.x, location.y
        try:
            diff_x = location_x - target_x
            diff_y = location_y - target_y
            return math.pow(diff_x, 2) + math.pow(diff_y, 2) < math.pow(target_radius, 2)
        except TypeError:
            traceback.print_exc()
            return False

    @staticmethod
    def __find_temporary_target(scan, scan_distance, drive_angle):
        best_distance = scan_distance
        best_angle = drive_angle
        for angle, distance in sorted(scan.points, key=lambda (a, _): abs(a - drive_angle)):
            if distance > best_distance:
                best_distance = distance
                best_angle = angle
        x, y = logic.convert_polar_to_grid(best_distance - ROBO_WIDTH, best_angle)
        return x, y, ROBO_WIDTH * 0.7

    @staticmethod
    def __compute_drive_angle_distance(location, target):
        target_x, target_y, _ = target

        location_x, location_y, location_angle = location.x, location.y, location.angle
        location_angle = drive_to_point_logic.normalize_angle(location_angle)

        if location_x is None or location_y is None or location_angle is None:
            return None, None

        diff_y = target_y - location_y
        diff_x = target_x - location_x
        target_angle = math.atan2(target_y - location_y, target_x - location_x)
        drive_angle = target_angle - location_angle
        drive_angle = drive_to_point_logic.normalize_angle(drive_angle)
        drive_angle = -drive_angle  # mirrored map
        drive_distance = math.sqrt(diff_y * diff_y + diff_x * diff_x) * 1000.0

        return drive_angle, drive_distance

    def __compute_speed(self, drive_angle, drive_distance):
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
        left, right = logic.sign(left) * _left, logic.sign(right) * _right

        return left, right

    @staticmethod
    def compute_change(drive_angle, driving_alpha):
        return driving_alpha * drive_angle / math.pi * MAX_SPEED

    def __send_commands(self, drive_angle, drive_distance):
        left, right = self.__compute_speed(drive_angle, drive_distance)
        current_timestamp = time.time()
        scan_factor = logic.compute_data_trust(self.__scan.timestamp / 1000.0, current_timestamp)
        location_factor = logic.compute_data_trust(self.__locator.get_last_update_timestamp(), current_timestamp)
        left = left * scan_factor * location_factor
        right = right * scan_factor * location_factor
        left, right = int(left), int(right)
        self.__driver_proxy.send_motors_command(left, right, left, right)
