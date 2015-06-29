import math
import time

import os

from amberdriver.tools import config
from amberdriver.tools.logic import Value, convert_grid_to_polar, convert_polar_to_grid

__author__ = 'paoolo'

pwd = os.path.dirname(os.path.abspath(__file__))
config.add_config_ini('%s/drive_to_point.ini' % pwd)

ROBO_WIDTH = float(config.ROBO_WIDTH)

""" Data polar/grid functions, conversion, etc. """


def convert_speed_grid_to_polar(velocity_x, velocity_y):
    return convert_grid_to_polar(velocity_x, velocity_y)


def convert_speed_polar_to_grid(velocity, angle):
    return convert_polar_to_grid(velocity, angle)


def convert_map_grid_to_polar(map_grid):
    map_polar = []
    for x, y in map_grid:
        angle, distance = convert_grid_to_polar(x, y)
        map_polar.append((angle, distance))
    return map_polar


def convert_map_polar_to_grid(map_polar):
    map_grid = []
    for angle, distance in map_polar:
        x, y = convert_polar_to_grid(distance, angle)
        map_grid.append((x, y))
    return map_grid


def normalize_angle(angle):
    if angle < -math.pi:
        angle += 2 * math.pi
    elif angle > math.pi:
        angle -= 2 * math.pi
    return angle


""" Data analyzer """


class LocationAnalyzer(object):
    def __init__(self):
        pass

    @staticmethod
    def get_location_data(location):
        x, y, probability, angle, _ = location.get_location()
        return Location(x, y, angle, time.time())

    def __call__(self, location):
        location = self.get_location_data(location)


""" Mechanism """


class Locator(object):
    def __init__(self):
        self.__timestamp, self.__last_update_ts = 0.0, 0.0
        self.__relative_x, self.__relative_y, self.__relative_angle = 0.0, 0.0, 0.0
        self.__absolute_x, self.__absolute_y, self.__absolute_angle = 0.0, 0.0, 0.0
        self.__absolute_probability = 0.0

    def __get_delta_timestamp(self):
        current_timestamp = time.time()
        delta_timestamp = current_timestamp - self.__timestamp
        self.__timestamp = current_timestamp
        return delta_timestamp

    def update_absolute_location(self, location):
        self.__last_update_ts = time.time()
        x, y, probability, angle, _ = location
        angle = normalize_angle(angle)
        self.__absolute_x, self.__absolute_y, self.__absolute_angle = x, y, angle
        self.__absolute_probability = probability
        if self.__absolute_probability > 0.9:
            self.__relative_x, self.__relative_y, self.__relative_angle = x, y, angle

    def calculate_relative_location(self, speed_left, speed_right):
        self.__last_update_ts = time.time()
        delta_timestamp = self.__get_delta_timestamp()

        if speed_right == speed_left:
            x = self.__relative_x + speed_left * delta_timestamp * math.cos(self.__relative_angle)
            y = self.__relative_y + speed_right * delta_timestamp * math.sin(self.__relative_angle)

            angle = self.__relative_angle

        else:
            a = 0.5 * ROBO_WIDTH * (speed_right + speed_left) / (speed_right - speed_left)
            angle = self.__relative_angle + (speed_right - speed_left) / ROBO_WIDTH * delta_timestamp

            x = self.__relative_x + a * (math.sin(angle) - math.sin(self.__relative_angle))
            y = self.__relative_y - a * (math.cos(angle) - math.cos(self.__relative_angle))

            angle = normalize_angle(angle)

        self.__relative_x, self.__relative_y, self.__relative_angle = x, y, angle

    def get_location(self):
        # correlate data calculated and absolute
        if self.__absolute_probability > 0.8:
            return Location(self.__absolute_x, self.__absolute_y, self.__absolute_angle, self.__last_update_ts)
        elif self.__absolute_probability < 0.3:
            return Location(self.__relative_x, self.__relative_y, self.__relative_angle, self.__last_update_ts)
        else:
            probability = self.__absolute_probability
            x = (1.0 - probability) * self.__relative_x + probability * self.__absolute_x
            y = (1.0 - probability) * self.__relative_y + probability * self.__absolute_y
            angle = (1.0 - probability) * self.__relative_angle + probability * self.__absolute_angle
            return Location(x, y, angle, self.__last_update_ts)

    def __call__(self, speeds):
        self.calculate_relative_location(speeds.speed_left, speeds.speed_right)
        return self.get_location()


class Mapper(object):
    def __init__(self):
        self.data_grid = {}

    def add_polar(self, polar, location):
        current_timestamp = time.time()
        for angle, distance in polar:
            angle = angle + location.angle
            x, y = convert_polar_to_grid(distance, angle)
            x, y = x + location.x, y + location.y
            x, y = round(x), round(y)
            if x not in self.data_grid:
                self.data_grid[x] = {}
            self.data_grid[x][y] = current_timestamp

    def flush(self, offset=0.5):
        current_timestamp = time.time()
        to_remove = []
        for x in self.data_grid:
            for y in self.data_grid[x]:
                if self.data_grid[x][y] < current_timestamp - offset:
                    to_remove.append((x, y))
        for x, y in to_remove:
            del self.data_grid[x][y]


class ObstacleMapper(object):
    def __init__(self):
        self.__scan, self.__location = None, None
        self.__data_grid = {}

    def update_scan(self, scan):
        self.__scan = scan
        self.__update_map()

    def update_location(self, location):
        self.__location = location
        self.__update_map()

    def __update_map(self):
        scan = self.__scan
        location = self.__location
        if scan is not None and location is not None:
            scan.points = sorted(scan.points, key=lambda (a, _): a)
            # transform scan


""" Objects class """


class Location(Value):
    def __init__(self, x, y, angle, timestamp):
        Value.__init__(self, timestamp)
        self.x, self.y, self.angle = x, y, angle

    def __str__(self):
        return 'location: x: %f, y: %f, angle: %f' % (self.x, self.y, self.angle)
