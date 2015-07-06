import math
import time

import os

from amberdriver.tools import config, logic

__author__ = 'paoolo'

pwd = os.path.dirname(os.path.abspath(__file__))
config.add_config_ini('%s/drive_to_point.ini' % pwd)

ROBO_WIDTH = float(config.ROBO_WIDTH)

""" Data polar/grid functions, conversion, etc. """


def convert_map_grid_to_polar(map_grid):
    map_polar = []
    for x, y in map_grid:
        angle, distance = logic.convert_grid_to_polar(x, y)
        map_polar.append((angle, distance))
    return map_polar


def convert_map_polar_to_grid(map_polar):
    map_grid = []
    for angle, distance in map_polar:
        x, y = logic.convert_polar_to_grid(distance, angle)
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
        if x is None or y is None or probability is None or angle is None:
            return
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

    def get_absolute_location(self):
        return Location(self.__absolute_x, self.__absolute_y, self.__absolute_angle, self.__last_update_ts)

    def get_last_update_timestamp(self):
        return self.__last_update_ts

    def __call__(self, speeds):
        self.calculate_relative_location(speeds.speed_left, speeds.speed_right)
        return self.get_location()


class Mapper(object):
    def __init__(self):
        self.data_grid = {}

    def update_scan(self, scan, location):
        current_timestamp = time.time()
        for angle, distance in scan.points:
            angle = angle + location.angle
            x, y = logic.convert_polar_to_grid(distance, angle)

            x, y = x + location.x, y + location.y
            x, y = logic.dround(x), logic.dround(y)

            if x not in self.data_grid:
                self.data_grid[x] = {}
            if y not in self.data_grid:
                self.data_grid[x][y] = logic.Object()

            self.data_grid[x][y].scan_update_ts = current_timestamp

    def update_location(self, location):
        current_timestamp = time.time()

        x = logic.dround(location.x)
        y = logic.dround(location.y)

        self.__update_location(x, y, current_timestamp)

    def __update_location(self, x, y, current_timestamp, steps=None):
        if steps is not None and steps == 0:
            return

        if x not in self.data_grid:
            self.data_grid[x] = {}

        if y not in self.data_grid[x]:
            self.data_grid[x][y] = logic.Object()

        self.data_grid[x][y].location_update_ts = current_timestamp
        if hasattr(self.data_grid[x][y], 'location_update_weight'):
            self.data_grid[x][y].location_update_weight += 1
            if steps is None:
                steps = self.data_grid[x][y].location_update_weight
            self.__update_location(x + 0.25, y, current_timestamp, steps - 1)
            self.__update_location(x - 0.25, y, current_timestamp, steps - 1)
            self.__update_location(x, y + 0.25, current_timestamp, steps - 1)
            self.__update_location(x, y - 0.25, current_timestamp, steps - 1)

        else:
            self.data_grid[x][y].location_update_weight = 1

    def get_scan(self, location):
        scan = logic.Object()
        scan.points = []

        angle_start = location.angle - 2.0943951023931953
        angle_stop = location.angle + 2.0943951023931953
        angle_step = 0.35208516886930985

        for angle in logic.drange(angle_start, angle_stop, angle_step):
            added = False
            for distance in logic.drange(0, 3000, 10):
                x = logic.dround(distance * math.cos(angle) + location.x)
                y = logic.dround(distance * math.sin(angle) + location.y)

                if x in self.data_grid and y in self.data_grid[x]:
                    scan.points.append((angle, distance))
                    added = True
                    break
            if not added:
                scan.points.append((angle, 0))

        return scan

    def flush(self, offset=0.5):
        pass


""" Objects class """


class Location(logic.Value):
    def __init__(self, x, y, angle, timestamp):
        logic.Value.__init__(self, timestamp)
        self.x, self.y, self.angle = x, y, angle

    def __str__(self):
        return 'location: x: %f, y: %f, angle: %f' % (self.x, self.y, self.angle)
