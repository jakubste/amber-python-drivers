import math
import time

import os

from amberdriver.tools import config, logic

__author__ = 'paoolo'

pwd = os.path.dirname(os.path.abspath(__file__))
config.add_config_ini('%s/drive_to_point.ini' % pwd)

ROBO_WIDTH = float(config.ROBO_WIDTH)

""" Data polar/grid functions, conversion, etc. """


def normalize_angle(angle):
    if angle < -math.pi:
        angle += 2 * math.pi
    elif angle > math.pi:
        angle -= 2 * math.pi
    return angle


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


""" Objects class """


class Location(logic.Value):
    def __init__(self, x, y, angle, timestamp):
        logic.Value.__init__(self, timestamp)
        self.x, self.y, self.angle = x, y, angle

    def __str__(self):
        return 'location: x: %f, y: %f, angle: %f' % (self.x, self.y, self.angle)
