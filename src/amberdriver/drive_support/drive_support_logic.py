import math
import time

import os

from amberdriver.tools import config


__author__ = 'paoolo'

pwd = os.path.dirname(os.path.abspath(__file__))
config.add_config_ini('%s/drive_support.ini' % pwd)

ROBO_WIDTH = float(config.ROBO_WIDTH)
ROBO_MASS = float(config.ROBO_MASS)

MAX_SPEED = float(config.MAX_SPEED)
MAX_ROTATING_SPEED = float(config.MAX_ROTATING_SPEED)
SOFT_LIMIT = float(config.SOFT_LIMIT)
HARD_LIMIT = float(config.HARD_LIMIT)

SCANNER_DIST_OFFSET = float(config.SCANNER_DIST_OFFSET)
ANGLE_RANGE = float(config.ANGLE_RANGE)

DISTANCE_ALPHA = float(config.DISTANCE_ALPHA)
RODEO_SWAP_ALPHA = float(config.RODEO_SWAP_ALPHA)

""" Other """


def average(*values):
    return (float(sum(values)) / float(len(values))) if len(values) > 0 else 0.0


""" Angles function, conversion, etc.  """


def get_angle(left, right, robo_width):
    return math.atan2(left - right, float(robo_width))


def normalize_angle(angle):
    if angle < -math.pi:
        angle += 2 * math.pi
    elif angle > math.pi:
        angle -= 2 * math.pi
    return angle


""" Trust data function """


def data_trust(data_ts, curr_ts, exp_base=4.0 / 3.0):
    val = data_ts - curr_ts
    return math.pow(exp_base, val)


def location_trust(location):
    _, _, location_probability, _, location_timestamp = location
    current_timestamp = time.time()
    return location_probability * data_trust(location_timestamp / 1000.0, current_timestamp)


""" Data polar/grid functions, conversion, etc. """


def convert_grid_to_polar(x, y):
    angle = math.atan2(y, x)
    value = math.sqrt(x ** 2 + y ** 2)
    return angle, value


def convert_polar_to_grid(value, angle):
    x = value * math.cos(angle)
    y = value * math.cos(angle)
    return x, y


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


""" Speed function, limitation, etc. """


def get_speed(left, right):
    return (left + right) / 2.0


def get_radius(left, right):
    return (right * ROBO_WIDTH) / (left - right) + ROBO_WIDTH / 2.0


def get_centripetal_force(velocity, radius):
    return (ROBO_MASS * velocity ** 2) / radius


def get_hard_limit(distance, soft_limit, hard_limit, max_speed):
    return max_speed / (soft_limit - hard_limit) * float(distance) - \
           (max_speed * hard_limit) / (soft_limit - hard_limit)


def get_soft_limit(current_speed, max_speed, soft_limit, hard_limit, alpha):
    return alpha * soft_limit * (current_speed / max_speed) + hard_limit + 50.0


def distance_to_speed(distance, angle, angle_offset=0.0,
                      distance_function=lambda d: d,
                      angle_function=lambda a: a):
    return angle_function(angle) * distance_function(distance)


def compute_speed_limits(map_polar,
                         distance_function=distance_to_speed):
    speed_limits = []
    for angle, distance in map_polar:
        distance = distance_function(distance, angle)
        speed_limits.append((angle, distance))
    return speed_limits


def distance_to_force(distance, angle, angle_offset=0.0,
                      distance_function=lambda d: d,
                      angle_function=lambda a: a):
    return angle_function(angle) * distance_function(distance)


def compute_environmental_forces(map_polar,
                                 distance_to_force_function=distance_to_force):
    forces_polar = []
    for angle, distance in map_polar:
        force = distance_to_force_function(distance, angle)
        forces_polar.append((angle, force))
    return forces_polar


""" Additional filter, etc. """


class LowPassFilter(object):
    def __init__(self, alpha, *args):
        self.__values = args
        self.__alpha = alpha

    def __call__(self, *args):
        self.__values = map(lambda (prev, curr): (prev + self.__alpha * (curr - prev)), zip(self.__values, args))
        return self.__values


class DelayingFilter(object):
    pass


class KalmanFilter(object):
    pass


class Accumulator(list):
    def __init__(self, values=list(), max_length=10):
        super(Accumulator, self).__init__(values)
        self.__max_length = max_length

    def append(self, x):
        super(Accumulator, self).append(x)
        if len(self) > self.__max_length:
            self.pop(0)

    def average_diff(self):
        return average(*self.diffs())

    def diffs(self):
        diffs = []
        if len(self) > 1:
            _prev = self[0]
            for _next in self[1:]:
                diffs.append(_next - _prev)
                _prev = _next
        return diffs