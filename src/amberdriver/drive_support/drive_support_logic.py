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


class Values(object):
    def append(self, values):
        pass


def define_operation(operation):
    def operation_func(A, B):
        if type(A) == type(B) == tuple:
            return tuple(map(lambda (a, b): operation(a, b), zip(A, B)))
        elif type(A) == type(B) != tuple:
            return operation(A, B)
        elif type(A) != type(B) and type(A) == tuple:
            return tuple(map(lambda a: operation(a, B), A))
        elif type(A) != type(B) and type(B) == tuple:
            return tuple(map(lambda b: operation(A, b), B))
        else:
            raise TypeError()

    return operation_func


add = define_operation(lambda a, b: a + b)
subtract = define_operation(lambda a, b: a - b)
multiply = define_operation(lambda a, b: a * b)

divide = define_operation(lambda a, b: a / b)


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


def location_trust(location, exp_base=4.0 / 3.0):
    _, _, location_probability, _, location_timestamp = location
    current_timestamp = time.time()
    return location_probability * data_trust(location_timestamp / 1000.0, current_timestamp, exp_base)


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


class DelayFilter(object):
    def __init__(self):
        self.all_values = []

    def __call__(self, values, delay=0.5):
        current_timestamp = time.time()
        self.all_values.append((tuple(values), current_timestamp))
        filtered_values = filter(lambda (value, timestamp): timestamp < current_timestamp - delay, self.all_values)
        if len(filtered_values) > 0:
            for filtered_value in filtered_values:
                self.all_values.remove(filtered_value)
            return filtered_values[0][0]
        return None


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


class SpeedStabilizer(object):
    def __init__(self, interval=0.1):
        self.interval = interval
        self.user_speeds, self.user_speeds_timestamp = None, 0.0
        self.last_speeds, self.last_speeds_timestamp = None, None

    def set_speeds(self, speeds):
        if self.user_speeds is None:
            self.user_speeds = speeds
            self.user_speeds_timestamp = time.time()

    def run(self):
        while True:
            if self.user_speeds is not None:
                current_timestamp = time.time()
                if self.last_speeds is not None:
                    A = divide(subtract(self.user_speeds, self.last_speeds),
                               subtract(self.user_speeds_timestamp, self.last_speeds_timestamp))
                    B = subtract(self.user_speeds,
                                 multiply(A, self.user_speeds_timestamp))
                    current_speeds = add(multiply(A, current_timestamp), B)
                else:
                    current_speeds = self.user_speeds
                self.last_speeds = current_speeds
                self.last_speeds_timestamp = current_timestamp
                self.user_speeds = None
                # set current_speeds
            time.sleep(self.interval)


def round(value, granularity=0.25):
    new_value = int(value / granularity) * granularity
    if value - new_value >= granularity / 2.0:
        new_value += granularity
    return new_value


def accel(A, B):
    if abs(B.timestamp - A.timestamp) > 0.0:
        return (B.linear_speed - A.linear_speed) / (B.linear_speed - A.timestamp)
    return 0.0


class SpeedsAnalyzer(object):
    def __init__(self):
        self.__front_left = LowPassFilter(0.2, 0.0)
        self.__front_right = LowPassFilter(0.2, 0.0)
        self.__rear_left = LowPassFilter(0.2, 0.0)
        self.__rear_right = LowPassFilter(0.2, 0.0)

    def filter_speeds(self, speeds):
        speeds.front_left = self.__front_left(speeds.front_left)
        speeds.front_right = self.__front_right(speeds.front_right)
        speeds.rear_left = self.__front_left(speeds.rear_left)
        speeds.rear_right = self.__front_left(speeds.rear_right)

    @staticmethod
    def compute_speeds(speeds):
        speeds.left = average(speeds.front_left, speeds.rear_left)
        speeds.right = average(speeds.front_right, speeds.rear_right)
        speeds.front = average(speeds.front_left, speeds.front_right)
        speeds.rear = average(speeds.rear_left, speeds.rear_right)

    @staticmethod
    def compute_speed_radius(speeds):
        speed_left_right = average(speeds.left, speeds.right)
        speed_front_rear = average(speeds.front, speeds.rear)
        speeds.linear_speed = average(speed_left_right, speed_front_rear)
        if abs(speeds.left - speeds.right) > 0.0:
            speeds.radius = speeds.right * ROBO_WIDTH / (speeds.left - speeds.right) + (ROBO_WIDTH / 2.0)
            speeds.rotational_speed = speeds.linear_speed / speeds.radius
        else:
            speeds.radius = 0.0
            speeds.rotational_speed = 0.0

    def compute_accel(self, speeds):
        speeds.linear_accel = average(map(accel, zip(speeds, speeds[1:])))

    def set_speeds(self, speeds):
        self.filter_speeds(speeds)
        self.compute_speeds(speeds)
        self.compute_speed_radius(speeds)


class MotionAnalyzer(object):
    def __init__(self):
        self.__gravity_alpha = 0.8
        self.__gravity_forward, self.__gravity_side = 0.0, 0.0

        self.__motion = None
        self.__motion_filter = LowPassFilter(0.3, 0.0, 0.0, 0.0)

        self.__speed_computed_from_motion = 0.0
        self.__radius_computed_from_motion = 0.0

    @staticmethod
    def get_motion_data(motion):
        accel = motion.get_accel()
        gyro = motion.get_gyro()

        accel_forward, accel_side, = accel.y_axis / 100.0, accel.x_axis / 100.0
        speed_rotational = math.radians(gyro.z_axis)

        return accel_forward, accel_side, speed_rotational

    def compute_accel(self, accel_forward, accel_side):
        self.__gravity_forward = self.__gravity_alpha * self.__gravity_forward + \
                                 (1 - self.__gravity_alpha) * accel_forward
        accel_forward -= self.__gravity_forward

        self.__gravity_side = self.__gravity_alpha * self.__gravity_side + \
                              (1 - self.__gravity_alpha) * accel_side
        accel_side -= self.__gravity_side

        return accel_forward, accel_side

    @staticmethod
    def compute_speeds(motion):
        if motion.speed_rotational > 0.0 or motion.speed_rotational < 0.0:
            speed = motion.accel_side / motion.speed_rotational
            radius = speed / motion.speed_rotational
            return speed * 1000.0, radius * 1000.0
        return 0.0, 0.0

    def set_motion(self, motion):
        accel_forward, accel_side, speed_rotational = MotionAnalyzer.get_motion_data(motion)
        accel_forward, accel_side = self.compute_accel(accel_forward, accel_side)
        accel_forward, accel_side, speed_rotational = self.__motion_filter(accel_forward, accel_side, speed_rotational)
        motion = Motion(accel_forward, accel_side, speed_rotational)
        MotionAnalyzer.compute_speeds(motion)
        self.__motion = motion


class PowerAnalyzer(object):
    def __init__(self):
        self.__voltages, self.__currents = Values(), Values()

    def set_power(self, power):
        self.__voltages.append(power.voltages)
        self.__currents.append(power.currents)


class LocationAnalyzer(object):
    def __init__(self):
        self.__x, self.__y = Values(), Values()
        self.__angle = Values()

    def set_location(self, location):
        self.__x.append(location.x)
        self.__y.append(location.y)
        self.__angle.append(location.angle)


class ScanAnalyzer(object):
    def __init__(self):
        pass

    def set_scan(self, scan):
        pass


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


class Locator(object):
    def __init__(self):
        self.__time_stamp = 0.0
        self.__relative_x, self.__relative_y, self.__relative_angle = 0.0, 0.0, 0.0
        self.__absolute_x, self.__absolute_y, self.__absolute_angle = 0.0, 0.0, 0.0

    def __get_delta_timestamp(self):
        current_timestamp = time.time()
        delta_timestamp = current_timestamp - self.__time_stamp
        self.__time_stamp = current_timestamp
        return delta_timestamp

    def update_absolute_location(self, x, y, angle):
        self.__absolute_x, self.__absolute_y, self.__absolute_angle = x, y, angle

    def calculate_relative_location(self, speed_left, speed_right):
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
        return x, y, angle

    def get_location(self):
        # correlate data calculated and absolute
        return Location(self.__relative_x, self.__relative_y, self.__relative_angle)


class Driver(object):
    pass


class Value():
    def __init__(self):
        self.timestamp = time.time() * 1000.0


class Scan(Value):
    def __init__(self, points):
        Value.__init__(self)
        self.points = points


class Speed(Value):
    def __init__(self, front_left, front_right, rear_left, rear_right):
        Value.__init__(self)
        self.speed_front_left, self.speed_front_right = front_left, front_right
        self.speed_rear_left, self.speed_rear_right = rear_left, rear_right


class Motion(Value):
    def __init__(self, acc_forward, acc_side, speed_rotational):
        Value.__init__(self)
        self.accel_forward = acc_forward
        self.accel_side = acc_side
        self.speed_rotational = speed_rotational


class Current(Value):
    def __init__(self, front_left, front_right, rear_left, rear_right):
        Value.__init__(self)
        self.current_front_left, self.current_front_right = front_left, front_right
        self.current_rear_left, self.current_rear_right = rear_left, rear_right


class Voltage(Value):
    def __init__(self, front, rear):
        Value.__init__(self)
        self.voltage_front, self.voltage_rear = front, rear


class Location(Value):
    def __init__(self, x, y, angle):
        Value.__init__(self)
        self.x, self.y, self.angle = x, y, angle