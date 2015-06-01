import math
import time
import collections

import os

from amberdriver.tools import config
from amberdriver.tools.logic import Value, LowPassFilter
from amberdriver.tools import logic


__author__ = 'paoolo'

pwd = os.path.dirname(os.path.abspath(__file__))
config.add_config_ini('%s/drive_support.ini' % pwd)

ROBO_WIDTH = float(config.ROBO_WIDTH)
ROBO_MASS = float(config.ROBO_MASS)

MAX_SPEED = float(config.MAX_SPEED)

SOFT_LIMIT = float(config.SOFT_LIMIT)
HARD_LIMIT = float(config.HARD_LIMIT)

DISTANCE_ALPHA = float(config.DISTANCE_ALPHA)

ANGLE_RANGE = float(config.ANGLE_RANGE)
SCANNER_DIST_OFFSET = float(config.SCANNER_DIST_OFFSET)

MAX_ACCELERATION_FORWARD = float(config.MAX_ACCELERATION_FORWARD)
MAX_ACCELERATION_SIDE = float(config.MAX_ACCELERATION_SIDE)

""" Other """


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


""" Speed function, limitation, etc. """


def get_speed(left, right):
    return (left + right) / 2.0


def get_radius(left, right):
    if abs(left - right) > 0:
        return (right * ROBO_WIDTH) / (left - right) + ROBO_WIDTH / 2.0
    else:
        return None


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


def accel(A, B):
    if abs(B.timestamp - A.timestamp) > 0.0:
        return (B.linear_speed - A.linear_speed) / (B.linear_speed - A.timestamp)
    return 0.0


""" Data analyzer """


class SpeedsAnalyzer(object):
    def __init__(self):
        self.__front_left = LowPassFilter(0.9, 0.0)
        self.__front_right = LowPassFilter(0.9, 0.0)
        self.__rear_left = LowPassFilter(0.9, 0.0)
        self.__rear_right = LowPassFilter(0.9, 0.0)

    @staticmethod
    def get_speeds_data(speeds):
        # unit is mm/s
        front_left, front_right, rear_left, rear_right = speeds
        return Speeds(front_left, front_right, rear_left, rear_right)

    def filter_speeds(self, speeds):
        if 0 <= speeds.speed_front_left < 5000:
            speeds.speed_front_left = self.__front_left(speeds.speed_front_left)
        if 0 <= speeds.speed_front_right < 5000:
            speeds.speed_front_right = self.__front_right(speeds.speed_front_right)
        if 0 <= speeds.speed_rear_left < 5000:
            speeds.speed_rear_left = self.__rear_left(speeds.speed_rear_left)
        if 0 <= speeds.speed_rear_right < 5000:
            speeds.speed_rear_right = self.__rear_right(speeds.speed_rear_right)

    @staticmethod
    def compute_linear_speeds(speeds):
        speeds.speed_left = average(speeds.speed_front_left, speeds.speed_rear_left)
        speeds.speed_right = average(speeds.speed_front_right, speeds.speed_rear_right)
        speeds.speed_front = average(speeds.speed_front_left, speeds.speed_front_right)
        speeds.speed_rear = average(speeds.speed_rear_left, speeds.speed_rear_right)
        speed_left_right = average(speeds.speed_left, speeds.speed_right)
        speed_front_rear = average(speeds.speed_front, speeds.speed_rear)
        speeds.linear_speed = average(speed_left_right, speed_front_rear)

    @staticmethod
    def compute_rotational_speed(speeds):
        if abs(speeds.speed_left - speeds.speed_right) > 0.0:
            speeds.radius = speeds.speed_right * ROBO_WIDTH / \
                            (speeds.speed_left - speeds.speed_right) + (ROBO_WIDTH / 2.0)
            speeds.rotational_speed = speeds.linear_speed / speeds.radius
        else:
            speeds.radius = 0.0
            speeds.rotational_speed = 0.0

    def __call__(self, speeds):
        speeds = self.get_speeds_data(speeds)
        self.filter_speeds(speeds)
        self.compute_linear_speeds(speeds)
        self.compute_rotational_speed(speeds)
        return speeds


class MotionAnalyzer(object):
    def __init__(self):
        self.__gravity_alpha = 0.8
        self.__gravity_forward, self.__gravity_side = 0.0, 0.0

        self.__acceleration_forward_filter = LowPassFilter(0.3, 0.0)
        self.__acceleration_side_filter = LowPassFilter(0.3, 0.0)
        self.__speed_rotational_filter = LowPassFilter(0.7, 0.0)

    @staticmethod
    def get_motion_data(motion):
        accel = motion.get_accel()
        gyro = motion.get_gyro()

        # unit is cm/s2
        acceleration_forward, acceleration_side, = accel.y_axis / 10.0, accel.x_axis / 10.0
        # unit is rad/s
        speed_rotational = math.radians(gyro.z_axis)

        return Motion(acceleration_forward, acceleration_side, speed_rotational)

    def compute_motion(self, motion):
        self.__gravity_forward = self.__gravity_alpha * self.__gravity_forward + \
                                 (1 - self.__gravity_alpha) * motion.acceleration_forward
        motion.acceleration_forward -= self.__gravity_forward

        self.__gravity_side = self.__gravity_alpha * self.__gravity_side + \
                              (1 - self.__gravity_alpha) * motion.acceleration_side
        motion.acceleration_side -= self.__gravity_side

    def filter_motion(self, motion):
        motion.acceleration_forward = self.__acceleration_forward_filter(motion.acceleration_forward)
        motion.acceleration_side = self.__acceleration_side_filter(motion.acceleration_side)
        motion.speed_rotational = self.__speed_rotational_filter(motion.speed_rotational)

    @staticmethod
    def compute_speeds(motion):
        if abs(motion.speed_rotational) > 0.0:
            speed_linear = motion.acceleration_side / motion.speed_rotational
            radius = speed_linear / motion.speed_rotational
            motion.speed_linear = speed_linear
            motion.radius = radius
        else:
            motion.speed_linear = 0.0
            motion.radius = 0.0

    def __call__(self, motion):
        motion = self.get_motion_data(motion)
        self.compute_motion(motion)
        self.filter_motion(motion)
        self.compute_speeds(motion)
        return motion


""" Mechanism """


def get_max_speed(distance, soft_limit, hard_limit, max_speed):
    return max_speed / (soft_limit - hard_limit) * float(distance) - \
           (max_speed * hard_limit) / (soft_limit - hard_limit)


def calculate_new_left_right(left, right, max_speed, current_speed):
    if current_speed > 0:
        divide = max_speed / current_speed
        return left * divide, right * divide
    else:
        return left, right


def get_min_distance(scan, current_angle, scanner_dist_offset, angle_range):
    min_distance = None
    min_distance_angle = None

    for angle, distance in scan.points:
        if distance > scanner_dist_offset \
                and current_angle - angle_range < angle < current_angle + angle_range:
            if min_distance is None or distance < min_distance:
                min_distance = distance
                min_distance_angle = angle

    return min_distance, min_distance_angle


def sign(value):
    return -1 if value < 0.0 else 1


def compute_circle(radius, stop_angle):
    if radius <= 0.0:
        start_angle = 0.0
    else:
        start_angle = math.pi
        stop_angle = math.pi - stop_angle
    sign_radius = sign(radius)
    circle = []
    for angle in logic.drange(start_angle, stop_angle, -sign_radius * 0.017453292519943295):
        point = logic.convert_polar_to_grid(radius, angle)
        circle.append(point)
    circle = map(lambda (x, y): (x + radius, y), circle)
    circle = map(lambda (x, y): logic.convert_grid_to_polar(x, y), circle)
    return circle


def compute_acceleration(last_speeds):
    acceleration = 0.0
    if len(last_speeds) > 1:
        all_speeds_iter = iter(last_speeds)
        try:
            prev_speeds = all_speeds_iter.next()
            while True:
                speeds = all_speeds_iter.next()
                acceleration += (
                    (speeds.speed_linear - prev_speeds.speed_linear) / (speeds.timestamp - speeds.timestamp))
        except StopIteration:
            pass
        return acceleration / (len(acceleration) - 1.0)
    return 0.0


class Limiter(object):
    def __init__(self):
        self.__speeds_filter = LowPassFilter(0.4, 0.0, 0.0, 0.0, 0.0)

        self.__scan = None
        self.__motion = None
        self.__measured_speeds = None

        self.__distance_factor = 0.0
        self.__acceleration_factor, self.__rotational_factor = 0.0, 0.0
        self.__speed_factor = 0.0

        self.last_speeds = collections.deque(maxlen=20)

    @staticmethod
    def compute_factor_due_to_distance(scan):
        factor_cosines = 0.0
        factor_gauss = 0.0
        weights_cosines = 0.0
        weights_gauss = 0.0
        for angle, distance in scan.points:
            w_c = math.cos(3.0 / 4.0 * angle)
            w_g = math.exp(-math.pow(angle, 2.0))
            weights_cosines += w_c
            weights_gauss += w_g
            if 10.0 < distance < 1200.0:
                factor_cosines += w_c * (distance / 1200.0)
                factor_gauss += w_g * (distance / 1200.0)
            else:
                factor_cosines += w_c
                factor_gauss += w_g
        factor_cosines = factor_cosines / weights_cosines
        factor_gauss = factor_gauss / weights_gauss
        return 0.4 * factor_cosines + 0.6 * factor_gauss

    @staticmethod
    def compute_robot_trajectory(speeds):
        circle = compute_circle(speeds.radius, speeds.speed_linear / speeds.radius * 10.0)
        circle = sorted(circle, key=lambda (a, _): -sign(speeds.radius) * a)
        return circle

    @staticmethod
    def order_scan(speeds, scan):
        sorted(scan.points, key=lambda (a, _): sign(speeds.radius) * a)

    @staticmethod
    def analyze_scan_rotational(speeds, scan, robot_trajectory):
        speeds.radius_limited_by_scan = speeds.radius

        scan_iterator = iter(scan.points)
        robot_trajectory_iterator = iter(robot_trajectory)

        (scan_angle, prev_scan_distance) = scan_iterator.next()
        (robot_trajectory_angle, robot_trajectory_distance) = robot_trajectory_iterator.next()
        scan_distance = prev_scan_distance

        try:
            while True:
                if abs(robot_trajectory_angle) < abs(scan_angle):
                    (scan_angle, new_scan_distance) = scan_iterator.next()
                    scan_distance = average(prev_scan_distance, new_scan_distance)
                    prev_scan_distance = new_scan_distance

                else:
                    if scan_distance < robot_trajectory_distance * 0.8:
                        radius = (robot_trajectory_distance * 0.8) / (2 * math.sin(robot_trajectory_angle))
                        if speeds.radius_limited_by_scan > radius:
                            speeds.radius_limited_by_scan = radius
                    (robot_trajectory_angle, robot_trajectory_distance) = robot_trajectory_iterator.next()

        except StopIteration:
            pass

        speeds.speed_limited_by_scan = (speeds.speed_linear * speeds.radius_limited_by_scan) / speeds.radius

    @staticmethod
    def analyze_scan_forward(speeds, scan):
        bond_angle = math.radians(20.0)
        min_distance_angle = -bond_angle
        min_distance = None

        for angle, distance in scan.points:
            if min_distance_angle < angle < bond_angle:
                if min_distance is None or min_distance > distance:
                    min_distance = distance
                    min_distance_angle = angle

        soft_limit = get_soft_limit(speeds.speed_linear,
                                    MAX_SPEED, SOFT_LIMIT * 1.3, HARD_LIMIT * 1.3, DISTANCE_ALPHA)
        limited_speed = get_max_speed(min_distance, soft_limit,
                                      HARD_LIMIT * 1.3, MAX_SPEED)

        speeds.radius_limited_by_scan = None
        speeds.speed_limited_by_scan = limited_speed

    def limit_speed_due_to_distance(self, speeds):
        speeds.distance_factor = self.__distance_factor

        scan = self.__scan
        if scan is not None:
            if speeds.radius is not None and speeds.radius != 0.0:
                robot_trajectory = self.compute_robot_trajectory(speeds)
                self.order_scan(speeds, scan)
                self.analyze_scan_rotational(speeds, scan, robot_trajectory)

            else:
                self.analyze_scan_forward(speeds, scan)

    @staticmethod
    def compute_factor_due_to_motion(motion):
        # value of forward acceleration could not be higher than 20 dm/s2
        # value of side acceleration could not be higher than 15 dm/s2
        acceleration_factor = (1 - abs(motion.acceleration_forward) / 20.0) * \
                              (1 - abs(motion.acceleration_side) / 15.0)
        # value of rotational speed could not be higher than 1.8 rad/s
        rotational_factor = (1 - abs(motion.speed_rotational) / 1.8)
        return acceleration_factor, rotational_factor

    @staticmethod
    def analyze_motion(speeds, motion, last_speeds):
        delta_time = last_speeds[-1].timestamp - last_speeds[-2].timestamp
        if speeds.acceleration_forward > MAX_ACCELERATION_FORWARD or \
                        motion.acceleration_forward > MAX_ACCELERATION_FORWARD:
            speeds.speed_limited_by_motion = MAX_ACCELERATION_FORWARD * delta_time + last_speeds[-2].speed

    def limit_speed_due_to_motion(self, speeds):
        speeds.acceleration_factor = self.__acceleration_factor
        speeds.rotational_factor = self.__rotational_factor

        motion = self.__motion
        if motion is not None:
            self.analyze_motion(speeds, motion, self.last_speeds)

    @staticmethod
    def compute_factor_due_to_speed(speeds):
        return 1.0

    def limit_speed_due_to_speed(self, speeds):
        pass

    def compute_other_values(self, speeds):
        self.last_speeds.append(speeds)
        speeds.speed_left = average(speeds.speed_front_left, speeds.speed_rear_left)
        speeds.speed_right = average(speeds.speed_front_right, speeds.speed_rear_right)
        speeds.speed_linear = average(speeds.speed_left, speeds.speed_right)
        speeds.radius = get_radius(speeds.speed_left, speeds.speed_right)
        speeds.acceleration_forward = compute_acceleration(self.last_speeds)
        if speeds.radius is not None:
            speeds.acceleration_side = math.pow(speeds.speed_linear, 2.0) / speeds.radius
        else:
            speeds.acceleration_side = None
        if speeds.radius is not None:
            speeds.speed_rotational = speeds.speed_linear / speeds.radius
        else:
            speeds.speed_rotational = None

    def detect_oscillation(self, speeds):
        pass

    def __call__(self, speeds):
        self.compute_other_values(speeds)
        self.detect_oscillation(speeds)
        self.limit_speed_due_to_distance(speeds)
        self.limit_speed_due_to_motion(speeds)
        self.limit_speed_due_to_speed(speeds)
        # apply changes to speeds
        factor = 0.0
        weight = 0.0
        if hasattr(speeds, 'rotational_factor'):
            # sys.stderr.write('rotational: %f\n' % speeds.rotational_factor)
            factor += 0.4 * speeds.rotational_factor
            weight += 0.4
        if hasattr(speeds, 'acceleration_factor'):
            # sys.stderr.write('acceleration: %f\n' % speeds.acceleration_factor)
            factor += 0.3 * speeds.acceleration_factor
            weight += 0.3
        if hasattr(speeds, 'distance_factor'):
            # sys.stderr.write('distance: %f\n' % speeds.distance_factor)
            factor += 1.3 * speeds.distance_factor
            weight += 1.3
        if weight > 0.0:
            factor /= weight
        else:
            factor = 1.0
        # sys.stderr.write('factor: %f\n' % factor)
        motion_factor = speeds.speed_limited_by_motion / speeds.speed_linear
        distance_factor = speeds.speed_limited_by_distance / speeds.speed_linear
        factor = factor * motion_factor * distance_factor
        speeds.speed_front_left *= factor
        speeds.speed_front_right *= factor
        speeds.speed_rear_left *= factor
        speeds.speed_rear_right *= factor

    def update_scan(self, scan):
        self.__scan = scan
        self.__distance_factor = self.compute_factor_due_to_distance(scan)

    def update_motion(self, motion):
        self.__motion = motion
        self.__acceleration_factor, self.__rotational_factor = self.compute_factor_due_to_motion(motion)

    def update_measured_speeds(self, measured_speeds):
        self.__measured_speeds = measured_speeds
        self.__speed_factor = self.compute_factor_due_to_speed(measured_speeds)


class Stabilizer(object):
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


""" Objects class """


class Speeds(Value):
    def __init__(self, front_left, front_right, rear_left, rear_right):
        Value.__init__(self)
        self.speed_front_left, self.speed_front_right = front_left, front_right
        self.speed_rear_left, self.speed_rear_right = rear_left, rear_right

    def __str__(self):
        return 'speed: front_left: %d, front_rear: %d, rear_left: %d, rear_right: %d' % \
               (self.speed_front_left, self.speed_front_right,
                self.speed_rear_left, self.speed_rear_right)


class Motion(Value):
    def __init__(self, acceleration_forward, acceleration_side, speed_rotational):
        Value.__init__(self)
        self.acceleration_forward = acceleration_forward
        self.acceleration_side = acceleration_side
        self.speed_rotational = speed_rotational

    def __str__(self):
        return 'motion: acceleration_forward: %f, acceleration_side: %f, speed_rotational: %f' % \
               (self.acceleration_forward, self.acceleration_side, self.speed_rotational)