import math
import collections

import os

from amberdriver.tools import config
from amberdriver.tools.logic import Value, LowPassFilter, sign, average
from amberdriver.tools import logic

__author__ = 'paoolo'

pwd = os.path.dirname(os.path.abspath(__file__))
config.add_config_ini('%s/drive_support.ini' % pwd)

ROBO_WIDTH = float(config.ROBO_WIDTH)
ROBO_MASS = float(config.ROBO_MASS)

MAX_SPEED = float(config.MAX_SPEED)

SOFT_DISTANCE_LIMIT = float(config.SOFT_DISTANCE_LIMIT)
HARD_DISTANCE_LIMIT = float(config.HARD_DISTANCE_LIMIT)

DISTANCE_ALPHA = float(config.DISTANCE_ALPHA)

ANGLE_RANGE = float(config.ANGLE_RANGE)
SCANNER_DIST_OFFSET = float(config.SCANNER_DIST_OFFSET)

MAX_ACCELERATION_FORWARD = float(config.MAX_ACCELERATION_FORWARD)
MAX_ACCELERATION_SIDE = float(config.MAX_ACCELERATION_SIDE)
MAX_ROTATIONAL_SPEED = float(config.MAX_ROTATIONAL_SPEED)


def get_angle(left, right, robo_width):
    return math.atan2(left - right, float(robo_width))


def compute_max_speed(max_speed, distance, soft_limit, hard_limit):
    if 0.0 < soft_limit and 0.0 < hard_limit < soft_limit:
        return max_speed * (distance - hard_limit) / (soft_limit - hard_limit)
    return 0.0


def compute_circle(radius, stop_angle):
    if radius < 0.0:
        start_angle = 0.0
        stop_angle = stop_angle
        step = 0.017453292519943295
    else:
        start_angle = math.pi
        stop_angle = math.pi - stop_angle
        step = -0.017453292519943295

    circle = []
    for angle in logic.drange(start_angle, stop_angle, step):
        point = logic.convert_polar_to_grid(abs(radius), angle)
        circle.append(point)
    circle = map(lambda (x, y): (x + radius, y), circle)
    circle = map(lambda (x, y): logic.convert_grid_to_polar(x, y), circle)

    return circle


def compute_robot_trajectory(speeds):
    if speeds.radius is not None and abs(speeds.radius) > 0.0:
        robot_trajectory = compute_circle(speeds.radius, speeds.linear_speed / abs(speeds.radius) * 10.0)
        robot_trajectory = map(lambda (a, d): (-(a - math.pi / 2.0), d), robot_trajectory)
        return robot_trajectory
    return []


def compute_radius(left, right):
    if abs(left - right) > 0.0:
        return (right * ROBO_WIDTH) / (left - right) + ROBO_WIDTH / 2.0
    return None


def compute_acceleration(list_of_speeds):
    acceleration = 0.0
    if len(list_of_speeds) > 1:
        list_of_speeds_iter = iter(list_of_speeds)
        count = 0
        try:
            prev_speeds = list_of_speeds_iter.next()
            while True:
                speeds = list_of_speeds_iter.next()
                if abs(speeds.timestamp - prev_speeds.timestamp) > 0.0:
                    value = (speeds.linear_speed - prev_speeds.linear_speed) / (
                        speeds.timestamp - prev_speeds.timestamp)
                    acceleration += value
                    count += 1
        except StopIteration:
            pass
        if count > 0:
            return acceleration / float(count)
    return 0.0


class Speeds(Value):
    def __init__(self, speeds, last_speeds=None):
        Value.__init__(self)
        front_left, front_right, rear_left, rear_right = speeds

        self.speed_front_left, self.speed_front_right = front_left, front_right
        self.speed_rear_left, self.speed_rear_right = rear_left, rear_right

        self.speed_left, self.speed_right, self.speed_front, self.speed_rear = None, None, None, None
        self.linear_speed, self.rotational_speed = None, None
        self.radius, self.acceleration_forward, self.acceleration_side = None, None, None

        self.compute_other_speed(last_speeds)

    def compute_other_speed(self, last_speeds=None):
        self.speed_left = average(self.speed_front_left, self.speed_rear_left)
        self.speed_right = average(self.speed_front_right, self.speed_rear_right)
        self.speed_front = average(self.speed_front_left, self.speed_front_right)
        self.speed_rear = average(self.speed_rear_left, self.speed_rear_right)

        self.linear_speed = average(self.speed_left, self.speed_right,
                                    self.speed_front, self.speed_rear)

        self.radius = compute_radius(self.speed_left, self.speed_right)

        if last_speeds is not None:
            last_speeds.append(self)
            self.acceleration_forward = compute_acceleration(last_speeds)
        else:
            self.acceleration_forward = None

        if self.radius is not None and abs(self.radius) > 0.0:
            self.acceleration_side = math.pow(self.linear_speed, 2.0) / self.radius
        else:
            self.acceleration_side = None

        if self.radius is not None and abs(self.radius) > 0.0:
            self.rotational_speed = self.linear_speed / self.radius
        else:
            self.rotational_speed = None

    def __str__(self):
        return 'speed: front_left: %d, front_rear: %d, rear_left: %d, rear_right: %d' % \
               (self.speed_front_left, self.speed_front_right,
                self.speed_rear_left, self.speed_rear_right)


class Motion(Value):
    def __init__(self, acceleration_forward, acceleration_side, rotational_speed):
        Value.__init__(self)
        self.acceleration_forward = acceleration_forward
        self.acceleration_side = acceleration_side
        self.rotational_speed = rotational_speed

        self.linear_speed, self.radius = None, None

    def __str__(self):
        return 'motion: acceleration_forward: %f, acceleration_side: %f, rotational_speed: %f' % \
               (self.acceleration_forward, self.acceleration_side, self.rotational_speed)


class SpeedsAnalyzer(object):
    def __init__(self):
        self.__speeds_filter = LowPassFilter(0.9, 0.0, 0.0, 0.0, 0.0)
        self.__last_speeds = collections.deque(maxlen=20)

    def __call__(self, speeds):
        speeds = self.__speeds_filter(*speeds)
        return Speeds(speeds, self.__last_speeds)


class MotionAnalyzer(object):
    def __init__(self):
        self.__gravity_alpha = 0.8
        self.__gravity_forward, self.__gravity_side = 0.0, 0.0

        self.__acceleration_forward_filter = LowPassFilter(0.3, 0.0)
        self.__acceleration_side_filter = LowPassFilter(0.3, 0.0)
        self.__rotational_speed_filter = LowPassFilter(0.7, 0.0)

    @staticmethod
    def get_motion_data(motion):
        accel = motion.get_accel()
        gyro = motion.get_gyro()

        # unit is cm/s2
        acceleration_forward, acceleration_side, = accel.y_axis / 10.0, accel.x_axis / 10.0
        # unit is rad/s
        rotational_speed = math.radians(gyro.z_axis)

        return Motion(acceleration_forward, acceleration_side, rotational_speed)

    def compute_other_motions(self, motion):
        self.__gravity_forward = self.__gravity_alpha * self.__gravity_forward + \
                                 (1 - self.__gravity_alpha) * motion.acceleration_forward
        motion.acceleration_forward -= self.__gravity_forward

        self.__gravity_side = self.__gravity_alpha * self.__gravity_side + \
                              (1 - self.__gravity_alpha) * motion.acceleration_side
        motion.acceleration_side -= self.__gravity_side

    def filter_motions(self, motion):
        motion.acceleration_forward = self.__acceleration_forward_filter(motion.acceleration_forward)
        motion.acceleration_side = self.__acceleration_side_filter(motion.acceleration_side)
        motion.rotational_speed = self.__rotational_speed_filter(motion.rotational_speed)

    @staticmethod
    def __compute_other_values(motion):
        if abs(motion.rotational_speed) > 0.0:
            motion.linear_speed = motion.acceleration_side / motion.rotational_speed
            motion.radius = motion.linear_speed / motion.rotational_speed
        else:
            motion.linear_speed = 0.0
            motion.radius = 0.0

    def __call__(self, motion):
        motion = self.get_motion_data(motion)
        self.compute_other_motions(motion)
        self.filter_motions(motion)
        self.__compute_other_values(motion)
        return motion


class DistanceLimiter(object):
    def __init__(self):
        self.__scan = None
        self.__distance_factor = None

    def update_scan(self, scan):
        self.__scan = scan
        self.__distance_factor = compute_factor_due_to_distance(scan)

    def __call__(self, speeds):
        apply_factor(speeds, self.__distance_factor)

        scan = self.__scan
        if scan is not None:
            avoid(speeds, scan)
            limit_speed(speeds, scan)
            if speeds.radius is not None and 0.0 < abs(speeds.radius) and \
                            abs(speeds.speed_left - speeds.speed_right) > 25.0:
                robot_trajectory = compute_robot_trajectory(speeds)
                scan.points = sorted(scan.points, key=lambda (a, _): a, reverse=(speeds.radius < 0.0))
                limit_speed_rotational(speeds, scan, robot_trajectory)


class MotionLimiter(object):
    def __init__(self):
        self.__motion = None
        self.__acceleration_factor, self.__rotational_factor = None, None

    def update_motion(self, motion):
        self.__motion = motion
        self.__acceleration_factor, self.__rotational_factor = compute_factor_due_to_motion(motion)

    def __call__(self, speeds):
        apply_factor(speeds, self.__acceleration_factor)
        apply_factor(speeds, self.__rotational_factor)

        motion = self.__motion
        if motion is not None:
            pass


def apply_factor(speeds, factor):
    if 0.0 <= factor <= 1.0:
        speeds.speed_front_left *= factor
        speeds.speed_front_right *= factor
        speeds.speed_rear_left *= factor
        speeds.speed_rear_right *= factor


def find_minimas_maximas_inters(scan):
    minimas, maximas, inters = [], [], []
    scan.points = sorted(scan.points, key=lambda (a, _): a)
    scan_iterator = iter(scan.points)
    try:
        (prev_angle, prev_distance) = scan_iterator.next()
        prev_diff = 0.0
        while True:
            (angle, distance) = scan_iterator.next()
            diff = distance - prev_distance
            if abs(diff - prev_diff) > 10.0:
                inters.append(average(prev_angle, angle))
            else:
                if prev_diff < 0.0 < diff:
                    maximas.append((prev_angle, prev_distance))
                elif prev_diff > 0.0 > diff:
                    minimas.append((prev_angle, prev_distance))
            prev_diff = diff
            (prev_angle, prev_distance) = (angle, distance)
    except StopIteration:
        pass
    return minimas, maximas, inters


def avoid(speeds, scan):
    scan.points = sorted(scan.points, key=lambda (a, _): abs(a))
    scan_iterator = iter(scan.points)
    best_angle, best_distance = 0.0, 50.0
    try:
        while True:
            (angle, distance) = scan_iterator.next()
            if 1200.0 > distance > best_distance and angle < 45.0:
                best_angle = angle
                best_distance = distance
    except StopIteration:
        pass

    if speeds.speed_left * speeds.speed_right > 0.0:
        change_speed(speeds, math.radians(best_angle))


def limit_speed(speeds, scan):
    center_angle = get_angle(speeds.speed_left, speeds.speed_right, ROBO_WIDTH)
    min_distance_angle = center_angle - 30.0
    min_distance = None

    scan.points = sorted(scan.points, key=lambda (a, _): a)
    for angle, distance in scan.points:
        if min_distance_angle < angle < center_angle + 30.0:
            if 60.0 < distance < 5000.0 and (min_distance is None or min_distance > distance):
                min_distance = distance
                min_distance_angle = angle

    if min_distance is not None:
        if min_distance < HARD_DISTANCE_LIMIT:
            if speeds.linear_speed > 0.0:
                speeds.speed_front_left = 0.0
                speeds.speed_front_right = 0.0
                speeds.speed_rear_left = 0.0
                speeds.speed_rear_right = 0.0
        else:
            if min_distance < SOFT_DISTANCE_LIMIT:
                max_speed = compute_max_speed(MAX_SPEED, min_distance, SOFT_DISTANCE_LIMIT, HARD_DISTANCE_LIMIT)
                reduce_speed(speeds, max_speed)
                speeds.compute_other_speed()


def limit_speed_rotational(speeds, scan, robot_trajectory):
    radius_limited_by_scan = speeds.radius

    robot_trajectory = sorted(robot_trajectory, key=lambda (a, _): a, reverse=(speeds.radius > 0.0))

    scan_iterator = iter(scan.points)
    robot_trajectory_iterator = iter(robot_trajectory)

    try:
        (scan_angle, prev_scan_distance) = scan_iterator.next()
        (robot_trajectory_angle, robot_trajectory_distance) = robot_trajectory_iterator.next()
        scan_distance = prev_scan_distance
        while True:
            if abs(robot_trajectory_angle) < abs(scan_angle):
                (scan_angle, new_scan_distance) = scan_iterator.next()
                scan_distance = average(prev_scan_distance, new_scan_distance)
                prev_scan_distance = new_scan_distance

            else:
                value = math.sin(robot_trajectory_angle)
                if scan_distance < robot_trajectory_distance * 0.8 and abs(value) > 0.0:
                    radius = (robot_trajectory_distance * 0.8) / (2 * value)
                    if radius_limited_by_scan > radius:
                        radius_limited_by_scan = radius
                (robot_trajectory_angle, robot_trajectory_distance) = robot_trajectory_iterator.next()

    except StopIteration:
        pass

    if abs(radius_limited_by_scan - speeds.radius) > 25.0:
        change_radius(speeds, radius_limited_by_scan)
        speeds.compute_other_speed()


def change_radius(speeds, radius):
    if radius is None:
        return

    if abs(radius) > 0.0:
        if abs(2.0 * radius + ROBO_WIDTH) > 0.0:
            if radius < 0.0:
                # turn left
                speed_left = speeds.speed_right * (2.0 * radius - ROBO_WIDTH) / (2.0 * radius + ROBO_WIDTH)
                if abs(speeds.speed_left) > 0.0:
                    factor_for_left = abs(speed_left / speeds.speed_left)
                    speeds.speed_front_left = sign(speeds.speed_front_left) * factor_for_left * \
                                              abs(speeds.speed_front_left)
                    speeds.speed_rear_left = sign(speeds.speed_rear_left) * factor_for_left * \
                                             abs(speeds.speed_rear_left)
            else:
                # turn right
                speed_right = speeds.speed_left * (2.0 * radius - ROBO_WIDTH) / (2.0 * radius + ROBO_WIDTH)
                if abs(speeds.speed_right) > 0.0:
                    factor_for_right = abs(speed_right / speeds.speed_right)
                    speeds.speed_front_right = sign(speeds.speed_front_right) * factor_for_right * \
                                               abs(speeds.speed_front_right)
                    speeds.speed_rear_right = sign(speeds.speed_rear_right) * factor_for_right * \
                                              abs(speeds.speed_rear_right)
    else:
        # rotate in place
        if speeds.radius < 0:
            # rotate in left
            speeds.speed_front_left = -speeds.speed_front_right
            speeds.speed_rear_left = -speeds.speed_rear_right
        else:
            # rotate in right
            speeds.speed_front_right = -speeds.speed_front_left
            speeds.speed_rear_right = -speeds.speed_rear_left


def reduce_speed(speeds, speed):
    max_speed = max(abs(speeds.speed_front_left), abs(speeds.speed_front_right),
                    abs(speeds.speed_rear_left), abs(speeds.speed_rear_right))
    if abs(speeds.linear_speed) > 0.0:
        reduce_factor = 1.0
        if abs(speeds.linear_speed) > speed:
            reduce_factor = speed / abs(speeds.linear_speed)
        elif max_speed > (1.2 * speed):
            reduce_factor = (1.2 * speed) / max_speed
        speeds.speed_front_left *= reduce_factor
        speeds.speed_front_right *= reduce_factor
        speeds.speed_rear_left *= reduce_factor
        speeds.speed_rear_right *= reduce_factor


def change_speed(speeds, angle):
    if abs(angle) < math.pi / 2.0:
        if angle > 0.0:
            speeds.speed_front_right = -4.0 * speeds.speed_front_left * angle / math.pi + speeds.speed_front_left
            speeds.speed_rear_right = -4.0 * speeds.speed_rear_left * angle / math.pi + speeds.speed_rear_left
        elif angle < 0.0:
            speeds.speed_front_left = -4.0 * speeds.speed_front_right * angle / math.pi + speeds.speed_front_right
            speeds.speed_rear_left = -4.0 * speeds.speed_rear_right * angle / math.pi + speeds.speed_rear_right
    else:
        if angle < 0.0:
            speeds.speed_front_left = -speeds.speed_front_right
            speeds.speed_rear_left = -speeds.speed_rear_right
        else:
            speeds.speed_front_right = -speeds.speed_front_left
            speeds.speed_rear_right = -speeds.speed_rear_left


def compute_factor_due_to_motion(motion):
    # value of forward acceleration could not be higher than 30 dm/s2
    # value of side acceleration could not be higher than 20 dm/s2
    if abs(motion.acceleration_forward) < MAX_ACCELERATION_FORWARD and \
                    abs(motion.acceleration_side) < MAX_ACCELERATION_SIDE:
        acceleration_factor = (1.0 - abs(motion.acceleration_forward) / MAX_ACCELERATION_FORWARD) * \
                              (1.0 - abs(motion.acceleration_side) / MAX_ACCELERATION_SIDE)
    else:
        acceleration_factor = 0.0

    # value of rotational speed could not be higher than 6.28 rad/s
    if abs(motion.rotational_speed) < MAX_ROTATIONAL_SPEED:
        rotational_factor = (1.0 - abs(motion.rotational_speed) / MAX_ROTATIONAL_SPEED)
    else:
        rotational_factor = 0.0

    return acceleration_factor, rotational_factor


def compute_factor_due_to_distance(scan):
    factor_cosines = 0.0
    factor_gauss = 0.0
    weights_cosines = 0.0
    weights_gauss = 0.0
    for angle, distance in scan.points:
        w_c = math.cos(3.0 / 4.0 * math.radians(angle))
        w_g = math.exp(-math.pow(angle, 2.0))
        if 200.0 < distance:
            weights_cosines += w_c
            weights_gauss += w_g
            if 200.0 < distance <= 1200.0:
                f_c = (distance / 1000.0) - 0.2
                f_g = (distance / 1000.0) - 0.2
                factor_cosines += (w_c * f_c)
                factor_gauss += (w_g * f_g)
            elif 1200.0 < distance < 5600.0:
                factor_cosines += w_c
                factor_gauss += w_g
    if abs(weights_cosines) > 0.0:
        factor_cosines = factor_cosines / weights_cosines
    else:
        factor_cosines = 0.0
    if abs(weights_gauss) > 0.0:
        factor_gauss = factor_gauss / weights_gauss
    else:
        factor_gauss = 0.0
    return 0.4 * factor_cosines + 0.6 * factor_gauss
