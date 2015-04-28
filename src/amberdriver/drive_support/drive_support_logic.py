import math
import time

import os

from amberdriver.tools import config


__author__ = 'paoolo'

pwd = os.path.dirname(os.path.abspath(__file__))
config.add_config_ini('%s/drive_support.ini' % pwd)

ROBO_WIDTH = float(config.ROBO_WIDTH)

MAX_SPEED = float(config.MAX_SPEED)
MAX_ROTATING_SPEED = float(config.MAX_ROTATING_SPEED)
SOFT_LIMIT = float(config.SOFT_LIMIT)
HARD_LIMIT = float(config.HARD_LIMIT)

SCANNER_DIST_OFFSET = float(config.SCANNER_DIST_OFFSET)
ANGLE_RANGE = float(config.ANGLE_RANGE)

DISTANCE_ALPHA = float(config.DISTANCE_ALPHA)
RODEO_SWAP_ALPHA = float(config.RODEO_SWAP_ALPHA)


def get_angle(left, right, robo_width):
    return math.atan2(left - right, float(robo_width))


def normalize_angle(angle):
    if angle < -math.pi:
        angle += 2 * math.pi
    elif angle > math.pi:
        angle -= 2 * math.pi
    return angle


def get_speed(left, right):
    return (left + right) / 2.0


def get_max_speed(distance, soft_limit, hard_limit, max_speed):
    return max_speed / (soft_limit - hard_limit) * float(distance) - \
           (max_speed * hard_limit) / (soft_limit - hard_limit)


def get_soft_limit(current_speed, max_speed, soft_limit, hard_limit, alpha):
    return alpha * soft_limit * (current_speed / max_speed) + hard_limit + 50.0


def convert_angles_to_radians(points):
    return map(lambda (angle, distance): (math.radians(angle), distance), points)


def convert_angles_to_degrees(points):
    return map(lambda (angle, distance): (math.degrees(angle), distance), points)


def get_min_distance(scan, current_angle, scanner_dist_offset, angle_range):
    points = convert_angles_to_radians(scan)
    min_distance = None
    min_distance_angle = None

    for angle, distance in points:
        if distance > scanner_dist_offset \
                and current_angle - angle_range < angle < current_angle + angle_range:
            if min_distance is None or distance < min_distance:
                min_distance = distance
                min_distance_angle = angle

    return min_distance, min_distance_angle


def get_max_distance(scan, current_angle, scanner_dist_offset, angle_range):
    points = convert_angles_to_radians(scan)
    max_distance = None
    max_distance_angle = None
    max_diff_angle = 0.0

    for angle, distance in points:
        if (distance > scanner_dist_offset or distance == 0) \
                and current_angle - angle_range < angle < current_angle + angle_range:
            diff_angle = abs(current_angle - angle)
            if (max_distance is None or distance > max_distance or distance == 0) and diff_angle > max_diff_angle:
                max_distance = distance
                max_distance_angle = angle
                max_diff_angle = diff_angle

    return max_distance, max_distance_angle


def simple_importance_angle_level(angle):
    if math.fabs(angle) < math.pi / 2.0:
        return 1.0 - 2.0 * math.fabs(angle) / math.pi
    return 0.0


def simple_distance_level(distance, max_distance=5000.0):
    """ max_distance in [mm] """
    return distance / max_distance


def average(values):
    return reduce(lambda x, y: x + y, values) / len(values)


def distance_danger_level(scan,
                          importance_angle_level_func=simple_importance_angle_level,
                          distance_level_func=simple_distance_level,
                          compute_level_func=average):
    points = convert_angles_to_radians(scan)
    levels = []
    for angle, distance in points:
        importance_angle_level = importance_angle_level_func(angle)
        distance_level = distance_level_func(distance)
        levels.append(importance_angle_level * distance_level)
    return compute_level_func(levels)


def find_continuities(scan, max_continuity_interval=10):
    """ max_continuity_interval in [mm] for distance 100 [mm] """
    points = convert_angles_to_radians(scan)
    continuities = []
    old_x, old_y = None, None
    first_x, first_y = None, None
    for angle, distance in sorted(points.items()):
        x, y = distance * math.cos(angle), distance * math.sin(angle)
        if old_x is None and old_y is None:
            old_x, old_y = x, y
            first_x, first_y = x, y
        else:
            limit = math.pow(distance * max_continuity_interval * 0.01, 2)
            if math.pow(old_x - x, 2) > limit or math.pow(old_y - y, 2) > limit:
                first = (first_x, first_y)
                last = (old_x, old_y)
                continuities.append((first, last))
                first_x, first_y = x, y
            else:
                old_x, old_y = x, y
    return continuities


def find_locals_min_max(scan, interval=50, min_distance=100.0, max_distance=5000.0):
    """ interval in 50 [mm] """
    points = convert_angles_to_radians(scan)
    local_mini, local_maxi = None, None
    local_minis, local_maxis = [], []
    prev_distance = None
    for angle, distance in sorted(points.items()):
        if min_distance < distance < max_distance:
            if prev_distance is None:
                prev_distance = distance
            else:
                if distance - prev_distance > interval:
                    local_maxi = (angle, distance)
                    if local_mini is not None:
                        local_minis.append(local_mini)
                        local_mini = None
                elif distance - prev_distance < -interval:
                    local_mini = (angle, distance)
                    if local_maxi is not None:
                        local_maxis.append(local_maxi)
                        local_maxi = None
                prev_distance = distance
    return local_minis, local_maxis


def limit_due_to_distance(left, right, scan):
    if left > 0 or right > 0:
        current_angle = get_angle(left, right,
                                  ROBO_WIDTH)
        current_speed = get_speed(left, right)

        if scan is not None:
            min_distance, _ = get_min_distance(scan, current_angle,
                                               SCANNER_DIST_OFFSET, ANGLE_RANGE)

            if min_distance is not None:
                soft_limit = get_soft_limit(current_speed,
                                            MAX_SPEED, SOFT_LIMIT * 1.3, HARD_LIMIT * 1.3, DISTANCE_ALPHA)

                if HARD_LIMIT * 1.3 < min_distance < soft_limit:
                    max_speed = get_max_speed(min_distance, soft_limit,
                                              HARD_LIMIT * 1.3, MAX_SPEED)
                    if current_speed > max_speed:
                        left, right = __calculate_new_left_right(left, right, max_speed, current_speed)

                elif min_distance <= HARD_LIMIT * 1.3:
                    left, right = 0, 0

        else:
            print 'distance: no scan!'
            left, right = 0.0, 0.0

    return left, right


def __calculate_new_left_right(left, right, max_speed, current_speed):
    if current_speed > 0:
        divide = max_speed / current_speed
        return left * divide, right * divide
    else:
        return left, right


def limit_to_max_speed(left, right):
    left = __limit_to_max_speed(left)
    right = __limit_to_max_speed(right)

    return left, right


def __limit_to_max_speed(value):
    max_speed = MAX_SPEED
    return max_speed if value > max_speed \
        else -max_speed if value < -max_speed \
        else value


def data_trust(data_ts, curr_ts):
    val = data_ts / 1000.0 - curr_ts
    return math.pow(4.0 / 3.0, val)


def location_trust(location):
    _, _, location_probability, _, location_timestamp = location
    current_timestamp = time.time()
    return location_probability * data_trust(location_timestamp, current_timestamp)


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


class Map(object):
    def __init__(self):
        self.__map = {}

    def add(self, x, y, angle=None, distance=None):
        if angle is not None and distance is not None:
            _x, _y = convert_polar_to_grid(distance, angle)
            x += _x
            y += _y
        if x not in self.__map:
            self.__map[x] = {}
        if y not in self.__map[x]:
            self.__map[x][y] = time.time()


def motion_to_speed(motion,
                    acc_function=lambda acc: acc,
                    rev_function=lambda rev: rev):
    accel = motion.get_accel()
    gyro = motion.get_gyro()

    acc_x, acc_y = accel.x_axis, accel.y_axis
    acc_x, acc_y = acc_function(acc_x), acc_function(acc_y)
    acc, angle = convert_grid_to_polar(acc_x, acc_y)

    gyro_val = gyro.z_axis
    rev = rev_function(gyro_val)

    if not gyro_val == 0:
        pass

        # acc -> motion in any way
        # gyro -> turn left or right


def speed_to_motion(speed):
    pass


def compute_obstacle_forces(map_polar,
                            distance_function=lambda (distance, angle): distance,
                            force_function=lambda (force_x, force_y, angle): (force_x, force_y)):
    sum_force_x, sum_force_y = 0.0, 0.0
    for angle, distance in map_polar:
        distance = distance_function(distance, angle)
        force_x, force_y = convert_polar_to_grid(distance, angle)
        force_x, force_y = force_function(force_x, force_y, angle)
        sum_force_x += force_x
        sum_force_y += force_y


def compute_target_force():
    pass


def adjust_speed(fl, fr, rl, rr, motion_data, measured_speed):
    """
    * analyze robo motion: from gyro/accel and measured speed
    * adjust speed
    """
    return fl, fr, rl, rr


def limit_speed(fl, fr, rl, rr, environment_scan):
    """
    * lookahead around robot
    * verify state
    * limit if necessary
    """
    return fl, fr, rl, rr
