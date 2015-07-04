import math
import time

__author__ = 'paoolo'

""" Other """


def dround(value, granularity=0.25):
    new_value = int(value / granularity) * granularity
    if value - new_value >= granularity / 2.0:
        new_value += granularity
    return new_value


def drange(start, stop, step):
    r = start
    while (r < stop and step > 0.0) or (r > stop and step < 0.0):
        yield r
        r += step


def sign(value):
    return -1 if value < 0.0 else 1


def vector_operation(operation):
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


add = vector_operation(lambda a, b: a + b)
subtract = vector_operation(lambda a, b: a - b)
multiply = vector_operation(lambda a, b: a * b)
divide = vector_operation(lambda a, b: a / b)


def average(*values):
    return (float(sum(values)) / float(len(values))) if len(values) > 0 else 0.0


""" Trust data function """


def compute_data_trust(data_ts, curr_ts, exp_base=4.0 / 3.0):
    val = data_ts - curr_ts
    return math.pow(exp_base, val)


def compute_location_trust(location, exp_base=4.0 / 3.0):
    _, _, location_probability, _, location_timestamp = location
    current_timestamp = time.time()
    return location_probability * compute_data_trust(location_timestamp / 1000.0, current_timestamp, exp_base)


""" Angles function, conversion, etc.  """


def convert_angles_to_radians(points):
    return map(lambda (angle, distance): (math.radians(angle), distance), points)


def convert_angles_to_degrees(points):
    return map(lambda (angle, distance): (math.degrees(angle), distance), points)


def convert_polar_to_grid(value, angle):
    x = value * math.cos(angle)
    y = value * math.sin(angle)
    return x, y


def convert_grid_to_polar(x, y):
    angle = math.atan2(y, x)
    value = math.sqrt(x ** 2 + y ** 2)
    return angle, value


""" Data analyzer """


class ScanAnalyzer(object):
    def __init__(self):
        pass

    def __call__(self, scan):
        # units are mm and deg
        points = scan.get_points()
        convert_angles_to_radians(points)
        return Scan(points)


""" Objects class """


class Object(object):
    pass


class Value(object):
    def __init__(self, timestamp=None):
        self.timestamp = time.time() * 1000.0 if timestamp is None else timestamp


class Scan(Value):
    def __init__(self, points):
        Value.__init__(self)
        self.points = points

    def __str__(self):
        return 'scan: points: counts: %d' % len(self.points)


""" Filter """


class LowPassFilter(object):
    def __init__(self, alpha, *args):
        self.__values = args
        self.__alpha = alpha

    def __call__(self, *args):
        self.__values = map(lambda (prev, curr): (prev + self.__alpha * (curr - prev)), zip(self.__values, args))
        self.__values = map(lambda val: dround(val, 0.01), self.__values)
        self.__values = map(lambda (new, curr): curr if abs(new - curr) <= 0.01 else new, zip(self.__values, args))
        return self.__values[0] if len(self.__values) == 1 else self.__values
