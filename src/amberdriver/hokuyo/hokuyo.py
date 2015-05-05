import logging
import logging.config
import threading
import sys
import time

import os
import traceback
from ambercommon.common import runtime

from amberdriver.tools import config


__author__ = 'paoolo'

pwd = os.path.dirname(os.path.abspath(__file__))
logging.config.fileConfig('%s/hokuyo.ini' % pwd)
config.add_config_ini('%s/hokuyo.ini' % pwd)

LOGGER_NAME = 'Hokuyo'


def chunks(l, n):
    for i in xrange(0, len(l), n):
        yield l[i:i + n]


def decode(val):
    bin_str = '0b'
    for char in val:
        val = ord(char) - 0x30
        bin_str += '%06d' % int(bin(val)[2:])
    return int(bin_str, 2)


class Hokuyo(object):
    SHORT_COMMAND_LEN = 5
    MD_COMMAND_REPLY_LEN = 20

    LASER_ON = 'BM\n'
    LASER_OFF = 'QT\n'
    RESET = 'RS\n'

    VERSION_INFO = 'VV\n'
    SENSOR_STATE = 'II\n'
    SENSOR_SPECS = 'PP\n'

    CHARS_PER_VALUE = 3.0
    CHARS_PER_LINE = 66.0
    CHARS_PER_BLOCK = 64.0

    START_DEG = 119.885
    STEP_DEG = 0.35208516886930985

    START_STEP = 44
    STOP_STEP = 725

    VERSION_INFO_LINES = 6
    SENSOR_STATE_LINES = 8
    SENSOR_SPECS_LINES = 9

    def __init__(self, port):
        self.__port = port
        self.__port_lock = threading.RLock()

        runtime.add_shutdown_hook(self.terminate)

    def flush(self):
        self.__port_lock.acquire()
        try:
            self.__port.write('QT\nRS\nQT\n')
            self.__port.reset_port()

        finally:
            self.__port_lock.release()

    def terminate(self):
        self.__port_lock.acquire()
        try:
            self.flush()
            self.__port.close()

        finally:
            self.__port_lock.release()

    def __offset(self):
        count = 3
        result = ''

        self.__port_lock.acquire()
        try:
            a = self.__port.read(1)
            b = self.__port.read(1)
            c = self.__port.read(1)

            while not (c == '' and ((a == '\n' and b == '\n') or (a == '\n' and b == '') or (a == '' and b == ''))):
                result += a
                a = b
                b = c
                c = self.__port.read(1)
                count += 1

        finally:
            self.__port_lock.release()

        result += a
        result += b
        result += c

        sys.stderr.write(
            '\n===============\nREAD %d EXTRA BYTES\n===============\n%s\n===============\n' % (count, str(result)))

    def __execute_command(self, command):
        self.__port_lock.acquire()
        try:
            self.__port.write(command)
            result = self.__port.read(len(command))
            assert result == command
            return result

        finally:
            self.__port_lock.release()

    def __short_command(self, command, check_response=True):
        result = ''
        self.__port_lock.acquire()
        try:
            result += self.__execute_command(command)
            result += self.__port.read(Hokuyo.SHORT_COMMAND_LEN)

            if check_response:
                assert result[-5:-2] == '00P'
            assert result[-2:] == '\n\n'

            return result

        except BaseException:
            sys.stderr.write('RESULT: "%s"\n' % result)
            traceback.print_exc()
            traceback.print_stack()
            self.__offset()

        finally:
            self.__port_lock.release()

    def __long_command(self, cmd, lines, check_response=True):
        result = ''
        self.__port_lock.acquire()
        try:
            result += self.__execute_command(cmd)

            result += self.__port.read(4)
            if check_response:
                assert result[-4:-1] == '00P'
            assert result[-1:] == '\n'

            line = 0
            while line < lines:
                char = self.__port.read_byte()
                if char is not None:
                    char = chr(char)
                    result += char
                    if char == '\n':
                        line += 1
                else:  # char is None
                    line += 1

            assert result[-2:] == '\n\n'

            return result

        except BaseException:
            sys.stderr.write('RESULT: "%s"\n' % result)
            traceback.print_exc()
            traceback.print_stack()
            self.__offset()

        finally:
            self.__port_lock.release()

    def laser_on(self):
        return self.__short_command(Hokuyo.LASER_ON, check_response=True)

    def laser_off(self):
        return self.__short_command(Hokuyo.LASER_OFF)

    def reset(self):
        return self.__short_command(Hokuyo.RESET)

    def set_motor_speed(self, motor_speed=99):
        return self.__short_command('CR' + '%02d' % motor_speed + '\n', check_response=False)

    def set_high_sensitive(self, high_sensitive=True):
        return self.__short_command('HS' + ('1\n' if high_sensitive else '0\n'), check_response=False)

    def get_version_info(self):
        return self.__long_command(Hokuyo.VERSION_INFO, Hokuyo.VERSION_INFO_LINES)

    def get_sensor_state(self):
        return self.__long_command(Hokuyo.SENSOR_STATE, Hokuyo.SENSOR_STATE_LINES)

    def get_sensor_specs(self):
        return self.__long_command(Hokuyo.SENSOR_SPECS, Hokuyo.SENSOR_SPECS_LINES)

    def __get_and_parse_scan(self, cluster_count, start_step, stop_step):
        distances = {}
        result = ''

        count = ((stop_step - start_step) * Hokuyo.CHARS_PER_VALUE * Hokuyo.CHARS_PER_LINE)
        count /= (Hokuyo.CHARS_PER_BLOCK * cluster_count)
        count += 5.0  # magic number
        count = int(count)

        self.__port_lock.acquire()
        try:
            result += self.__port.read(count)
        finally:
            self.__port_lock.release()

        assert result[-2:] == '\n\n'

        result = result.split('\n')
        result = map(lambda line: line[:-1], result)
        result = ''.join(result)

        i = 0
        start = (-Hokuyo.START_DEG + Hokuyo.STEP_DEG * cluster_count * (start_step - Hokuyo.START_STEP))
        for chunk in chunks(result, 3):
            distances[- ((Hokuyo.STEP_DEG * cluster_count * i) + start)] = decode(chunk)
            i += 1

        return distances

    def get_single_scan(self, start_step=START_STEP, stop_step=STOP_STEP, cluster_count=1):
        self.__port_lock.acquire()
        try:
            cmd = 'GD%04d%04d%02d\n' % (start_step, stop_step, cluster_count)
            self.__port.write(cmd)

            result = self.__port.read(len(cmd))
            assert result == cmd

            result += self.__port.read(4)
            assert result[-4:-1] == '00P'
            assert result[-1] == '\n'

            result = self.__port.read(6)
            assert result[-1] == '\n'

            scan = self.__get_and_parse_scan(cluster_count, start_step, stop_step)
            return scan

        except BaseException:
            traceback.print_exc()
            self.__offset()

        finally:
            self.__port_lock.release()

    def get_multiple_scans(self, start_step=START_STEP, stop_step=STOP_STEP, cluster_count=1,
                           scan_interval=0, number_of_scans=0):
        self.__port_lock.acquire()
        try:
            cmd = 'MD%04d%04d%02d%01d%02d\n' % (start_step, stop_step, cluster_count, scan_interval, number_of_scans)
            self.__port.write(cmd)

            result = self.__port.read(len(cmd))
            assert result == cmd

            result += self.__port.read(Hokuyo.SHORT_COMMAND_LEN)
            assert result[-2:] == '\n\n'

            index = number_of_scans
            while number_of_scans == 0 or index > 0:
                index -= 1

                try:
                    result = self.__port.read(Hokuyo.MD_COMMAND_REPLY_LEN)
                    assert result[:13] == cmd[:13]

                    result = self.__port.read(6)
                    assert result[-1] == '\n'

                    scan = self.__get_and_parse_scan(cluster_count, start_step, stop_step)
                    yield scan

                except AssertionError:
                    sys.stderr.write('Assert error!\n')
                    traceback.print_exc()
                    self.__offset()

        except GeneratorExit:
            sys.stderr.write('Multi scan interrupted!\n')
            self.__offset()

        except BaseException:
            traceback.print_exc()
            self.__offset()

        finally:
            self.__port_lock.release()


class HokuyoDriver(object):
    def __init__(self, scanner, motor_speed=0, high_sensitive=False, multi_scanning_allowed=False):
        self.__scanner = scanner
        self.__hokuyo_lock = threading.RLock()

        self.__scan = ([], [], 0)

        self.__motor_speed = motor_speed
        self.__high_sensitive = high_sensitive
        self.__multi_scanning_allowed = multi_scanning_allowed

        self.__is_active = True

        self.__logger = logging.getLogger(LOGGER_NAME)

        runtime.add_shutdown_hook(self.terminate)

        self.__reset()

    def terminate(self):
        self.__multi_scanning_allowed = False
        self.__is_active = False

    def enable_multi_scanning(self, scanning_allowed):
        self.__multi_scanning_allowed = scanning_allowed

    def get_scan(self):
        if not self.__multi_scanning_allowed:
            scan = self.__scanner.get_single_scan()
            self.__set_scan(scan)
        return self.__scan

    def __reset(self):
        self.__hokuyo_lock.acquire()
        try:
            self.__scanner.flush()
            self.__setup()
        finally:
            self.__hokuyo_lock.release()

    def __setup(self):
        self.__scanner.set_motor_speed(self.__motor_speed)
        self.__scanner.set_high_sensitive(self.__high_sensitive)

    def scanning_loop(self):
        while self.__is_active:
            if self.__multi_scanning_allowed:
                for scan in self.__scanner.get_multiple_scans():
                    self.__set_scan(scan)
                    if not self.__multi_scanning_allowed or not self.__is_active:
                        self.__reset()
                        self.__scanner.laser_on()
                        break
            time.sleep(0.1)

    def __set_scan(self, scan):
        if scan is not None:
            timestamp = int(time.time() * 1000.0)
            angles, distances = HokuyoDriver.__parse_scan(scan)
            self.__scan = (angles, distances, timestamp)

    @staticmethod
    def __parse_scan(scan):
        angles = sorted(scan.keys())
        distances = map(scan.get, angles)
        return angles, distances