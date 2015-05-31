import logging
import logging.config
import threading
import time
import math

from ambercommon.common import runtime
import os

from amberdriver.tools import config


pwd = os.path.dirname(os.path.abspath(__file__))
logging.config.fileConfig('%s/roboclaw.ini' % pwd)
config.add_config_ini('%s/roboclaw.ini' % pwd)

LOGGER_NAME = 'Roboclaw'

WHEEL_RADIUS = float(config.ROBOCLAW_WHEEL_RADIUS)
PULSES_PER_REVOLUTION = float(config.ROBOCLAW_PULSES_PER_REVOLUTION)

STOP_IDLE_TIMEOUT = float(config.ROBOCLAW_STOP_IDLE_TIMEOUT)
RESET_IDLE_TIMEOUT = float(config.ROBOCLAW_RESET_IDLE_TIMEOUT)

ERROR_MONITOR_INTERVAL = float(config.ROBOCLAW_ERROR_MONITOR_INTERVAL)
CRITICAL_READ_REPEATS = int(config.ROBOCLAW_CRITICAL_READ_REPEATS)

RESET_DELAY = float(config.ROBOCLAW_RESET_DELAY)
RESET_GPIO_PATH = str(config.ROBOCLAW_RESET_GPIO_PATH)

LED1_GPIO_PATH = str(config.ROBOCLAW_LED1_GPIO_PATH)
LED2_GPIO_PATH = str(config.ROBOCLAW_LED2_GPIO_PATH)

__author__ = 'paoolo'


class Roboclaw(object):
    def __init__(self, port, rc_address=128):
        self.__port = port
        self.__rc_address = rc_address
        self.__port_lock = threading.RLock()

        runtime.add_shutdown_hook(self.terminate)

    def reset_port(self):
        self.__port_lock.acquire()
        try:
            self.drive_mixed_with_signed_duty_cycle(0, 0)
            self.__port.reset_port()

        finally:
            self.__port_lock.release()

    def flush(self):
        self.__port.flush()

    def terminate(self):
        self.__port_lock.acquire()
        try:
            self.reset_port()
            self.__port.close()

        finally:
            self.__port_lock.release()

    def drive_forward_m1(self, val):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 0)
            self.__port.write_byte(val)
            self.__port.write_byte(self.__port.get_checksum())
        finally:
            self.__port_lock.release()

    def drive_backwards_m1(self, val):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 1)
            self.__port.write_byte(val)
            self.__port.write_byte(self.__port.get_checksum())
        finally:
            self.__port_lock.release()

    def set_min_main_voltage(self, val):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 2)
            self.__port.write_byte(val)
            self.__port.write_byte(self.__port.get_checksum())
        finally:
            self.__port_lock.release()

    def set_max_main_voltage(self, val):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 3)
            self.__port.write_byte(val)
            self.__port.write_byte(self.__port.get_checksum())
        finally:
            self.__port_lock.release()

    def drive_forward_m2(self, val):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 4)
            self.__port.write_byte(val)
            self.__port.write_byte(self.__port.get_checksum())
        finally:
            self.__port_lock.release()

    def drive_backwards_m2(self, val):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 5)
            self.__port.write_byte(val)
            self.__port.write_byte(self.__port.get_checksum())
        finally:
            self.__port_lock.release()

    def drive_m1(self, val):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 6)
            self.__port.write_byte(val)
            self.__port.write_byte(self.__port.get_checksum())
        finally:
            self.__port_lock.release()

    def drive_m2(self, val):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 7)
            self.__port.write_byte(val)
            self.__port.write_byte(self.__port.get_checksum())
        finally:
            self.__port_lock.release()

    def drive_forward(self, val):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 8)
            self.__port.write_byte(val)
            self.__port.write_byte(self.__port.get_checksum())
        finally:
            self.__port_lock.release()

    def drive_backwards(self, val):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 9)
            self.__port.write_byte(val)
            self.__port.write_byte(self.__port.get_checksum())
        finally:
            self.__port_lock.release()

    def turn_right(self, val):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 10)
            self.__port.write_byte(val)
            self.__port.write_byte(self.__port.get_checksum())
        finally:
            self.__port_lock.release()

    def turn_left(self, val):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 11)
            self.__port.write_byte(val)
            self.__port.write_byte(self.__port.get_checksum())
        finally:
            self.__port_lock.release()

    def drive_forward_or_backwards(self, val):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 12)
            self.__port.write_byte(val)
            self.__port.write_byte(self.__port.get_checksum())
        finally:
            self.__port_lock.release()

    def turn_left_or_right(self, val):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 13)
            self.__port.write_byte(val)
            self.__port.write_byte(self.__port.get_checksum())
        finally:
            self.__port_lock.release()

    def read_quad_encoder_register_m1(self):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 16)
            enc = self.__port.read_slong()
            status = self.__port.read_byte()
            crc = self.__port.get_checksum()
            if crc == self.__port.read_byte():
                return enc, status
            return -1, -1
        finally:
            self.__port_lock.release()

    def read_quad_encoder_register_m2(self):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 17)
            enc = self.__port.read_slong()
            status = self.__port.read_byte()
            crc = self.__port.get_checksum()
            if crc == self.__port.read_byte():
                return enc, status
            return -1, -1
        finally:
            self.__port_lock.release()

    def read_speed_m1(self):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 18)
            enc = self.__port.read_slong()
            status = self.__port.read_byte()
            crc = self.__port.get_checksum()
            if crc == self.__port.read_byte():
                return enc, status
            return -1, -1
        finally:
            self.__port_lock.release()

    def read_speed_m2(self):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 19)
            enc = self.__port.read_slong()
            status = self.__port.read_byte()
            crc = self.__port.get_checksum()
            if crc == self.__port.read_byte():
                return enc, status
            return -1, -1
        finally:
            self.__port_lock.release()

    def reset_quad_encoder_counters(self):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 20)
            self.__port.write_byte(self.__port.get_checksum())
        finally:
            self.__port_lock.release()

    def read_firmware_version(self):
        self.__port_lock.acquire()
        try:
            self.__port.send_command(self.__rc_address, 21)
            # FIXME(paoolo): Check docs
            return self.__port.read(32)
        finally:
            self.__port_lock.release()

    def read_main_battery_voltage_level(self):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 24)
            val = self.__port.read_word()
            crc = self.__port.get_checksum()
            if crc == self.__port.read_byte():
                return val
            return -1
        finally:
            self.__port_lock.release()

    def read_logic_battery_voltage_level(self):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 25)
            val = self.__port.read_word()
            crc = self.__port.get_checksum()
            if crc == self.__port.read_byte():
                return val
            return -1
        finally:
            self.__port_lock.release()

    def set_min_logic_voltage_level(self, val):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 26)
            self.__port.write_byte(val)
            self.__port.write_byte(self.__port.get_checksum())
        finally:
            self.__port_lock.release()

    def set_max_logic_voltage_level(self, val):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 27)
            self.__port.write_byte(val)
            self.__port.write_byte(self.__port.get_checksum())
        finally:
            self.__port_lock.release()

    def set_pid_constants_m1(self, p, i, d, qpps):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 28)
            self.__port.write_long(d)
            self.__port.write_long(p)
            self.__port.write_long(i)
            self.__port.write_long(qpps)
            self.__port.write_byte(self.__port.get_checksum())
        finally:
            self.__port_lock.release()

    def set_pid_constants_m2(self, p, i, d, qpps):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 29)
            self.__port.write_long(d)
            self.__port.write_long(p)
            self.__port.write_long(i)
            self.__port.write_long(qpps)
            self.__port.write_byte(self.__port.get_checksum())
        finally:
            self.__port_lock.release()

    def read_current_speed_m1(self):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 30)
            enc = self.__port.read_slong()
            status = self.__port.read_byte()
            crc = self.__port.get_checksum()
            if crc == self.__port.read_byte():
                return enc, status
            return -1, -1
        finally:
            self.__port_lock.release()

    def read_current_speed_m2(self):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 31)
            enc = self.__port.read_slong()
            status = self.__port.read_byte()
            crc = self.__port.get_checksum()
            if crc == self.__port.read_byte():
                return enc, status
            return -1, -1
        finally:
            self.__port_lock.release()

    def drive_m1_with_signed_duty_cycle(self, val):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 32)
            self.__port.write_sword(val)
            self.__port.write_byte(self.__port.get_checksum())
        finally:
            self.__port_lock.release()

    def drive_m2_with_signed_duty_cycle(self, val):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 33)
            self.__port.write_sword(val)
            self.__port.write_byte(self.__port.get_checksum())
        finally:
            self.__port_lock.release()

    def drive_mixed_with_signed_duty_cycle(self, m1, m2):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 34)
            self.__port.write_sword(m1)
            self.__port.write_sword(m2)
            self.__port.write_byte(self.__port.get_checksum())
        finally:
            self.__port_lock.release()

    def drive_m1_with_signed_speed(self, val):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 35)
            self.__port.write_slong(val)
            self.__port.write_byte(self.__port.get_checksum())
        finally:
            self.__port_lock.release()

    def drive_m2_with_signed_speed(self, val):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 36)
            self.__port.write_slong(val)
            self.__port.write_byte(self.__port.get_checksum())
        finally:
            self.__port_lock.release()

    def drive_mixed_with_signed_speed(self, m1, m2):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 37)
            self.__port.write_slong(m1)
            self.__port.write_slong(m2)
            self.__port.write_byte(self.__port.get_checksum())
        finally:
            self.__port_lock.release()

    def drive_m1_with_signed_speed_accel(self, accel, speed):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 38)
            self.__port.write_long(accel)
            self.__port.write_slong(speed)
            self.__port.write_byte(self.__port.get_checksum())
        finally:
            self.__port_lock.release()

    def drive_m2_with_signed_speed_accel(self, accel, speed):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 39)
            self.__port.write_long(accel)
            self.__port.write_slong(speed)
            self.__port.write_byte(self.__port.get_checksum())
        finally:
            self.__port_lock.release()

    def drive_mixed_with_signed_speed_accel(self, accel, speed1, speed2):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 40)
            self.__port.write_long(accel)
            self.__port.write_slong(speed1)
            self.__port.write_slong(speed2)
            self.__port.write_byte(self.__port.get_checksum())
        finally:
            self.__port_lock.release()

    def buffered_m1_drive_with_signed_speed_distance(self, speed, distance, buf):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 41)
            self.__port.write_slong(speed)
            self.__port.write_long(distance)
            self.__port.write_byte(buf)
            self.__port.write_byte(self.__port.get_checksum())
        finally:
            self.__port_lock.release()

    def buffered_m2_drive_with_signed_speed_distance(self, speed, distance, buf):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 42)
            self.__port.write_slong(speed)
            self.__port.write_long(distance)
            self.__port.write_byte(buf)
            self.__port.write_byte(self.__port.get_checksum())
        finally:
            self.__port_lock.release()

    def buffered_drive_mixed_with_signed_speed_distance(self, speed1, distance1, speed2, distance2, buf):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 43)
            self.__port.write_slong(speed1)
            self.__port.write_long(distance1)
            self.__port.write_slong(speed2)
            self.__port.write_long(distance2)
            self.__port.write_byte(buf)
            self.__port.write_byte(self.__port.get_checksum())
        finally:
            self.__port_lock.release()

    def buffered_m1_drive_with_signed_speed_accel_distance(self, accel, speed, distance, buf):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 44)
            self.__port.write_long(accel)
            self.__port.write_slong(speed)
            self.__port.write_long(distance)
            self.__port.write_byte(buf)
            self.__port.write_byte(self.__port.get_checksum())
        finally:
            self.__port_lock.release()

    def buffered_m2_drive_with_signed_speed_accel_distance(self, accel, speed, distance, buf):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 45)
            self.__port.write_long(accel)
            self.__port.write_slong(speed)
            self.__port.write_long(distance)
            self.__port.write_byte(buf)
            self.__port.write_byte(self.__port.get_checksum())
        finally:
            self.__port_lock.release()

    def drive_mixed_with_signed_speed_accel_distance(self, accel, speed1, distance1, speed2, distance2, buf):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 46)
            self.__port.write_long(accel)
            self.__port.write_slong(speed1)
            self.__port.write_long(distance1)
            self.__port.write_slong(speed2)
            self.__port.write_long(distance2)
            self.__port.write_byte(buf)
            self.__port.write_byte(self.__port.get_checksum())
        finally:
            self.__port_lock.release()

    def read_buffer_length(self):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 47)
            buffer1 = self.__port.read_byte()
            buffer2 = self.__port.read_byte()
            crc = self.__port.get_checksum()
            if crc == self.__port.read_byte():
                return buffer1, buffer2
            return -1, -1
        finally:
            self.__port_lock.release()

    def read_motor_currents(self):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 49)
            motor1 = self.__port.read_word()
            motor2 = self.__port.read_word()
            crc = self.__port.get_checksum()
            if crc == self.__port.read_byte():
                return motor1, motor2
            return -1, -1
        finally:
            self.__port_lock.release()

    def drive_mixed_with_speed_individual_accel(self, accel1, speed1, accel2, speed2):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 50)
            self.__port.write_long(accel1)
            self.__port.write_slong(speed1)
            self.__port.write_long(accel2)
            self.__port.write_slong(speed2)
            self.__port.write_byte(self.__port.get_checksum())
        finally:
            self.__port_lock.release()

    def drive_mixed_with_speed_individual_accel_distance(self,
                                                         accel1, speed1, distance1, accel2, speed2, distance2, buf):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 51)
            self.__port.write_long(accel1)
            self.__port.write_slong(speed1)
            self.__port.write_long(distance1)
            self.__port.write_long(accel2)
            self.__port.write_slong(speed2)
            self.__port.write_long(distance2)
            self.__port.write_byte(buf)
            self.__port.write_byte(self.__port.get_checksum())
        finally:
            self.__port_lock.release()

    def drive_m1_with_signed_duty_accel(self, accel, duty):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 52)
            self.__port.write_sword(duty)
            self.__port.write_word(accel)
            self.__port.write_byte(self.__port.get_checksum())
        finally:
            self.__port_lock.release()

    def drive_m2_with_signed_duty_accel(self, accel, duty):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 53)
            self.__port.write_sword(duty)
            self.__port.write_word(accel)
            self.__port.write_byte(self.__port.get_checksum())
        finally:
            self.__port_lock.release()

    def drive_mixed_with_signed_duty_accel(self, accel1, duty1, accel2, duty2):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 54)
            self.__port.write_sword(duty1)
            self.__port.write_word(accel1)
            self.__port.write_sword(duty2)
            self.__port.write_word(accel2)
            self.__port.write_byte(self.__port.get_checksum())
        finally:
            self.__port_lock.release()

    def read_m1_pidq_settings(self):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 55)
            p = self.__port.read_long()
            i = self.__port.read_long()
            d = self.__port.read_long()
            qpps = self.__port.read_long()
            crc = self.__port.get_checksum()
            if crc == self.__port.read_byte():
                return p, i, d, qpps
            return -1, -1, -1, -1
        finally:
            self.__port_lock.release()

    def read_m2_pidq_settings(self):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 56)
            p = self.__port.read_long()
            i = self.__port.read_long()
            d = self.__port.read_long()
            qpps = self.__port.read_long()
            crc = self.__port.get_checksum()
            if crc == self.__port.read_byte():
                return p, i, d, qpps
            return -1, -1, -1, -1
        finally:
            self.__port_lock.release()

    def set_main_battery_voltages(self, _min, _max):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 57)
            self.__port.write_word(_min)
            self.__port.write_word(_max)
            self.__port.write_byte(self.__port.get_checksum())
        finally:
            self.__port_lock.release()

    def set_logic_battery_voltages(self, _min, _max):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 58)
            self.__port.write_word(_min)
            self.__port.write_word(_max)
            self.__port.write_byte(self.__port.get_checksum())
        finally:
            self.__port_lock.release()

    def read_main_battery_voltage_settings(self):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 59)
            _min = self.__port.read_word()
            _max = self.__port.read_word()
            crc = self.__port.get_checksum()
            if crc == self.__port.read_byte():
                return _min, _max
            return -1, -1
        finally:
            self.__port_lock.release()

    def read_logic_battery_voltage_settings(self):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 60)
            _min = self.__port.read_word()
            _max = self.__port.read_word()
            crc = self.__port.get_checksum()
            if crc == self.__port.read_byte():
                return _min, _max
            return -1, -1
        finally:
            self.__port_lock.release()

    def set_m1_position_pid_constants(self, kp, ki, kd, ki_max, deadzone, _min, _max):
        self.__port_lock.acquire()
        try:
            self.__port.send_command(self.__rc_address, 61)
            self.__port.write_long(kp)
            self.__port.write_long(ki)
            self.__port.write_long(kd)
            self.__port.write_long(ki_max)
            self.__port.write_long(deadzone)
            self.__port.write_long(_min)
            self.__port.write_long(_max)
        finally:
            self.__port_lock.release()

    def set_m2_position_pid_constants(self, kp, ki, kd, ki_max, deadzone, _min, _max):
        self.__port_lock.acquire()
        try:
            self.__port.send_command(self.__rc_address, 62)
            self.__port.write_long(kp)
            self.__port.write_long(ki)
            self.__port.write_long(kd)
            self.__port.write_long(ki_max)
            self.__port.write_long(deadzone)
            self.__port.write_long(_min)
            self.__port.write_long(_max)
        finally:
            self.__port_lock.release()

    def read_m1_position_pid_constants(self):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 63)
            p = self.__port.read_long()
            i = self.__port.read_long()
            d = self.__port.read_long()
            i_max = self.__port.read_long()
            deadzone = self.__port.read_long()
            _min = self.__port.read_long()
            _max = self.__port.read_long()
            crc = self.__port.get_checksum()
            if crc == self.__port.read_byte():
                return p, i, d, i_max, deadzone, _min, _max
            return -1, -1, -1, -1, -1, -1, -1
        finally:
            self.__port_lock.release()

    def read_m2_position_pid_constants(self):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 64)
            p = self.__port.read_long()
            i = self.__port.read_long()
            d = self.__port.read_long()
            i_max = self.__port.read_long()
            deadzone = self.__port.read_long()
            _min = self.__port.read_long()
            _max = self.__port.read_long()
            crc = self.__port.get_checksum()
            if crc == self.__port.read_byte():
                return p, i, d, i_max, deadzone, _min, _max
            return -1, -1, -1, -1, -1, -1, -1
        finally:
            self.__port_lock.release()

    def drive_m1_with_signed_speed_accel_deccel_position(self, accel, speed, deccel, position, buf):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 65)
            self.__port.write_long(accel)
            self.__port.write_long(speed)
            self.__port.write_long(deccel)
            self.__port.write_long(position)
            self.__port.write_byte(buf)
            self.__port.write_byte(self.__port.get_checksum())
        finally:
            self.__port_lock.release()

    def drive_m2_with_signed_speed_accel_deccel_position(self, accel, speed, deccel, position, buf):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 66)
            self.__port.write_long(accel)
            self.__port.write_long(speed)
            self.__port.write_long(deccel)
            self.__port.write_long(position)
            self.__port.write_byte(buf)
            self.__port.write_byte(self.__port.get_checksum())
        finally:
            self.__port_lock.release()

    def drive_mixed_with_signed_speed_accel_deccel_position(self,
                                                            accel1, speed1, deccel1, position1,
                                                            accel2, speed2, deccel2, position2,
                                                            buf):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 67)
            self.__port.write_long(accel1)
            self.__port.write_long(speed1)
            self.__port.write_long(deccel1)
            self.__port.write_long(position1)
            self.__port.write_long(accel2)
            self.__port.write_long(speed2)
            self.__port.write_long(deccel2)
            self.__port.write_long(position2)
            self.__port.write_byte(buf)
            self.__port.write_byte(self.__port.get_checksum())
        finally:
            self.__port_lock.release()

    def read_temperature(self):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 82)
            val = self.__port.read_word()
            crc = self.__port.get_checksum()
            if crc == self.__port.read_byte():
                return val
            return -1
        finally:
            self.__port_lock.release()

    def read_error_state(self):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 90)
            val = self.__port.read_byte()
            crc = self.__port.get_checksum()
            if crc == self.__port.read_byte():
                return val
            return -1
        finally:
            self.__port_lock.release()

    def read_encoder_mode(self):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 91)
            mode1 = self.__port.read_byte()
            mode2 = self.__port.read_byte()
            crc = self.__port.get_checksum()
            if crc == self.__port.read_byte():
                return mode1, mode2
            return -1
        finally:
            self.__port_lock.release()

    def set_m1_encoder_mode(self, val):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 92)
            self.__port.write_byte(val)
            self.__port.write_byte(self.__port.get_checksum())
        finally:
            self.__port_lock.release()

    def set_m2_encoder_mode(self, val):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 93)
            self.__port.write_byte(val)
            self.__port.write_byte(self.__port.get_checksum())
        finally:
            self.__port_lock.release()

    def write_settings_to_eeprom(self):
        self.__port_lock.acquire()
        try:
            self.__port.reset_checksum()
            self.__port.send_command(self.__rc_address, 94)
            crc = self.__port.get_checksum()
            if crc == self.__port.read_byte():
                return crc
            return -1
        finally:
            self.__port_lock.release()


def to_mmps(val):
    return int(val * WHEEL_RADIUS * math.pi * 2.0 / PULSES_PER_REVOLUTION)


def to_qpps(val):
    rps = val / (WHEEL_RADIUS * math.pi * 2.0)
    return int(rps * PULSES_PER_REVOLUTION)


class RoboclawDriver(object):
    def __init__(self, front, rear, p=0x10000, i=0x10000, d=0x0, qpps=10000):
        self.__front, self.__rear = front, rear
        self.__roboclaw_lock = threading.RLock()

        self.__timeout_lock = threading.Lock()
        self.__motors_stop_timer_enabled = False
        self.__reset_time, self.__motors_stop_time = 0.0, 0.0

        self.__reset_gpio = open(RESET_GPIO_PATH, mode='ab')
        self.__led1_gpio = open(LED1_GPIO_PATH, mode='ab')
        self.__led2_gpio = open(LED2_GPIO_PATH, mode='ab')

        self.__p_const = p
        self.__i_const = i
        self.__d_const = d
        self.__qpps_const = qpps
        self.__driving_allowed = True

        self.__is_active = True

        self.__logger = logging.getLogger(LOGGER_NAME)

        runtime.add_shutdown_hook(self.terminate)

        self.__front.flush()
        self.__rear.flush()
        self.__setup()

        self.__green_led(True)
        self.__red_led(False)

    def __green_led(self, enable):
        self.__led1_gpio.write('0' if enable else '1')
        self.__led1_gpio.flush()

    def __red_led(self, enable):
        self.__led2_gpio.write('0' if enable else '1')
        self.__led2_gpio.flush()

    def terminate(self):
        self.__driving_allowed = False
        self.__is_active = False
        self.__reset_gpio.close()
        self.__led1_gpio.close()
        self.__led2_gpio.close()

    def get_currents(self):
        self.__roboclaw_lock.acquire()
        try:
            front_right_current, front_left_current = self.__front.read_motor_currents()
            rear_right_current, rear_left_current = self.__rear.read_motor_currents()

            return (front_right_current / 10.0, front_left_current / 10.0,
                    rear_right_current / 10.0, rear_left_current / 10.0)
        finally:
            self.__roboclaw_lock.release()

    def get_voltages(self):
        self.__roboclaw_lock.acquire()
        try:
            front_voltage = self.__front.read_main_battery_voltage_level()
            rear_voltage = self.__rear.read_main_battery_voltage_level()

            return front_voltage / 10.0, rear_voltage / 10.0
        finally:
            self.__roboclaw_lock.release()

    def get_speeds(self):
        if not self.__driving_allowed:
            return 0, 0, 0, 0

        self.__roboclaw_lock.acquire()
        try:
            front_right, _ = self.__front.read_speed_m1()
            front_left, _ = self.__front.read_speed_m2()
            rear_right, _ = self.__rear.read_speed_m1()
            rear_left, _ = self.__rear.read_speed_m2()
        finally:
            self.__roboclaw_lock.release()

        front_left = to_mmps(front_left)
        front_right = to_mmps(front_right)
        rear_left = to_mmps(rear_left)
        rear_right = to_mmps(rear_right)

        return front_left, front_right, rear_left, rear_right

    def set_speeds(self, front_left, front_right, rear_left, rear_right):
        if not self.__driving_allowed:
            return

        front_left = to_qpps(front_left)
        front_right = to_qpps(front_right)
        rear_left = to_qpps(rear_left)
        rear_right = to_qpps(rear_right)

        self.__reset_timeouts()

        self.__roboclaw_lock.acquire()
        try:
            self.__green_led(False)
            self.__front.drive_mixed_with_signed_speed(front_right, front_left)
            self.__front.flush()
            self.__rear.drive_mixed_with_signed_speed(rear_right, rear_left)
            self.__rear.flush()
            self.__green_led(True)
        finally:
            self.__roboclaw_lock.release()

    def stop(self):
        self.__roboclaw_lock.acquire()
        try:
            self.__front.drive_mixed_with_signed_speed(0, 0)
            self.__rear.drive_mixed_with_signed_speed(0, 0)
        finally:
            self.__roboclaw_lock.release()

    def __reset(self):
        self.__roboclaw_lock.acquire()
        try:
            self.__driving_allowed = False
            self.__reset_gpio.write('1')
            self.__reset_gpio.flush()
            time.sleep(0.0001)
            self.__reset_gpio.write('0')
            self.__reset_gpio.flush()
            time.sleep(0.0001)
            self.__reset_gpio.write('1')
            self.__reset_gpio.flush()
            time.sleep(RESET_DELAY / 1000.0)
            self.__front.reset_port()
            self.__rear.reset_port()
            self.__setup()
            self.__driving_allowed = True
        finally:
            self.__roboclaw_lock.release()

    def __setup(self):
        self.__front.set_pid_constants_m1(self.__p_const, self.__i_const, self.__d_const, self.__qpps_const)
        self.__front.set_pid_constants_m2(self.__p_const, self.__i_const, self.__d_const, self.__qpps_const)
        self.__rear.set_pid_constants_m1(self.__p_const, self.__i_const, self.__d_const, self.__qpps_const)
        self.__rear.set_pid_constants_m2(self.__p_const, self.__i_const, self.__d_const, self.__qpps_const)

        self.__front.set_m1_encoder_mode(0)
        self.__front.set_m2_encoder_mode(0)
        self.__rear.set_m1_encoder_mode(0)
        self.__rear.set_m2_encoder_mode(0)

    def timeout_monitor_loop(self):
        self.__reset_timeouts()
        while self.__is_active:
            current_time = time.time()

            self.__timeout_lock.acquire()
            try:
                do_stop = False
                if self.__motors_stop_timer_enabled and self.__motors_stop_time < current_time:
                    do_stop = True
                    self.__motors_stop_timer_enabled = False

                do_reset = False
                if self.__reset_time < current_time:
                    do_reset = True
                    self.__reset_time = current_time + RESET_IDLE_TIMEOUT / 1000.0
            finally:
                self.__timeout_lock.release()

            if do_stop:
                self.stop()
            if do_reset:
                self.__reset()

            time.sleep(0.1)

    def __read_error_codes(self):
        self.__roboclaw_lock.acquire()
        try:
            front_error_code = self.__front.read_error_state()
            rear_error_code = self.__rear.read_error_state()
            return front_error_code, rear_error_code

        finally:
            self.__roboclaw_lock.release()

    def __get_error_codes(self):
        front_error_codes, rear_error_codes = self.__read_error_codes()
        if front_error_codes == 0 and rear_error_codes == 0:
            return 0x0, 0x0

        front_error_codes_tmp = front_error_codes
        rear_error_codes_tmp = rear_error_codes

        same_errors = True
        for _ in range(CRITICAL_READ_REPEATS):
            front_error_codes, rear_error_codes = self.__read_error_codes()
            if front_error_codes != front_error_codes_tmp or rear_error_codes != rear_error_codes_tmp:
                same_errors = False
                break

        return (front_error_codes, rear_error_codes) if same_errors else (0x0, 0x0)

    def error_monitor_loop(self):
        while self.__is_active:
            time.sleep(ERROR_MONITOR_INTERVAL / 1000.0)

            front_error_status, rear_error_status = self.__get_error_codes()

            if front_error_status > 0 or rear_error_status > 0:
                self.__logger.warn('Error: front: %f, rear: %f, reset!', front_error_status, rear_error_status)
                self.__red_led(True)
                self.__reset()

                if front_error_status == 0x20 or rear_error_status == 0x20:
                    self.__driving_allowed = False
                    self.__logger.critical('Battery low!')

            elif front_error_status < 0 or rear_error_status < 0:
                self.__logger.warn('Bad feelings: error status(es) less than zero. '
                                   'It looks that CRC is wrong or nothing is returned '
                                   'from Roboclaw.')

    def __reset_timeouts(self):
        act_time = time.time()

        self.__timeout_lock.acquire()
        try:
            self.__reset_time = act_time + RESET_IDLE_TIMEOUT / 1000.0
            self.__motors_stop_time = act_time + STOP_IDLE_TIMEOUT / 1000.0
            self.__motors_stop_timer_enabled = True
        finally:
            self.__timeout_lock.release()