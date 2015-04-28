import struct

__author__ = 'paoolo'


class SerialPort(object):
    def __init__(self, serial_port):
        self.__port = serial_port
        self.__checksum = 0

    def close(self):
        self.__port.close()

    def get_checksum(self):
        return self.__checksum

    def reset_checksum(self):
        self.__checksum = 0

    def read(self, size):
        return self.__port.read(size)

    def write(self, char):
        self.__port.write(char)
        self.__port.flush()

    def send_command(self, address, command):
        self.__checksum = address
        self.__port.write(chr(address))
        self.__checksum += command
        self.__port.write(chr(command))
        self.__port.flushInput()

    def read_byte(self):
        res = self.__port.read(1)
        if res == '':
            self.__port.flushInput()
            res = self.__port.read(1)
        if len(res) == 1:
            val = struct.unpack('>B', res)
            self.__checksum += val[0] & 0xFF
            return val[0]
        return None

    def read_sbyte(self):
        res = self.__port.read(1)
        if res == '':
            self.__port.flushInput()
            res = self.__port.read(1)
        if len(res) == 1:
            val = struct.unpack('>b', res)
            self.__checksum += val[0] & 0xFF
            return val[0]
        return None

    def read_word(self):
        res = self.__port.read(2)
        if res == '':
            self.__port.flushInput()
            res = self.__port.read(2)
        if len(res) == 2:
            val = struct.unpack('>H', res)
            self.__checksum += val[0] & 0xFF
            self.__checksum += (val[0] >> 8) & 0xFF
            return val[0]
        return None

    def read_sword(self):
        res = self.__port.read(2)
        if res == '':
            self.__port.flushInput()
            res = self.__port.read(2)
        if len(res) == 2:
            val = struct.unpack('>h', res)
            self.__checksum += val[0] & 0xFF
            self.__checksum += (val[0] >> 8) & 0xFF
            return val[0]
        return None

    def read_long(self):
        res = self.__port.read(4)
        if res == '':
            self.__port.flushInput()
            res = self.__port.read(4)
        if len(res) == 4:
            val = struct.unpack('>L', res)
            self.__checksum += val[0] & 0xFF
            self.__checksum += (val[0] >> 8) & 0xFF
            self.__checksum += (val[0] >> 16) & 0xFF
            self.__checksum += (val[0] >> 24) & 0xFF
            return val[0]
        return None

    def read_slong(self):
        res = self.__port.read(4)
        if res == '':
            self.__port.flushInput()
            res = self.__port.read(4)
        if len(res) == 4:
            val = struct.unpack('>l', res)
            self.__checksum += val[0] & 0xFF
            self.__checksum += (val[0] >> 8) & 0xFF
            self.__checksum += (val[0] >> 16) & 0xFF
            self.__checksum += (val[0] >> 24) & 0xFF
            return val[0]
        return None

    def write_byte(self, val):
        self.__checksum += val & 0xFF
        try:
            return self.__port.write(struct.pack('>B', val))
        finally:
            self.__port.flush()

    def write_sbyte(self, val):
        self.__checksum += val & 0xFF
        try:
            return self.__port.write(struct.pack('>b', val))
        finally:
            self.__port.flush()

    def write_word(self, val):
        self.__checksum += val & 0xFF
        self.__checksum += (val >> 8) & 0xFF
        try:
            return self.__port.write(struct.pack('>H', val))
        finally:
            self.__port.flush()

    def write_sword(self, val):
        self.__checksum += val & 0xFF
        self.__checksum += (val >> 8) & 0xFF
        try:
            return self.__port.write(struct.pack('>h', val))
        finally:
            self.__port.flush()

    def write_long(self, val):
        self.__checksum += val & 0xFF
        self.__checksum += (val >> 8) & 0xFF
        self.__checksum += (val >> 16) & 0xFF
        self.__checksum += (val >> 24) & 0xFF
        try:
            return self.__port.write(struct.pack('>L', val))
        finally:
            self.__port.flush()

    def write_slong(self, val):
        self.__checksum += val & 0xFF
        self.__checksum += (val >> 8) & 0xFF
        self.__checksum += (val >> 16) & 0xFF
        self.__checksum += (val >> 24) & 0xFF
        try:
            return self.__port.write(struct.pack('>l', val))
        finally:
            self.__port.flush()