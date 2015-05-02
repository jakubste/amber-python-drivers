import struct
import sys

__author__ = 'paoolo'


class SerialPort(object):
    def __init__(self, serial_port):
        self.__port = serial_port
        self.__checksum = 0

    def close(self):
        self.__port.close()

    def flush(self):
        result = ''
        flushing = True

        self.__port.flush()
        self.__port.flushInput()
        self.__port.flushOutput()

        while flushing:
            char = self.__port.read(1)
            flushing = (char != '')
            result += char

        sys.stderr.write('\n===============\nFLUSH SERIAL PORT\n===============\n%s\n===============\n' % result)

    def get_checksum(self, mask=0x7F):
        return self.__checksum & mask

    def reset_checksum(self, value=0x0):
        self.__checksum = value

    def read(self, size):
        return self.__port.read(size)

    def write(self, char):
        self.__port.write(char)
        self.__port.flush()

    def send_command(self, address, command):
        self.write_byte(address)
        self.write_byte(command)

    def read_byte(self):
        res = self.__port.read(1)
        if res == '':
            self.__port.flushInput()
            res = self.__port.read(1)
        if len(res) == 1:
            val = struct.unpack('>B', res)
            self.__checksum = (val[0] + self.__checksum) & 0xFF
            return val[0]
        raise Exception('No byte read')

    def read_sbyte(self):
        res = self.__port.read(1)
        if res == '':
            self.__port.flushInput()
            res = self.__port.read(1)
        if len(res) == 1:
            val = struct.unpack('>b', res)
            self.__checksum = (val[0] + self.__checksum) & 0xFF
            return val[0]
        raise Exception('No signed-byte read')

    def read_word(self):
        res = self.__port.read(2)
        if res == '':
            self.__port.flushInput()
            res = self.__port.read(2)
        if len(res) == 2:
            val = struct.unpack('>H', res)
            self.__checksum = (val[0] + self.__checksum) & 0xFF
            self.__checksum = ((val[0] >> 8) + self.__checksum) & 0xFF
            return val[0]
        raise Exception('No word read')

    def read_sword(self):
        res = self.__port.read(2)
        if res == '':
            self.__port.flushInput()
            res = self.__port.read(2)
        if len(res) == 2:
            val = struct.unpack('>h', res)
            self.__checksum = (val[0] + self.__checksum) & 0xFF
            self.__checksum = ((val[0] >> 8) + self.__checksum) & 0xFF
            return val[0]
        raise Exception('No signed-word read')

    def read_long(self):
        res = self.__port.read(4)
        if res == '':
            self.__port.flushInput()
            res = self.__port.read(4)
        if len(res) == 4:
            val = struct.unpack('>L', res)
            self.__checksum = (val[0] + self.__checksum) & 0xFF
            self.__checksum = ((val[0] >> 8) + self.__checksum) & 0xFF
            self.__checksum = ((val[0] >> 16) + self.__checksum) & 0xFF
            self.__checksum = ((val[0] >> 24) + self.__checksum) & 0xFF
            return val[0]
        raise Exception('No long read')

    def read_slong(self):
        res = self.__port.read(4)
        if res == '':
            self.__port.flushInput()
            res = self.__port.read(4)
        if len(res) == 4:
            val = struct.unpack('>l', res)
            self.__checksum = (val[0] + self.__checksum) & 0xFF
            self.__checksum = ((val[0] >> 8) + self.__checksum) & 0xFF
            self.__checksum = ((val[0] >> 16) + self.__checksum) & 0xFF
            self.__checksum = ((val[0] >> 24) + self.__checksum) & 0xFF
            return val[0]
        raise Exception('No signed-long read')

    def write_byte(self, val):
        self.__checksum = (val + self.__checksum) & 0xFF
        try:
            return self.__port.write(struct.pack('>B', val))
        finally:
            self.__port.flush()

    def write_sbyte(self, val):
        self.__checksum = (val + self.__checksum) & 0xFF
        try:
            return self.__port.write(struct.pack('>b', val))
        finally:
            self.__port.flush()

    def write_word(self, val):
        self.__checksum = (val + self.__checksum) & 0xFF
        self.__checksum = ((val >> 8) + self.__checksum) & 0xFF
        try:
            return self.__port.write(struct.pack('>H', val))
        finally:
            self.__port.flush()

    def write_sword(self, val):
        self.__checksum = (val + self.__checksum) & 0xFF
        self.__checksum = ((val >> 8) + self.__checksum) & 0xFF
        try:
            return self.__port.write(struct.pack('>h', val))
        finally:
            self.__port.flush()

    def write_long(self, val):
        self.__checksum = (val + self.__checksum) & 0xFF
        self.__checksum = ((val >> 8) + self.__checksum) & 0xFF
        self.__checksum = ((val >> 16) + self.__checksum) & 0xFF
        self.__checksum = ((val >> 24) + self.__checksum) & 0xFF
        try:
            return self.__port.write(struct.pack('>L', val))
        finally:
            self.__port.flush()

    def write_slong(self, val):
        self.__checksum = (val + self.__checksum) & 0xFF
        self.__checksum = ((val >> 8) + self.__checksum) & 0xFF
        self.__checksum = ((val >> 16) + self.__checksum) & 0xFF
        self.__checksum = ((val >> 24) + self.__checksum) & 0xFF
        try:
            return self.__port.write(struct.pack('>l', val))
        finally:
            self.__port.flush()