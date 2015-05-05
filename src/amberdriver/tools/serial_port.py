import struct
import sys

__author__ = 'paoolo'


class SerialPort(object):
    def __init__(self, serial_port):
        self.__port = serial_port
        self.__checksum = 0

    def close(self):
        self.__port.close()

    def reset_port(self):
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

    def send_command(self, address, command):
        self.write_byte(address)
        self.write_byte(command)

    def read_byte(self):
        res = self.__port.read(1)
        if len(res) == 1:
            val = struct.unpack('>B', res)
            self.__checksum = (val[0] + self.__checksum) & 0xFF
            return val[0]
        return None

    def read_sbyte(self):
        res = self.__port.read(1)
        if len(res) == 1:
            val = struct.unpack('>b', res)
            self.__checksum = (val[0] + self.__checksum) & 0xFF
            return val[0]
        return None

    def read_word(self):
        res = self.__port.read(2)
        if len(res) == 2:
            val = struct.unpack('>H', res)
            self.__checksum = (val[0] + self.__checksum) & 0xFF
            self.__checksum = ((val[0] >> 8) + self.__checksum) & 0xFF
            return val[0]
        return None

    def read_sword(self):
        res = self.__port.read(2)
        if len(res) == 2:
            val = struct.unpack('>h', res)
            self.__checksum = (val[0] + self.__checksum) & 0xFF
            self.__checksum = ((val[0] >> 8) + self.__checksum) & 0xFF
            return val[0]
        return None

    def read_long(self):
        res = self.__port.read(4)
        if len(res) == 4:
            val = struct.unpack('>L', res)
            self.__checksum = (val[0] + self.__checksum) & 0xFF
            self.__checksum = ((val[0] >> 8) + self.__checksum) & 0xFF
            self.__checksum = ((val[0] >> 16) + self.__checksum) & 0xFF
            self.__checksum = ((val[0] >> 24) + self.__checksum) & 0xFF
            return val[0]
        return None

    def read_slong(self):
        res = self.__port.read(4)
        if len(res) == 4:
            val = struct.unpack('>l', res)
            self.__checksum = (val[0] + self.__checksum) & 0xFF
            self.__checksum = ((val[0] >> 8) + self.__checksum) & 0xFF
            self.__checksum = ((val[0] >> 16) + self.__checksum) & 0xFF
            self.__checksum = ((val[0] >> 24) + self.__checksum) & 0xFF
            return val[0]
        return None

    def write_byte(self, val):
        self.__checksum = (val + self.__checksum) & 0xFF
        return self.__port.write(struct.pack('>B', val))

    def write_sbyte(self, val):
        self.__checksum = (val + self.__checksum) & 0xFF
        return self.__port.write(struct.pack('>b', val))

    def write_word(self, val):
        self.__checksum = (val + self.__checksum) & 0xFF
        self.__checksum = ((val >> 8) + self.__checksum) & 0xFF
        return self.__port.write(struct.pack('>H', val))

    def write_sword(self, val):
        self.__checksum = (val + self.__checksum) & 0xFF
        self.__checksum = ((val >> 8) + self.__checksum) & 0xFF
        return self.__port.write(struct.pack('>h', val))

    def write_long(self, val):
        self.__checksum = (val + self.__checksum) & 0xFF
        self.__checksum = ((val >> 8) + self.__checksum) & 0xFF
        self.__checksum = ((val >> 16) + self.__checksum) & 0xFF
        self.__checksum = ((val >> 24) + self.__checksum) & 0xFF
        return self.__port.write(struct.pack('>L', val))

    def write_slong(self, val):
        self.__checksum = (val + self.__checksum) & 0xFF
        self.__checksum = ((val >> 8) + self.__checksum) & 0xFF
        self.__checksum = ((val >> 16) + self.__checksum) & 0xFF
        self.__checksum = ((val >> 24) + self.__checksum) & 0xFF
        return self.__port.write(struct.pack('>l', val))