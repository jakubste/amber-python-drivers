import struct
import unittest

import mock

from amberdriver.tools import serial_port


__author__ = 'paoolo'


class SerialPortTestCase(unittest.TestCase):
    def setUp(self):
        self.mocked_serial_port = mock.Mock()
        self.port = serial_port.SerialPort(self.mocked_serial_port)

    def tearDown(self):
        pass


class CloseTestCase(SerialPortTestCase):
    def runTest(self):
        self.port.close()
        self.mocked_serial_port.close.assert_called_once_with()


class ReadTestCase(SerialPortTestCase):
    def runTest(self):
        size = mock.Mock()
        self.port.read(size)
        self.mocked_serial_port.read.assert_called_once_with(size)


class WriteTestCase(SerialPortTestCase):
    def runTest(self):
        char = mock.Mock()
        self.port.write(char)
        self.mocked_serial_port.write.assert_called_once_with(char)


class SendCommandTestCase(SerialPortTestCase):
    def runTest(self):
        addresses = range(128, 136)
        commands = range(0, 128)
        for address in addresses:
            for command in commands:
                calls = [mock.call.write(struct.pack('>B', address)), mock.call.write(struct.pack('>B', command))]
                self.port.reset_checksum()
                self.port.send_command(address, command)
                self.mocked_serial_port.write.assert_has_calls(calls)
                checksum = ((address & 0xFF) + (command & 0xFF)) & 0x7F
                self.assertEqual(checksum, self.port.get_checksum())


class ReadByteTestCase(SerialPortTestCase):
    def runTest(self):
        # TODO(paoolo): check how to mock string
        values = range(0, 256)
        for value in values:
            value = struct.pack('>B', value)
            self.mocked_serial_port.read = mock.Mock(return_value=value)
            self.port.reset_checksum()
            result = struct.unpack('>B', value)
            result = result[0]
            self.assertEqual(self.port.read_byte(), result)
            self.mocked_serial_port.read.assert_called_once_with(len(value))
            checksum = (result & 0xFF) & 0x7F
            self.assertEqual(self.port.get_checksum(), checksum)
            self.mocked_serial_port.read = mock.Mock(return_value=str())
            self.assertIsNone(self.port.read_byte())


class ReadSignedByteTestCase(SerialPortTestCase):
    def runTest(self):
        values = range(-128, 128)
        for value in values:
            value = struct.pack('>b', value)
            self.mocked_serial_port.read = mock.Mock(return_value=value)
            self.port.reset_checksum()
            result = struct.unpack('>b', value)
            result = result[0]
            self.assertEqual(self.port.read_sbyte(), result)
            self.mocked_serial_port.read.assert_called_once_with(len(value))
            checksum = (result & 0xFF) & 0x7F
            self.assertEqual(self.port.get_checksum(), checksum)
            self.mocked_serial_port.read = mock.Mock(return_value=str())
            self.assertIsNone(self.port.read_sbyte())


class ReadWordTestCase(SerialPortTestCase):
    def runTest(self):
        values = range(0, 256 * 256, 256)
        for value in values:
            value = struct.pack('>H', value)
            self.mocked_serial_port.read = mock.Mock(return_value=value)
            self.port.reset_checksum()
            result = struct.unpack('>H', value)
            result = result[0]
            self.assertEqual(self.port.read_word(), result)
            self.mocked_serial_port.read.assert_called_once_with(len(value))
            checksum = ((result & 0XFF) + ((result >> 8) & 0xFF)) & 0x7F
            self.assertEqual(self.port.get_checksum(), checksum)
            self.mocked_serial_port.read = mock.Mock(return_value=str())
            self.assertIsNone(self.port.read_word())


class ReadSignedWordTestCase(SerialPortTestCase):
    def runTest(self):
        values = range(-256 * 256 / 2, 256 * 256 / 2, 256)
        for value in values:
            value = struct.pack('>h', value)
            self.mocked_serial_port.read = mock.Mock(return_value=value)
            self.port.reset_checksum()
            result = struct.unpack('>h', value)
            result = result[0]
            self.assertEqual(self.port.read_sword(), result)
            self.mocked_serial_port.read.assert_called_once_with(len(value))
            # TODO(paoolo): check signed checksum computation
            checksum = ((result & 0xFF) + ((result >> 8) & 0xFF)) & 0x7F
            self.assertEqual(self.port.get_checksum(), checksum)
            self.mocked_serial_port.read = mock.Mock(return_value=str())
            self.assertIsNone(self.port.read_sword())


class ReadLongTestCase(SerialPortTestCase):
    def runTest(self):
        values = range(0, 256 ** 4, 256 ** 3)
        for value in values:
            value = struct.pack('>L', value)
            self.mocked_serial_port.read = mock.Mock(return_value=value)
            self.port.reset_checksum()
            result = struct.unpack('>L', value)
            result = result[0]
            self.assertEqual(self.port.read_long(), result)
            self.mocked_serial_port.read.assert_called_once_with(len(value))
            checksum = ((result & 0xFF) + ((result >> 8) & 0xFF) +
                        ((result >> 16) & 0xFF) + ((result >> 24) & 0xFF)) & 0x7F
            self.assertEqual(self.port.get_checksum(), checksum)
            self.mocked_serial_port.read = mock.Mock(return_value=str())
            self.assertIsNone(self.port.read_long())


class ReadSignedLongTestCase(SerialPortTestCase):
    def runTest(self):
        values = range(-(256 ** 4) / 2, 256 ** 4 / 2, 256 ** 3)
        for value in values:
            value = struct.pack('>l', value)
            self.mocked_serial_port.read = mock.Mock(return_value=value)
            self.port.reset_checksum()
            result = struct.unpack('>l', value)
            result = result[0]
            self.assertEqual(self.port.read_slong(), result)
            self.mocked_serial_port.read.assert_called_once_with(len(value))
            checksum = ((result & 0xFF) + ((result >> 8) & 0xFF) +
                        ((result >> 16) & 0xFF) + ((result >> 24) & 0xFF)) & 0x7F
            self.assertEqual(self.port.get_checksum(), checksum)
            self.mocked_serial_port.read = mock.Mock(return_value=str())
            self.assertIsNone(self.port.read_slong())


class WriteByteTestCase(SerialPortTestCase):
    def runTest(self):
        values = range(0, 256)
        for value in values:
            packed_value = struct.pack('>B', value)
            self.mocked_serial_port.write = mock.Mock(return_value=len(packed_value))
            self.port.reset_checksum()
            self.assertEqual(self.port.write_byte(value), len(packed_value))
            self.mocked_serial_port.write.assert_called_once_with(packed_value)
            checksum = (value & 0xFF) & 0x7F
            self.assertEqual(self.port.get_checksum(), checksum)


class WriteSignedByteTestCase(SerialPortTestCase):
    def runTest(self):
        values = range(-128, 128)
        for value in values:
            packed_value = struct.pack('>b', value)
            self.mocked_serial_port.write = mock.Mock(return_value=len(packed_value))
            self.port.reset_checksum()
            self.assertEqual(self.port.write_sbyte(value), len(packed_value))
            self.mocked_serial_port.write.assert_called_once_with(packed_value)
            checksum = (value & 0xFF) & 0x7F
            self.assertEqual(self.port.get_checksum(), checksum)


class WriteWordTestCase(SerialPortTestCase):
    def runTest(self):
        values = range(0, 256 * 256, 256)
        for value in values:
            packed_value = struct.pack('>H', value)
            self.mocked_serial_port.write = mock.Mock(return_value=len(packed_value))
            self.port.reset_checksum()
            self.assertEqual(self.port.write_word(value), len(packed_value))
            self.mocked_serial_port.write.assert_called_once_with(packed_value)
            checksum = ((value & 0xFF) + ((value >> 8) & 0xFF)) & 0x7F
            self.assertEqual(self.port.get_checksum(), checksum)


class WriteSignedWordTestCase(SerialPortTestCase):
    def runTest(self):
        values = range(-256 * 256 / 2, 256 * 256 / 2, 256)
        for value in values:
            packed_value = struct.pack('>h', value)
            self.mocked_serial_port.write = mock.Mock(return_value=len(packed_value))
            self.port.reset_checksum()
            self.assertEqual(self.port.write_sword(value), len(packed_value))
            self.mocked_serial_port.write.assert_called_once_with(packed_value)
            checksum = ((value & 0xFF) + ((value >> 8) & 0xFF)) & 0x7F
            self.assertEqual(self.port.get_checksum(), checksum)


class WriteLongTestCase(SerialPortTestCase):
    def runTest(self):
        values = range(0, 256 ** 4, 256 ** 3)
        for value in values:
            packed_value = struct.pack('>L', value)
            self.mocked_serial_port.write = mock.Mock(return_value=len(packed_value))
            self.port.reset_checksum()
            self.assertEqual(self.port.write_long(value), len(packed_value))
            self.mocked_serial_port.write.assert_called_once_with(packed_value)
            checksum = ((value & 0xFF) + ((value >> 8) & 0xFF) + ((value >> 16) & 0xFF) + ((value >> 24) & 0xFF)) & 0x7F
            self.assertEqual(self.port.get_checksum(), checksum)


class WriteSignedLongTestCase(SerialPortTestCase):
    def runTest(self):
        values = range(-(256 ** 4) / 2, 256 ** 4 / 2, 256 ** 3)
        for value in values:
            packed_value = struct.pack('>l', value)
            self.mocked_serial_port.write = mock.Mock(return_value=len(packed_value))
            self.port.reset_checksum()
            self.assertEqual(self.port.write_slong(value), len(packed_value))
            self.mocked_serial_port.write.assert_called_once_with(packed_value)
            checksum = ((value & 0xFF) + ((value >> 8) & 0xFF) + ((value >> 16) & 0xFF) + ((value >> 24) & 0xFF)) & 0x7F
            self.assertEqual(self.port.get_checksum(), checksum)