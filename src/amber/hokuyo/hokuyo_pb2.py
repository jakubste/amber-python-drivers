# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: hokuyo.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)


from amber.common import drivermsg_pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='hokuyo.proto',
  package='amber.hokuyo_proto',
  serialized_pb='\n\x0chokuyo.proto\x12\x12\x61mber.hokuyo_proto\x1a\x0f\x64rivermsg.proto\"\"\n\x0fSubscribeAction\x12\x0f\n\x07\x65nabled\x18\x01 \x01(\x08\"\x1c\n\x08Response\x12\x10\n\x08response\x18\x01 \x01(\t\"p\n\x07Version\x12\x10\n\x08response\x18\x01 \x01(\t\x12\x0e\n\x06vendor\x18\x02 \x01(\t\x12\x0f\n\x07product\x18\x03 \x01(\t\x12\x10\n\x08\x66irmware\x18\x04 \x01(\t\x12\x10\n\x08protocol\x18\x05 \x01(\t\x12\x0e\n\x06serial\x18\x06 \x01(\t\"\xca\x01\n\x05Specs\x12\x10\n\x08response\x18\x01 \x01(\t\x12\r\n\x05model\x18\x02 \x01(\t\x12\x18\n\x10\x64istance_minimum\x18\x03 \x01(\r\x12\x18\n\x10\x64istance_maximum\x18\x04 \x01(\r\x12\x17\n\x0f\x61rea_resolution\x18\x05 \x01(\r\x12\x14\n\x0c\x61rea_minimum\x18\x06 \x01(\r\x12\x14\n\x0c\x61rea_maximum\x18\x07 \x01(\r\x12\x12\n\narea_front\x18\x08 \x01(\r\x12\x13\n\x0bmotor_speed\x18\t \x01(\r\"\x96\x01\n\x05State\x12\x10\n\x08response\x18\x01 \x01(\t\x12\r\n\x05model\x18\x02 \x01(\t\x12\r\n\x05laser\x18\x03 \x01(\x08\x12\x13\n\x0bmotor_speed\x18\x04 \x01(\t\x12\x14\n\x0cmeasure_mode\x18\x05 \x01(\t\x12\x10\n\x08\x62it_rate\x18\x06 \x01(\t\x12\x0c\n\x04time\x18\x07 \x01(\t\x12\x12\n\ndiagnostic\x18\x08 \x01(\t\"&\n\x04Scan\x12\x10\n\x08response\x18\x01 \x01(\t\x12\x0c\n\x04scan\x18\x02 \x01(\t:\"\n\x08laser_on\x12\x10.amber.DriverMsg\x18\x0b \x01(\x08:#\n\tlaser_off\x12\x10.amber.DriverMsg\x18\x0c \x01(\x08:\x1f\n\x05reset\x12\x10.amber.DriverMsg\x18\r \x01(\x08:)\n\x0fset_motor_speed\x12\x10.amber.DriverMsg\x18\x0e \x01(\x08:%\n\x0bmotor_speed\x12\x10.amber.DriverMsg\x18\x0f \x01(\r:,\n\x12set_high_sensitive\x12\x10.amber.DriverMsg\x18\x10 \x01(\x08:(\n\x0ehigh_sensitive\x12\x10.amber.DriverMsg\x18\x11 \x01(\x08:*\n\x10get_version_info\x12\x10.amber.DriverMsg\x18\x12 \x01(\x08:*\n\x10get_sensor_state\x12\x10.amber.DriverMsg\x18\x13 \x01(\x08:*\n\x10get_sensor_specs\x12\x10.amber.DriverMsg\x18\x14 \x01(\x08:)\n\x0fget_single_scan\x12\x10.amber.DriverMsg\x18\x15 \x01(\x08:$\n\nstart_step\x12\x10.amber.DriverMsg\x18\x16 \x01(\r:#\n\tstop_step\x12\x10.amber.DriverMsg\x18\x17 \x01(\r:\'\n\rcluster_count\x12\x10.amber.DriverMsg\x18\x18 \x01(\r:@\n\x08response\x12\x10.amber.DriverMsg\x18\x19 \x01(\x0b\x32\x1c.amber.hokuyo_proto.Response:>\n\x07version\x12\x10.amber.DriverMsg\x18\x1a \x01(\x0b\x32\x1b.amber.hokuyo_proto.Version::\n\x05specs\x12\x10.amber.DriverMsg\x18\x1b \x01(\x0b\x32\x19.amber.hokuyo_proto.Specs::\n\x05state\x12\x10.amber.DriverMsg\x18\x1c \x01(\x0b\x32\x19.amber.hokuyo_proto.State:8\n\x04scan\x12\x10.amber.DriverMsg\x18\x1d \x01(\x0b\x32\x18.amber.hokuyo_proto.Scan:N\n\x0fsubscribeAction\x12\x10.amber.DriverMsg\x18\x1e \x01(\x0b\x32#.amber.hokuyo_proto.SubscribeActionB,\n\x1dpl.edu.agh.amber.hokuyo.protoB\x0bHokuyoProto')


LASER_ON_FIELD_NUMBER = 11
laser_on = _descriptor.FieldDescriptor(
  name='laser_on', full_name='amber.hokuyo_proto.laser_on', index=0,
  number=11, type=8, cpp_type=7, label=1,
  has_default_value=False, default_value=False,
  message_type=None, enum_type=None, containing_type=None,
  is_extension=True, extension_scope=None,
  options=None)
LASER_OFF_FIELD_NUMBER = 12
laser_off = _descriptor.FieldDescriptor(
  name='laser_off', full_name='amber.hokuyo_proto.laser_off', index=1,
  number=12, type=8, cpp_type=7, label=1,
  has_default_value=False, default_value=False,
  message_type=None, enum_type=None, containing_type=None,
  is_extension=True, extension_scope=None,
  options=None)
RESET_FIELD_NUMBER = 13
reset = _descriptor.FieldDescriptor(
  name='reset', full_name='amber.hokuyo_proto.reset', index=2,
  number=13, type=8, cpp_type=7, label=1,
  has_default_value=False, default_value=False,
  message_type=None, enum_type=None, containing_type=None,
  is_extension=True, extension_scope=None,
  options=None)
SET_MOTOR_SPEED_FIELD_NUMBER = 14
set_motor_speed = _descriptor.FieldDescriptor(
  name='set_motor_speed', full_name='amber.hokuyo_proto.set_motor_speed', index=3,
  number=14, type=8, cpp_type=7, label=1,
  has_default_value=False, default_value=False,
  message_type=None, enum_type=None, containing_type=None,
  is_extension=True, extension_scope=None,
  options=None)
MOTOR_SPEED_FIELD_NUMBER = 15
motor_speed = _descriptor.FieldDescriptor(
  name='motor_speed', full_name='amber.hokuyo_proto.motor_speed', index=4,
  number=15, type=13, cpp_type=3, label=1,
  has_default_value=False, default_value=0,
  message_type=None, enum_type=None, containing_type=None,
  is_extension=True, extension_scope=None,
  options=None)
SET_HIGH_SENSITIVE_FIELD_NUMBER = 16
set_high_sensitive = _descriptor.FieldDescriptor(
  name='set_high_sensitive', full_name='amber.hokuyo_proto.set_high_sensitive', index=5,
  number=16, type=8, cpp_type=7, label=1,
  has_default_value=False, default_value=False,
  message_type=None, enum_type=None, containing_type=None,
  is_extension=True, extension_scope=None,
  options=None)
HIGH_SENSITIVE_FIELD_NUMBER = 17
high_sensitive = _descriptor.FieldDescriptor(
  name='high_sensitive', full_name='amber.hokuyo_proto.high_sensitive', index=6,
  number=17, type=8, cpp_type=7, label=1,
  has_default_value=False, default_value=False,
  message_type=None, enum_type=None, containing_type=None,
  is_extension=True, extension_scope=None,
  options=None)
GET_VERSION_INFO_FIELD_NUMBER = 18
get_version_info = _descriptor.FieldDescriptor(
  name='get_version_info', full_name='amber.hokuyo_proto.get_version_info', index=7,
  number=18, type=8, cpp_type=7, label=1,
  has_default_value=False, default_value=False,
  message_type=None, enum_type=None, containing_type=None,
  is_extension=True, extension_scope=None,
  options=None)
GET_SENSOR_STATE_FIELD_NUMBER = 19
get_sensor_state = _descriptor.FieldDescriptor(
  name='get_sensor_state', full_name='amber.hokuyo_proto.get_sensor_state', index=8,
  number=19, type=8, cpp_type=7, label=1,
  has_default_value=False, default_value=False,
  message_type=None, enum_type=None, containing_type=None,
  is_extension=True, extension_scope=None,
  options=None)
GET_SENSOR_SPECS_FIELD_NUMBER = 20
get_sensor_specs = _descriptor.FieldDescriptor(
  name='get_sensor_specs', full_name='amber.hokuyo_proto.get_sensor_specs', index=9,
  number=20, type=8, cpp_type=7, label=1,
  has_default_value=False, default_value=False,
  message_type=None, enum_type=None, containing_type=None,
  is_extension=True, extension_scope=None,
  options=None)
GET_SINGLE_SCAN_FIELD_NUMBER = 21
get_single_scan = _descriptor.FieldDescriptor(
  name='get_single_scan', full_name='amber.hokuyo_proto.get_single_scan', index=10,
  number=21, type=8, cpp_type=7, label=1,
  has_default_value=False, default_value=False,
  message_type=None, enum_type=None, containing_type=None,
  is_extension=True, extension_scope=None,
  options=None)
START_STEP_FIELD_NUMBER = 22
start_step = _descriptor.FieldDescriptor(
  name='start_step', full_name='amber.hokuyo_proto.start_step', index=11,
  number=22, type=13, cpp_type=3, label=1,
  has_default_value=False, default_value=0,
  message_type=None, enum_type=None, containing_type=None,
  is_extension=True, extension_scope=None,
  options=None)
STOP_STEP_FIELD_NUMBER = 23
stop_step = _descriptor.FieldDescriptor(
  name='stop_step', full_name='amber.hokuyo_proto.stop_step', index=12,
  number=23, type=13, cpp_type=3, label=1,
  has_default_value=False, default_value=0,
  message_type=None, enum_type=None, containing_type=None,
  is_extension=True, extension_scope=None,
  options=None)
CLUSTER_COUNT_FIELD_NUMBER = 24
cluster_count = _descriptor.FieldDescriptor(
  name='cluster_count', full_name='amber.hokuyo_proto.cluster_count', index=13,
  number=24, type=13, cpp_type=3, label=1,
  has_default_value=False, default_value=0,
  message_type=None, enum_type=None, containing_type=None,
  is_extension=True, extension_scope=None,
  options=None)
RESPONSE_FIELD_NUMBER = 25
response = _descriptor.FieldDescriptor(
  name='response', full_name='amber.hokuyo_proto.response', index=14,
  number=25, type=11, cpp_type=10, label=1,
  has_default_value=False, default_value=None,
  message_type=None, enum_type=None, containing_type=None,
  is_extension=True, extension_scope=None,
  options=None)
VERSION_FIELD_NUMBER = 26
version = _descriptor.FieldDescriptor(
  name='version', full_name='amber.hokuyo_proto.version', index=15,
  number=26, type=11, cpp_type=10, label=1,
  has_default_value=False, default_value=None,
  message_type=None, enum_type=None, containing_type=None,
  is_extension=True, extension_scope=None,
  options=None)
SPECS_FIELD_NUMBER = 27
specs = _descriptor.FieldDescriptor(
  name='specs', full_name='amber.hokuyo_proto.specs', index=16,
  number=27, type=11, cpp_type=10, label=1,
  has_default_value=False, default_value=None,
  message_type=None, enum_type=None, containing_type=None,
  is_extension=True, extension_scope=None,
  options=None)
STATE_FIELD_NUMBER = 28
state = _descriptor.FieldDescriptor(
  name='state', full_name='amber.hokuyo_proto.state', index=17,
  number=28, type=11, cpp_type=10, label=1,
  has_default_value=False, default_value=None,
  message_type=None, enum_type=None, containing_type=None,
  is_extension=True, extension_scope=None,
  options=None)
SCAN_FIELD_NUMBER = 29
scan = _descriptor.FieldDescriptor(
  name='scan', full_name='amber.hokuyo_proto.scan', index=18,
  number=29, type=11, cpp_type=10, label=1,
  has_default_value=False, default_value=None,
  message_type=None, enum_type=None, containing_type=None,
  is_extension=True, extension_scope=None,
  options=None)
SUBSCRIBEACTION_FIELD_NUMBER = 30
subscribeAction = _descriptor.FieldDescriptor(
  name='subscribeAction', full_name='amber.hokuyo_proto.subscribeAction', index=19,
  number=30, type=11, cpp_type=10, label=1,
  has_default_value=False, default_value=None,
  message_type=None, enum_type=None, containing_type=None,
  is_extension=True, extension_scope=None,
  options=None)


_SUBSCRIBEACTION = _descriptor.Descriptor(
  name='SubscribeAction',
  full_name='amber.hokuyo_proto.SubscribeAction',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='enabled', full_name='amber.hokuyo_proto.SubscribeAction.enabled', index=0,
      number=1, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  extension_ranges=[],
  serialized_start=53,
  serialized_end=87,
)


_RESPONSE = _descriptor.Descriptor(
  name='Response',
  full_name='amber.hokuyo_proto.Response',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='response', full_name='amber.hokuyo_proto.Response.response', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=unicode("", "utf-8"),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  extension_ranges=[],
  serialized_start=89,
  serialized_end=117,
)


_VERSION = _descriptor.Descriptor(
  name='Version',
  full_name='amber.hokuyo_proto.Version',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='response', full_name='amber.hokuyo_proto.Version.response', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=unicode("", "utf-8"),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='vendor', full_name='amber.hokuyo_proto.Version.vendor', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=unicode("", "utf-8"),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='product', full_name='amber.hokuyo_proto.Version.product', index=2,
      number=3, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=unicode("", "utf-8"),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='firmware', full_name='amber.hokuyo_proto.Version.firmware', index=3,
      number=4, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=unicode("", "utf-8"),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='protocol', full_name='amber.hokuyo_proto.Version.protocol', index=4,
      number=5, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=unicode("", "utf-8"),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='serial', full_name='amber.hokuyo_proto.Version.serial', index=5,
      number=6, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=unicode("", "utf-8"),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  extension_ranges=[],
  serialized_start=119,
  serialized_end=231,
)


_SPECS = _descriptor.Descriptor(
  name='Specs',
  full_name='amber.hokuyo_proto.Specs',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='response', full_name='amber.hokuyo_proto.Specs.response', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=unicode("", "utf-8"),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='model', full_name='amber.hokuyo_proto.Specs.model', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=unicode("", "utf-8"),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='distance_minimum', full_name='amber.hokuyo_proto.Specs.distance_minimum', index=2,
      number=3, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='distance_maximum', full_name='amber.hokuyo_proto.Specs.distance_maximum', index=3,
      number=4, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='area_resolution', full_name='amber.hokuyo_proto.Specs.area_resolution', index=4,
      number=5, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='area_minimum', full_name='amber.hokuyo_proto.Specs.area_minimum', index=5,
      number=6, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='area_maximum', full_name='amber.hokuyo_proto.Specs.area_maximum', index=6,
      number=7, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='area_front', full_name='amber.hokuyo_proto.Specs.area_front', index=7,
      number=8, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='motor_speed', full_name='amber.hokuyo_proto.Specs.motor_speed', index=8,
      number=9, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  extension_ranges=[],
  serialized_start=234,
  serialized_end=436,
)


_STATE = _descriptor.Descriptor(
  name='State',
  full_name='amber.hokuyo_proto.State',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='response', full_name='amber.hokuyo_proto.State.response', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=unicode("", "utf-8"),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='model', full_name='amber.hokuyo_proto.State.model', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=unicode("", "utf-8"),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='laser', full_name='amber.hokuyo_proto.State.laser', index=2,
      number=3, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='motor_speed', full_name='amber.hokuyo_proto.State.motor_speed', index=3,
      number=4, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=unicode("", "utf-8"),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='measure_mode', full_name='amber.hokuyo_proto.State.measure_mode', index=4,
      number=5, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=unicode("", "utf-8"),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='bit_rate', full_name='amber.hokuyo_proto.State.bit_rate', index=5,
      number=6, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=unicode("", "utf-8"),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='time', full_name='amber.hokuyo_proto.State.time', index=6,
      number=7, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=unicode("", "utf-8"),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='diagnostic', full_name='amber.hokuyo_proto.State.diagnostic', index=7,
      number=8, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=unicode("", "utf-8"),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  extension_ranges=[],
  serialized_start=439,
  serialized_end=589,
)


_SCAN = _descriptor.Descriptor(
  name='Scan',
  full_name='amber.hokuyo_proto.Scan',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='response', full_name='amber.hokuyo_proto.Scan.response', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=unicode("", "utf-8"),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='scan', full_name='amber.hokuyo_proto.Scan.scan', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=unicode("", "utf-8"),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  extension_ranges=[],
  serialized_start=591,
  serialized_end=629,
)

DESCRIPTOR.message_types_by_name['SubscribeAction'] = _SUBSCRIBEACTION
DESCRIPTOR.message_types_by_name['Response'] = _RESPONSE
DESCRIPTOR.message_types_by_name['Version'] = _VERSION
DESCRIPTOR.message_types_by_name['Specs'] = _SPECS
DESCRIPTOR.message_types_by_name['State'] = _STATE
DESCRIPTOR.message_types_by_name['Scan'] = _SCAN

class SubscribeAction(_message.Message):
  __metaclass__ = _reflection.GeneratedProtocolMessageType
  DESCRIPTOR = _SUBSCRIBEACTION

  # @@protoc_insertion_point(class_scope:amber.hokuyo_proto.SubscribeAction)

class Response(_message.Message):
  __metaclass__ = _reflection.GeneratedProtocolMessageType
  DESCRIPTOR = _RESPONSE

  # @@protoc_insertion_point(class_scope:amber.hokuyo_proto.Response)

class Version(_message.Message):
  __metaclass__ = _reflection.GeneratedProtocolMessageType
  DESCRIPTOR = _VERSION

  # @@protoc_insertion_point(class_scope:amber.hokuyo_proto.Version)

class Specs(_message.Message):
  __metaclass__ = _reflection.GeneratedProtocolMessageType
  DESCRIPTOR = _SPECS

  # @@protoc_insertion_point(class_scope:amber.hokuyo_proto.Specs)

class State(_message.Message):
  __metaclass__ = _reflection.GeneratedProtocolMessageType
  DESCRIPTOR = _STATE

  # @@protoc_insertion_point(class_scope:amber.hokuyo_proto.State)

class Scan(_message.Message):
  __metaclass__ = _reflection.GeneratedProtocolMessageType
  DESCRIPTOR = _SCAN

  # @@protoc_insertion_point(class_scope:amber.hokuyo_proto.Scan)

drivermsg_pb2.DriverMsg.RegisterExtension(laser_on)
drivermsg_pb2.DriverMsg.RegisterExtension(laser_off)
drivermsg_pb2.DriverMsg.RegisterExtension(reset)
drivermsg_pb2.DriverMsg.RegisterExtension(set_motor_speed)
drivermsg_pb2.DriverMsg.RegisterExtension(motor_speed)
drivermsg_pb2.DriverMsg.RegisterExtension(set_high_sensitive)
drivermsg_pb2.DriverMsg.RegisterExtension(high_sensitive)
drivermsg_pb2.DriverMsg.RegisterExtension(get_version_info)
drivermsg_pb2.DriverMsg.RegisterExtension(get_sensor_state)
drivermsg_pb2.DriverMsg.RegisterExtension(get_sensor_specs)
drivermsg_pb2.DriverMsg.RegisterExtension(get_single_scan)
drivermsg_pb2.DriverMsg.RegisterExtension(start_step)
drivermsg_pb2.DriverMsg.RegisterExtension(stop_step)
drivermsg_pb2.DriverMsg.RegisterExtension(cluster_count)
response.message_type = _RESPONSE
drivermsg_pb2.DriverMsg.RegisterExtension(response)
version.message_type = _VERSION
drivermsg_pb2.DriverMsg.RegisterExtension(version)
specs.message_type = _SPECS
drivermsg_pb2.DriverMsg.RegisterExtension(specs)
state.message_type = _STATE
drivermsg_pb2.DriverMsg.RegisterExtension(state)
scan.message_type = _SCAN
drivermsg_pb2.DriverMsg.RegisterExtension(scan)
subscribeAction.message_type = _SUBSCRIBEACTION
drivermsg_pb2.DriverMsg.RegisterExtension(subscribeAction)

DESCRIPTOR.has_options = True
DESCRIPTOR._options = _descriptor._ParseOptions(descriptor_pb2.FileOptions(), '\n\035pl.edu.agh.amber.hokuyo.protoB\013HokuyoProto')
# @@protoc_insertion_point(module_scope)
