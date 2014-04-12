# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: dummy.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)


import drivermsg_pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='dummy.proto',
  package='amber.dummy_proto',
  serialized_pb='\n\x0b\x64ummy.proto\x12\x11\x61mber.dummy_proto\x1a\x0f\x64rivermsg.proto\"\"\n\x0fSubscribeAction\x12\x0f\n\x07\x65nabled\x18\x01 \x01(\x08\"\x1c\n\x08Response\x12\x10\n\x08response\x18\x01 \x01(\t: \n\x06\x65nable\x12\x10.amber.DriverMsg\x18\x0b \x01(\x08:!\n\x07message\x12\x10.amber.DriverMsg\x18\x0c \x01(\t:?\n\x08response\x12\x10.amber.DriverMsg\x18\r \x01(\x0b\x32\x1b.amber.dummy_proto.Response:M\n\x0fsubscribeAction\x12\x10.amber.DriverMsg\x18\x0e \x01(\x0b\x32\".amber.dummy_proto.SubscribeAction:$\n\nget_status\x12\x10.amber.DriverMsg\x18\x0f \x01(\x08\x42*\n\x1cpl.edu.agh.amber.dummy.protoB\nDummyProto')


ENABLE_FIELD_NUMBER = 11
enable = _descriptor.FieldDescriptor(
  name='enable', full_name='amber.dummy_proto.enable', index=0,
  number=11, type=8, cpp_type=7, label=1,
  has_default_value=False, default_value=False,
  message_type=None, enum_type=None, containing_type=None,
  is_extension=True, extension_scope=None,
  options=None)
MESSAGE_FIELD_NUMBER = 12
message = _descriptor.FieldDescriptor(
  name='message', full_name='amber.dummy_proto.message', index=1,
  number=12, type=9, cpp_type=9, label=1,
  has_default_value=False, default_value=unicode("", "utf-8"),
  message_type=None, enum_type=None, containing_type=None,
  is_extension=True, extension_scope=None,
  options=None)
RESPONSE_FIELD_NUMBER = 13
response = _descriptor.FieldDescriptor(
  name='response', full_name='amber.dummy_proto.response', index=2,
  number=13, type=11, cpp_type=10, label=1,
  has_default_value=False, default_value=None,
  message_type=None, enum_type=None, containing_type=None,
  is_extension=True, extension_scope=None,
  options=None)
SUBSCRIBEACTION_FIELD_NUMBER = 14
subscribeAction = _descriptor.FieldDescriptor(
  name='subscribeAction', full_name='amber.dummy_proto.subscribeAction', index=3,
  number=14, type=11, cpp_type=10, label=1,
  has_default_value=False, default_value=None,
  message_type=None, enum_type=None, containing_type=None,
  is_extension=True, extension_scope=None,
  options=None)
GET_STATUS_FIELD_NUMBER = 15
get_status = _descriptor.FieldDescriptor(
  name='get_status', full_name='amber.dummy_proto.get_status', index=4,
  number=15, type=8, cpp_type=7, label=1,
  has_default_value=False, default_value=False,
  message_type=None, enum_type=None, containing_type=None,
  is_extension=True, extension_scope=None,
  options=None)


_SUBSCRIBEACTION = _descriptor.Descriptor(
  name='SubscribeAction',
  full_name='amber.dummy_proto.SubscribeAction',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='enabled', full_name='amber.dummy_proto.SubscribeAction.enabled', index=0,
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
  serialized_start=51,
  serialized_end=85,
)


_RESPONSE = _descriptor.Descriptor(
  name='Response',
  full_name='amber.dummy_proto.Response',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='response', full_name='amber.dummy_proto.Response.response', index=0,
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
  serialized_start=87,
  serialized_end=115,
)

DESCRIPTOR.message_types_by_name['SubscribeAction'] = _SUBSCRIBEACTION
DESCRIPTOR.message_types_by_name['Response'] = _RESPONSE

class SubscribeAction(_message.Message):
  __metaclass__ = _reflection.GeneratedProtocolMessageType
  DESCRIPTOR = _SUBSCRIBEACTION

  # @@protoc_insertion_point(class_scope:amber.dummy_proto.SubscribeAction)

class Response(_message.Message):
  __metaclass__ = _reflection.GeneratedProtocolMessageType
  DESCRIPTOR = _RESPONSE

  # @@protoc_insertion_point(class_scope:amber.dummy_proto.Response)

drivermsg_pb2.DriverMsg.RegisterExtension(enable)
drivermsg_pb2.DriverMsg.RegisterExtension(message)
response.message_type = _RESPONSE
drivermsg_pb2.DriverMsg.RegisterExtension(response)
subscribeAction.message_type = _SUBSCRIBEACTION
drivermsg_pb2.DriverMsg.RegisterExtension(subscribeAction)
drivermsg_pb2.DriverMsg.RegisterExtension(get_status)

DESCRIPTOR.has_options = True
DESCRIPTOR._options = _descriptor._ParseOptions(descriptor_pb2.FileOptions(), '\n\034pl.edu.agh.amber.dummy.protoB\nDummyProto')
# @@protoc_insertion_point(module_scope)
