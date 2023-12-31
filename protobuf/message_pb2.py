# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: message.proto
"""Generated protocol buffer code."""
from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\rmessage.proto\"H\n\x0cPacketHeader\x12\x12\n\nsystem_uid\x18\x01 \x01(\r\x12\x15\n\rms_from_start\x18\x02 \x01(\r\x12\r\n\x05\x65poch\x18\x03 \x01(\x04\"\x85\x01\n\x13SimpleSensorReading\x12\r\n\x05index\x18\x01 \x01(\r\x12\x16\n\x0etimestamp_unix\x18\x02 \x01(\r\x12\x13\n\x0btemperature\x18\x03 \x01(\x02\x12\x10\n\x08humidity\x18\x04 \x01(\x02\x12\x0b\n\x03\x63o2\x18\x05 \x01(\x02\x12\x13\n\x0blight_level\x18\x06 \x01(\x02\"\xb8\x02\n\rSensorReading\x12\x14\n\x0cpacket_index\x18\x01 \x01(\r\x12\x15\n\rsample_period\x18\x02 \x01(\r\x12\'\n\x07payload\x18\x03 \x03(\x0b\x32\x16.SensorReading.Payload\x1a\xd0\x01\n\x07Payload\x12\x18\n\x10timestamp_sensor\x18\x01 \x01(\x06\x12\x16\n\x0etimestamp_unix\x18\x02 \x01(\x04\x12\x1f\n\x17timestamp_ms_from_start\x18\x03 \x01(\r\x12\x0e\n\x06signal\x18\x04 \x01(\x02\x12\x19\n\x11signal_dimensions\x18\x05 \x01(\r\x12$\n\tsensor_id\x18\x06 \x01(\x0e\x32\x11.SignalIdentifier\x12!\n\x08\x61\x63\x63uracy\x18\x07 \x01(\x0e\x32\x0f.SensorAccuracy\"q\n\x0cSensorConfig\x12\x18\n\x10sample_period_ms\x18\x01 \x01(\r\x12\x1a\n\x12\x65nable_temperature\x18\x02 \x01(\x08\x12\x17\n\x0f\x65nable_humidity\x18\x03 \x01(\x08\x12\x12\n\nenable_gas\x18\x04 \x01(\x08\"d\n\x0bSDCardState\x12\x10\n\x08\x64\x65tected\x18\x01 \x01(\x08\x12\x17\n\x0fspace_remaining\x18\x02 \x01(\x04\x12*\n\"estimated_remaining_recording_time\x18\x03 \x01(\x04\"N\n\tMarkState\x12\x14\n\x0c\x62\x65\x65p_enabled\x18\x01 \x01(\x08\x12\x13\n\x0bmark_number\x18\x02 \x01(\r\x12\x16\n\x0etimestamp_unix\x18\x03 \x01(\x04\"4\n\nMarkPacket\x12\x17\n\nannotation\x18\x01 \x01(\tH\x00\x88\x01\x01\x42\r\n\x0b_annotation\"\x7f\n\x11\x44iscoveredDevices\x12\x19\n\x11number_of_devices\x18\x01 \x01(\r\x12)\n\x06\x64\x65vice\x18\x02 \x03(\x0b\x32\x19.DiscoveredDevices.Device\x1a$\n\x06\x44\x65vice\x12\x0b\n\x03UID\x18\x01 \x01(\r\x12\r\n\x05range\x18\x02 \x01(\x02\"Y\n\x0c\x42\x61tteryState\x12\x10\n\x08\x63harging\x18\x01 \x01(\x08\x12\x0f\n\x07voltage\x18\x02 \x01(\x02\x12\x17\n\npercentage\x18\x03 \x01(\x02H\x00\x88\x01\x01\x42\r\n\x0b_percentage\"\xba\x02\n\x10SystemInfoPacket\x12!\n\x19number_discovered_devices\x18\x01 \x01(\r\x12\x33\n\x15simple_sensor_reading\x18\x02 \x01(\x0b\x32\x14.SimpleSensorReading\x12\x18\n\x10\x64\x65vice_recording\x18\x03 \x01(\x08\x12\"\n\x0csdcard_state\x18\x04 \x01(\x0b\x32\x0c.SDCardState\x12\x1e\n\nmark_state\x18\x05 \x01(\x0b\x32\n.MarkState\x12$\n\rbattery_state\x18\x06 \x01(\x0b\x32\r.BatteryState\x12\x33\n\x12\x64iscovered_devices\x18\x07 \x01(\x0b\x32\x12.DiscoveredDevicesH\x00\x88\x01\x01\x42\x15\n\x13_discovered_devices\"k\n\x10\x41udioCompression\x12\x0f\n\x07\x65nabled\x18\x01 \x01(\x08\x12*\n\x10\x63ompression_type\x18\x02 \x01(\x0e\x32\x10.CompressionType\x12\x1a\n\x12\x63ompression_factor\x18\x03 \x01(\r\"\xd0\x01\n\x0b\x41udioConfig\x12\x11\n\tchannel_1\x18\x01 \x01(\x08\x12\x11\n\tchannel_2\x18\x02 \x01(\x08\x12#\n\x0bsample_freq\x18\x03 \x01(\x0e\x32\x0e.MicSampleFreq\x12)\n\x0e\x62it_resolution\x18\x04 \x01(\x0e\x32\x11.MicBitResolution\x12,\n\x11\x61udio_compression\x18\x05 \x01(\x0b\x32\x11.AudioCompression\x12\x1d\n\x15\x65stimated_record_time\x18\x06 \x01(\x02\"\xda\x01\n\x0eScheduleConfig\x12\x0e\n\x06Sunday\x18\x01 \x01(\x08\x12\x0e\n\x06Monday\x18\x02 \x01(\x08\x12\x0f\n\x07Tuesday\x18\x03 \x01(\x08\x12\x11\n\tWednesday\x18\x04 \x01(\x08\x12\x10\n\x08Thursday\x18\x05 \x01(\x08\x12\x0e\n\x06\x46riday\x18\x06 \x01(\x08\x12\x10\n\x08Saturday\x18\x07 \x01(\x08\x12\x12\n\nstart_hour\x18\x08 \x01(\r\x12\x14\n\x0cstart_minute\x18\t \x01(\r\x12\x11\n\tstop_hour\x18\n \x01(\r\x12\x13\n\x0bstop_minute\x18\x0b \x01(\r\"&\n\x0eLowPowerConfig\x12\x14\n\x0clowPowerMode\x18\x01 \x01(\x08\"Z\n\rCameraControl\x12 \n\x18pair_with_nearby_cameras\x18\x01 \x01(\x08\x12\x16\n\x0ewakeup_cameras\x18\x02 \x01(\x08\x12\x0f\n\x07\x63\x61pture\x18\x03 \x01(\x08\"n\n\x0cNetworkState\x12$\n\x1cnumber_of_discovered_devices\x18\x01 \x01(\r\x12\x1d\n\x15\x64iscovered_device_UID\x18\x02 \x03(\r\x12\x19\n\x11\x66orce_rediscovery\x18\x03 \x01(\x08\"\x95\x02\n\x0c\x43onfigPacket\x12\"\n\x0c\x61udio_config\x18\x01 \x01(\x0b\x32\x0c.AudioConfig\x12(\n\x0fschedule_config\x18\x02 \x03(\x0b\x32\x0f.ScheduleConfig\x12$\n\rsensor_config\x18\x03 \x01(\x0b\x32\r.SensorConfig\x12)\n\x10low_power_config\x18\x04 \x01(\x0b\x32\x0f.LowPowerConfig\x12&\n\x0e\x63\x61mera_control\x18\x05 \x01(\x0b\x32\x0e.CameraControl\x12$\n\rnetwork_state\x18\x06 \x01(\x0b\x32\r.NetworkState\x12\x18\n\x10\x65nable_recording\x18\x07 \x01(\x08\"\xa3\x01\n\x0fSpecialFunction\x12\x17\n\rformat_sdcard\x18\x01 \x01(\x08H\x00\x12\x14\n\nfunction_2\x18\x02 \x01(\x08H\x00\x12\x14\n\nfunction_3\x18\x03 \x01(\x08H\x00\x12\x14\n\nfunction_4\x18\x04 \x01(\x08H\x00\x12\x14\n\nfunction_5\x18\x05 \x01(\x08H\x00\x12\x14\n\nfunction_6\x18\x06 \x01(\x08H\x00\x42\t\n\x07payload\"\xdd\x01\n\x06Packet\x12\x1d\n\x06header\x18\x01 \x01(\x0b\x32\r.PacketHeader\x12/\n\x12system_info_packet\x18\x02 \x01(\x0b\x32\x11.SystemInfoPacketH\x00\x12\"\n\x0bmark_packet\x18\x03 \x01(\x0b\x32\x0b.MarkPacketH\x00\x12&\n\rconfig_packet\x18\x04 \x01(\x0b\x32\r.ConfigPacketH\x00\x12,\n\x10special_function\x18\x05 \x01(\x0b\x32\x10.SpecialFunctionH\x00\x42\t\n\x07payload*\x94\x02\n\x10SignalIdentifier\x12\r\n\tUNDEFINED\x10\x00\x12\x07\n\x03IAQ\x10\x01\x12\x0e\n\nSTATIC_IAQ\x10\x02\x12\n\n\x06\x43O2_EQ\x10\x03\x12\x11\n\rBREATH_VOC_EQ\x10\x04\x12\x13\n\x0fRAW_TEMPERATURE\x10\x06\x12\x10\n\x0cRAW_PRESSURE\x10\x07\x12\x10\n\x0cRAW_HUMIDITY\x10\x08\x12\x0b\n\x07RAW_GAS\x10\t\x12\x18\n\x14STABILIZATION_STATUS\x10\x0c\x12\x11\n\rRUN_IN_STATUS\x10\r\x12\x1b\n\x17SENSOR_HEAT_COMPEN_TEMP\x10\x0e\x12\x15\n\x11HEAT_COMPEN_HUMID\x10\x0f\x12\x12\n\x0eGAS_PERCENTAGE\x10\x15*Z\n\x0eSensorAccuracy\x12\x0e\n\nUNRELIABLE\x10\x00\x12\x10\n\x0cLOW_ACCURACY\x10\x01\x12\x13\n\x0fMEDIUM_ACCURACY\x10\x02\x12\x11\n\rHIGH_ACCURACY\x10\x03*\x82\x01\n\rMicSampleFreq\x12\x15\n\x11SAMPLE_RATE_16000\x10\x00\x12\x15\n\x11SAMPLE_RATE_20500\x10\x01\x12\x15\n\x11SAMPLE_RATE_44100\x10\x02\x12\x15\n\x11SAMPLE_RATE_48000\x10\x03\x12\x15\n\x11SAMPLE_RATE_96000\x10\x04*1\n\x10MicBitResolution\x12\r\n\tBIT_RES_8\x10\x00\x12\x0e\n\nBIT_RES_16\x10\x01*\x1b\n\x0f\x43ompressionType\x12\x08\n\x04OPUS\x10\x00\x62\x06proto3')

_SIGNALIDENTIFIER = DESCRIPTOR.enum_types_by_name['SignalIdentifier']
SignalIdentifier = enum_type_wrapper.EnumTypeWrapper(_SIGNALIDENTIFIER)
_SENSORACCURACY = DESCRIPTOR.enum_types_by_name['SensorAccuracy']
SensorAccuracy = enum_type_wrapper.EnumTypeWrapper(_SENSORACCURACY)
_MICSAMPLEFREQ = DESCRIPTOR.enum_types_by_name['MicSampleFreq']
MicSampleFreq = enum_type_wrapper.EnumTypeWrapper(_MICSAMPLEFREQ)
_MICBITRESOLUTION = DESCRIPTOR.enum_types_by_name['MicBitResolution']
MicBitResolution = enum_type_wrapper.EnumTypeWrapper(_MICBITRESOLUTION)
_COMPRESSIONTYPE = DESCRIPTOR.enum_types_by_name['CompressionType']
CompressionType = enum_type_wrapper.EnumTypeWrapper(_COMPRESSIONTYPE)
UNDEFINED = 0
IAQ = 1
STATIC_IAQ = 2
CO2_EQ = 3
BREATH_VOC_EQ = 4
RAW_TEMPERATURE = 6
RAW_PRESSURE = 7
RAW_HUMIDITY = 8
RAW_GAS = 9
STABILIZATION_STATUS = 12
RUN_IN_STATUS = 13
SENSOR_HEAT_COMPEN_TEMP = 14
HEAT_COMPEN_HUMID = 15
GAS_PERCENTAGE = 21
UNRELIABLE = 0
LOW_ACCURACY = 1
MEDIUM_ACCURACY = 2
HIGH_ACCURACY = 3
SAMPLE_RATE_16000 = 0
SAMPLE_RATE_20500 = 1
SAMPLE_RATE_44100 = 2
SAMPLE_RATE_48000 = 3
SAMPLE_RATE_96000 = 4
BIT_RES_8 = 0
BIT_RES_16 = 1
OPUS = 0


_PACKETHEADER = DESCRIPTOR.message_types_by_name['PacketHeader']
_SIMPLESENSORREADING = DESCRIPTOR.message_types_by_name['SimpleSensorReading']
_SENSORREADING = DESCRIPTOR.message_types_by_name['SensorReading']
_SENSORREADING_PAYLOAD = _SENSORREADING.nested_types_by_name['Payload']
_SENSORCONFIG = DESCRIPTOR.message_types_by_name['SensorConfig']
_SDCARDSTATE = DESCRIPTOR.message_types_by_name['SDCardState']
_MARKSTATE = DESCRIPTOR.message_types_by_name['MarkState']
_MARKPACKET = DESCRIPTOR.message_types_by_name['MarkPacket']
_DISCOVEREDDEVICES = DESCRIPTOR.message_types_by_name['DiscoveredDevices']
_DISCOVEREDDEVICES_DEVICE = _DISCOVEREDDEVICES.nested_types_by_name['Device']
_BATTERYSTATE = DESCRIPTOR.message_types_by_name['BatteryState']
_SYSTEMINFOPACKET = DESCRIPTOR.message_types_by_name['SystemInfoPacket']
_AUDIOCOMPRESSION = DESCRIPTOR.message_types_by_name['AudioCompression']
_AUDIOCONFIG = DESCRIPTOR.message_types_by_name['AudioConfig']
_SCHEDULECONFIG = DESCRIPTOR.message_types_by_name['ScheduleConfig']
_LOWPOWERCONFIG = DESCRIPTOR.message_types_by_name['LowPowerConfig']
_CAMERACONTROL = DESCRIPTOR.message_types_by_name['CameraControl']
_NETWORKSTATE = DESCRIPTOR.message_types_by_name['NetworkState']
_CONFIGPACKET = DESCRIPTOR.message_types_by_name['ConfigPacket']
_SPECIALFUNCTION = DESCRIPTOR.message_types_by_name['SpecialFunction']
_PACKET = DESCRIPTOR.message_types_by_name['Packet']
PacketHeader = _reflection.GeneratedProtocolMessageType('PacketHeader', (_message.Message,), {
  'DESCRIPTOR' : _PACKETHEADER,
  '__module__' : 'message_pb2'
  # @@protoc_insertion_point(class_scope:PacketHeader)
  })
_sym_db.RegisterMessage(PacketHeader)

SimpleSensorReading = _reflection.GeneratedProtocolMessageType('SimpleSensorReading', (_message.Message,), {
  'DESCRIPTOR' : _SIMPLESENSORREADING,
  '__module__' : 'message_pb2'
  # @@protoc_insertion_point(class_scope:SimpleSensorReading)
  })
_sym_db.RegisterMessage(SimpleSensorReading)

SensorReading = _reflection.GeneratedProtocolMessageType('SensorReading', (_message.Message,), {

  'Payload' : _reflection.GeneratedProtocolMessageType('Payload', (_message.Message,), {
    'DESCRIPTOR' : _SENSORREADING_PAYLOAD,
    '__module__' : 'message_pb2'
    # @@protoc_insertion_point(class_scope:SensorReading.Payload)
    })
  ,
  'DESCRIPTOR' : _SENSORREADING,
  '__module__' : 'message_pb2'
  # @@protoc_insertion_point(class_scope:SensorReading)
  })
_sym_db.RegisterMessage(SensorReading)
_sym_db.RegisterMessage(SensorReading.Payload)

SensorConfig = _reflection.GeneratedProtocolMessageType('SensorConfig', (_message.Message,), {
  'DESCRIPTOR' : _SENSORCONFIG,
  '__module__' : 'message_pb2'
  # @@protoc_insertion_point(class_scope:SensorConfig)
  })
_sym_db.RegisterMessage(SensorConfig)

SDCardState = _reflection.GeneratedProtocolMessageType('SDCardState', (_message.Message,), {
  'DESCRIPTOR' : _SDCARDSTATE,
  '__module__' : 'message_pb2'
  # @@protoc_insertion_point(class_scope:SDCardState)
  })
_sym_db.RegisterMessage(SDCardState)

MarkState = _reflection.GeneratedProtocolMessageType('MarkState', (_message.Message,), {
  'DESCRIPTOR' : _MARKSTATE,
  '__module__' : 'message_pb2'
  # @@protoc_insertion_point(class_scope:MarkState)
  })
_sym_db.RegisterMessage(MarkState)

MarkPacket = _reflection.GeneratedProtocolMessageType('MarkPacket', (_message.Message,), {
  'DESCRIPTOR' : _MARKPACKET,
  '__module__' : 'message_pb2'
  # @@protoc_insertion_point(class_scope:MarkPacket)
  })
_sym_db.RegisterMessage(MarkPacket)

DiscoveredDevices = _reflection.GeneratedProtocolMessageType('DiscoveredDevices', (_message.Message,), {

  'Device' : _reflection.GeneratedProtocolMessageType('Device', (_message.Message,), {
    'DESCRIPTOR' : _DISCOVEREDDEVICES_DEVICE,
    '__module__' : 'message_pb2'
    # @@protoc_insertion_point(class_scope:DiscoveredDevices.Device)
    })
  ,
  'DESCRIPTOR' : _DISCOVEREDDEVICES,
  '__module__' : 'message_pb2'
  # @@protoc_insertion_point(class_scope:DiscoveredDevices)
  })
_sym_db.RegisterMessage(DiscoveredDevices)
_sym_db.RegisterMessage(DiscoveredDevices.Device)

BatteryState = _reflection.GeneratedProtocolMessageType('BatteryState', (_message.Message,), {
  'DESCRIPTOR' : _BATTERYSTATE,
  '__module__' : 'message_pb2'
  # @@protoc_insertion_point(class_scope:BatteryState)
  })
_sym_db.RegisterMessage(BatteryState)

SystemInfoPacket = _reflection.GeneratedProtocolMessageType('SystemInfoPacket', (_message.Message,), {
  'DESCRIPTOR' : _SYSTEMINFOPACKET,
  '__module__' : 'message_pb2'
  # @@protoc_insertion_point(class_scope:SystemInfoPacket)
  })
_sym_db.RegisterMessage(SystemInfoPacket)

AudioCompression = _reflection.GeneratedProtocolMessageType('AudioCompression', (_message.Message,), {
  'DESCRIPTOR' : _AUDIOCOMPRESSION,
  '__module__' : 'message_pb2'
  # @@protoc_insertion_point(class_scope:AudioCompression)
  })
_sym_db.RegisterMessage(AudioCompression)

AudioConfig = _reflection.GeneratedProtocolMessageType('AudioConfig', (_message.Message,), {
  'DESCRIPTOR' : _AUDIOCONFIG,
  '__module__' : 'message_pb2'
  # @@protoc_insertion_point(class_scope:AudioConfig)
  })
_sym_db.RegisterMessage(AudioConfig)

ScheduleConfig = _reflection.GeneratedProtocolMessageType('ScheduleConfig', (_message.Message,), {
  'DESCRIPTOR' : _SCHEDULECONFIG,
  '__module__' : 'message_pb2'
  # @@protoc_insertion_point(class_scope:ScheduleConfig)
  })
_sym_db.RegisterMessage(ScheduleConfig)

LowPowerConfig = _reflection.GeneratedProtocolMessageType('LowPowerConfig', (_message.Message,), {
  'DESCRIPTOR' : _LOWPOWERCONFIG,
  '__module__' : 'message_pb2'
  # @@protoc_insertion_point(class_scope:LowPowerConfig)
  })
_sym_db.RegisterMessage(LowPowerConfig)

CameraControl = _reflection.GeneratedProtocolMessageType('CameraControl', (_message.Message,), {
  'DESCRIPTOR' : _CAMERACONTROL,
  '__module__' : 'message_pb2'
  # @@protoc_insertion_point(class_scope:CameraControl)
  })
_sym_db.RegisterMessage(CameraControl)

NetworkState = _reflection.GeneratedProtocolMessageType('NetworkState', (_message.Message,), {
  'DESCRIPTOR' : _NETWORKSTATE,
  '__module__' : 'message_pb2'
  # @@protoc_insertion_point(class_scope:NetworkState)
  })
_sym_db.RegisterMessage(NetworkState)

ConfigPacket = _reflection.GeneratedProtocolMessageType('ConfigPacket', (_message.Message,), {
  'DESCRIPTOR' : _CONFIGPACKET,
  '__module__' : 'message_pb2'
  # @@protoc_insertion_point(class_scope:ConfigPacket)
  })
_sym_db.RegisterMessage(ConfigPacket)

SpecialFunction = _reflection.GeneratedProtocolMessageType('SpecialFunction', (_message.Message,), {
  'DESCRIPTOR' : _SPECIALFUNCTION,
  '__module__' : 'message_pb2'
  # @@protoc_insertion_point(class_scope:SpecialFunction)
  })
_sym_db.RegisterMessage(SpecialFunction)

Packet = _reflection.GeneratedProtocolMessageType('Packet', (_message.Message,), {
  'DESCRIPTOR' : _PACKET,
  '__module__' : 'message_pb2'
  # @@protoc_insertion_point(class_scope:Packet)
  })
_sym_db.RegisterMessage(Packet)

if _descriptor._USE_C_DESCRIPTORS == False:

  DESCRIPTOR._options = None
  _SIGNALIDENTIFIER._serialized_start=2886
  _SIGNALIDENTIFIER._serialized_end=3162
  _SENSORACCURACY._serialized_start=3164
  _SENSORACCURACY._serialized_end=3254
  _MICSAMPLEFREQ._serialized_start=3257
  _MICSAMPLEFREQ._serialized_end=3387
  _MICBITRESOLUTION._serialized_start=3389
  _MICBITRESOLUTION._serialized_end=3438
  _COMPRESSIONTYPE._serialized_start=3440
  _COMPRESSIONTYPE._serialized_end=3467
  _PACKETHEADER._serialized_start=17
  _PACKETHEADER._serialized_end=89
  _SIMPLESENSORREADING._serialized_start=92
  _SIMPLESENSORREADING._serialized_end=225
  _SENSORREADING._serialized_start=228
  _SENSORREADING._serialized_end=540
  _SENSORREADING_PAYLOAD._serialized_start=332
  _SENSORREADING_PAYLOAD._serialized_end=540
  _SENSORCONFIG._serialized_start=542
  _SENSORCONFIG._serialized_end=655
  _SDCARDSTATE._serialized_start=657
  _SDCARDSTATE._serialized_end=757
  _MARKSTATE._serialized_start=759
  _MARKSTATE._serialized_end=837
  _MARKPACKET._serialized_start=839
  _MARKPACKET._serialized_end=891
  _DISCOVEREDDEVICES._serialized_start=893
  _DISCOVEREDDEVICES._serialized_end=1020
  _DISCOVEREDDEVICES_DEVICE._serialized_start=984
  _DISCOVEREDDEVICES_DEVICE._serialized_end=1020
  _BATTERYSTATE._serialized_start=1022
  _BATTERYSTATE._serialized_end=1111
  _SYSTEMINFOPACKET._serialized_start=1114
  _SYSTEMINFOPACKET._serialized_end=1428
  _AUDIOCOMPRESSION._serialized_start=1430
  _AUDIOCOMPRESSION._serialized_end=1537
  _AUDIOCONFIG._serialized_start=1540
  _AUDIOCONFIG._serialized_end=1748
  _SCHEDULECONFIG._serialized_start=1751
  _SCHEDULECONFIG._serialized_end=1969
  _LOWPOWERCONFIG._serialized_start=1971
  _LOWPOWERCONFIG._serialized_end=2009
  _CAMERACONTROL._serialized_start=2011
  _CAMERACONTROL._serialized_end=2101
  _NETWORKSTATE._serialized_start=2103
  _NETWORKSTATE._serialized_end=2213
  _CONFIGPACKET._serialized_start=2216
  _CONFIGPACKET._serialized_end=2493
  _SPECIALFUNCTION._serialized_start=2496
  _SPECIALFUNCTION._serialized_end=2659
  _PACKET._serialized_start=2662
  _PACKET._serialized_end=2883
# @@protoc_insertion_point(module_scope)
