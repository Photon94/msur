from pydantic import BaseModel, validator
from pydantic.generics import GenericModel
from typing import TypeVar, Generic
import struct


packet_telemetry = struct.Struct('!BBffffffffffffBBBBf')
packet_control = struct.Struct('!BBBBBBfffffBBBffB')
packet_pid = struct.Struct('!BBfff')
packet_reboot = struct.Struct('!BBBBBBBB')
packet_crc = struct.Struct('!H')

telemetry_keys = [
    '0', 'id', 'roll', 'pitch', 'yaw', 'gyro_z', 'depth', 'altitude', 'velocity_x', 'velocity_y', 'pos_x', 'pos_y',
     'voltage', 'current', 'pid_stat', 'devices_stat', 'leak', 'device_error', 'reserved_0', 'reserved_1', 'reserved_2',
     'reserved_3', 'crc'
]


class IBase(BaseModel):

    def encode(self, packet: list) -> list:
        pass


class PidStats(IBase):
    roll: bool
    pitch: bool
    yaw: bool
    depth: bool
    altitude: bool
    speed_x: bool
    speed_y: bool

    def encode(self, packet: list) -> list:
        value = int(''.join([str(int(i)) for i in reversed(
            [self.roll, self.pitch, self.depth, self.altitude, self.yaw, self.speed_x, self.speed_y, 0])]), 2)
        packet[11] = value
        return packet


class PidConfig(IBase):
    p: float
    i: float
    d: float

    def encode(self, packet: list) -> list:
        packet[2] = self.p
        packet[3] = self.i
        packet[4] = self.d
        return packet


class DepthPidConfig(PidConfig):
    pass


class AltitudePidConfig(PidConfig):
    pass


class RollPidConfig(PidConfig):
    pass


class PitchPidConfig(PidConfig):
    pass


class YawPidConfig(PidConfig):
    pass


class VelXPidConfig(PidConfig):
    pass


class VelYPidConfig(PidConfig):
    pass


class GyroPidConfig(PidConfig):
    pass


class RebootConfig(IBase):
    stm: bool
    pc: bool
    hydro: bool

    def encode(self, packet: list) -> list:
        packet[2] = int(self.stm)
        packet[3] = int(self.pc)
        packet[4] = int(self.hydro)
        return packet


class ExternalDevices(IBase):
    em_1: bool
    em_2: bool

    def encode(self, packet: list) -> list:
        value = int(''.join([str(int(i)) for i in reversed([self.em_1, self.em_2, 0, 0, 0, 0, 0, 0])]), 2)
        packet[12] = value
        return packet


class LeakStatus(BaseModel):
    main: bool
    navigation: bool


class DevicesError(BaseModel):
    pressure_sensor: bool
    navigation_module: bool


class Telemetry(BaseModel):
    roll: float
    pitch: float
    yaw: float
    gyro_z: float
    depth: float
    altitude: float
    velocity_x: float
    velocity_y: float
    pos_x: float
    pos_y: float
    voltage: float
    current: float
    pid_stat: PidStats
    devices_stat: ExternalDevices
    leak: LeakStatus
    device_error: DevicesError

    @validator('pid_stat', pre=True)
    def validate_pid_stat(cls, v):
        b = '{0:08b}'.format(v)
        return PidStats(roll=bool(int(b[-1])), pitch=bool(int(b[-2])), yaw=bool(int(b[-5])), depth=bool(int(b[-3])),
                        altitude=bool(int(b[-4])), speed_x=bool(int(b[-6])), speed_y=bool(int(b[-7])))

    @validator('devices_stat', pre=True)
    def validate_devices_stat(cls, v):
        b = '{0:08b}'.format(v)
        return ExternalDevices(em_1=bool(int(b[-1])), em_2=bool(int(b[-2])))

    @validator('leak', pre=True)
    def validate_leak(cls, v):
        b = '{0:08b}'.format(v)
        return LeakStatus(main=bool(int(b[-1])), navigation=bool(int(b[-2])))

    @validator('device_error', pre=True)
    def validate_device_error(cls, v):
        b = '{0:08b}'.format(v)
        return DevicesError(pressure_sensor=bool(int(b[-1])), navigation_module=bool(int(b[-2])))


class Thrust(IBase):
    value: int

    @validator('value', pre=True)
    def validate_value(cls, v):
        if v > 100:
            return 100
        elif v < -100:
            return -100
        return v


class FloatValue(IBase):
    value: float


class XThrust(Thrust):

    def encode(self, packet: list) -> list:
        packet[2] = self.value
        return packet


class YThrust(Thrust):
    def encode(self, packet: list) -> list:
        packet[3] = self.value
        return packet


class WThrust(Thrust):
    def encode(self, packet: list) -> list:
        packet[4] = self.value
        return packet


class ZThrust(Thrust):
    def encode(self, packet: list) -> list:
        packet[5] = self.value
        return packet


class Depth(FloatValue):
    def encode(self, packet: list) -> list:
        packet[6] = self.value
        return packet


class AltSet(FloatValue):
    def encode(self, packet: list) -> list:
        packet[7] = self.value
        return packet


class Yaw(FloatValue):
    def encode(self, packet: list) -> list:
        packet[8] = self.value
        return packet


class XVelocity(FloatValue):
    def encode(self, packet: list) -> list:
        packet[9] = self.value
        return packet


class YVelocity(FloatValue):
    def encode(self, packet: list) -> list:
        packet[10] = self.value
        return packet


class NavFlag(BaseModel):
    value: bool

    def encode(self, packet: list) -> list:
        packet[13] = int(self.value)
        return packet
