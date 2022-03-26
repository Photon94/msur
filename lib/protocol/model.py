from pydantic import BaseModel, validator
import struct


packet_structure = struct.Struct('!BBffffffffffffBBBBBBh')
keys = ['0', 'id', 'roll', 'pitch', 'yaw', 'gyro_z', 'depth', 'altitude', 'velocity_x', 'velocity_y', 'pos_x', 'pos_y',
     'voltage', 'current', 'pid_stat', 'devices_stat', 'leak', 'device_error', 'reserved_0', 'reserved_1', 'reserved_2',
     'reserved_3', 'crc']


class PidStats(BaseModel):
    roll: bool
    pitch: bool
    yaw: bool
    depth: bool
    altitude: bool
    speed_x: bool
    speed_y: bool


class DevicesStats(BaseModel):
    em_1: bool
    em_2: bool


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
    devices_stat: DevicesStats
    leak: LeakStatus
    device_error: DevicesError

    @validator('pid_stat', pre=True)
    def validate_pid_stat(cls, v):
        b = '{0:08b}'.format(v)
        return PidStats(roll=bool(int(b[-1])), pitch=bool(int(b[-2])), yaw=bool(int(b[-3])), depth=bool(int(b[-4])),
                        altitude=bool(int(b[-5])), speed_x=bool(int(b[-6])), speed_y=bool(int(b[-7])))

    @validator('devices_stat', pre=True)
    def validate_devices_stat(cls, v):
        b = '{0:08b}'.format(v)
        return DevicesStats(em_1=bool(int(b[-1])), em_2=bool(int(b[-2])))

    @validator('leak', pre=True)
    def validate_leak(cls, v):
        b = '{0:08b}'.format(v)
        return LeakStatus(main=bool(int(b[-1])), navigation=bool(int(b[-2])))

    @validator('device_error', pre=True)
    def validate_device_error(cls, v):
        b = '{0:08b}'.format(v)
        return DevicesError(pressure_sensor=bool(int(b[-1])), navigation_module=bool(int(b[-2])))
