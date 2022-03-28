import socket
from typing import Callable
from crc16 import crc16xmodem as crc16
from protocol.model import *
from collections import defaultdict


class Receiver:
    def __init__(self, address: (str, int)):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(address)
        self.socket.setblocking(False)
        self.packet_counter = 0
        self.errors_counter = 0

    def loop(self, func: Callable):
        while True:
            self.ones(func)

    def ones(self, func: Callable):
        try:
            data, address = self.socket.recvfrom(packet_telemetry.size)
        except BlockingIOError:
            return func(None)
        crc = packet_crc.unpack(data[-2:])
        data = data[:-2]
        if crc != crc16(data):
            self.errors_counter += 1
            return func(Telemetry())
        data = packet_telemetry.unpack(data)
        dict_ = dict(zip(telemetry_keys, data))
        self.packet_counter += 1
        return func(Telemetry(**dict_))


message_map = {
    (230, 15): [XThrust, YThrust, ZThrust, Depth, AltSet, Yaw, XVelocity, YVelocity, PidStats, ExternalDevices, NavFlag],
    (110, 3): [DepthPidConfig],
    (111, 3): [AltitudePidConfig],
    (112, 3): [RollPidConfig],
    (113, 3): [PitchPidConfig],
    (114, 3): [YawPidConfig],
    (115, 3): [VelXPidConfig],
    (116, 3): [VelYPidConfig],
    (117, 3): [GyroPidConfig],
    (133, 6): [RebootConfig],
}

# arg_map = [arg: type_ for type_, args in message_map.items()]

byte_map = {
    XThrust: 2, YThrust: 3, WThrust: 4, ZThrust: 5, Depth: 6, AltSet: 7, Yaw: 8, XVelocity: 9, YVelocity: 10,
    PidStats: 11, ExternalDevices: 12, NavFlag: 13,
}


class Sender:
    def __init__(self, address: (str, int)):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.address = address
        self.packet_counter = 0

    @staticmethod
    def _split_packets(*args) -> []:
        types = defaultdict(list)
        for arg in args:
            pass
        pass

    @staticmethod
    def _get_settings(*args) -> (int, int):
        return 1, 1

    @classmethod
    def pack(cls, *args) -> []:
        id_, len_ = cls._get_settings(*args)
        packet = [0, id_, *[0]*len_]
        for arg in args:
            arg.encode(packet)
        return packet

    def loop(self, func: Callable):
        while True:
            self.ones(func)

    def ones(self, func: Callable):
        packet = packet_control.pack(*self.pack(*func()))
        crc = packet_crc.pack(crc16(packet))
        self.socket.sendto(packet+crc, self.address)
        self.packet_counter += 1
