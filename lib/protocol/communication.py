import socket
from typing import Callable
from crc16 import crc16xmodem as crc16
from protocol.model import *


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
        dict_ = dict(zip(keys, data))
        self.packet_counter += 1
        return func(Telemetry(**dict_))


class Sender:
    def __init__(self, address: (str, int)):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.address = address
        self.packet_counter = 0

    @staticmethod
    def pack(*args) -> []:
        packet = [0, 230, *[0]*15]

        for arg in args:
            if isinstance(arg, (XThrust, YThrust, ZThrust)):
                packet[{XThrust: 2, YThrust: 3, WThrust: 4, ZThrust: 5}[arg.__class__]] = arg.value
            elif isinstance(arg, (Depth, AltSet, Yaw, XVelocity, YVelocity)):
                packet[{Depth: 6, AltSet: 7, Yaw: 8, XVelocity: 9, YVelocity: 10}[arg.__class__]] = arg.value
            elif isinstance(arg, PidStats):
                packet[11] = arg.encode()
            elif isinstance(arg, DevicesStats):
                packet[12] = arg.encode()
            elif isinstance(arg, NavFlag):
                packet[13] = arg.value
        return packet

    def loop(self, func: Callable):
        while True:
            self.ones(func)

    def ones(self, func: Callable):
        packet = packet_control.pack(*self.pack(*func()))
        crc = packet_crc.pack(crc16(packet))
        self.socket.sendto(packet+crc, self.address)
        self.packet_counter += 1
