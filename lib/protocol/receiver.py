import socket
from typing import Callable
from protocol.model import Telemetry, packet_structure, keys


class Receiver:
    def __init__(self, address: (str, int)):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(address)
        self.socket.setblocking(False)
        self.packet_counter = 0

    def loop(self, func: Callable):
        while True:
            self.ones(func)

    def ones(self, func: Callable):
        try:
            data, address = self.socket.recvfrom(packet_structure.size)
        except BlockingIOError:
            return func(None)
        data = packet_structure.unpack(data)
        dict_ = dict(zip(keys, data))
        self.packet_counter += 1
        return func(Telemetry(**dict_))
