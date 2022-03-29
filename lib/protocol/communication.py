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


class IClient(ABC):

    def __init__(self, address):
        self.address = address

    @abstractmethod
    def send(self, packet: list, structure: struct.Struct) -> None:
        pass


class Client(IClient):

    def __init__(self, address):
        super().__init__(address)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, packet: list, structure: struct.Struct) -> None:
        packet = structure.pack(*packet)
        crc = packet_crc.pack(crc16(packet))
        self.socket.sendto(packet+crc, self.address)


class Sender:
    """
    Приниает на вход функцию которая возвращает список объектов для отправки,
    разделяет их по типам и последовательно отправляет,
    Терминология:
    [XThrust, YThrust, RollPidSettings, Reboot] - список объектов управления, все эти объекты должны быть отправлены в
    разных пакетах, с разными заголовками, такой список пакетов называется смешанный список объектов управления
    [[XThrust, YThrust], [RollPidSettings], [Reboot]] это следующая стадия, список списка объектов, каждый список это
    отдельный udp пакет, каждый этот список может быть отправлен одним сообщениям даже если он содержит множество
    объектов. такая структура называется списком пакетов управления
    [0, 230, 0 ...] - непосредственно пакет управления, закодированный и готовый к отправки
    """
    def __init__(self, address: (str, int)):
        self.client = self.get_client(address)
        self.packet_counter = 0

    @staticmethod
    def get_client(address) -> IClient:
        return Client(address)

    @staticmethod
    def _split_packets(*args) -> []:
        types = defaultdict(list)
        for arg in args:
            types[arg_map[arg.__class__]].append(arg)
        return list(types.values())

    @staticmethod
    def _get_packet_settings(*args) -> (int, int):
        return arg_map[args[0].__class__]

    @staticmethod
    def _get_struct_settings(*args) -> struct.Struct:
        return structure_map[args[0].__class__]

    @classmethod
    def make_packet(cls, *args) -> []:
        """
        :param args: список объектов управления или настроек
        :return: пакет отправляемы на аппарат
        """
        # получаем настройки для данного типа пакетов, важно что в эту функцию приходят пакеты отправляемые вместе
        id_, len_ = cls._get_packet_settings(*args)

        packet = [0, id_, *[0]*len_]
        for arg in args:
            arg.encode(packet)
        return packet

    def loop(self, func: Callable):
        while True:
            self.ones(func)

    def ones(self, func: Callable):
        # тут пакеты уже разделены по типам
        packets = self._split_packets(*func())
        for packet in packets:
            encoded_packet = self.make_packet(*packet)
            self.client.send(encoded_packet, self._get_struct_settings(*packet))
            self.packet_counter += 1
