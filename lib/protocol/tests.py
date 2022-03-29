import unittest
from protocol.communication import Sender, IClient
from protocol.model import *
import struct


class TestClient(IClient):

    def __init__(self, address):
        super().__init__(address)
        self. packets = []
        self.structures = []

    def send(self, packet: list, structure: struct.Struct) -> None:
        self.packets.append(packet)
        self.structures.append(structure)


class TestSender(Sender):

    def get_client(self, address) -> IClient:
        return TestClient(address)


class SenderTest(unittest.TestCase):

    def test_split_packets(self):
        sender = TestSender(('127.0.0.1', 5555))

        packet = [XThrust(value=100), YThrust(value=50), RebootConfig(stm=True), RollPidConfig(p=1, i=0, d=6)]

        sender.ones(lambda: packet)
        self.assertEqual(sender.client.package, [
            [0, 230, 100, 50, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 133, 1, 0, 0, 0, 0, 0],
            [0, 112, 1.0, 0.0, 6.0]])
