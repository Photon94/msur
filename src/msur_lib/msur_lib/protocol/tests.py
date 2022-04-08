import unittest
from protocol.communication import Sender, IClient
from typing import Callable
from protocol.model import *
from protocol.communication import Receiver
import math


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


class TestReceiver(Receiver):
    def __init__(self, address: (str, int)):
        super().__init__(address)
        self.i = 0

    def ones(self, func: Callable):
        p = PidStats(
            roll=True,
            pitch=False,
            yaw=False,
            depth=False,
            altitude=False,
            speed_x=False,
            speed_y=False,
        )

        e = ExternalDevices(em_1=False, em_2=False)
        l = LeakStatus(main=False, imu=False)
        d = DevicesError(pressure_sensor=False, imu_module=False)
        i = self.i
        telemetry = Telemetry(
            roll=math.sin(0.15 * i),
            pitch=math.sin(0.16 * i),
            yaw=math.sin(0.17 * i),
            gyro_z=math.sin(0.18 * i),
            depth=math.sin(0.18 * i),
            altitude=math.sin(0.18 * i),
            velocity_x=math.sin(0.18 * i),
            velocity_y=math.sin(0.18 * i),
            pos_x=math.sin(0.18 * i),
            pos_y=math.sin(0.18 * i),
            voltage=math.sin(0.18 * i) * 3,
            current=math.sin(0.18 * i),
            pid_stat=p,
            devices_stat=e,
            leak=l,
            device_error=d
        )
        func(telemetry)
        self.i += 1


