import pyxel
from protocol.communication import Receiver
from protocol.model import Telemetry
import socket


ADDRESS = ('192.168.90.77', 2065)


class App:
    def __init__(self):
        self.receiver = Receiver(ADDRESS)
        pyxel.init(200, 150, title="Telemetry MSUR monitor")
        self.connected = False
        pyxel.run(self.update, self.draw)

    def update(self):
        pass

    def draw(self):
        self.receiver.ones(self.telemetry)
        self.wait()

    def wait(self):
        if self.connected:
            return
        pyxel.cls(0)
        offset = pyxel.sin(pyxel.frame_count * 9) * 2

        pyxel.text(70, 30 + offset, "wait connection", 7)
        pyxel.text(71, 51, f"{ADDRESS[0]}:{ADDRESS[1]}", 4)
        pyxel.text(70, 50, f"{ADDRESS[0]}:{ADDRESS[1]}", 9)

    def telemetry(self, telemetry: Telemetry):
        if telemetry is None:
            return
        self.connected = True

        pyxel.cls(0)
        x, y = 10, 15
        pyxel.text(20, 5, "position", 5)
        pyxel.text(x, y, f"Pitch: {telemetry.pitch:10.3f}°", 7)
        pyxel.text(x, y := y + 10, f"Yaw: {telemetry.yaw:12.3f}", 7)
        pyxel.text(x, y := y + 10, f"Roll: {telemetry.roll:11.3f}", 7)
        pyxel.text(x, y := y + 10, f"Depth: {telemetry.depth:10.3f}", 7)
        pyxel.text(x, y := y + 10, f"Altitude: {telemetry.altitude:7.3f}", 7)

        pyxel.text(16, y := y + 15, "electricity", 5)
        pyxel.text(x, y := y + 10, f"Voltage: {telemetry.voltage:8.3f}v", 7)
        pyxel.text(x, y := y + 10, f"Current: {telemetry.current:8.3f}A", 7)

        x, y = 100, 15
        pyxel.text(120, 5, "speed", 5)
        pyxel.text(x, y, f"Velocity X: {telemetry.velocity_x:8.3f}m/s", 7)
        pyxel.text(x, y := y + 10, f"Velocity y: {telemetry.velocity_y:8.3f}m/s", 7)

        pyxel.text(115, y := y + 15, "position", 5)
        pyxel.text(x, y := y + 10, f"Position X: {telemetry.pos_x:8.3f}m", 7)
        pyxel.text(x, y := y + 10, f"Position Y: {telemetry.pos_y:8.3f}m", 7)

        pyxel.text(80, y := 105, "PID status", 5)
        true_color = 3
        false_color = 8
        pyxel.text(x := 10, y := y + 10, f"roll", true_color if telemetry.pid_stat.roll else false_color)
        pyxel.text(x := x + 22, y, f"pitch", true_color if telemetry.pid_stat.pitch else false_color)
        pyxel.text(x := x + 26, y, f"yaw", true_color if telemetry.pid_stat.yaw else false_color)
        pyxel.text(x := x + 18, y, f"depth", true_color if telemetry.pid_stat.depth else false_color)
        pyxel.text(x := x + 26, y, f"attitude", true_color if telemetry.pid_stat.altitude else false_color)
        pyxel.text(x := x + 38, y, f"spd x", true_color if telemetry.pid_stat.speed_x else false_color)
        pyxel.text(x := x + 26, y, f"spd y", true_color if telemetry.pid_stat.speed_y else false_color)

        pyxel.text(10, y := y + 15, f"Packet counter: {self.receiver.packet_counter}", 9)
App()
