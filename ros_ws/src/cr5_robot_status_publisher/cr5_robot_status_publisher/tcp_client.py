import math
import socket
import struct


class TCPMonitorClient:
    def __init__(self, ip: str, port: int = 30004, timeout: float = 5.0):
        self.ip = ip
        self.port = port
        self.timeout = timeout
        self.sock: socket.socket | None = None

    def connect(self):
        """Establish TCP connection"""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(self.timeout)
        self.sock.connect((self.ip, self.port))
        print(f"Connected to {self.ip}:{self.port}")

    def disconnect(self):
        """Close TCP connection"""
        if self.sock:
            self.sock.close()
            self.sock = None
            print("Disconnected")

    def read_packet(self) -> bytes:
        """Read exactly 1440 bytes"""
        if not self.sock:
            raise ConnectionError("Not connected")

        buf = b""
        while len(buf) < 1440:
            chunk = self.sock.recv(1440 - len(buf))
            if not chunk:
                raise ConnectionError("Connection closed by robot")
            buf += chunk
        return buf

    @staticmethod
    def sanitize_value(values: list[float], epsilon: float = 1e-3) -> list[float]:
        """Set very small float values to 0.0 to remove noise."""
        return [0.0 if abs(v) < epsilon else v for v in values]

    @staticmethod
    def parse_dio_bits(dio_value: int, count: int = 24) -> list[bool]:
        """Convert a 64-bit integer to a list of booleans representing DI/DO bits."""
        return [(dio_value >> i) & 1 == 1 for i in range(count)]

    def parse_packet(self, packet: bytes) -> dict:
        """Parse essential fields"""

        # Digital inputs/outputs
        digital_inputs = self.parse_dio_bits(
            struct.unpack_from("<Q", packet, 8)[0])
        digital_outputs = self.parse_dio_bits(
            struct.unpack_from("<Q", packet, 16)[0])

        # Robot mode mapping
        robot_mode_value = struct.unpack_from("<Q", packet, 24)[0]
        robot_mode = {
            0: "No Controller",
            1: "Disconnected",
            2: "Confirm Safety",
            3: "Booting",
            4: "Power Off",
            5: "Power On",
            6: "Idle",
            7: "Backdrive",
            8: "Running",
            9: "Updating Firmware",
        }.get(robot_mode_value, f"Unknown ({robot_mode_value})")

        # Joint positions (QActual) 6 doubles
        q_actual = self.sanitize_value(struct.unpack_from("<6d", packet, 432))

        q_actual = [math.radians(val) for val in q_actual]

        # Joint speeds (QDActual) 6 doubles
        qd_actual = self.sanitize_value(struct.unpack_from("<6d", packet, 480))

        # TCP actual Cartesian coordinates (ToolVectorActual) 6 doubles
        tcp_actual = self.sanitize_value(
            struct.unpack_from("<6d", packet, 624))

        return {
            "digital_inputs": digital_inputs,
            "digital_outputs": digital_outputs,
            "robot_mode": robot_mode,
            "joints": {
                "position": q_actual,
                "speed": qd_actual,
            },
            "tcp": tcp_actual,
        }
