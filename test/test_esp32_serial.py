from collections import deque
import unittest
from unittest.mock import patch

from thruster_control.esp32_serial import Esp32ThrusterSerial


class FakeSerial:
    def __init__(self, *_args, **_kwargs) -> None:
        self.responses = deque()
        self.writes: list[bytes] = []
        self.closed = False

    def reset_input_buffer(self) -> None:
        return

    def write(self, payload: bytes) -> None:
        self.writes.append(payload)
        if payload == b"PING\n":
            self.responses.extend(
                [b"STALE neutral\n", b"PONG L1500 R1500\n"]
            )
        elif payload.startswith(b"PWM L"):
            self.responses.append(b"OK L1500 R1500\n")
        elif payload == b"STOP\n":
            self.responses.append(b"OK STOP\n")

    def flush(self) -> None:
        return

    def readline(self) -> bytes:
        if self.responses:
            return self.responses.popleft()
        return b""

    def close(self) -> None:
        self.closed = True


class Esp32ThrusterSerialTests(unittest.TestCase):
    def create_device(self) -> tuple[Esp32ThrusterSerial, FakeSerial]:
        fake = FakeSerial()
        with (
            patch(
                "thruster_control.esp32_serial.serial.Serial",
                return_value=fake,
            ),
            patch("thruster_control.esp32_serial.time.sleep"),
        ):
            device = Esp32ThrusterSerial(
                "/dev/mtr_esp32",
                ready_timeout_s=0.0,
            )
        return device, fake

    def test_pong_verifies_pair_firmware_without_ready_banner(self) -> None:
        device, fake = self.create_device()

        identity = device.probe_dual_firmware(timeout_s=0.1)

        self.assertEqual(identity, "PONG L1500 R1500")
        self.assertEqual(fake.writes, [b"PING\n"])

    def test_pair_command_returns_firmware_acknowledgement(self) -> None:
        device, fake = self.create_device()

        response = device.send_pwm_pair(1500, 1500)

        self.assertEqual(response, "OK L1500 R1500")
        self.assertEqual(fake.writes, [b"PWM L1500 R1500\n"])
        self.assertEqual(device.last_response, response)


if __name__ == "__main__":
    unittest.main()
