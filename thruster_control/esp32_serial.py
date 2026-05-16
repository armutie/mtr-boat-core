from __future__ import annotations

import time
from dataclasses import dataclass

import serial

from radar_nav.models import NavOutput


@dataclass
class ThrusterCommand:
    pwm_us: int
    reason: str


@dataclass
class ThrusterMapping:
    neutral_us: int = 1500
    forward_min_us: int = 1520
    forward_max_us: int = 1600
    hard_min_us: int = 1350
    hard_max_us: int = 2000
    steering_slowdown: float = 0.35

    def clamp_pwm(self, value: float) -> int:
        return int(round(max(self.hard_min_us, min(self.hard_max_us, value))))


def nav_output_to_thruster(output: NavOutput | None, mapping: ThrusterMapping | None = None) -> ThrusterCommand:
    mapping = mapping or ThrusterMapping()
    if output is None:
        return ThrusterCommand(mapping.neutral_us, "no nav output")

    if output.command == "stop" or output.throttle <= 0.0:
        return ThrusterCommand(mapping.neutral_us, f"{output.command}: neutral")

    throttle = max(0.0, min(1.0, output.throttle))
    steering = max(-1.0, min(1.0, output.steering))
    throttle *= 1.0 - mapping.steering_slowdown * abs(steering)

    if throttle <= 0.01:
        return ThrusterCommand(mapping.neutral_us, "throttle near zero")

    pwm = mapping.forward_min_us + throttle * (mapping.forward_max_us - mapping.forward_min_us)
    return ThrusterCommand(mapping.clamp_pwm(pwm), f"{output.command}: throttle={output.throttle:.2f} steering={output.steering:+.2f}")


class Esp32ThrusterSerial:
    def __init__(self, port: str, baud: int = 115200, timeout: float = 0.25):
        self.ser = serial.Serial(port, baudrate=baud, timeout=timeout)
        self.last_pwm_us: int | None = None
        time.sleep(2.0)

    def send_pwm(self, pwm_us: int) -> None:
        self.ser.write(f"PWM {int(pwm_us)}\n".encode("ascii"))
        self.ser.flush()
        self.last_pwm_us = int(pwm_us)

    def stop(self) -> None:
        self.ser.write(b"STOP\n")
        self.ser.flush()
        self.last_pwm_us = None

    def close(self) -> None:
        self.ser.close()
