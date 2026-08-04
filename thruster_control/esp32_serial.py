"""Serial bridge to the ESP32 (or Arduino) thruster firmware.

Wire protocol over USB serial (ASCII, line-terminated with ``\\n``):

- ``PWM <us>``           - single thruster, microseconds duty cycle
- ``PWM L<us> R<us>``    - differential pair, left then right (whitespace separated)
- ``STOP``               - immediate neutral on every channel

Firmware is expected to:

- Map a value of ``neutral_us`` (default 1500) to "stalled" output on every
  channel.
- Clamp incoming values into a hard min/max range before driving the ESCs.
- On ``STOP`` (or loss of serial) drive every channel back to neutral.

Two-channel firmware is a small extension of the UWMedTechRobotics
`thruster-control-experimentation` Arduino sketch: attach two ``Servo``
instances on PWM-capable pins, parse ``PWM L<us> R<us>``, write each value
with ``writeMicroseconds`` after clamping to the configured range.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass

import serial

from radar_nav.models import NavOutput


@dataclass
class ThrusterCommand:
    pwm_us: int
    reason: str


@dataclass
class ThrusterPairCommand:
    left_us: int
    right_us: int
    reason: str


@dataclass
class ThrusterMapping:
    neutral_us: int = 1500
    forward_min_us: int = 1520
    forward_max_us: int = 1600
    reverse_level1_us: int = 1460
    reverse_level2_us: int = 1445
    reverse_level3_us: int = 1425
    hard_min_us: int = 1350
    hard_max_us: int = 2000
    steering_slowdown: float = 0.35
    pivot_turn_start: float = 0.75
    pivot_reverse_ratio: float = 1.0

    def clamp_pwm(self, value: float) -> int:
        return int(round(max(self.hard_min_us, min(self.hard_max_us, value))))

    def throttle_to_us(self, throttle: float) -> int:
        """Map signed thrust into the measured forward/reverse PWM bands.

        Anything within ~1% of zero returns neutral. Reverse uses the three
        measured levels so half reverse lands on level 2 exactly.
        """

        throttle = max(-1.0, min(1.0, throttle))
        if abs(throttle) <= 0.01:
            return self.neutral_us
        if throttle > 0.0:
            pwm = self.forward_min_us + throttle * (
                self.forward_max_us - self.forward_min_us
            )
            return self.clamp_pwm(pwm)

        reverse = abs(throttle)
        if reverse <= 0.5:
            pwm = self.reverse_level1_us + reverse * 2.0 * (
                self.reverse_level2_us - self.reverse_level1_us
            )
        else:
            pwm = self.reverse_level2_us + (reverse - 0.5) * 2.0 * (
                self.reverse_level3_us - self.reverse_level2_us
            )
        return self.clamp_pwm(pwm)

    def pwm_to_throttle(self, pwm_us: float) -> float:
        value = float(pwm_us)
        if value == self.neutral_us:
            return 0.0
        if value > self.neutral_us:
            span = max(1, self.forward_max_us - self.forward_min_us)
            throttle = (value - self.forward_min_us) / span
            return max(0.02, min(1.0, throttle))
        if value >= self.reverse_level1_us:
            return -0.02
        if value >= self.reverse_level2_us:
            span = max(
                1,
                self.reverse_level1_us - self.reverse_level2_us,
            )
            ratio = (self.reverse_level1_us - value) / span
            return -(0.02 + ratio * 0.48)
        span = max(
            1,
            self.reverse_level2_us - self.reverse_level3_us,
        )
        ratio = (self.reverse_level2_us - value) / span
        return -max(0.5, min(1.0, 0.5 + ratio * 0.5))


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


def manual_to_pair(
    throttle: float,
    steering: float,
    mapping: ThrusterMapping | None = None,
    enabled: bool = True,
    allow_pivot_reverse: bool = True,
) -> ThrusterPairCommand:
    """Convert a (throttle, steering) intent into a left/right PWM pair.

    Ordinary steering uses the forward differential mix. Near full lock, the
    inside thruster blends through neutral into calibrated reverse. Reverse is
    multiplied by throttle, so zero throttle always means two neutral outputs.

    When ``enabled`` is False, both sides are neutral regardless of input.
    """

    mapping = mapping or ThrusterMapping()
    if not enabled:
        return ThrusterPairCommand(mapping.neutral_us, mapping.neutral_us, "manual disabled")

    throttle = max(0.0, min(1.0, throttle))
    steering = max(-1.0, min(1.0, steering))

    if throttle <= 0.01:
        return ThrusterPairCommand(
            mapping.neutral_us,
            mapping.neutral_us,
            f"throttle={throttle:.2f} steering={steering:+.2f} (neutral)",
        )

    steering_magnitude = abs(steering)
    outer = min(1.0, throttle * (1.0 + steering_magnitude))
    inner = throttle * (1.0 - steering_magnitude)
    pivot_start = max(0.0, min(0.99, mapping.pivot_turn_start))
    reverse_ratio = (
        max(0.0, min(1.0, mapping.pivot_reverse_ratio))
        if allow_pivot_reverse
        else 0.0
    )
    if reverse_ratio > 0.0 and steering_magnitude > pivot_start:
        blend = (
            (steering_magnitude - pivot_start)
            / (1.0 - pivot_start)
        )
        inner_at_start = throttle * (1.0 - pivot_start)
        inner_at_full = -throttle * reverse_ratio
        inner = (
            inner_at_start * (1.0 - blend)
            + inner_at_full * blend
        )

    if steering >= 0.0:
        left, right = outer, inner
    else:
        left, right = inner, outer
    return ThrusterPairCommand(
        mapping.throttle_to_us(left),
        mapping.throttle_to_us(right),
        f"throttle={throttle:.2f} steering={steering:+.2f} L={left:.2f} R={right:.2f}",
    )


def pair_to_manual(
    left_us: float,
    right_us: float,
    mapping: ThrusterMapping | None = None,
) -> tuple[float, float]:
    """Convert a PWM pair into display/control throttle and steering."""

    mapping = mapping or ThrusterMapping()
    left = mapping.pwm_to_throttle(left_us)
    right = mapping.pwm_to_throttle(right_us)
    total_effort = abs(left) + abs(right)
    if total_effort <= 0.02:
        return 0.0, 0.0
    return (
        max(0.0, min(1.0, total_effort / 2.0)),
        max(-1.0, min(1.0, (left - right) / total_effort)),
    )


class Esp32ThrusterSerial:
    def __init__(
        self,
        port: str,
        baud: int = 115200,
        timeout: float = 0.25,
        ready_timeout_s: float = 8.0,
    ):
        self.ser = serial.Serial(port, baudrate=baud, timeout=timeout)
        self.last_pwm_us: int | None = None
        self.last_pair: tuple[int, int] | None = None
        self.last_response: str | None = None
        self.ready_banner: str | None = None
        self._command_lock = threading.Lock()
        # Allow the ESP32 USB-CDC interface to enumerate before we read.
        time.sleep(0.5)
        self._wait_for_ready(ready_timeout_s)

    def _wait_for_ready(self, timeout_s: float) -> None:
        """Block until the firmware emits its `READY ...` banner, or until
        the timeout expires.

        The bundled ESP32 sketch holds a 5 s neutral-pulse arming window
        before accepting commands; talking to the ESCs during that window
        can put them into setup/calibration mode. The READY line tells us
        arming has finished. For legacy single-channel firmware (no READY
        line) the timeout itself doubles as the arming window, so callers
        always see at least ~5 s of held-neutral before the first command.
        """

        deadline = time.monotonic() + max(0.0, timeout_s)
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass
        while time.monotonic() < deadline:
            try:
                line = self.ser.readline()
            except Exception:
                line = b""
            if not line:
                continue
            text = line.decode("ascii", errors="replace").strip()
            if text.startswith("READY"):
                self.ready_banner = text
                self.last_response = text
                return

    def _request(
        self,
        command: str,
        response_prefixes: tuple[str, ...],
        *,
        timeout_s: float = 0.5,
    ) -> str | None:
        with self._command_lock:
            self.ser.write(f"{command}\n".encode("ascii"))
            self.ser.flush()
            deadline = time.monotonic() + max(timeout_s, 0.0)
            while time.monotonic() < deadline:
                line = self.ser.readline()
                if not line:
                    continue
                text = line.decode("ascii", errors="replace").strip()
                self.last_response = text
                if text.startswith(response_prefixes):
                    return text
        return None

    def probe_dual_firmware(self, timeout_s: float = 1.0) -> str | None:
        """Return evidence that the connected firmware supports PWM pairs."""
        banner = self.ready_banner or ""
        if banner.startswith("READY L=") and " R=" in banner:
            return banner
        response = self._request(
            "PING",
            ("PONG L",),
            timeout_s=timeout_s,
        )
        if response is not None and " R" in response:
            return response
        return None

    def send_pwm(self, pwm_us: int) -> str | None:
        value = int(pwm_us)
        response = self._request(
            f"PWM {value}",
            ("OK PWM ", "ERR "),
        )
        self.last_pwm_us = value
        self.last_pair = None
        return response

    def send_pwm_pair(self, left_us: int, right_us: int) -> str | None:
        left = int(left_us)
        right = int(right_us)
        response = self._request(
            f"PWM L{left} R{right}",
            ("OK L", "ERR "),
        )
        self.last_pair = (left, right)
        self.last_pwm_us = None
        return response

    def stop(self) -> str | None:
        response = self._request("STOP", ("OK STOP", "ERR "))
        self.last_pwm_us = None
        self.last_pair = None
        return response

    def close(self) -> None:
        try:
            self.stop()
        except Exception:
            pass
        self.ser.close()
