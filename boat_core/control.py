from __future__ import annotations

from dataclasses import dataclass

from thruster_control import (
    ThrusterMapping,
    manual_to_pair,
    pair_to_manual,
)


VALID_CONTROL_MODES = ("off", "manual", "auto")


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, float(value)))


@dataclass(frozen=True)
class VelocityCommand:
    linear_x: float = 0.0
    angular_z: float = 0.0


@dataclass(frozen=True)
class PwmCommand:
    left_us: int
    right_us: int


class ControlSupervisor:
    """Select one final left/right actuator command.

    Manual input is converted from velocity intent to PWM here. Autonomous
    input is already an actuator command and passes through unchanged unless
    the operator is actively applying a trim.
    """

    def __init__(
        self,
        command_timeout_s: float = 0.5,
        max_linear_mps: float = 1.0,
        max_angular_rps: float = 1.0,
        throttle_slew_per_s: float = 0.0,
        mapping: ThrusterMapping | None = None,
    ) -> None:
        self.command_timeout_s = max(0.05, command_timeout_s)
        self.max_linear_mps = max(0.01, max_linear_mps)
        self.max_angular_rps = max(0.01, max_angular_rps)
        self.throttle_slew_per_s = max(0.0, throttle_slew_per_s)
        self.mapping = mapping or ThrusterMapping()
        self.mode = "off"
        self._operator = VelocityCommand()
        self._automatic = self._neutral()
        self._operator_at: float | None = None
        self._automatic_at: float | None = None
        self._manual_throttle = 0.0
        self._last_output_at: float | None = None

    def set_mode(self, mode: str) -> None:
        if mode not in VALID_CONTROL_MODES:
            raise ValueError(f"mode must be one of {VALID_CONTROL_MODES}")
        if mode != self.mode:
            self._operator_at = None
            self._automatic_at = None
            self._manual_throttle = 0.0
            self._last_output_at = None
        self.mode = mode

    def update_operator(self, command: VelocityCommand, now: float) -> None:
        self._operator = self._bounded(command)
        self._operator_at = now

    def update_auto(self, command: PwmCommand, now: float) -> None:
        self._automatic = PwmCommand(
            self.mapping.clamp_pwm(command.left_us),
            self.mapping.clamp_pwm(command.right_us),
        )
        self._automatic_at = now

    def output(self, now: float) -> PwmCommand:
        if self.mode == "manual":
            if not self._fresh(self._operator_at, now):
                self._manual_throttle = 0.0
                self._last_output_at = now
                return self._neutral()
            return self._manual_output(now)

        if self.mode == "auto":
            if not self._fresh(self._automatic_at, now):
                return self._neutral()
            if not self._fresh(self._operator_at, now):
                return self._automatic
            if (
                abs(self._operator.linear_x) <= 1e-9
                and abs(self._operator.angular_z) <= 1e-9
            ):
                return self._automatic
            return self._trimmed_auto_output()

        return self._neutral()

    def _manual_output(self, now: float) -> PwmCommand:
        target_throttle = clamp(
            self._operator.linear_x / self.max_linear_mps,
            0.0,
            1.0,
        )
        steering = clamp(
            self._operator.angular_z / self.max_angular_rps,
            -1.0,
            1.0,
        )
        dt = (
            0.0
            if self._last_output_at is None
            else max(0.0, now - self._last_output_at)
        )
        self._last_output_at = now
        if (
            target_throttle <= self._manual_throttle
            or self.throttle_slew_per_s <= 0.0
        ):
            self._manual_throttle = target_throttle
        else:
            self._manual_throttle = min(
                target_throttle,
                self._manual_throttle + self.throttle_slew_per_s * dt,
            )
        pair = manual_to_pair(
            self._manual_throttle,
            steering,
            self.mapping,
        )
        return PwmCommand(pair.left_us, pair.right_us)

    def _trimmed_auto_output(self) -> PwmCommand:
        throttle, steering = pair_to_manual(
            self._automatic.left_us,
            self._automatic.right_us,
            self.mapping,
        )
        throttle += self._operator.linear_x / self.max_linear_mps
        steering += self._operator.angular_z / self.max_angular_rps
        pair = manual_to_pair(
            clamp(throttle, 0.0, 1.0),
            clamp(steering, -1.0, 1.0),
            self.mapping,
        )
        return PwmCommand(pair.left_us, pair.right_us)

    def _neutral(self) -> PwmCommand:
        return PwmCommand(
            self.mapping.neutral_us,
            self.mapping.neutral_us,
        )

    def _fresh(self, updated_at: float | None, now: float) -> bool:
        return (
            updated_at is not None
            and 0.0 <= now - updated_at <= self.command_timeout_s
        )

    def _bounded(self, command: VelocityCommand) -> VelocityCommand:
        return VelocityCommand(
            linear_x=clamp(
                command.linear_x,
                -self.max_linear_mps,
                self.max_linear_mps,
            ),
            angular_z=clamp(
                command.angular_z,
                -self.max_angular_rps,
                self.max_angular_rps,
            ),
        )
