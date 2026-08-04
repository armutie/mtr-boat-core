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


class ActuatorArmLatch:
    """Require an observed off state before enabling actuator output."""

    def __init__(self) -> None:
        self.mode: str | None = None
        self.armed = False
        self._off_observed = False

    def update_mode(self, mode: str) -> bool:
        if mode not in VALID_CONTROL_MODES:
            raise ValueError(f"mode must be one of {VALID_CONTROL_MODES}")

        previous_mode = self.mode
        self.mode = mode
        if mode == "off":
            self._off_observed = True
            self.armed = False
        elif self._off_observed and previous_mode == "off":
            self.armed = True
        return self.armed


class ControlSupervisor:
    """Select one final left/right actuator command.

    Manual input is converted from velocity intent to PWM here. Autonomous
    input is already an actuator command and passes through unchanged unless
    the operator is actively holding steering authority.
    """

    def __init__(
        self,
        command_timeout_s: float = 0.5,
        max_linear_mps: float = 1.0,
        max_angular_rps: float = 1.0,
        throttle_slew_per_s: float = 0.0,
        direction_change_neutral_s: float = 0.2,
        mapping: ThrusterMapping | None = None,
    ) -> None:
        self.command_timeout_s = max(0.05, command_timeout_s)
        self.max_linear_mps = max(0.01, max_linear_mps)
        self.max_angular_rps = max(0.01, max_angular_rps)
        self.throttle_slew_per_s = max(0.0, throttle_slew_per_s)
        self.direction_change_neutral_s = max(
            0.0,
            direction_change_neutral_s,
        )
        self.mapping = mapping or ThrusterMapping()
        self.mode = "off"
        self._operator = VelocityCommand()
        self._automatic = self._neutral()
        self._operator_at: float | None = None
        self._automatic_at: float | None = None
        self._steering_takeover_active = False
        self._steering_takeover = 0.0
        self._steering_takeover_at: float | None = None
        self._manual_throttle = 0.0
        self._last_output_at: float | None = None
        self._actuator_session: str | None = None
        self._directions = [0, 0]
        self._neutral_since: list[float | None] = [None, None]

    def set_mode(self, mode: str) -> None:
        if mode not in VALID_CONTROL_MODES:
            raise ValueError(f"mode must be one of {VALID_CONTROL_MODES}")
        if mode != self.mode:
            self._operator_at = None
            self._automatic_at = None
            self._steering_takeover_active = False
            self._steering_takeover_at = None
            self._manual_throttle = 0.0
            self._last_output_at = None
        self.mode = mode

    def register_actuator_session(self, session_id: str) -> bool:
        session_id = session_id.strip()
        if not session_id:
            raise ValueError("actuator session ID cannot be empty")
        if session_id == self._actuator_session:
            return False

        self._actuator_session = session_id
        self.set_mode("off")
        return True

    def update_operator(self, command: VelocityCommand, now: float) -> None:
        self._operator = self._bounded(command)
        self._operator_at = now

    def update_auto(self, command: PwmCommand, now: float) -> None:
        self._automatic = PwmCommand(
            self.mapping.clamp_pwm(command.left_us),
            self.mapping.clamp_pwm(command.right_us),
        )
        self._automatic_at = now

    def update_steering_takeover(
        self,
        active: bool,
        steering: float,
        now: float,
    ) -> None:
        self._steering_takeover_active = bool(active)
        self._steering_takeover = clamp(steering, -1.0, 1.0)
        self._steering_takeover_at = now

    def output(self, now: float) -> PwmCommand:
        if self.mode == "manual":
            if not self._fresh(self._operator_at, now):
                self._manual_throttle = 0.0
                self._last_output_at = now
                desired = self._neutral()
            else:
                desired = self._manual_output(now)
        elif self.mode == "auto":
            if not self._fresh(self._automatic_at, now):
                desired = self._neutral()
            elif (
                not self._steering_takeover_active
                or not self._fresh(self._steering_takeover_at, now)
            ):
                desired = self._automatic
            else:
                desired = self._steering_takeover_output()
        else:
            desired = self._neutral()
        return self._guard_direction_change(desired, now)

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

    def _steering_takeover_output(self) -> PwmCommand:
        throttle, _auto_steering = pair_to_manual(
            self._automatic.left_us,
            self._automatic.right_us,
            self.mapping,
        )
        pair = manual_to_pair(
            clamp(throttle, 0.0, 1.0),
            self._steering_takeover,
            self.mapping,
        )
        return PwmCommand(pair.left_us, pair.right_us)

    def _neutral(self) -> PwmCommand:
        return PwmCommand(
            self.mapping.neutral_us,
            self.mapping.neutral_us,
        )

    def _guard_direction_change(
        self,
        command: PwmCommand,
        now: float,
    ) -> PwmCommand:
        values = (command.left_us, command.right_us)
        guarded = [
            self._guard_channel(index, value, now)
            for index, value in enumerate(values)
        ]
        return PwmCommand(guarded[0], guarded[1])

    def _guard_channel(
        self,
        index: int,
        requested_us: int,
        now: float,
    ) -> int:
        neutral = self.mapping.neutral_us
        requested_direction = (
            1
            if requested_us > neutral
            else -1
            if requested_us < neutral
            else 0
        )
        previous_direction = self._directions[index]

        if requested_direction == 0:
            if (
                previous_direction != 0
                and self._neutral_since[index] is None
            ):
                self._neutral_since[index] = now
            return neutral

        if previous_direction in (0, requested_direction):
            self._directions[index] = requested_direction
            self._neutral_since[index] = None
            return requested_us

        neutral_since = self._neutral_since[index]
        if neutral_since is None:
            self._neutral_since[index] = now
            return neutral
        if now - neutral_since < self.direction_change_neutral_s:
            return neutral

        self._directions[index] = requested_direction
        self._neutral_since[index] = None
        return requested_us

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
