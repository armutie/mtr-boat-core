from __future__ import annotations

from dataclasses import dataclass


VALID_CONTROL_MODES = ("off", "manual", "auto")


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, float(value)))


@dataclass(frozen=True)
class VelocityCommand:
    linear_x: float = 0.0
    angular_z: float = 0.0


class ControlSupervisor:
    """Select manual commands or blend operator trim into auto commands."""

    def __init__(
        self,
        command_timeout_s: float = 0.5,
        max_linear_mps: float = 1.0,
        max_angular_rps: float = 1.0,
    ) -> None:
        self.command_timeout_s = max(0.05, command_timeout_s)
        self.max_linear_mps = max(0.01, max_linear_mps)
        self.max_angular_rps = max(0.01, max_angular_rps)
        self.mode = "off"
        self._operator = VelocityCommand()
        self._automatic = VelocityCommand()
        self._operator_at: float | None = None
        self._automatic_at: float | None = None

    def set_mode(self, mode: str) -> None:
        if mode not in VALID_CONTROL_MODES:
            raise ValueError(f"mode must be one of {VALID_CONTROL_MODES}")
        if mode != self.mode:
            self._operator_at = None
            self._automatic_at = None
        self.mode = mode

    def update_operator(self, command: VelocityCommand, now: float) -> None:
        self._operator = self._bounded(command)
        self._operator_at = now

    def update_auto(self, command: VelocityCommand, now: float) -> None:
        self._automatic = self._bounded(command)
        self._automatic_at = now

    def output(self, now: float) -> VelocityCommand:
        if self.mode == "manual":
            if not self._fresh(self._operator_at, now):
                return VelocityCommand()
            return self._operator

        if self.mode == "auto":
            if not self._fresh(self._automatic_at, now):
                return VelocityCommand()
            operator = (
                self._operator
                if self._fresh(self._operator_at, now)
                else VelocityCommand()
            )
            return self._bounded(
                VelocityCommand(
                    linear_x=self._automatic.linear_x + operator.linear_x,
                    angular_z=self._automatic.angular_z + operator.angular_z,
                )
            )

        return VelocityCommand()

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
