import unittest

from boat_core.control import (
    ControlSupervisor,
    PwmCommand,
    VelocityCommand,
)
from thruster_control import ThrusterMapping


class ControlSupervisorTests(unittest.TestCase):
    def setUp(self) -> None:
        self.mapping = ThrusterMapping(
            neutral_us=1500,
            forward_min_us=1565,
            forward_max_us=1650,
        )
        self.control = ControlSupervisor(
            command_timeout_s=0.5,
            max_linear_mps=2.0,
            max_angular_rps=1.0,
            mapping=self.mapping,
        )

    def test_starts_off_at_neutral(self) -> None:
        self.assertEqual(
            self.control.output(10.0),
            PwmCommand(1500, 1500),
        )

    def test_manual_maps_fresh_operator_command_once(self) -> None:
        self.control.set_mode("manual")
        self.control.update_operator(VelocityCommand(1.2, -0.4), 10.0)

        self.assertEqual(
            self.control.output(10.25),
            PwmCommand(1596, 1636),
        )
        self.assertEqual(
            self.control.output(10.6),
            PwmCommand(1500, 1500),
        )

    def test_auto_pwm_passes_through_exactly(self) -> None:
        self.control.set_mode("auto")
        self.control.update_auto(PwmCommand(1565, 1575), 10.0)

        self.assertEqual(
            self.control.output(10.2),
            PwmCommand(1565, 1575),
        )

    def test_zero_operator_input_does_not_round_trip_auto_pwm(self) -> None:
        self.control.set_mode("auto")
        self.control.update_auto(PwmCommand(1565, 1565), 10.0)
        self.control.update_operator(VelocityCommand(), 10.1)

        self.assertEqual(
            self.control.output(10.2),
            PwmCommand(1565, 1565),
        )

    def test_fresh_operator_input_trims_auto(self) -> None:
        self.control.set_mode("auto")
        self.control.update_auto(PwmCommand(1600, 1600), 10.0)
        self.control.update_operator(VelocityCommand(-0.25, 0.4), 10.1)

        self.assertEqual(
            self.control.output(10.2),
            PwmCommand(1599, 1580),
        )

    def test_auto_continues_when_operator_trim_expires(self) -> None:
        self.control.set_mode("auto")
        self.control.update_auto(PwmCommand(1600, 1575), 10.0)
        self.control.update_operator(VelocityCommand(-0.25, 0.4), 9.6)

        self.assertEqual(
            self.control.output(10.2),
            PwmCommand(1600, 1575),
        )

    def test_auto_stops_when_auto_command_expires(self) -> None:
        self.control.set_mode("auto")
        self.control.update_auto(PwmCommand(1600, 1575), 10.0)

        self.assertEqual(
            self.control.output(10.6),
            PwmCommand(1500, 1500),
        )

    def test_auto_pwm_is_hard_clamped(self) -> None:
        self.control.set_mode("auto")
        self.control.update_auto(PwmCommand(2200, 1200), 10.0)

        self.assertEqual(
            self.control.output(10.1),
            PwmCommand(2000, 1350),
        )

    def test_manual_rise_slew_does_not_affect_auto(self) -> None:
        control = ControlSupervisor(
            command_timeout_s=0.5,
            max_linear_mps=2.0,
            max_angular_rps=1.0,
            throttle_slew_per_s=0.5,
            mapping=self.mapping,
        )
        control.set_mode("manual")
        control.update_operator(VelocityCommand(2.0, 0.0), 10.0)

        self.assertEqual(control.output(10.0), PwmCommand(1500, 1500))
        self.assertEqual(control.output(10.2), PwmCommand(1574, 1574))

        control.set_mode("auto")
        control.update_auto(PwmCommand(1650, 1500), 11.0)
        self.assertEqual(control.output(11.0), PwmCommand(1650, 1500))


if __name__ == "__main__":
    unittest.main()
