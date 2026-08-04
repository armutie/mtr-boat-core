import unittest

from boat_core.control import (
    ActuatorArmLatch,
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

    def test_sharp_manual_turn_reverses_inside_thruster(self) -> None:
        self.control.set_mode("manual")
        self.control.update_operator(VelocityCommand(1.0, 1.0), 10.0)

        self.assertEqual(
            self.control.output(10.0),
            PwmCommand(1650, 1445),
        )

    def test_direction_change_holds_neutral_before_reverse(self) -> None:
        self.control.set_mode("manual")
        self.control.update_operator(VelocityCommand(2.0, 0.0), 10.0)
        self.assertEqual(
            self.control.output(10.0),
            PwmCommand(1650, 1650),
        )

        self.control.update_operator(VelocityCommand(2.0, 1.0), 10.1)
        self.assertEqual(
            self.control.output(10.1),
            PwmCommand(1650, 1500),
        )
        self.assertEqual(
            self.control.output(10.2),
            PwmCommand(1650, 1500),
        )
        self.assertEqual(
            self.control.output(10.31),
            PwmCommand(1650, 1425),
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

    def test_active_takeover_replaces_auto_steering(self) -> None:
        self.control.set_mode("auto")
        self.control.update_auto(PwmCommand(1600, 1600), 10.0)
        self.control.update_steering_takeover(True, 0.4, 10.1)

        self.assertEqual(
            self.control.output(10.2),
            PwmCommand(1614, 1586),
        )

    def test_centered_takeover_overrides_auto_turn(self) -> None:
        self.control.set_mode("auto")
        self.control.update_auto(PwmCommand(1600, 1575), 10.0)
        self.control.update_steering_takeover(True, 0.0, 10.1)

        self.assertEqual(
            self.control.output(10.2),
            PwmCommand(1588, 1588),
        )

    def test_auto_continues_when_steering_takeover_expires(self) -> None:
        self.control.set_mode("auto")
        self.control.update_auto(PwmCommand(1600, 1575), 10.0)
        self.control.update_steering_takeover(True, 0.4, 9.6)

        self.assertEqual(
            self.control.output(10.2),
            PwmCommand(1600, 1575),
        )

    def test_releasing_takeover_returns_to_auto_immediately(self) -> None:
        self.control.set_mode("auto")
        self.control.update_auto(PwmCommand(1600, 1575), 10.0)
        self.control.update_steering_takeover(True, -0.7, 10.1)
        self.control.update_steering_takeover(False, 0.0, 10.2)

        self.assertEqual(
            self.control.output(10.2),
            PwmCommand(1600, 1575),
        )

    def test_manual_operator_command_cannot_fight_auto(self) -> None:
        self.control.set_mode("auto")
        self.control.update_auto(PwmCommand(1600, 1575), 10.0)
        self.control.update_operator(VelocityCommand(2.0, -1.0), 10.1)

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

    def test_new_actuator_session_forces_off(self) -> None:
        self.assertTrue(self.control.register_actuator_session("first"))
        self.control.set_mode("auto")
        self.control.update_auto(PwmCommand(1600, 1575), 10.0)
        self.assertEqual(
            self.control.output(10.1),
            PwmCommand(1600, 1575),
        )

        self.assertFalse(self.control.register_actuator_session("first"))
        self.assertEqual(self.control.mode, "auto")
        self.assertTrue(self.control.register_actuator_session("second"))
        self.assertEqual(self.control.mode, "off")
        self.assertEqual(
            self.control.output(10.2),
            PwmCommand(1500, 1500),
        )

    def test_rejects_an_empty_actuator_session(self) -> None:
        with self.assertRaises(ValueError):
            self.control.register_actuator_session(" ")


class ActuatorArmLatchTests(unittest.TestCase):
    def test_requires_off_before_first_arm(self) -> None:
        latch = ActuatorArmLatch()

        self.assertFalse(latch.update_mode("auto"))
        self.assertFalse(latch.armed)
        self.assertFalse(latch.update_mode("off"))
        self.assertTrue(latch.update_mode("auto"))
        self.assertTrue(latch.armed)

    def test_off_disarms_and_a_new_selection_rearms(self) -> None:
        latch = ActuatorArmLatch()

        latch.update_mode("off")
        latch.update_mode("manual")
        self.assertTrue(latch.armed)
        self.assertFalse(latch.update_mode("off"))
        self.assertFalse(latch.armed)
        self.assertTrue(latch.update_mode("manual"))

    def test_rejects_an_unknown_mode(self) -> None:
        latch = ActuatorArmLatch()

        with self.assertRaises(ValueError):
            latch.update_mode("cruise")


if __name__ == "__main__":
    unittest.main()
