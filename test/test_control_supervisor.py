import unittest

from boat_core.control import ControlSupervisor, VelocityCommand


class ControlSupervisorTests(unittest.TestCase):
    def setUp(self) -> None:
        self.control = ControlSupervisor(
            command_timeout_s=0.5,
            max_linear_mps=2.0,
            max_angular_rps=1.0,
        )

    def test_starts_off(self) -> None:
        self.assertEqual(self.control.output(10.0), VelocityCommand())

    def test_manual_uses_fresh_operator_command(self) -> None:
        self.control.set_mode("manual")
        self.control.update_operator(VelocityCommand(1.2, -0.4), 10.0)

        self.assertEqual(
            self.control.output(10.25),
            VelocityCommand(1.2, -0.4),
        )
        self.assertEqual(self.control.output(10.6), VelocityCommand())

    def test_auto_blends_fresh_operator_trim(self) -> None:
        self.control.set_mode("auto")
        self.control.update_auto(VelocityCommand(1.0, 0.2), 10.0)
        self.control.update_operator(VelocityCommand(-0.25, 0.4), 10.1)

        output = self.control.output(10.2)
        self.assertAlmostEqual(output.linear_x, 0.75)
        self.assertAlmostEqual(output.angular_z, 0.6)

    def test_auto_continues_when_operator_trim_expires(self) -> None:
        self.control.set_mode("auto")
        self.control.update_auto(VelocityCommand(1.0, 0.2), 10.0)
        self.control.update_operator(VelocityCommand(-0.25, 0.4), 9.6)

        self.assertEqual(
            self.control.output(10.2),
            VelocityCommand(1.0, 0.2),
        )

    def test_auto_stops_when_auto_command_expires(self) -> None:
        self.control.set_mode("auto")
        self.control.update_auto(VelocityCommand(1.0, 0.2), 10.0)

        self.assertEqual(self.control.output(10.6), VelocityCommand())

    def test_blended_output_is_clamped(self) -> None:
        self.control.set_mode("auto")
        self.control.update_auto(VelocityCommand(1.8, 0.8), 10.0)
        self.control.update_operator(VelocityCommand(1.0, 0.8), 10.0)

        self.assertEqual(
            self.control.output(10.1),
            VelocityCommand(2.0, 1.0),
        )


if __name__ == "__main__":
    unittest.main()
