from pathlib import Path
import unittest


class RosPwmPathTests(unittest.TestCase):
    def test_autonomy_publishes_pwm_without_twist_conversion(self) -> None:
        source = Path("boat_ros/autonomy_node.py").read_text(
            encoding="utf-8",
        )
        self.assertIn("Int32MultiArray", source)
        self.assertIn("command.data = [left_us, right_us]", source)
        self.assertNotIn("command.linear.x", source)
        self.assertNotIn("command.angular.z", source)

    def test_supervisor_owns_final_pwm_topic(self) -> None:
        source = Path("boat_ros/control_supervisor_node.py").read_text(
            encoding="utf-8",
        )
        self.assertIn('"thrusters/auto"', source)
        self.assertIn('"thrusters/command"', source)
        self.assertIn(
            "output.data = [command.left_us, command.right_us]",
            source,
        )

    def test_thruster_node_sends_supervised_pair_directly(self) -> None:
        source = Path("boat_ros/thruster_node.py").read_text(
            encoding="utf-8",
        )
        self.assertIn(
            "self.serial.send_pwm_pair(left_us, right_us)",
            source,
        )
        self.assertNotIn("manual_to_pair", source)
        self.assertNotIn("pair_to_manual", source)


if __name__ == "__main__":
    unittest.main()
