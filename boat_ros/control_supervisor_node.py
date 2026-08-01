from __future__ import annotations

import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, String

from boat_core.control import (
    ControlSupervisor,
    PwmCommand,
    VelocityCommand,
)
from thruster_control import ThrusterMapping


class ControlSupervisorNode(Node):
    def __init__(self) -> None:
        super().__init__("control_supervisor_node")

        self.declare_parameter("operator_topic", "cmd_vel/operator")
        self.declare_parameter("auto_topic", "thrusters/auto")
        self.declare_parameter("output_topic", "thrusters/command")
        self.declare_parameter("mode_request_topic", "control/mode_request")
        self.declare_parameter("mode_topic", "control/mode")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("command_timeout_s", 0.5)
        self.declare_parameter("max_linear_mps", 1.0)
        self.declare_parameter("max_angular_rps", 1.0)
        self.declare_parameter("throttle_slew_per_s", 4.0)
        self.declare_parameter("neutral_us", 1500)
        self.declare_parameter("forward_min_us", 1565)
        self.declare_parameter("forward_max_us", 1650)
        self.declare_parameter("hard_min_us", 1350)
        self.declare_parameter("hard_max_us", 2000)

        self.control = ControlSupervisor(
            command_timeout_s=float(
                self.get_parameter("command_timeout_s").value
            ),
            max_linear_mps=float(
                self.get_parameter("max_linear_mps").value
            ),
            max_angular_rps=float(
                self.get_parameter("max_angular_rps").value
            ),
            throttle_slew_per_s=float(
                self.get_parameter("throttle_slew_per_s").value
            ),
            mapping=ThrusterMapping(
                neutral_us=int(self.get_parameter("neutral_us").value),
                forward_min_us=int(
                    self.get_parameter("forward_min_us").value
                ),
                forward_max_us=int(
                    self.get_parameter("forward_max_us").value
                ),
                hard_min_us=int(
                    self.get_parameter("hard_min_us").value
                ),
                hard_max_us=int(
                    self.get_parameter("hard_max_us").value
                ),
            ),
        )

        self.output_publisher = self.create_publisher(
            Int32MultiArray,
            str(self.get_parameter("output_topic").value),
            10,
        )
        self.mode_publisher = self.create_publisher(
            String,
            str(self.get_parameter("mode_topic").value),
            10,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("operator_topic").value),
            self.on_operator,
            10,
        )
        self.create_subscription(
            Int32MultiArray,
            str(self.get_parameter("auto_topic").value),
            self.on_auto,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("mode_request_topic").value),
            self.on_mode,
            10,
        )

        rate_hz = max(
            1.0,
            float(self.get_parameter("publish_rate_hz").value),
        )
        self.create_timer(1.0 / rate_hz, self.publish_command)
        self.get_logger().info("Control supervisor started in off mode")

    def on_operator(self, message: Twist) -> None:
        self.control.update_operator(
            VelocityCommand(message.linear.x, message.angular.z),
            time.monotonic(),
        )

    def on_auto(self, message: Int32MultiArray) -> None:
        if len(message.data) < 2:
            self.get_logger().warning(
                "Ignored auto command without left/right PWM"
            )
            return
        self.control.update_auto(
            PwmCommand(int(message.data[0]), int(message.data[1])),
            time.monotonic(),
        )

    def on_mode(self, message: String) -> None:
        previous_mode = self.control.mode
        try:
            self.control.set_mode(message.data.strip().lower())
        except ValueError as exc:
            self.get_logger().warning(str(exc))
            return
        if self.control.mode != previous_mode:
            self.get_logger().info(
                f"Control mode: {self.control.mode}"
            )

    def publish_command(self) -> None:
        command = self.control.output(time.monotonic())
        output = Int32MultiArray()
        output.data = [command.left_us, command.right_us]
        self.output_publisher.publish(output)

        mode = String()
        mode.data = self.control.mode
        self.mode_publisher.publish(mode)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControlSupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
