from __future__ import annotations

import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

from boat_core.control import clamp
from thruster_control import (
    Esp32ThrusterSerial,
    ThrusterMapping,
    manual_to_pair,
)


class ThrusterNode(Node):
    def __init__(self) -> None:
        super().__init__("thruster_node")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("command_topic", "cmd_vel")
        self.declare_parameter("send_rate_hz", 20.0)
        self.declare_parameter("command_timeout_s", 0.5)
        self.declare_parameter("throttle_slew_per_s", 4.0)
        self.declare_parameter("max_linear_mps", 1.0)
        self.declare_parameter("max_angular_rps", 1.0)
        self.declare_parameter("neutral_us", 1500)
        self.declare_parameter("forward_min_us", 1565)
        self.declare_parameter("forward_max_us", 1650)
        self.declare_parameter("hard_min_us", 1350)
        self.declare_parameter("hard_max_us", 2000)
        self.declare_parameter("steering_slowdown", 0.35)
        self.declare_parameter("require_dual_firmware", True)

        self.mapping = ThrusterMapping(
            neutral_us=int(self.get_parameter("neutral_us").value),
            forward_min_us=int(
                self.get_parameter("forward_min_us").value
            ),
            forward_max_us=int(
                self.get_parameter("forward_max_us").value
            ),
            hard_min_us=int(self.get_parameter("hard_min_us").value),
            hard_max_us=int(self.get_parameter("hard_max_us").value),
            steering_slowdown=float(
                self.get_parameter("steering_slowdown").value
            ),
        )
        self.max_linear_mps = max(
            0.01,
            float(self.get_parameter("max_linear_mps").value),
        )
        self.max_angular_rps = max(
            0.01,
            float(self.get_parameter("max_angular_rps").value),
        )
        self.command_timeout_s = max(
            0.05,
            float(self.get_parameter("command_timeout_s").value),
        )
        self.throttle_slew_per_s = max(
            0.0,
            float(self.get_parameter("throttle_slew_per_s").value),
        )

        port = str(self.get_parameter("port").value)
        baud = int(self.get_parameter("baud").value)
        self.serial = Esp32ThrusterSerial(port, baud=baud)
        if bool(self.get_parameter("require_dual_firmware").value):
            banner = self.serial.ready_banner or ""
            if not banner.startswith("READY L="):
                self.serial.close()
                raise RuntimeError(
                    "dual-thruster firmware is required for ROS control"
                )

        self._command = Twist()
        self._command_at: float | None = None
        self._throttle = 0.0
        self._last_tick = time.monotonic()

        self.create_subscription(
            Twist,
            str(self.get_parameter("command_topic").value),
            self.on_command,
            10,
        )
        send_rate_hz = max(
            1.0,
            float(self.get_parameter("send_rate_hz").value),
        )
        self.create_timer(1.0 / send_rate_hz, self.send_command)
        self.get_logger().info(
            f"Thruster node owns {port} at {baud} baud"
        )

    def on_command(self, message: Twist) -> None:
        self._command = message
        self._command_at = time.monotonic()

    def send_command(self) -> None:
        now = time.monotonic()
        dt = max(0.0, now - self._last_tick)
        self._last_tick = now
        fresh = (
            self._command_at is not None
            and now - self._command_at <= self.command_timeout_s
        )

        target_throttle = 0.0
        steering = 0.0
        if fresh:
            target_throttle = clamp(
                self._command.linear.x / self.max_linear_mps,
                0.0,
                1.0,
            )
            steering = clamp(
                self._command.angular.z / self.max_angular_rps,
                -1.0,
                1.0,
            )

        if (
            target_throttle <= self._throttle
            or self.throttle_slew_per_s <= 0.0
        ):
            self._throttle = target_throttle
        else:
            self._throttle = min(
                target_throttle,
                self._throttle + self.throttle_slew_per_s * dt,
            )

        pair = manual_to_pair(
            self._throttle,
            steering,
            self.mapping,
            enabled=fresh,
        )
        self.serial.send_pwm_pair(pair.left_us, pair.right_us)

    def destroy_node(self) -> bool:
        try:
            self.serial.stop()
            self.serial.close()
        finally:
            return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ThrusterNode()
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
