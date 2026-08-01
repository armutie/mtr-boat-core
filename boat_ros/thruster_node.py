from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, String

from thruster_control import (
    Esp32ThrusterSerial,
    ThrusterMapping,
)


class ThrusterNode(Node):
    def __init__(self) -> None:
        super().__init__("thruster_node")

        self.declare_parameter("port", "/dev/mtr_esp32")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("command_topic", "thrusters/command")
        self.declare_parameter("status_topic", "thrusters/status")
        self.declare_parameter("send_rate_hz", 20.0)
        self.declare_parameter("command_timeout_s", 0.5)
        self.declare_parameter("neutral_us", 1500)
        self.declare_parameter("hard_min_us", 1350)
        self.declare_parameter("hard_max_us", 2000)
        self.declare_parameter("require_dual_firmware", True)

        self.mapping = ThrusterMapping(
            neutral_us=int(self.get_parameter("neutral_us").value),
            hard_min_us=int(self.get_parameter("hard_min_us").value),
            hard_max_us=int(self.get_parameter("hard_max_us").value),
        )
        self.command_timeout_s = max(
            0.05,
            float(self.get_parameter("command_timeout_s").value),
        )
        port = str(self.get_parameter("port").value)
        baud = int(self.get_parameter("baud").value)
        self.serial = Esp32ThrusterSerial(port, baud=baud)
        if bool(self.get_parameter("require_dual_firmware").value):
            identity = self.serial.probe_dual_firmware()
            if identity is None:
                self.serial.close()
                raise RuntimeError(
                    "dual-thruster firmware is required for ROS control"
                )
            self.get_logger().info(
                f"Verified dual-thruster firmware: {identity}"
            )

        self._left_us = self.mapping.neutral_us
        self._right_us = self.mapping.neutral_us
        self._command_at: float | None = None
        self.status_publisher = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            10,
        )

        self.create_subscription(
            Int32MultiArray,
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

    def on_command(self, message: Int32MultiArray) -> None:
        if len(message.data) < 2:
            self.get_logger().warning(
                "Ignored thruster command without left/right PWM"
            )
            return
        self._left_us = self.mapping.clamp_pwm(message.data[0])
        self._right_us = self.mapping.clamp_pwm(message.data[1])
        self._command_at = time.monotonic()

    def send_command(self) -> None:
        now = time.monotonic()
        fresh = (
            self._command_at is not None
            and now - self._command_at <= self.command_timeout_s
        )
        left_us = (
            self._left_us if fresh else self.mapping.neutral_us
        )
        right_us = (
            self._right_us if fresh else self.mapping.neutral_us
        )
        response = self.serial.send_pwm_pair(left_us, right_us)
        if response is None:
            self.get_logger().warning(
                "No acknowledgement from ESP32 thruster firmware"
            )
            return
        message = String()
        message.data = response
        self.status_publisher.publish(message)

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
