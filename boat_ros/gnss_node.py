from __future__ import annotations

import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus

from gnss import NmeaReader


class GnssNode(Node):
    """Publish parsed NMEA position fixes using the standard ROS message."""

    def __init__(self) -> None:
        super().__init__("gnss_node")

        self.declare_parameter("port", "/dev/mtr_gnss")
        self.declare_parameter("baud", 38400)
        self.declare_parameter("frame_id", "gnss_link")
        self.declare_parameter("topic", "gnss/fix")
        self.declare_parameter("velocity_topic", "gnss/velocity")
        self.declare_parameter("velocity_frame_id", "map")
        self.declare_parameter("diagnostics_topic", "/diagnostics")
        self.declare_parameter("diagnostics_rate_hz", 1.0)
        self.declare_parameter("poll_hz", 20.0)
        self.declare_parameter("serial_timeout_s", 0.05)

        port = str(self.get_parameter("port").value)
        baud = int(self.get_parameter("baud").value)
        timeout_s = float(self.get_parameter("serial_timeout_s").value)
        if not port:
            raise ValueError("GNSS parameter 'port' must not be empty")

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.publisher = self.create_publisher(
            NavSatFix,
            str(self.get_parameter("topic").value),
            qos_profile_sensor_data,
        )
        self.velocity_publisher = self.create_publisher(
            TwistStamped,
            str(self.get_parameter("velocity_topic").value),
            qos_profile_sensor_data,
        )
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter("diagnostics_topic").value),
            10,
        )
        self.velocity_frame_id = str(
            self.get_parameter("velocity_frame_id").value
        )
        diagnostics_rate_hz = max(
            float(self.get_parameter("diagnostics_rate_hz").value),
            0.1,
        )
        self.diagnostics_period_s = 1.0 / diagnostics_rate_hz
        self._last_diagnostics_at = 0.0
        self._latest_satellites: int | None = None
        self._latest_hdop: float | None = None
        self._latest_speed_mps: float | None = None
        self._latest_heading_deg: float | None = None
        self.port = port
        self.reader = NmeaReader(port, baud=baud, timeout=max(timeout_s, 0.0))
        self._last_error_log_s = 0.0

        poll_hz = max(float(self.get_parameter("poll_hz").value), 0.1)
        self.timer = self.create_timer(1.0 / poll_hz, self.poll_once)
        self.get_logger().info(
            f"Reading GNSS from {port} at {baud} baud; "
            f"publishing {self.get_parameter('topic').value}"
        )

    def poll_once(self) -> None:
        try:
            fix = self.reader.read_fix()
        except Exception as exc:
            self._log_read_error(exc)
            return

        if fix is None:
            return

        if fix.satellites is not None:
            self._latest_satellites = fix.satellites
        if fix.hdop is not None:
            self._latest_hdop = fix.hdop
        if fix.speed_mps is not None:
            self._latest_speed_mps = fix.speed_mps
        if fix.heading_deg is not None:
            self._latest_heading_deg = fix.heading_deg

        message = NavSatFix()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        message.status.status = (
            NavSatStatus.STATUS_FIX if fix.fix != "none" else NavSatStatus.STATUS_NO_FIX
        )
        message.status.service = NavSatStatus.SERVICE_GPS
        message.latitude = fix.lat if fix.lat is not None else math.nan
        message.longitude = fix.lon if fix.lon is not None else math.nan
        message.altitude = fix.altitude_m if fix.altitude_m is not None else math.nan
        message.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.publisher.publish(message)
        self._publish_velocity(fix, message.header.stamp)
        self._publish_diagnostics(fix, message.header.stamp)

    def _publish_velocity(self, fix, stamp) -> None:
        if fix.speed_mps is None or fix.heading_deg is None:
            return

        course_rad = math.radians(float(fix.heading_deg))
        message = TwistStamped()
        message.header.stamp = stamp
        message.header.frame_id = self.velocity_frame_id
        message.twist.linear.x = float(fix.speed_mps) * math.sin(course_rad)
        message.twist.linear.y = float(fix.speed_mps) * math.cos(course_rad)
        self.velocity_publisher.publish(message)

    def _publish_diagnostics(self, fix, stamp) -> None:
        now = time.monotonic()
        if now - self._last_diagnostics_at < self.diagnostics_period_s:
            return
        self._last_diagnostics_at = now

        status = DiagnosticStatus()
        status.name = "GNSS"
        status.hardware_id = self.port
        has_fix = fix.fix != "none"
        status.level = (
            DiagnosticStatus.OK if has_fix else DiagnosticStatus.WARN
        )
        status.message = "fix" if has_fix else "no fix"
        status.values = [
            KeyValue(
                key="satellites",
                value=(
                    ""
                    if self._latest_satellites is None
                    else str(self._latest_satellites)
                ),
            ),
            KeyValue(
                key="hdop",
                value=(
                    ""
                    if self._latest_hdop is None
                    else str(self._latest_hdop)
                ),
            ),
            KeyValue(
                key="speed_mps",
                value=(
                    ""
                    if self._latest_speed_mps is None
                    else str(self._latest_speed_mps)
                ),
            ),
            KeyValue(
                key="heading_deg",
                value=(
                    ""
                    if self._latest_heading_deg is None
                    else str(self._latest_heading_deg)
                ),
            ),
        ]
        message = DiagnosticArray()
        message.header.stamp = stamp
        message.status = [status]
        self.diagnostics_publisher.publish(message)

    def _log_read_error(self, exc: Exception) -> None:
        now = time.monotonic()
        if now - self._last_error_log_s >= 5.0:
            self.get_logger().error(f"GNSS read failed: {exc}")
            self._last_error_log_s = now

    def destroy_node(self) -> bool:
        self.reader.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GnssNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
