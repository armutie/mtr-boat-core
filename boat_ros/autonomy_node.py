from __future__ import annotations

import copy
from dataclasses import fields
import json
import math
import threading
import time

from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import String

from boat_core.autonomy import AutoConfig, AutoController
from radar_nav.waypoint import WaypointNavConfig
from thruster_control import ThrusterMapping, pair_to_manual


class SnapshotReader:
    def __init__(self, initial: dict, stale_after_s: float) -> None:
        self._lock = threading.Lock()
        self._latest = initial
        self._updated_at: float | None = None
        self.stale_after_s = stale_after_s

    def update(self, values: dict) -> None:
        with self._lock:
            self._latest.update(values)
            self._updated_at = time.time()

    def snapshot(self) -> tuple[str, dict]:
        now = time.time()
        with self._lock:
            latest = copy.deepcopy(self._latest)
            updated_at = self._updated_at
        if updated_at is None:
            return "waiting", latest
        latest["age_s"] = round(now - updated_at, 2)
        return (
            "stale" if now - updated_at > self.stale_after_s else "live",
            latest,
        )


class AutonomyControlState:
    """Small adapter between the ROS runtime and the existing control core."""

    def __init__(self, neutral_us: int) -> None:
        self._lock = threading.Lock()
        self.neutral_us = neutral_us
        self.mode = "off"
        self.left_us = neutral_us
        self.right_us = neutral_us
        self.reason = "off"
        self.auto_status: dict = {
            "state": "idle",
            "reason": "no route",
        }

    def set_mode(self, mode: str) -> None:
        with self._lock:
            self.mode = mode if mode in ("off", "manual", "auto") else "off"
            if self.mode != "auto":
                self.left_us = self.neutral_us
                self.right_us = self.neutral_us

    def apply_auto_pwm(
        self,
        left_us: int,
        right_us: int,
        reason: str,
        status: dict | None = None,
    ) -> None:
        with self._lock:
            if self.mode == "auto":
                self.left_us = int(left_us)
                self.right_us = int(right_us)
                self.reason = reason
            if status is not None:
                self.auto_status = dict(status)

    def set_auto_status(self, status: dict) -> None:
        with self._lock:
            self.auto_status = dict(status)

    def output(self) -> tuple[str, int, int, str, dict]:
        with self._lock:
            return (
                self.mode,
                self.left_us,
                self.right_us,
                self.reason,
                dict(self.auto_status),
            )


class AutonomyNode(Node):
    """Run waypoint autonomy as a ROS sensor consumer and command publisher."""

    def __init__(self) -> None:
        super().__init__("autonomy_node")

        self.declare_parameter("fix_topic", "gnss/fix")
        self.declare_parameter("velocity_topic", "gnss/velocity")
        self.declare_parameter("imu_topic", "imu/data")
        self.declare_parameter("mode_topic", "control/mode")
        self.declare_parameter("route_topic", "autonomy/route")
        self.declare_parameter("status_topic", "autonomy/status")
        self.declare_parameter("command_topic", "cmd_vel/auto")
        self.declare_parameter("max_linear_mps", 1.0)
        self.declare_parameter("max_angular_rps", 1.0)

        defaults = AutoConfig()
        for item in fields(AutoConfig):
            if item.name != "waypoint":
                self.declare_parameter(item.name, getattr(defaults, item.name))
        self.declare_parameter(
            "reach_radius_m",
            defaults.waypoint.reach_radius_m,
        )
        self.declare_parameter(
            "approach_slow_radius_m",
            defaults.waypoint.approach_slow_radius_m,
        )

        config_values = {
            item.name: self.get_parameter(item.name).value
            for item in fields(AutoConfig)
            if item.name != "waypoint"
        }
        config_values["waypoint"] = WaypointNavConfig(
            reach_radius_m=float(
                self.get_parameter("reach_radius_m").value
            ),
            approach_slow_radius_m=float(
                self.get_parameter("approach_slow_radius_m").value
            ),
        )
        self.config = AutoConfig(**config_values)
        self.mapping = ThrusterMapping(
            neutral_us=self.config.neutral_us,
            forward_min_us=self.config.level1_us,
            forward_max_us=self.config.level3_us,
        )
        self.max_linear_mps = float(
            self.get_parameter("max_linear_mps").value
        )
        self.max_angular_rps = float(
            self.get_parameter("max_angular_rps").value
        )

        self.gnss_reader = SnapshotReader(
            {
                "lat": None,
                "lon": None,
                "fix": "unavailable",
                "speed_mps": None,
                "heading_deg": None,
            },
            self.config.gnss_stale_s,
        )
        self.imu_reader = SnapshotReader(
            {
                "yaw_relative_deg": None,
                "gyro_z_dps": None,
            },
            self.config.imu_stale_s,
        )
        self.control = AutonomyControlState(self.config.neutral_us)
        self.controller = AutoController(
            self.control,
            self.gnss_reader,
            self.imu_reader,
            self.config,
        )

        route_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.command_publisher = self.create_publisher(
            Twist,
            str(self.get_parameter("command_topic").value),
            10,
        )
        self.status_publisher = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            10,
        )
        self.create_subscription(
            NavSatFix,
            str(self.get_parameter("fix_topic").value),
            self.on_fix,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            TwistStamped,
            str(self.get_parameter("velocity_topic").value),
            self.on_velocity,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            str(self.get_parameter("imu_topic").value),
            self.on_imu,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("mode_topic").value),
            self.on_mode,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("route_topic").value),
            self.on_route,
            route_qos,
        )
        self.create_timer(
            1.0 / max(float(self.config.control_hz), 1.0),
            self.update_control,
        )
        self.get_logger().info(
            "Autonomy consumes GNSS/IMU and publishes cmd_vel/auto"
        )

    def on_fix(self, message: NavSatFix) -> None:
        self.gnss_reader.update(
            {
                "lat": self._finite_or_none(message.latitude),
                "lon": self._finite_or_none(message.longitude),
                "fix": "fix" if int(message.status.status) >= 0 else "none",
            }
        )

    def on_velocity(self, message: TwistStamped) -> None:
        east_mps = float(message.twist.linear.x)
        north_mps = float(message.twist.linear.y)
        speed_mps = math.hypot(east_mps, north_mps)
        heading_deg = None
        if speed_mps > 1e-6:
            heading_deg = math.degrees(
                math.atan2(east_mps, north_mps)
            ) % 360.0
        self.gnss_reader.update(
            {
                "speed_mps": speed_mps,
                "heading_deg": heading_deg,
            }
        )

    def on_imu(self, message: Imu) -> None:
        orientation = message.orientation
        norm = math.sqrt(
            orientation.w * orientation.w
            + orientation.x * orientation.x
            + orientation.y * orientation.y
            + orientation.z * orientation.z
        )
        yaw_deg = None
        if norm > 1e-6:
            sin_yaw = 2.0 * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            )
            cos_yaw = 1.0 - 2.0 * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            )
            yaw_deg = math.degrees(math.atan2(sin_yaw, cos_yaw)) % 360.0
        self.imu_reader.update(
            {
                "yaw_relative_deg": yaw_deg,
                "gyro_z_dps": math.degrees(
                    float(message.angular_velocity.z)
                ),
            }
        )

    def on_mode(self, message: String) -> None:
        self.control.set_mode(message.data.strip().lower())

    def on_route(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            waypoints = (
                payload.get("waypoints", [])
                if isinstance(payload, dict)
                else payload
            )
            if not isinstance(waypoints, list):
                raise ValueError("route must contain a waypoint list")
            self.controller.set_waypoints(waypoints)
            self.get_logger().info(
                f"Loaded autonomy route with {len(waypoints)} waypoints"
            )
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"Rejected autonomy route: {exc}")

    def update_control(self) -> None:
        self.controller.tick()
        mode, left_us, right_us, reason, status = self.control.output()
        throttle, steering = pair_to_manual(
            left_us,
            right_us,
            self.mapping,
        )
        if mode != "auto":
            throttle = 0.0
            steering = 0.0

        command = Twist()
        command.linear.x = throttle * self.max_linear_mps
        command.angular.z = steering * self.max_angular_rps
        self.command_publisher.publish(command)

        status["reason"] = status.get("reason", reason)
        status["output"] = {
            "left_us": left_us,
            "right_us": right_us,
            "throttle": throttle,
            "steering": steering,
        }
        message = String()
        message.data = json.dumps(status, separators=(",", ":"))
        self.status_publisher.publish(message)

    @staticmethod
    def _finite_or_none(value) -> float | None:
        number = float(value)
        return number if math.isfinite(number) else None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AutonomyNode()
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
