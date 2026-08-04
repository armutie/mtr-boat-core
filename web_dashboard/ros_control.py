from __future__ import annotations

import json
import threading
import time

from thruster_control import (
    ThrusterMapping,
    manual_to_pair,
    pair_to_manual,
)


class RosCommandBridge:
    """Publish dashboard manual and auto intent into the ROS control graph."""

    log_path = None

    def __init__(
        self,
        control_state,
        mapping: ThrusterMapping,
        send_hz: float = 20.0,
        max_linear_mps: float = 1.0,
        max_angular_rps: float = 1.0,
        operator_topic: str = "cmd_vel/operator",
        steering_takeover_topic: str = "control/steering_takeover",
        mode_topic: str = "control/mode_request",
        route_topic: str = "autonomy/route",
        heading_reset_topic: str = "autonomy/relearn_heading",
        autonomy_status_topic: str = "autonomy/status",
        thruster_command_topic: str = "thrusters/command",
    ) -> None:
        self.control_state = control_state
        self.mapping = mapping
        self.send_hz = max(1.0, send_hz)
        self.max_linear_mps = max(0.01, max_linear_mps)
        self.max_angular_rps = max(0.01, max_angular_rps)
        self.operator_topic = operator_topic
        self.steering_takeover_topic = steering_takeover_topic
        self.mode_topic = mode_topic
        self.route_topic = route_topic
        self.heading_reset_topic = heading_reset_topic
        self.autonomy_status_topic = autonomy_status_topic
        self.thruster_command_topic = thruster_command_topic
        self._lock = threading.Lock()
        self._status = "ros-control starting"
        self._waypoints: list[dict] = []
        self._route_version = 0
        self._heading_reset_version = 0
        self._steering_takeover_active = False
        self._steering_takeover = 0.0
        self._steering_takeover_at: float | None = None
        self._steering_takeover_timeout_s = 0.35
        self._autonomy_status = {
            "state": "idle",
            "reason": "no route",
        }
        self._effective = {
            "throttle": 0.0,
            "steering": 0.0,
            "left_us": mapping.neutral_us,
            "right_us": mapping.neutral_us,
            "send_hz": self.send_hz,
            "error": None,
        }
        self._stop_event = threading.Event()
        self._thread = threading.Thread(
            target=self._run,
            daemon=True,
            name="ros-command-bridge",
        )

    def start(self) -> None:
        self._thread.start()

    def shutdown(self) -> None:
        self.control_state.stop()
        self._stop_event.set()
        self._thread.join(timeout=1.0)

    def status_label(self) -> str:
        with self._lock:
            return self._status

    def effective_output(self) -> dict:
        with self._lock:
            return {
                **self._effective,
                "steering_takeover": {
                    "active": self._takeover_active_locked(),
                    "steering": round(self._steering_takeover, 4),
                },
            }

    def set_waypoints(self, records: list[dict]) -> dict:
        waypoints = []
        for index, record in enumerate(records):
            if not isinstance(record, dict):
                raise ValueError(f"waypoint {index + 1} must be an object")
            try:
                latitude = float(record["lat"])
                longitude = float(record["lon"])
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError(
                    f"waypoint {index + 1} requires numeric lat/lon"
                ) from exc
            waypoints.append(
                {
                    "lat": latitude,
                    "lon": longitude,
                    "label": str(record.get("label", "")),
                }
            )
        with self._lock:
            self._waypoints = waypoints
            self._route_version += 1
        # The autonomy node owns route progress. Keep its last confirmed status
        # until it acknowledges this route update instead of briefly inventing
        # active_index 0 in the dashboard.
        return self.route_snapshot()

    def route_snapshot(self) -> dict:
        with self._lock:
            return {
                "waypoints": [dict(item) for item in self._waypoints],
                "status": dict(self._autonomy_status),
            }

    def relearn_heading(self) -> dict:
        with self._lock:
            self._heading_reset_version += 1
            self._autonomy_status = {
                **self._autonomy_status,
                "state": "acquiring_heading",
                "reason": "heading reset; drive straight in manual",
                "heading": {
                    "state": "uninitialized",
                    "reason": (
                        "heading reset; waiting for straight manual motion"
                    ),
                    "heading_deg": None,
                    "source": "uninitialized",
                    "confidence": "none",
                },
            }
            status = dict(self._autonomy_status)
        self.control_state.set_auto_status(status)
        return status

    def set_steering_takeover(
        self,
        active: bool,
        steering: float,
    ) -> dict:
        with self._lock:
            self._steering_takeover_active = bool(active)
            self._steering_takeover = max(
                -1.0,
                min(1.0, float(steering)),
            )
            self._steering_takeover_at = time.monotonic()
            return {
                "active": self._steering_takeover_active,
                "steering": round(self._steering_takeover, 4),
            }

    def status(self) -> dict:
        with self._lock:
            return dict(self._autonomy_status)

    def can_arm(self) -> tuple[bool, str]:
        with self._lock:
            has_route = bool(self._waypoints)
        if not has_route:
            return False, "auto requires at least one waypoint"
        return True, "auto ready"

    def _set_status(
        self,
        status: str,
        *,
        throttle: float = 0.0,
        steering: float = 0.0,
        error: str | None = None,
    ) -> None:
        pair = manual_to_pair(
            throttle,
            steering,
            self.mapping,
            enabled=throttle > 0.01,
        )
        with self._lock:
            takeover = self._takeover_active_locked()
            self._status = status
            self._effective = {
                "throttle": round(throttle, 4),
                "steering": round(steering, 4),
                "left_us": pair.left_us,
                "right_us": pair.right_us,
                "send_hz": self.send_hz,
                "error": error,
                "steering_takeover": {
                    "active": takeover,
                    "steering": round(self._steering_takeover, 4),
                },
            }

    def _run(self) -> None:
        try:
            import rclpy
            from geometry_msgs.msg import Twist
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import (
                DurabilityPolicy,
                QoSProfile,
                ReliabilityPolicy,
            )
            from std_msgs.msg import Empty, Int32MultiArray, String

            context = rclpy.context.Context()
            rclpy.init(context=context)
            node = rclpy.create_node(
                "dashboard_control_bridge",
                context=context,
            )
            operator_publisher = node.create_publisher(
                Twist,
                self.operator_topic,
                10,
            )
            route_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            route_publisher = node.create_publisher(
                String,
                self.route_topic,
                route_qos,
            )
            steering_takeover_publisher = node.create_publisher(
                String,
                self.steering_takeover_topic,
                10,
            )
            heading_reset_publisher = node.create_publisher(
                Empty,
                self.heading_reset_topic,
                10,
            )
            mode_publisher = node.create_publisher(
                String,
                self.mode_topic,
                10,
            )
            node.create_subscription(
                String,
                self.autonomy_status_topic,
                self._on_autonomy_status,
                10,
            )
            node.create_subscription(
                Int32MultiArray,
                self.thruster_command_topic,
                self._on_thruster_command,
                10,
            )
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
        except Exception as exc:
            self._set_status("ros-control error", error=str(exc))
            return

        period = 1.0 / self.send_hz
        published_route_version = -1
        published_heading_reset_version = 0
        try:
            while not self._stop_event.is_set():
                started = time.monotonic()
                executor.spin_once(timeout_sec=0.0)
                intent = self.control_state.command_snapshot()
                mode = str(intent.get("mode", "off"))
                throttle = 0.0
                steering = 0.0

                with self._lock:
                    route_version = self._route_version
                    waypoints = [dict(item) for item in self._waypoints]
                    heading_reset_version = self._heading_reset_version
                    takeover_active = (
                        mode == "auto"
                        and self._takeover_active_locked()
                    )
                    takeover_steering = self._steering_takeover
                if route_version != published_route_version:
                    route_message = String()
                    route_message.data = json.dumps(
                        {"waypoints": waypoints},
                        separators=(",", ":"),
                    )
                    route_publisher.publish(route_message)
                    published_route_version = route_version
                if heading_reset_version != published_heading_reset_version:
                    heading_reset_publisher.publish(Empty())
                    published_heading_reset_version = heading_reset_version

                mode_message = String()
                mode_message.data = mode
                mode_publisher.publish(mode_message)

                takeover_message = String()
                takeover_message.data = json.dumps(
                    {
                        "active": takeover_active,
                        "steering": (
                            takeover_steering if takeover_active else 0.0
                        ),
                    },
                    separators=(",", ":"),
                )
                steering_takeover_publisher.publish(takeover_message)

                if mode == "manual":
                    if not intent.get("stale", False):
                        throttle = float(intent.get("throttle", 0.0))
                        steering = float(intent.get("steering", 0.0))
                    operator = Twist()
                    operator.linear.x = throttle * self.max_linear_mps
                    operator.angular.z = (
                        steering * self.max_angular_rps
                    )
                    operator_publisher.publish(operator)

                if mode != "auto":
                    self._set_status(
                        "ros-control",
                        throttle=throttle,
                        steering=steering,
                    )
                elapsed = time.monotonic() - started
                self._stop_event.wait(max(0.0, period - elapsed))
        except Exception as exc:
            self._set_status("ros-control error", error=str(exc))
        finally:
            try:
                mode_message = String()
                mode_message.data = "off"
                mode_publisher.publish(mode_message)
                executor.remove_node(node)
                executor.shutdown()
                node.destroy_node()
                rclpy.shutdown(context=context)
            except Exception:
                pass

    def _on_autonomy_status(self, message) -> None:
        try:
            status = json.loads(message.data)
            if not isinstance(status, dict):
                return
        except (TypeError, json.JSONDecodeError):
            return

        output = status.get("output", {})
        left_us = output.get("left_us")
        right_us = output.get("right_us")
        with self._lock:
            self._autonomy_status = dict(status)
            self._status = "ros-control"
        if left_us is not None and right_us is not None:
            self.control_state.apply_auto_pwm(
                int(left_us),
                int(right_us),
                str(status.get("reason", "autonomy update")),
                status,
            )
        else:
            self.control_state.set_auto_status(status)

    def _on_thruster_command(self, message) -> None:
        if len(message.data) < 2:
            return
        left_us = int(message.data[0])
        right_us = int(message.data[1])
        throttle, steering = pair_to_manual(
            left_us,
            right_us,
            self.mapping,
        )
        with self._lock:
            self._effective = {
                "throttle": round(throttle, 4),
                "steering": round(steering, 4),
                "left_us": left_us,
                "right_us": right_us,
                "send_hz": self.send_hz,
                "error": None,
                "steering_takeover": {
                    "active": self._takeover_active_locked(),
                    "steering": round(self._steering_takeover, 4),
                },
            }

    def _takeover_active_locked(self) -> bool:
        return (
            self._steering_takeover_active
            and self._steering_takeover_at is not None
            and time.monotonic() - self._steering_takeover_at
            <= self._steering_takeover_timeout_s
        )
