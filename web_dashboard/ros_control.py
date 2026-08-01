from __future__ import annotations

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
        auto_topic: str = "cmd_vel/auto",
        mode_topic: str = "control/mode_request",
    ) -> None:
        self.control_state = control_state
        self.mapping = mapping
        self.send_hz = max(1.0, send_hz)
        self.max_linear_mps = max(0.01, max_linear_mps)
        self.max_angular_rps = max(0.01, max_angular_rps)
        self.operator_topic = operator_topic
        self.auto_topic = auto_topic
        self.mode_topic = mode_topic
        self._lock = threading.Lock()
        self._status = "ros-control starting"
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
            return dict(self._effective)

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
            self._status = status
            self._effective = {
                "throttle": round(throttle, 4),
                "steering": round(steering, 4),
                "left_us": pair.left_us,
                "right_us": pair.right_us,
                "send_hz": self.send_hz,
                "error": error,
            }

    def _run(self) -> None:
        try:
            import rclpy
            from geometry_msgs.msg import Twist
            from std_msgs.msg import String

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
            auto_publisher = node.create_publisher(
                Twist,
                self.auto_topic,
                10,
            )
            mode_publisher = node.create_publisher(
                String,
                self.mode_topic,
                10,
            )
        except Exception as exc:
            self._set_status("ros-control error", error=str(exc))
            return

        period = 1.0 / self.send_hz
        try:
            while not self._stop_event.is_set():
                started = time.monotonic()
                intent = self.control_state.command_snapshot()
                mode = str(intent.get("mode", "off"))
                throttle = 0.0
                steering = 0.0

                mode_message = String()
                mode_message.data = mode
                mode_publisher.publish(mode_message)

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

                elif mode == "auto":
                    left_us = intent.get("left_us")
                    right_us = intent.get("right_us")
                    if (
                        not intent.get("stale", False)
                        and left_us is not None
                        and right_us is not None
                    ):
                        throttle, steering = pair_to_manual(
                            float(left_us),
                            float(right_us),
                            self.mapping,
                        )
                    automatic = Twist()
                    automatic.linear.x = (
                        throttle * self.max_linear_mps
                    )
                    automatic.angular.z = (
                        steering * self.max_angular_rps
                    )
                    auto_publisher.publish(automatic)

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
                node.destroy_node()
                rclpy.shutdown(context=context)
            except Exception:
                pass
