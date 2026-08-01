from __future__ import annotations

import copy
import math
import threading
import time


def empty_gnss_record() -> dict:
    return {
        "lat": None,
        "lon": None,
        "altitude_m": None,
        "speed_mps": None,
        "heading_deg": None,
        "fix": "unavailable",
        "satellites": None,
        "hdop": None,
        "age_s": None,
        "frame_id": None,
        "source": None,
    }


class RosGnssReader:
    """Collect standard ROS GNSS topics into a dashboard snapshot."""

    def __init__(
        self,
        *,
        fix_topic: str = "gnss/fix",
        velocity_topic: str = "gnss/velocity",
        diagnostics_topic: str = "/diagnostics",
        stale_after_s: float = 3.0,
        log=None,
    ) -> None:
        self.fix_topic = fix_topic
        self.velocity_topic = velocity_topic
        self.diagnostics_topic = diagnostics_topic
        self.stale_after_s = stale_after_s
        self.log = log
        self._lock = threading.Lock()
        self._latest = empty_gnss_record()
        self._last_fix_at: float | None = None
        self._error: str | None = None
        self._stop_event = threading.Event()
        self._context = None
        self._thread = threading.Thread(
            target=self._spin_ros,
            daemon=True,
            name="ros-gnss-reader",
        )
        self._thread.start()

    def shutdown(self) -> None:
        self._stop_event.set()
        context = self._context
        if context is not None and context.ok():
            context.shutdown()
        self._thread.join(timeout=2.0)

    def snapshot(self) -> tuple[str, dict]:
        now = time.time()
        with self._lock:
            latest = copy.deepcopy(self._latest)
            last_fix_at = self._last_fix_at
            error = self._error

        if error:
            latest["error"] = error
            return "error", latest
        if last_fix_at is None:
            return "waiting", latest

        latest["age_s"] = round(now - last_fix_at, 2)
        if now - last_fix_at > self.stale_after_s:
            return "stale", latest
        if latest["fix"] != "fix":
            return "waiting", latest
        return "live", latest

    def _spin_ros(self) -> None:
        try:
            import rclpy
            from diagnostic_msgs.msg import DiagnosticArray
            from geometry_msgs.msg import TwistStamped
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import qos_profile_sensor_data
            from sensor_msgs.msg import NavSatFix
        except ImportError as exc:
            with self._lock:
                self._error = f"ROS2 GNSS packages unavailable: {exc}"
            return

        outer = self

        class DashboardGnssNode(Node):
            def __init__(node_self, **kwargs) -> None:
                super().__init__("dashboard_gnss_reader", **kwargs)
                node_self.create_subscription(
                    NavSatFix,
                    outer.fix_topic,
                    outer._on_fix,
                    qos_profile_sensor_data,
                )
                node_self.create_subscription(
                    TwistStamped,
                    outer.velocity_topic,
                    outer._on_velocity,
                    qos_profile_sensor_data,
                )
                node_self.create_subscription(
                    DiagnosticArray,
                    outer.diagnostics_topic,
                    outer._on_diagnostics,
                    10,
                )

        context = rclpy.context.Context()
        self._context = context
        node = None
        executor = None
        try:
            rclpy.init(context=context)
            if not self._stop_event.is_set():
                node = DashboardGnssNode(context=context)
                executor = SingleThreadedExecutor(context=context)
                rclpy.spin(node, executor=executor)
        except Exception as exc:
            if not self._stop_event.is_set():
                with self._lock:
                    self._error = f"ROS2 GNSS subscriber failed: {exc}"
        finally:
            if executor is not None:
                executor.shutdown()
            if node is not None:
                node.destroy_node()
            if context.ok():
                rclpy.shutdown(context=context)
            self._context = None

    def _on_fix(self, message) -> None:
        now = time.time()
        latitude = self._finite_or_none(message.latitude)
        longitude = self._finite_or_none(message.longitude)
        altitude = self._finite_or_none(message.altitude)
        has_fix = int(message.status.status) >= 0
        record = {
            "lat": latitude,
            "lon": longitude,
            "altitude_m": altitude,
            "fix": "fix" if has_fix else "none",
            "frame_id": message.header.frame_id,
            "source": self.fix_topic,
        }
        with self._lock:
            self._latest.update(record)
            self._last_fix_at = now
            self._error = None
            snapshot = copy.deepcopy(self._latest)
        if self.log is not None:
            self.log.write(snapshot)

    def _on_velocity(self, message) -> None:
        east_mps = float(message.twist.linear.x)
        north_mps = float(message.twist.linear.y)
        speed_mps = math.hypot(east_mps, north_mps)
        heading_deg = None
        if speed_mps > 1e-6:
            heading_deg = math.degrees(
                math.atan2(east_mps, north_mps)
            ) % 360.0
        with self._lock:
            self._latest["speed_mps"] = speed_mps
            self._latest["heading_deg"] = heading_deg

    def _on_diagnostics(self, message) -> None:
        diagnostic = next(
            (
                item
                for item in message.status
                if "gnss" in item.name.lower()
            ),
            None,
        )
        if diagnostic is None:
            return
        values = {item.key: item.value for item in diagnostic.values}
        satellites = self._int_or_none(values.get("satellites"))
        hdop = self._float_or_none(values.get("hdop"))
        with self._lock:
            self._latest["satellites"] = satellites
            self._latest["hdop"] = hdop

    @staticmethod
    def _finite_or_none(value) -> float | None:
        number = float(value)
        return number if math.isfinite(number) else None

    @staticmethod
    def _float_or_none(value) -> float | None:
        try:
            return float(value) if value not in (None, "") else None
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _int_or_none(value) -> int | None:
        try:
            return int(value) if value not in (None, "") else None
        except (TypeError, ValueError):
            return None
