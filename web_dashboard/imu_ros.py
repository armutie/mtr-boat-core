from __future__ import annotations

import copy
import math
import threading
import time


STANDARD_GRAVITY_MPS2 = 9.80665


def empty_imu_record() -> dict:
    return {
        "accel_x_mps2": None,
        "accel_y_mps2": None,
        "accel_z_mps2": None,
        "accel_mag_mps2": None,
        "linear_accel_x_mps2": None,
        "linear_accel_y_mps2": None,
        "linear_accel_z_mps2": None,
        "gravity_x_mps2": None,
        "gravity_y_mps2": None,
        "gravity_z_mps2": None,
        "gyro_x_rad_s": None,
        "gyro_y_rad_s": None,
        "gyro_z_rad_s": None,
        "roll_deg": None,
        "pitch_deg": None,
        "yaw_deg": None,
        "orientation_available": False,
        "mag_x_ut": None,
        "mag_y_ut": None,
        "mag_z_ut": None,
        "mag_strength_ut": None,
        "temperature_c": None,
        "calibration": {
            "system": None,
            "gyroscope": None,
            "accelerometer": None,
            "magnetometer": None,
            "status": "waiting",
            "message": "waiting for BNO055 diagnostics",
        },
        "frame_id": None,
        "age_s": None,
        "source": None,
        "dt_s": None,
        # Compatibility fields used by the existing auto controller.
        "accel_x_g": None,
        "accel_y_g": None,
        "accel_z_g": None,
        "accel_mag_g": None,
        "gyro_x_dps": None,
        "gyro_y_dps": None,
        "gyro_z_dps": None,
        "yaw_relative_deg": None,
        "bias": {"x_dps": 0.0, "y_dps": 0.0, "z_dps": 0.0},
    }


def quaternion_to_euler_deg(
    w: float,
    x: float,
    y: float,
    z: float,
) -> tuple[float, float, float]:
    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll, cos_roll)

    sin_pitch = 2.0 * (w * y - z * x)
    if abs(sin_pitch) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sin_pitch)
    else:
        pitch = math.asin(sin_pitch)

    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(sin_yaw, cos_yaw)

    return tuple(math.degrees(value) for value in (roll, pitch, yaw))


class RosImuReader:
    """Collect the BNO055 ROS topic set into one dashboard snapshot."""

    def __init__(
        self,
        *,
        raw_topic: str = "imu/data_raw",
        data_topic: str = "imu/data",
        magnetic_topic: str = "imu/mag",
        temperature_topic: str = "imu/temperature",
        linear_acceleration_topic: str = "imu/linear_acceleration",
        gravity_topic: str = "imu/gravity",
        diagnostics_topic: str = "/diagnostics",
        stale_after_s: float = 2.0,
        log=None,
    ) -> None:
        self.raw_topic = raw_topic
        self.data_topic = data_topic
        self.magnetic_topic = magnetic_topic
        self.temperature_topic = temperature_topic
        self.linear_acceleration_topic = linear_acceleration_topic
        self.gravity_topic = gravity_topic
        self.diagnostics_topic = diagnostics_topic
        self.stale_after_s = stale_after_s
        self.log = log
        self._lock = threading.Lock()
        self._latest = empty_imu_record()
        self._last_raw_at: float | None = None
        self._last_orientation_at: float | None = None
        self._error: str | None = None
        self._thread = threading.Thread(
            target=self._spin_ros,
            daemon=True,
            name="ros-imu-reader",
        )
        self._thread.start()

    def snapshot(self) -> tuple[str, dict]:
        now = time.time()
        with self._lock:
            latest = copy.deepcopy(self._latest)
            last_raw_at = self._last_raw_at
            last_orientation_at = self._last_orientation_at
            error = self._error

        if error:
            latest["error"] = error
            return "error", latest
        if last_raw_at is None:
            return "waiting", latest

        latest["age_s"] = round(now - last_raw_at, 2)
        orientation_fresh = (
            last_orientation_at is not None
            and now - last_orientation_at <= self.stale_after_s
        )
        latest["orientation_available"] = orientation_fresh
        if not orientation_fresh:
            latest["roll_deg"] = None
            latest["pitch_deg"] = None
            latest["yaw_deg"] = None
            latest["yaw_relative_deg"] = None

        if now - last_raw_at > self.stale_after_s:
            return "stale", latest
        if latest["calibration"].get("status") == "error":
            return "error", latest
        return "live", latest

    def _spin_ros(self) -> None:
        try:
            import rclpy
            from diagnostic_msgs.msg import DiagnosticArray
            from geometry_msgs.msg import Vector3Stamped
            from rclpy.node import Node
            from rclpy.qos import qos_profile_sensor_data
            from sensor_msgs.msg import Imu, MagneticField, Temperature
        except ImportError as exc:
            with self._lock:
                self._error = f"ROS2 IMU packages unavailable: {exc}"
            return

        outer = self

        class DashboardImuNode(Node):
            def __init__(node_self, **kwargs) -> None:
                super().__init__("dashboard_imu_reader", **kwargs)
                node_self.create_subscription(
                    Imu,
                    outer.raw_topic,
                    outer._on_raw,
                    qos_profile_sensor_data,
                )
                node_self.create_subscription(
                    Imu,
                    outer.data_topic,
                    outer._on_fused,
                    qos_profile_sensor_data,
                )
                node_self.create_subscription(
                    MagneticField,
                    outer.magnetic_topic,
                    outer._on_magnetic,
                    qos_profile_sensor_data,
                )
                node_self.create_subscription(
                    Temperature,
                    outer.temperature_topic,
                    outer._on_temperature,
                    qos_profile_sensor_data,
                )
                node_self.create_subscription(
                    Vector3Stamped,
                    outer.linear_acceleration_topic,
                    outer._on_linear_acceleration,
                    qos_profile_sensor_data,
                )
                node_self.create_subscription(
                    Vector3Stamped,
                    outer.gravity_topic,
                    outer._on_gravity,
                    qos_profile_sensor_data,
                )
                node_self.create_subscription(
                    DiagnosticArray,
                    outer.diagnostics_topic,
                    outer._on_diagnostics,
                    10,
                )

        context = rclpy.context.Context()
        try:
            rclpy.init(context=context)
            node = DashboardImuNode(context=context)
            try:
                rclpy.spin(node)
            finally:
                node.destroy_node()
                rclpy.shutdown(context=context)
        except Exception as exc:
            with self._lock:
                self._error = f"ROS2 IMU subscriber failed: {exc}"

    def _on_raw(self, message) -> None:
        now = time.time()
        acceleration = message.linear_acceleration
        angular = message.angular_velocity
        magnitude = math.sqrt(
            acceleration.x ** 2
            + acceleration.y ** 2
            + acceleration.z ** 2
        )
        degrees = tuple(
            math.degrees(value)
            for value in (angular.x, angular.y, angular.z)
        )
        with self._lock:
            previous_at = self._last_raw_at
            self._latest.update(
                {
                    "accel_x_mps2": acceleration.x,
                    "accel_y_mps2": acceleration.y,
                    "accel_z_mps2": acceleration.z,
                    "accel_mag_mps2": magnitude,
                    "gyro_x_rad_s": angular.x,
                    "gyro_y_rad_s": angular.y,
                    "gyro_z_rad_s": angular.z,
                    "frame_id": message.header.frame_id,
                    "source": self.raw_topic,
                    "dt_s": None if previous_at is None else now - previous_at,
                    "accel_x_g": acceleration.x / STANDARD_GRAVITY_MPS2,
                    "accel_y_g": acceleration.y / STANDARD_GRAVITY_MPS2,
                    "accel_z_g": acceleration.z / STANDARD_GRAVITY_MPS2,
                    "accel_mag_g": magnitude / STANDARD_GRAVITY_MPS2,
                    "gyro_x_dps": degrees[0],
                    "gyro_y_dps": degrees[1],
                    "gyro_z_dps": degrees[2],
                }
            )
            self._last_raw_at = now
            record = copy.deepcopy(self._latest)
        if self.log is not None:
            self.log.write(record)

    def _on_fused(self, message) -> None:
        orientation = message.orientation
        roll, pitch, yaw = quaternion_to_euler_deg(
            orientation.w,
            orientation.x,
            orientation.y,
            orientation.z,
        )
        with self._lock:
            self._latest.update(
                {
                    "roll_deg": roll,
                    "pitch_deg": pitch,
                    "yaw_deg": yaw,
                    "yaw_relative_deg": yaw,
                    "orientation_available": True,
                }
            )
            self._last_orientation_at = time.time()

    def _on_magnetic(self, message) -> None:
        field = message.magnetic_field
        values = tuple(
            value * 1_000_000.0
            for value in (field.x, field.y, field.z)
        )
        with self._lock:
            self._latest.update(
                {
                    "mag_x_ut": values[0],
                    "mag_y_ut": values[1],
                    "mag_z_ut": values[2],
                    "mag_strength_ut": math.sqrt(
                        values[0] ** 2
                        + values[1] ** 2
                        + values[2] ** 2
                    ),
                }
            )

    def _on_temperature(self, message) -> None:
        with self._lock:
            self._latest["temperature_c"] = message.temperature

    def _on_linear_acceleration(self, message) -> None:
        vector = message.vector
        with self._lock:
            self._latest.update(
                {
                    "linear_accel_x_mps2": vector.x,
                    "linear_accel_y_mps2": vector.y,
                    "linear_accel_z_mps2": vector.z,
                }
            )

    def _on_gravity(self, message) -> None:
        vector = message.vector
        with self._lock:
            self._latest.update(
                {
                    "gravity_x_mps2": vector.x,
                    "gravity_y_mps2": vector.y,
                    "gravity_z_mps2": vector.z,
                }
            )

    def _on_diagnostics(self, message) -> None:
        diagnostic = next(
            (
                status
                for status in message.status
                if status.name == "BNO055 IMU"
            ),
            None,
        )
        if diagnostic is None:
            return
        values = {item.key: item.value for item in diagnostic.values}

        def calibration_value(key: str) -> int | None:
            try:
                return int(values[key])
            except (KeyError, TypeError, ValueError):
                return None

        status = (
            "error"
            if int(diagnostic.level) >= 2
            else "calibrating"
            if int(diagnostic.level) == 1
            else "ready"
        )
        with self._lock:
            self._latest["calibration"] = {
                "system": calibration_value("system_calibration"),
                "gyroscope": calibration_value("gyroscope_calibration"),
                "accelerometer": calibration_value(
                    "accelerometer_calibration"
                ),
                "magnetometer": calibration_value(
                    "magnetometer_calibration"
                ),
                "status": status,
                "message": diagnostic.message,
            }
