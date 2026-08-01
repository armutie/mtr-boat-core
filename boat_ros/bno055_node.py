from __future__ import annotations

import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Vector3Stamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, MagneticField, Temperature

from imu import Bno055


def covariance_diagonal(values: list[float]) -> list[float]:
    if len(values) != 3:
        raise ValueError("covariance diagonal requires exactly three values")
    return [
        float(values[0]),
        0.0,
        0.0,
        0.0,
        float(values[1]),
        0.0,
        0.0,
        0.0,
        float(values[2]),
    ]


class Bno055Node(Node):
    """Publish BNO055 raw axes and optional device-fused orientation."""

    def __init__(self) -> None:
        super().__init__("bno055_node")

        self.declare_parameter("bus", 2)
        self.declare_parameter("address", "0x29")
        self.declare_parameter("placement", "P1")
        self.declare_parameter("reset_on_start", True)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("publish_fused_orientation", False)
        self.declare_parameter("data_topic", "imu/data")
        self.declare_parameter("raw_topic", "imu/data_raw")
        self.declare_parameter("magnetic_field_topic", "imu/mag")
        self.declare_parameter("temperature_topic", "imu/temperature")
        self.declare_parameter(
            "linear_acceleration_topic",
            "imu/linear_acceleration",
        )
        self.declare_parameter("gravity_topic", "imu/gravity")
        self.declare_parameter("diagnostics_topic", "/diagnostics")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("status_rate_hz", 1.0)
        self.declare_parameter("orientation_variance", [0.0, 0.0, 0.0])
        self.declare_parameter("angular_velocity_variance", [0.0, 0.0, 0.0])
        self.declare_parameter("linear_acceleration_variance", [0.0, 0.0, 0.0])
        self.declare_parameter("magnetic_field_variance", [0.0, 0.0, 0.0])

        bus = int(self.get_parameter("bus").value)
        address = int(str(self.get_parameter("address").value), 0)
        placement = str(self.get_parameter("placement").value)
        reset_on_start = bool(self.get_parameter("reset_on_start").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.publish_fused_orientation = bool(
            self.get_parameter("publish_fused_orientation").value
        )
        self.hardware_id = f"i2c-{bus}:0x{address:02x}"

        self.orientation_covariance = self._covariance("orientation_variance")
        self.angular_velocity_covariance = self._covariance(
            "angular_velocity_variance"
        )
        self.linear_acceleration_covariance = self._covariance(
            "linear_acceleration_variance"
        )
        self.magnetic_field_covariance = self._covariance(
            "magnetic_field_variance"
        )

        self.data_publisher = self.create_publisher(
            Imu,
            str(self.get_parameter("data_topic").value),
            qos_profile_sensor_data,
        )
        self.raw_publisher = self.create_publisher(
            Imu,
            str(self.get_parameter("raw_topic").value),
            qos_profile_sensor_data,
        )
        self.magnetic_publisher = self.create_publisher(
            MagneticField,
            str(self.get_parameter("magnetic_field_topic").value),
            qos_profile_sensor_data,
        )
        self.temperature_publisher = self.create_publisher(
            Temperature,
            str(self.get_parameter("temperature_topic").value),
            qos_profile_sensor_data,
        )
        self.linear_acceleration_publisher = self.create_publisher(
            Vector3Stamped,
            str(self.get_parameter("linear_acceleration_topic").value),
            qos_profile_sensor_data,
        )
        self.gravity_publisher = self.create_publisher(
            Vector3Stamped,
            str(self.get_parameter("gravity_topic").value),
            qos_profile_sensor_data,
        )
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter("diagnostics_topic").value),
            10,
        )

        self.imu = Bno055(
            bus=bus,
            address=address,
            placement=placement,
            reset_on_start=reset_on_start,
        )
        self._last_error_log_s = 0.0

        publish_rate_hz = max(
            float(self.get_parameter("publish_rate_hz").value),
            0.1,
        )
        status_rate_hz = max(
            float(self.get_parameter("status_rate_hz").value),
            0.1,
        )
        self.data_timer = self.create_timer(
            1.0 / publish_rate_hz,
            self.publish_data,
        )
        self.status_timer = self.create_timer(
            1.0 / status_rate_hz,
            self.publish_status,
        )

        self.get_logger().info(
            f"Reading BNO055 on {self.hardware_id} in NDOF mode; "
            f"raw IMU topic {self.get_parameter('raw_topic').value}; "
            f"device-fused orientation publishing is "
            f"{'enabled' if self.publish_fused_orientation else 'disabled'}"
        )

    def _covariance(self, parameter_name: str) -> list[float]:
        values = list(self.get_parameter(parameter_name).value)
        return covariance_diagonal(values)

    def publish_data(self) -> None:
        try:
            sample = self.imu.read_sample()
        except Exception as exc:
            self._log_read_error(exc)
            return

        stamp = self.get_clock().now().to_msg()

        raw = Imu()
        raw.header.stamp = stamp
        raw.header.frame_id = self.frame_id
        raw.orientation_covariance[0] = -1.0
        raw.angular_velocity.x = sample.angular_velocity_x_rad_s
        raw.angular_velocity.y = sample.angular_velocity_y_rad_s
        raw.angular_velocity.z = sample.angular_velocity_z_rad_s
        raw.angular_velocity_covariance = self.angular_velocity_covariance
        raw.linear_acceleration.x = sample.acceleration_x_mps2
        raw.linear_acceleration.y = sample.acceleration_y_mps2
        raw.linear_acceleration.z = sample.acceleration_z_mps2
        raw.linear_acceleration_covariance = (
            self.linear_acceleration_covariance
        )
        self.raw_publisher.publish(raw)

        if self.publish_fused_orientation:
            fused = Imu()
            fused.header.stamp = stamp
            fused.header.frame_id = self.frame_id
            fused.orientation.w = sample.orientation_w
            fused.orientation.x = sample.orientation_x
            fused.orientation.y = sample.orientation_y
            fused.orientation.z = sample.orientation_z
            fused.orientation_covariance = self.orientation_covariance
            fused.angular_velocity = raw.angular_velocity
            fused.angular_velocity_covariance = (
                self.angular_velocity_covariance
            )
            # REP-145 requires specific force, including gravity at rest, on
            # both imu/data and imu/data_raw.
            fused.linear_acceleration = raw.linear_acceleration
            fused.linear_acceleration_covariance = (
                self.linear_acceleration_covariance
            )
            self.data_publisher.publish(fused)

        magnetic = MagneticField()
        magnetic.header.stamp = stamp
        magnetic.header.frame_id = self.frame_id
        magnetic.magnetic_field.x = sample.magnetic_field_x_t
        magnetic.magnetic_field.y = sample.magnetic_field_y_t
        magnetic.magnetic_field.z = sample.magnetic_field_z_t
        magnetic.magnetic_field_covariance = self.magnetic_field_covariance
        self.magnetic_publisher.publish(magnetic)

        temperature = Temperature()
        temperature.header.stamp = stamp
        temperature.header.frame_id = self.frame_id
        temperature.temperature = sample.temperature_c
        temperature.variance = 0.0
        self.temperature_publisher.publish(temperature)

        linear_acceleration = Vector3Stamped()
        linear_acceleration.header.stamp = stamp
        linear_acceleration.header.frame_id = self.frame_id
        linear_acceleration.vector.x = sample.linear_acceleration_x_mps2
        linear_acceleration.vector.y = sample.linear_acceleration_y_mps2
        linear_acceleration.vector.z = sample.linear_acceleration_z_mps2
        self.linear_acceleration_publisher.publish(linear_acceleration)

        gravity = Vector3Stamped()
        gravity.header.stamp = stamp
        gravity.header.frame_id = self.frame_id
        gravity.vector.x = sample.gravity_x_mps2
        gravity.vector.y = sample.gravity_y_mps2
        gravity.vector.z = sample.gravity_z_mps2
        self.gravity_publisher.publish(gravity)

    def publish_status(self) -> None:
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        diagnostic = DiagnosticStatus()
        diagnostic.name = "BNO055 IMU"
        diagnostic.hardware_id = self.hardware_id

        try:
            status = self.imu.read_status()
        except Exception as exc:
            diagnostic.level = DiagnosticStatus.ERROR
            diagnostic.message = f"status read failed: {exc}"
        else:
            if status.system_error:
                diagnostic.level = DiagnosticStatus.ERROR
                diagnostic.message = f"system error {status.system_error}"
            elif not status.fully_calibrated:
                diagnostic.level = DiagnosticStatus.WARN
                diagnostic.message = "calibration incomplete"
            else:
                diagnostic.level = DiagnosticStatus.OK
                diagnostic.message = "fully calibrated"

            diagnostic.values = [
                KeyValue(
                    key="system_calibration",
                    value=str(status.system_calibration),
                ),
                KeyValue(
                    key="gyroscope_calibration",
                    value=str(status.gyroscope_calibration),
                ),
                KeyValue(
                    key="accelerometer_calibration",
                    value=str(status.accelerometer_calibration),
                ),
                KeyValue(
                    key="magnetometer_calibration",
                    value=str(status.magnetometer_calibration),
                ),
                KeyValue(key="system_status", value=str(status.system_status)),
                KeyValue(key="system_error", value=str(status.system_error)),
                KeyValue(
                    key="fused_orientation_published",
                    value=str(self.publish_fused_orientation).lower(),
                ),
            ]

        message.status = [diagnostic]
        self.diagnostics_publisher.publish(message)

    def _log_read_error(self, exc: Exception) -> None:
        now = time.monotonic()
        if now - self._last_error_log_s >= 5.0:
            self.get_logger().error(f"BNO055 read failed: {exc}")
            self._last_error_log_s = now

    def destroy_node(self) -> bool:
        self.imu.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Bno055Node()
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
