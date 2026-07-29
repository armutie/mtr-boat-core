from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from boat_ros.sensor_math import acceleration_g_to_mps2, angular_velocity_dps_to_rad_s
from imu import GyroBias, Mpu6050


class ImuNode(Node):
    """Publish MPU-6050 acceleration and angular velocity in SI units."""

    def __init__(self) -> None:
        super().__init__("imu_node")

        self.declare_parameter("bus", 2)
        self.declare_parameter("address", "0x68")
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("topic", "imu/data_raw")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("calibrate_on_start", True)
        self.declare_parameter("calibration_samples", 200)
        self.declare_parameter("calibration_delay_s", 0.01)

        bus = int(self.get_parameter("bus").value)
        address = int(str(self.get_parameter("address").value), 0)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.publisher = self.create_publisher(
            Imu,
            str(self.get_parameter("topic").value),
            qos_profile_sensor_data,
        )
        self.imu = Mpu6050(bus=bus, address=address)
        self._last_error_log_s = 0.0

        if bool(self.get_parameter("calibrate_on_start").value):
            samples = max(int(self.get_parameter("calibration_samples").value), 1)
            delay_s = max(float(self.get_parameter("calibration_delay_s").value), 0.0)
            self.get_logger().info(
                f"Keep the IMU still while calibrating {samples} gyro samples"
            )
            self.bias = self.imu.calibrate_gyro(samples=samples, delay_s=delay_s)
        else:
            self.bias = GyroBias()

        publish_rate_hz = max(
            float(self.get_parameter("publish_rate_hz").value),
            0.1,
        )
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_once)
        self.get_logger().info(
            f"Reading MPU-6050 on I2C bus {bus} at 0x{address:02x}; "
            f"publishing {self.get_parameter('topic').value}"
        )

    def publish_once(self) -> None:
        try:
            sample = self.imu.read_sample(bias=self.bias)
        except Exception as exc:
            self._log_read_error(exc)
            return

        message = Imu()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id

        # The MPU-6050 driver currently has no absolute orientation estimate.
        message.orientation_covariance[0] = -1.0
        message.angular_velocity.x = angular_velocity_dps_to_rad_s(sample.gyro_x_dps)
        message.angular_velocity.y = angular_velocity_dps_to_rad_s(sample.gyro_y_dps)
        message.angular_velocity.z = angular_velocity_dps_to_rad_s(sample.gyro_z_dps)
        message.linear_acceleration.x = acceleration_g_to_mps2(sample.accel_x_g)
        message.linear_acceleration.y = acceleration_g_to_mps2(sample.accel_y_g)
        message.linear_acceleration.z = acceleration_g_to_mps2(sample.accel_z_g)
        self.publisher.publish(message)

    def _log_read_error(self, exc: Exception) -> None:
        now = time.monotonic()
        if now - self._last_error_log_s >= 5.0:
            self.get_logger().error(f"IMU read failed: {exc}")
            self._last_error_log_s = now

    def destroy_node(self) -> bool:
        self.imu.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
