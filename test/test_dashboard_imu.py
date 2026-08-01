import math
from types import SimpleNamespace
import threading
import unittest

from web_dashboard.imu_ros import (
    RosImuReader,
    empty_imu_record,
    quaternion_to_euler_deg,
)


def vector(x: float, y: float, z: float) -> SimpleNamespace:
    return SimpleNamespace(x=x, y=y, z=z)


class DashboardImuTests(unittest.TestCase):
    def reader_without_ros(self) -> RosImuReader:
        reader = RosImuReader.__new__(RosImuReader)
        reader.raw_topic = "imu/data_raw"
        reader.stale_after_s = 2.0
        reader.log = None
        reader._lock = threading.Lock()
        reader._latest = empty_imu_record()
        reader._last_raw_at = None
        reader._last_orientation_at = None
        reader._error = None
        return reader

    def test_quaternion_converts_to_dashboard_angles(self) -> None:
        half_turn = math.radians(90.0) / 2.0
        roll, pitch, yaw = quaternion_to_euler_deg(
            math.cos(half_turn),
            0.0,
            0.0,
            math.sin(half_turn),
        )

        self.assertAlmostEqual(roll, 0.0)
        self.assertAlmostEqual(pitch, 0.0)
        self.assertAlmostEqual(yaw, 90.0)

    def test_raw_and_fused_messages_share_one_snapshot(self) -> None:
        reader = self.reader_without_ros()
        reader._on_raw(
            SimpleNamespace(
                header=SimpleNamespace(frame_id="imu_link"),
                linear_acceleration=vector(0.0, 0.0, 9.80665),
                angular_velocity=vector(0.0, 0.0, math.pi),
            )
        )
        reader._on_fused(
            SimpleNamespace(
                orientation=SimpleNamespace(
                    w=math.sqrt(0.5),
                    x=0.0,
                    y=0.0,
                    z=math.sqrt(0.5),
                )
            )
        )

        health, snapshot = reader.snapshot()

        self.assertEqual(health, "live")
        self.assertEqual(snapshot["frame_id"], "imu_link")
        self.assertAlmostEqual(snapshot["accel_mag_mps2"], 9.80665)
        self.assertAlmostEqual(snapshot["gyro_z_dps"], 180.0)
        self.assertAlmostEqual(snapshot["yaw_deg"], 90.0)
        self.assertTrue(snapshot["orientation_available"])

    def test_bno_diagnostics_become_calibration_state(self) -> None:
        reader = self.reader_without_ros()
        reader._on_diagnostics(
            SimpleNamespace(
                status=[
                    SimpleNamespace(
                        name="BNO055 IMU",
                        # Humble may expose diagnostic_msgs/DiagnosticStatus
                        # uint8 fields as a single byte rather than an int.
                        level=b"\x01",
                        message="calibration incomplete",
                        values=[
                            SimpleNamespace(
                                key="system_calibration",
                                value="2",
                            ),
                            SimpleNamespace(
                                key="gyroscope_calibration",
                                value="3",
                            ),
                            SimpleNamespace(
                                key="accelerometer_calibration",
                                value="3",
                            ),
                            SimpleNamespace(
                                key="magnetometer_calibration",
                                value="1",
                            ),
                        ],
                    )
                ]
            )
        )

        calibration = reader._latest["calibration"]
        self.assertEqual(calibration["status"], "calibrating")
        self.assertEqual(calibration["system"], 2)
        self.assertEqual(calibration["magnetometer"], 1)


if __name__ == "__main__":
    unittest.main()
