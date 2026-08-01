import math
import unittest

from imu.bno055 import Bno055, Bno055IdentityError


def signed_bytes(value: int) -> list[int]:
    encoded = value & 0xFFFF
    return [encoded & 0xFF, encoded >> 8]


class FakeBus:
    def __init__(self, chip_id: int = Bno055.CHIP_ID) -> None:
        self.byte_values = {
            Bno055.REG_CHIP_ID: chip_id,
            Bno055.REG_CALIBRATION_STATUS: 0b11100111,
            Bno055.REG_SYSTEM_STATUS: 5,
            Bno055.REG_SYSTEM_ERROR: 0,
        }
        self.block_values: dict[tuple[int, int], list[int]] = {}
        self.writes: list[tuple[int, int, int]] = []
        self.closed = False

    def read_byte_data(self, address: int, register: int) -> int:
        return self.byte_values[register]

    def write_byte_data(self, address: int, register: int, value: int) -> None:
        self.writes.append((address, register, value))

    def read_i2c_block_data(
        self,
        address: int,
        register: int,
        length: int,
    ) -> list[int]:
        return self.block_values[(register, length)]

    def close(self) -> None:
        self.closed = True


class Bno055Tests(unittest.TestCase):
    def test_configures_ndof_si_enu_and_requested_placement(self) -> None:
        bus = FakeBus()
        sleeps: list[float] = []

        imu = Bno055(
            bus=2,
            address=0x29,
            placement="P3",
            bus_factory=lambda _: bus,
            sleep=sleeps.append,
        )

        self.assertEqual(
            bus.writes,
            [
                (0x29, Bno055.REG_OPERATION_MODE, Bno055.MODE_CONFIG),
                (0x29, Bno055.REG_SYSTEM_TRIGGER, 0x20),
                (0x29, Bno055.REG_OPERATION_MODE, Bno055.MODE_CONFIG),
                (0x29, Bno055.REG_POWER_MODE, Bno055.POWER_NORMAL),
                (0x29, Bno055.REG_PAGE_ID, 0x00),
                (0x29, Bno055.REG_SYSTEM_TRIGGER, 0x00),
                (0x29, Bno055.REG_UNIT_SELECTION, Bno055.UNIT_SELECTION_ROS),
                (0x29, Bno055.REG_AXIS_MAP_CONFIG, 0x21),
                (0x29, Bno055.REG_AXIS_MAP_SIGN, 0x02),
                (0x29, Bno055.REG_OPERATION_MODE, Bno055.MODE_NDOF),
            ],
        )
        self.assertEqual(sleeps, [0.025, 0.700, 0.025, 0.500])
        imu.close()
        self.assertTrue(bus.closed)

    def test_rejects_a_different_chip(self) -> None:
        bus = FakeBus(chip_id=0x00)

        with self.assertRaises(Bno055IdentityError):
            Bno055(bus_factory=lambda _: bus, sleep=lambda _: None)

        self.assertTrue(bus.closed)

    def test_converts_sensor_registers_to_ros_units(self) -> None:
        bus = FakeBus()
        imu = Bno055(
            reset_on_start=False,
            bus_factory=lambda _: bus,
            sleep=lambda _: None,
        )

        sensor = [0] * 32
        sensor[0:6] = signed_bytes(981) + signed_bytes(-100) + signed_bytes(50)
        sensor[6:12] = signed_bytes(16) + signed_bytes(-32) + signed_bytes(48)
        sensor[12:18] = (
            signed_bytes(900)
            + signed_bytes(-450)
            + signed_bytes(0)
        )
        sensor[24:32] = (
            signed_bytes(8192)
            + signed_bytes(8192)
            + signed_bytes(8192)
            + signed_bytes(8192)
        )
        fusion = (
            signed_bytes(25)
            + signed_bytes(-50)
            + signed_bytes(100)
            + signed_bytes(0)
            + signed_bytes(0)
            + signed_bytes(981)
            + [0xFB]
        )
        bus.block_values[(Bno055.REG_ACCEL_DATA, 32)] = sensor
        bus.block_values[(Bno055.REG_LINEAR_ACCEL_DATA, 13)] = fusion

        sample = imu.read_sample()

        self.assertAlmostEqual(sample.acceleration_x_mps2, 9.81)
        self.assertAlmostEqual(sample.acceleration_y_mps2, -1.0)
        self.assertAlmostEqual(sample.angular_velocity_x_rad_s, 1.0)
        self.assertAlmostEqual(sample.angular_velocity_y_rad_s, -0.5)
        self.assertAlmostEqual(sample.magnetic_field_x_t, 1e-6)
        self.assertAlmostEqual(sample.magnetic_field_y_t, -2e-6)
        self.assertAlmostEqual(sample.linear_acceleration_z_mps2, 1.0)
        self.assertAlmostEqual(sample.gravity_z_mps2, 9.81)
        self.assertEqual(sample.temperature_c, -5.0)
        self.assertAlmostEqual(sample.orientation_w, 0.5)
        self.assertAlmostEqual(sample.orientation_x, 0.5)
        self.assertTrue(
            math.isclose(
                sum(
                    value * value
                    for value in (
                        sample.orientation_w,
                        sample.orientation_x,
                        sample.orientation_y,
                        sample.orientation_z,
                    )
                ),
                1.0,
            )
        )

    def test_decodes_calibration_and_system_status(self) -> None:
        bus = FakeBus()
        imu = Bno055(
            reset_on_start=False,
            bus_factory=lambda _: bus,
            sleep=lambda _: None,
        )

        status = imu.read_status()

        self.assertEqual(status.system_calibration, 3)
        self.assertEqual(status.gyroscope_calibration, 2)
        self.assertEqual(status.accelerometer_calibration, 1)
        self.assertEqual(status.magnetometer_calibration, 3)
        self.assertEqual(status.system_status, 5)
        self.assertEqual(status.system_error, 0)
        self.assertFalse(status.fully_calibrated)


if __name__ == "__main__":
    unittest.main()
