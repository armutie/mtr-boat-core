import math
import unittest

from boat_ros.sensor_math import (
    STANDARD_GRAVITY_MPS2,
    acceleration_g_to_mps2,
    angular_velocity_dps_to_rad_s,
)


class SensorMathTests(unittest.TestCase):
    def test_acceleration_uses_standard_gravity(self) -> None:
        self.assertAlmostEqual(acceleration_g_to_mps2(1.0), STANDARD_GRAVITY_MPS2)
        self.assertAlmostEqual(acceleration_g_to_mps2(-0.5), -STANDARD_GRAVITY_MPS2 / 2)

    def test_angular_velocity_is_converted_to_radians(self) -> None:
        self.assertAlmostEqual(angular_velocity_dps_to_rad_s(180.0), math.pi)
        self.assertAlmostEqual(angular_velocity_dps_to_rad_s(-90.0), -math.pi / 2)


if __name__ == "__main__":
    unittest.main()
