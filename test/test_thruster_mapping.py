import unittest

from thruster_control import (
    ThrusterMapping,
    manual_to_pair,
    pair_to_manual,
)


class ThrusterMappingTests(unittest.TestCase):
    def test_pair_conversion_round_trip(self) -> None:
        mapping = ThrusterMapping(
            neutral_us=1500,
            forward_min_us=1565,
            forward_max_us=1650,
        )
        pair = manual_to_pair(0.5, 0.2, mapping)

        throttle, steering = pair_to_manual(
            pair.left_us,
            pair.right_us,
            mapping,
        )

        self.assertAlmostEqual(throttle, 0.5, delta=0.02)
        self.assertAlmostEqual(steering, 0.2, delta=0.02)

    def test_neutral_pair_converts_to_stop(self) -> None:
        self.assertEqual(pair_to_manual(1500, 1500), (0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
