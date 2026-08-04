import unittest

from thruster_control import (
    ThrusterMapping,
    manual_to_pair,
    pair_to_manual,
)


class ThrusterMappingTests(unittest.TestCase):
    def setUp(self) -> None:
        self.mapping = ThrusterMapping(
            neutral_us=1500,
            forward_min_us=1565,
            forward_max_us=1650,
            reverse_level1_us=1460,
            reverse_level2_us=1445,
            reverse_level3_us=1425,
            pivot_turn_start=0.75,
            pivot_reverse_ratio=1.0,
        )

    def test_pair_conversion_round_trip(self) -> None:
        pair = manual_to_pair(0.5, 0.2, self.mapping)

        throttle, steering = pair_to_manual(
            pair.left_us,
            pair.right_us,
            self.mapping,
        )

        self.assertAlmostEqual(throttle, 0.5, delta=0.02)
        self.assertAlmostEqual(steering, 0.2, delta=0.02)

    def test_neutral_pair_converts_to_stop(self) -> None:
        self.assertEqual(pair_to_manual(1500, 1500), (0.0, 0.0))

    def test_measured_reverse_levels_are_mapped_exactly(self) -> None:
        self.assertEqual(self.mapping.throttle_to_us(-0.5), 1445)
        self.assertEqual(self.mapping.throttle_to_us(-1.0), 1425)

    def test_zero_throttle_stays_neutral_at_full_lock(self) -> None:
        for steering in (-1.0, 1.0):
            with self.subTest(steering=steering):
                pair = manual_to_pair(
                    0.0,
                    steering,
                    self.mapping,
                )
                self.assertEqual(
                    (pair.left_us, pair.right_us),
                    (1500, 1500),
                )

    def test_half_throttle_sharp_right_uses_reverse_level_2(self) -> None:
        pair = manual_to_pair(0.5, 1.0, self.mapping)

        self.assertEqual((pair.left_us, pair.right_us), (1650, 1445))

    def test_full_throttle_sharp_left_uses_reverse_level_3(self) -> None:
        pair = manual_to_pair(1.0, -1.0, self.mapping)

        self.assertEqual((pair.left_us, pair.right_us), (1425, 1650))

    def test_pivot_blend_begins_with_both_thrusters_forward(self) -> None:
        pair = manual_to_pair(0.5, 0.75, self.mapping)

        self.assertGreater(pair.left_us, 1500)
        self.assertGreater(pair.right_us, 1500)

    def test_full_pivot_pair_reports_full_right_steering(self) -> None:
        throttle, steering = pair_to_manual(1650, 1425, self.mapping)

        self.assertEqual(throttle, 1.0)
        self.assertEqual(steering, 1.0)


if __name__ == "__main__":
    unittest.main()
