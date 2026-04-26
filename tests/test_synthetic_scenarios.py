import unittest

from radar_nav import NavConfig, RadarNavPipeline, NavState
from radar_nav.clustering import cluster_points
from radar_nav.decision import choose_desired_command


def point(x, y, snr=280):
    return {"x": x, "y": y, "z": 0.0, "doppler": 0.0, "snr_raw": snr, "noise_raw": 900}


def front_cluster():
    return [
        point(-0.05, 0.65),
        point(0.03, 0.70),
        point(0.07, 0.62),
    ]


class SyntheticNavigationTests(unittest.TestCase):
    def test_no_points_scores_stay_low(self):
        pipeline = RadarNavPipeline(NavConfig())
        output = None
        for frame in range(100):
            output = pipeline.process_points([], frame_number=frame, now=frame * 0.1)

        self.assertIsNotNone(output)
        self.assertLess(output.front_score, 0.01)
        self.assertFalse(output.front_blocked)
        self.assertEqual(output.command, "forward")

    def test_persistent_front_obstacle_triggers_blocked(self):
        cfg = NavConfig(command_lock_s=0.0)
        pipeline = RadarNavPipeline(cfg)
        output = None
        for frame in range(30):
            output = pipeline.process_points(front_cluster(), frame_number=frame, now=frame * 0.1)

        self.assertIsNotNone(output)
        self.assertGreater(output.front_score, cfg.front_on_thresh)
        self.assertTrue(output.front_blocked)
        self.assertIn(output.command, ("stop", "turn_left", "turn_right"))

    def test_random_one_frame_noise_does_not_block_front(self):
        cfg = NavConfig(command_lock_s=0.0)
        pipeline = RadarNavPipeline(cfg)
        output = None
        for frame in range(80):
            points = [point(0.0, 0.7)] if frame % 10 == 0 else []
            output = pipeline.process_points(points, frame_number=frame, now=frame * 0.1)

        self.assertIsNotNone(output)
        self.assertLess(output.front_score, cfg.front_on_thresh)
        self.assertFalse(output.front_blocked)
        self.assertEqual(output.command, "forward")

    def test_object_disappears_uses_off_threshold(self):
        cfg = NavConfig(command_lock_s=0.0)
        pipeline = RadarNavPipeline(cfg)

        output = None
        for frame in range(30):
            output = pipeline.process_points(front_cluster(), frame_number=frame, now=frame * 0.1)
        self.assertTrue(output.front_blocked)

        seen_still_blocked = False
        seen_clear = False
        for frame in range(30, 70):
            output = pipeline.process_points([], frame_number=frame, now=frame * 0.1)
            if output.front_score >= cfg.front_off_thresh:
                seen_still_blocked = seen_still_blocked or output.front_blocked
            if output.front_score < cfg.front_off_thresh:
                seen_clear = seen_clear or not output.front_blocked

        self.assertTrue(seen_still_blocked)
        self.assertTrue(seen_clear)

    def test_left_right_tie_holds_previous_turn(self):
        cfg = NavConfig(side_margin=0.15)
        state = NavState(left_score=0.55, front_score=0.8, right_score=0.50, front_blocked=True, command="turn_left")
        desired, _reason = choose_desired_command(state, cfg)
        self.assertEqual(desired, "turn_left")

    def test_stronger_right_obstacle_turns_left(self):
        cfg = NavConfig(side_margin=0.15)
        state = NavState(left_score=0.1, front_score=0.8, right_score=0.7, front_blocked=True, command="stop")
        desired, _reason = choose_desired_command(state, cfg)
        self.assertEqual(desired, "turn_left")

    def test_stronger_left_obstacle_turns_right(self):
        cfg = NavConfig(side_margin=0.15)
        state = NavState(left_score=0.7, front_score=0.8, right_score=0.1, front_blocked=True, command="stop")
        desired, _reason = choose_desired_command(state, cfg)
        self.assertEqual(desired, "turn_right")

    def test_clustering_keeps_singletons_as_weak_evidence(self):
        cfg = NavConfig(cluster_min_points=2, keep_singletons=True)
        clusters = cluster_points([point(0.8, 1.0), point(-0.8, 1.2)], cfg)
        self.assertEqual(len(clusters), 2)
        self.assertTrue(all(cluster.is_singleton for cluster in clusters))
        self.assertTrue(all(0.0 < cluster.confidence < 0.25 for cluster in clusters))


if __name__ == "__main__":
    unittest.main()
