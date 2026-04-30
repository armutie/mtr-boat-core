import unittest

from radar_nav import NavConfig, RadarNavPipeline
from radar_nav.sim import (
    BoatConfig,
    BoatState,
    CandidateControl,
    CandidateControllerConfig,
    ControllerState,
    CircleObstacle,
    ObstacleCourse,
    RectObstacle,
    SimMetrics,
    WaypointConfig,
    WaypointState,
    blend_waypoint_and_avoidance,
    choose_candidate_control,
    compute_waypoint_control,
    generate_radar_points,
    obstacle_bounds,
    score_candidate_control,
    update_boat,
    update_metrics,
)


class BoatSimTests(unittest.TestCase):
    def test_no_obstacles_throttle_rises_and_boat_moves_forward(self):
        cfg = NavConfig()
        pipeline = RadarNavPipeline(cfg)
        boat = BoatState()
        boat_cfg = BoatConfig()

        output = None
        for frame in range(80):
            output = pipeline.process_points([], now=frame / 30.0)
            update_boat(boat, boat_cfg, output.throttle, output.steering, 1.0 / 30.0)

        self.assertIsNotNone(output)
        self.assertGreater(output.throttle, 0.85)
        self.assertGreater(boat.y, 1.0)
        self.assertAlmostEqual(boat.x, 0.0, delta=0.05)

    def test_centered_obstacle_reduces_throttle_before_collision_range(self):
        cfg = NavConfig()
        pipeline = RadarNavPipeline(cfg)
        boat = BoatState()
        course = ObstacleCourse([CircleObstacle(0.0, 0.9, 0.22)])

        output = None
        for frame in range(36):
            points = generate_radar_points(boat, course, cfg)
            output = pipeline.process_points(points, now=frame / 30.0)

        self.assertIsNotNone(output)
        self.assertTrue(output.front_blocked)
        self.assertLess(output.throttle, 0.6)

    def test_more_obstacle_evidence_on_right_steers_left(self):
        cfg = NavConfig()
        pipeline = RadarNavPipeline(cfg)
        boat = BoatState()
        course = ObstacleCourse(
            [
                CircleObstacle(0.0, 0.95, 0.18),
                CircleObstacle(0.45, 1.0, 0.22),
                CircleObstacle(0.72, 1.1, 0.20),
            ]
        )

        output = None
        for frame in range(50):
            points = generate_radar_points(boat, course, cfg)
            output = pipeline.process_points(points, now=frame / 30.0)

        self.assertIsNotNone(output)
        self.assertLess(output.steering, -0.15)

    def test_boat_speed_decays_when_throttle_is_low(self):
        boat = BoatState(speed=1.2)
        boat_cfg = BoatConfig()

        for _ in range(60):
            update_boat(boat, boat_cfg, throttle=0.0, steering=0.0, dt=1.0 / 30.0)

        self.assertLess(boat.speed, 0.8)

    def test_radar_points_exclude_behind_and_outside_bounds(self):
        cfg = NavConfig(max_y=2.0, lateral_limit=1.0)
        boat = BoatState()
        course = ObstacleCourse(
            [
                CircleObstacle(0.0, 1.0, 0.1),
                CircleObstacle(0.0, -1.0, 0.1),
                CircleObstacle(2.0, 1.0, 0.1),
            ]
        )

        points = generate_radar_points(boat, course, cfg, sample_spacing=0.1)

        self.assertGreater(len(points), 0)
        self.assertTrue(all(point["y"] >= cfg.min_y for point in points))
        self.assertTrue(all(point["y"] <= cfg.max_y for point in points))
        self.assertTrue(all(abs(point["x"]) <= cfg.lateral_limit for point in points))
        self.assertTrue(all(point["snr_raw"] >= cfg.min_snr_raw for point in points))

    def test_rectangle_radar_points_only_use_visible_face(self):
        cfg = NavConfig(max_y=3.0, lateral_limit=2.0)
        boat = BoatState(x=0.0, y=0.0)
        course = ObstacleCourse([RectObstacle(0.0, 1.0, 1.0, 0.4)])

        points = generate_radar_points(boat, course, cfg, sample_spacing=0.1)
        ys = [round(point["y"], 2) for point in points]

        self.assertGreater(len(points), 0)
        self.assertTrue(all(y <= 0.8 for y in ys))
        self.assertFalse(any(y >= 1.2 for y in ys))

    def test_circle_radar_points_use_near_arc(self):
        cfg = NavConfig(max_y=3.0, lateral_limit=2.0)
        boat = BoatState(x=0.0, y=0.0)
        course = ObstacleCourse([CircleObstacle(0.0, 1.0, 0.4)])

        points = generate_radar_points(boat, course, cfg, sample_spacing=0.05)

        self.assertGreater(len(points), 0)
        self.assertLess(max(point["y"] for point in points), 1.02)

    def test_touching_obstacle_marks_run_failed(self):
        boat = BoatState()
        boat_cfg = BoatConfig(collision_radius=0.18)
        course = ObstacleCourse([CircleObstacle(0.0, 0.0, 0.25)])
        metrics = SimMetrics()

        update_metrics(metrics, boat, boat_cfg, course, output=None, dt=1.0 / 30.0)

        self.assertTrue(boat.collided)
        self.assertTrue(boat.failed)
        self.assertTrue(metrics.failed)
        self.assertEqual(metrics.collisions, 1)

    def test_course_generates_obstacles_inside_arena(self):
        course = ObstacleCourse.default(seed=7)

        self.assertEqual(len(course.obstacles), course.obstacle_count)
        self.assertEqual(course.arena_radius, 20.0)
        for obstacle in course.obstacles:
            min_x, max_x, min_y, max_y = obstacle_bounds([obstacle])
            corners = [(min_x, min_y), (min_x, max_y), (max_x, min_y), (max_x, max_y)]
            self.assertTrue(all((x * x + y * y) ** 0.5 < course.arena_radius for x, y in corners))

    def test_course_reset_randomizes_obstacle_layout(self):
        course = ObstacleCourse.default(seed=7)
        first_seed = course.seed
        first_layout = [(type(obstacle).__name__, obstacle_bounds([obstacle])) for obstacle in course.obstacles]

        course.reset()
        second_layout = [(type(obstacle).__name__, obstacle_bounds([obstacle])) for obstacle in course.obstacles]

        self.assertNotEqual(course.seed, first_seed)
        self.assertNotEqual(second_layout, first_layout)

    def test_leaving_arena_marks_run_failed(self):
        course = ObstacleCourse.default(seed=7)
        boat = BoatState(x=course.arena_radius + 0.2, y=0.0)
        boat_cfg = BoatConfig(collision_radius=0.18)
        metrics = SimMetrics()

        update_metrics(metrics, boat, boat_cfg, course, output=None, dt=1.0 / 30.0)

        self.assertTrue(boat.failed)
        self.assertTrue(metrics.failed)
        self.assertEqual(metrics.failure_reason, "left arena")

    def test_waypoint_reset_places_target_inside_arena(self):
        course = ObstacleCourse.default(seed=7)
        boat = BoatState()
        waypoint = WaypointState(seed=1)
        waypoint.reset(boat, WaypointConfig(), course, randomize_seed=False)

        self.assertLess((waypoint.x * waypoint.x + waypoint.y * waypoint.y) ** 0.5, course.arena_radius)
        self.assertGreater(((waypoint.x - boat.x) ** 2 + (waypoint.y - boat.y) ** 2) ** 0.5, 4.0)

    def test_waypoint_control_steers_toward_target(self):
        boat = BoatState()
        waypoint = WaypointState(x=1.0, y=5.0, seed=1)
        control = compute_waypoint_control(boat, waypoint, WaypointConfig())

        self.assertGreater(control.steering, 0.0)
        self.assertGreater(control.throttle, 0.5)
        self.assertFalse(control.reached)

    def test_waypoint_control_marks_close_target_reached(self):
        boat = BoatState(x=0.0, y=0.0)
        waypoint = WaypointState(x=0.1, y=0.1, seed=1)
        control = compute_waypoint_control(boat, waypoint, WaypointConfig(reach_radius=0.3))

        self.assertTrue(control.reached)
        self.assertEqual(control.throttle, 0.0)
        self.assertEqual(control.steering, 0.0)

    def test_blended_control_uses_waypoint_and_avoidance(self):
        cfg = NavConfig()
        pipeline = RadarNavPipeline(cfg)
        output = None
        for frame in range(40):
            output = pipeline.process_points(
                [
                    {"x": 0.0, "y": 0.8, "z": 0.0, "doppler": 0.0, "snr_raw": 280, "noise_raw": 900},
                    {"x": 0.6, "y": 0.8, "z": 0.0, "doppler": 0.0, "snr_raw": 280, "noise_raw": 900},
                    {"x": 0.7, "y": 0.9, "z": 0.0, "doppler": 0.0, "snr_raw": 280, "noise_raw": 900},
                ],
                now=frame / 30.0,
            )
        waypoint_control = compute_waypoint_control(BoatState(), WaypointState(x=0.8, y=6.0), WaypointConfig())
        control = blend_waypoint_and_avoidance(waypoint_control, output, WaypointConfig())

        self.assertLess(control.steering, waypoint_control.steering)
        self.assertLessEqual(control.throttle, waypoint_control.throttle)

    def test_candidate_score_rewards_waypoint_progress(self):
        boat = BoatState()
        boat_cfg = BoatConfig()
        course = ObstacleCourse([], arena_radius=20.0)
        waypoint = WaypointState(x=0.0, y=8.0)
        cfg = CandidateControllerConfig()

        forward_cost, _ = score_candidate_control(
            boat,
            boat_cfg,
            waypoint,
            course,
            [],
            CandidateControl(throttle=0.85, steering=0.0),
            cfg,
        )
        slow_turn_cost, _ = score_candidate_control(
            boat,
            boat_cfg,
            waypoint,
            course,
            [],
            CandidateControl(throttle=0.28, steering=1.0),
            cfg,
        )

        self.assertLess(forward_cost, slow_turn_cost)

    def test_candidate_controller_avoids_visible_point_cloud(self):
        boat = BoatState()
        boat_cfg = BoatConfig()
        course = ObstacleCourse([], arena_radius=20.0)
        waypoint = WaypointState(x=0.0, y=8.0)
        nav_output = RadarNavPipeline(NavConfig()).process_points(
            [
                {"x": 0.35, "y": 0.7, "z": 0.0, "doppler": 0.0, "snr_raw": 280, "noise_raw": 900},
                {"x": 0.45, "y": 0.8, "z": 0.0, "doppler": 0.0, "snr_raw": 280, "noise_raw": 900},
                {"x": 0.55, "y": 0.9, "z": 0.0, "doppler": 0.0, "snr_raw": 280, "noise_raw": 900},
            ],
            now=0.0,
        )

        controller_state = ControllerState(throttle=0.7, steering=0.0)
        control = choose_candidate_control(boat, boat_cfg, waypoint, WaypointConfig(), nav_output, course, controller_state)

        self.assertLess(control.steering, 0.0)
        self.assertGreaterEqual(control.steering, -0.08)
        self.assertIsNotNone(control.min_predicted_clearance)

    def test_candidate_controller_uses_delta_from_current_control(self):
        boat = BoatState()
        boat_cfg = BoatConfig()
        course = ObstacleCourse([], arena_radius=20.0)
        waypoint = WaypointState(x=0.0, y=8.0)
        nav_output = RadarNavPipeline(NavConfig()).process_points([], now=0.0)
        controller_state = ControllerState(throttle=0.5, steering=0.2)

        control = choose_candidate_control(boat, boat_cfg, waypoint, WaypointConfig(), nav_output, course, controller_state)

        self.assertLessEqual(abs(control.steering - controller_state.steering), 0.081)
        self.assertLessEqual(abs(control.throttle - controller_state.throttle), 0.061)

    def test_candidate_controller_does_not_idle_when_front_is_clear(self):
        boat = BoatState()
        boat_cfg = BoatConfig()
        course = ObstacleCourse([], arena_radius=20.0)
        waypoint = WaypointState(x=0.0, y=8.0)
        nav_output = RadarNavPipeline(NavConfig()).process_points([], now=0.0)
        controller_state = ControllerState(throttle=0.0, steering=0.0)

        control = choose_candidate_control(boat, boat_cfg, waypoint, WaypointConfig(), nav_output, course, controller_state)

        self.assertGreater(control.throttle, 0.0)


if __name__ == "__main__":
    unittest.main()
