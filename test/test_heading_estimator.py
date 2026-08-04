import threading
import unittest

from boat_core.autonomy import AutoConfig, AutoController
from boat_core.heading import HeadingEstimator, HeadingEstimatorConfig


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


class FakeReader:
    def __init__(self, health: str, values: dict) -> None:
        self.health = health
        self.values = values

    def snapshot(self) -> tuple[str, dict]:
        return self.health, dict(self.values)


class FakeControlState:
    def __init__(self, mode: str = "manual") -> None:
        self._lock = threading.Lock()
        self.mode = mode
        self.output = (1500, 1500)
        self.auto_status = {}

    def set_auto_status(self, status: dict) -> None:
        self.auto_status = dict(status)

    def apply_auto_pwm(
        self,
        left_us: int,
        right_us: int,
        _reason: str,
        status: dict | None = None,
    ) -> None:
        self.output = (left_us, right_us)
        if status is not None:
            self.auto_status = dict(status)


class HeadingEstimatorTests(unittest.TestCase):
    def setUp(self) -> None:
        self.clock = FakeClock()
        self.estimator = HeadingEstimator(
            HeadingEstimatorConfig(
                learn_duration_s=1.0,
                learn_min_samples=5,
                correction_blend=0.02,
            ),
            clock=self.clock,
        )
        self.gnss = {
            "heading_deg": 100.0,
            "speed_mps": 0.8,
        }
        self.imu = {
            "yaw_relative_deg": 10.0,
            "age_s": 0.0,
            "calibration": {
                "status": "calibrating",
                "system": 1,
                "gyroscope": 3,
                "accelerometer": 1,
                "magnetometer": 0,
                "recovery_count": 0,
            },
        }
        self.forward = {
            "left_us": 1600,
            "right_us": 1600,
            "age_s": 0.0,
        }

    def update(
        self,
        command: dict | None = None,
        command_health: str = "live",
    ) -> dict:
        return self.estimator.update(
            self.gnss,
            "live",
            self.imu,
            command_health,
            command or self.forward,
        )

    def lock_heading(self) -> dict:
        result = {}
        for _ in range(5):
            result = self.update()
            self.clock.advance(0.25)
        result = self.update()
        self.assertEqual(result["state"], "locked")
        return result

    def test_neutral_rope_motion_does_not_initialize_heading(self) -> None:
        neutral = {"left_us": 1500, "right_us": 1500, "age_s": 0.0}
        for _ in range(10):
            result = self.update(neutral)
            self.clock.advance(0.25)

        self.assertEqual(result["state"], "provisional")
        self.assertIsNone(result["heading_deg"])
        self.assertEqual(result["learning"]["samples"], 0)

    def test_straight_forward_motion_locks_with_imperfect_calibration(self) -> None:
        result = self.lock_heading()

        self.assertAlmostEqual(result["heading_deg"], 100.0, places=1)
        self.assertEqual(result["calibration"]["accelerometer"], 1)

    def test_rope_motion_is_ignored_after_lock(self) -> None:
        self.lock_heading()
        self.gnss["heading_deg"] = 280.0
        neutral = {"left_us": 1500, "right_us": 1500, "age_s": 0.0}

        result = self.update(neutral)

        self.assertEqual(result["state"], "locked")
        self.assertAlmostEqual(result["heading_deg"], 100.0, places=1)
        self.assertFalse(result["gnss_correction"]["accepted"])

    def test_bno_yaw_remains_authoritative_after_lock(self) -> None:
        self.lock_heading()
        self.imu["yaw_relative_deg"] = 30.0
        self.gnss["heading_deg"] = 300.0
        neutral = {"left_us": 1500, "right_us": 1500, "age_s": 0.0}

        result = self.update(neutral)

        self.assertAlmostEqual(result["heading_deg"], 120.0, places=1)

    def test_turning_command_does_not_teach_heading(self) -> None:
        turning = {"left_us": 1650, "right_us": 1570, "age_s": 0.0}
        for _ in range(10):
            result = self.update(turning)
            self.clock.advance(0.25)

        self.assertEqual(result["state"], "provisional")
        self.assertEqual(result["learning"]["samples"], 0)
        self.assertIn("straight", result["reason"])

    def test_stale_imu_degrades_a_locked_heading(self) -> None:
        self.lock_heading()

        result = self.estimator.update(
            self.gnss,
            "stale",
            self.imu,
            "live",
            self.forward,
        )

        self.assertEqual(result["state"], "degraded")
        self.assertIsNone(result["heading_deg"])

    def test_bno_recovery_requires_a_new_straight_run(self) -> None:
        self.lock_heading()
        self.imu["calibration"]["recovery_count"] = 1
        neutral = {"left_us": 1500, "right_us": 1500, "age_s": 0.0}

        result = self.update(neutral)

        self.assertEqual(result["state"], "provisional")
        self.assertIsNone(result["heading_deg"])
        self.assertIn("recovered", result["reason"])

    def test_explicit_reset_forgets_a_locked_heading(self) -> None:
        self.lock_heading()
        self.estimator.reset()
        neutral = {"left_us": 1500, "right_us": 1500, "age_s": 0.0}

        result = self.update(neutral)

        self.assertEqual(result["state"], "provisional")
        self.assertIsNone(result["heading_deg"])


class AutoHeadingSafetyTests(unittest.TestCase):
    def test_behind_target_turn_uses_measured_reverse_level(self) -> None:
        controller = AutoController(
            FakeControlState(),
            FakeReader("waiting", {}),
            config=AutoConfig(),
        )

        self.assertEqual(
            controller._smooth_turn_to_pwm(
                1.0,
                pivot_reverse=True,
            ),
            (1650.0, 1425.0),
        )
        self.assertEqual(
            controller._smooth_turn_to_pwm(
                -1.0,
                pivot_reverse=True,
            ),
            (1425.0, 1650.0),
        )

    def test_auto_waits_at_neutral_until_manual_heading_is_locked(self) -> None:
        clock = FakeClock()
        control = FakeControlState(mode="auto")
        controller = AutoController(
            control,
            FakeReader(
                "live",
                {
                    "fix": "fix",
                    "lat": 43.0,
                    "lon": -79.0,
                    "heading_deg": 90.0,
                    "speed_mps": 0.8,
                    "age_s": 0.0,
                },
            ),
            FakeReader(
                "live",
                {
                    "yaw_relative_deg": 10.0,
                    "gyro_z_dps": 0.0,
                    "age_s": 0.0,
                },
            ),
            AutoConfig(),
            thruster_reader=FakeReader(
                "live",
                {
                    "left_us": 1500,
                    "right_us": 1500,
                    "age_s": 0.0,
                },
            ),
            clock=clock,
        )
        controller.set_waypoints(
            [{"lat": 43.001, "lon": -79.0, "label": "test"}]
        )

        controller.tick()

        self.assertEqual(control.output, (1500, 1500))
        self.assertEqual(
            control.auto_status["state"],
            "acquiring_heading",
        )
        self.assertIn("manual", control.auto_status["reason"])

    def test_relearn_heading_neutralizes_auto_output(self) -> None:
        control = FakeControlState(mode="auto")
        control.output = (1650, 1575)
        controller = AutoController(
            control,
            FakeReader("waiting", {}),
            FakeReader("waiting", {}),
            AutoConfig(),
        )

        status = controller.relearn_heading()

        self.assertEqual(control.output, (1500, 1500))
        self.assertEqual(status["heading"]["state"], "uninitialized")
        self.assertIn("drive straight in manual", status["reason"])


if __name__ == "__main__":
    unittest.main()
