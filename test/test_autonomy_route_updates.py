import unittest

from boat_core.autonomy import AutoController


class FakeControlState:
    def __init__(self) -> None:
        self.auto_status = {}

    def set_auto_status(self, status: dict) -> None:
        self.auto_status = dict(status)


class AutonomyRouteUpdateTests(unittest.TestCase):
    def setUp(self) -> None:
        self.controller = AutoController(
            FakeControlState(),
            gnss_reader=None,
        )
        self.first_two = [
            {"lat": 43.4700, "lon": -80.5400, "label": "one"},
            {"lat": 43.4701, "lon": -80.5400, "label": "two"},
        ]

    def test_appending_third_waypoint_preserves_completed_prefix(
        self,
    ) -> None:
        self.controller.set_waypoints(self.first_two)
        with self.controller._lock:
            self.controller._active_index = 2

        route = self.controller.set_waypoints(
            [
                *self.first_two,
                {"lat": 43.4702, "lon": -80.5400, "label": "three"},
            ]
        )

        self.assertEqual(route["status"]["active_index"], 2)
        self.assertEqual(route["status"]["total"], 3)

    def test_changing_completed_prefix_restarts_route(self) -> None:
        self.controller.set_waypoints(self.first_two)
        with self.controller._lock:
            self.controller._active_index = 2

        route = self.controller.set_waypoints(
            [
                {"lat": 43.4800, "lon": -80.5400, "label": "new one"},
                self.first_two[1],
            ]
        )

        self.assertEqual(route["status"]["active_index"], 0)


if __name__ == "__main__":
    unittest.main()
