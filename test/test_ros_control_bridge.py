import json
from types import SimpleNamespace
import unittest

from thruster_control import ThrusterMapping
from web_dashboard.ros_control import RosCommandBridge


class FakeControlState:
    def __init__(self) -> None:
        self.auto_status = None

    def set_auto_status(self, status: dict) -> None:
        self.auto_status = status


class RosCommandBridgeTests(unittest.TestCase):
    def setUp(self) -> None:
        self.control = FakeControlState()
        self.bridge = RosCommandBridge(
            self.control,
            ThrusterMapping(),
        )

    def test_route_is_validated_before_ros_publication(self) -> None:
        route = self.bridge.set_waypoints(
            [{"lat": 43.47, "lon": -80.54, "label": "dock"}]
        )

        self.assertEqual(route["waypoints"][0]["label"], "dock")
        self.assertEqual(self.bridge.can_arm(), (True, "auto ready"))
        self.assertEqual(len(route["waypoints"]), 1)

    def test_appending_waypoint_does_not_flash_progress_back_to_zero(
        self,
    ) -> None:
        route = [
            {"lat": 43.4700, "lon": -80.5400, "label": "one"},
            {"lat": 43.4701, "lon": -80.5400, "label": "two"},
        ]
        self.bridge.set_waypoints(route)
        self.bridge._on_autonomy_status(
            SimpleNamespace(
                data=json.dumps(
                    {
                        "state": "reached",
                        "reason": "route complete",
                        "active_index": 2,
                        "total": 2,
                    }
                )
            )
        )

        self.bridge.set_waypoints(
            [
                *route,
                {"lat": 43.4702, "lon": -80.5400, "label": "three"},
            ]
        )

        self.assertEqual(self.bridge.status()["active_index"], 2)

    def test_empty_route_disarms_autonomy(self) -> None:
        self.bridge.set_waypoints([])
        self.assertEqual(
            self.bridge.can_arm(),
            (False, "auto requires at least one waypoint"),
        )

    def test_invalid_route_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "numeric lat/lon"):
            self.bridge.set_waypoints([{"lat": "north"}])

    def test_relearn_heading_is_queued_for_ros_publication(self) -> None:
        status = self.bridge.relearn_heading()

        self.assertEqual(status["heading"]["state"], "uninitialized")
        self.assertEqual(self.bridge._heading_reset_version, 1)
        self.assertEqual(self.control.auto_status, status)

    def test_steering_takeover_tracks_latest_heartbeat(self) -> None:
        result = self.bridge.set_steering_takeover(True, 0.65)

        self.assertEqual(result, {"active": True, "steering": 0.65})
        self.assertTrue(
            self.bridge.effective_output()["steering_takeover"]["active"]
        )

        result = self.bridge.set_steering_takeover(False, 0.0)
        self.assertEqual(result, {"active": False, "steering": 0.0})


if __name__ == "__main__":
    unittest.main()
