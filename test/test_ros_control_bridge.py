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
        self.assertEqual(self.control.auto_status["total"], 1)

    def test_empty_route_disarms_autonomy(self) -> None:
        self.bridge.set_waypoints([])
        self.assertEqual(
            self.bridge.can_arm(),
            (False, "auto requires at least one waypoint"),
        )

    def test_invalid_route_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "numeric lat/lon"):
            self.bridge.set_waypoints([{"lat": "north"}])


if __name__ == "__main__":
    unittest.main()
