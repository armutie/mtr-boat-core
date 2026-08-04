from pathlib import Path
import unittest


class AutoDashboardUiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.html = Path("web_dashboard/index.html").read_text(
            encoding="utf-8"
        )
        cls.javascript = Path("web_dashboard/app.js").read_text(
            encoding="utf-8"
        )

    def test_map_tap_adds_without_an_add_mode(self) -> None:
        self.assertNotIn('id="auto-add-pin"', self.html)
        self.assertNotIn("addPinMode", self.javascript)
        self.assertIn(
            "addAutoWaypoint(evt.latlng.lat, evt.latlng.lng);",
            self.javascript,
        )

    def test_route_edits_have_explicit_actions(self) -> None:
        for control_id in (
            "auto-undo-route",
            "auto-clear-route",
            "auto-clear-confirm-button",
            "auto-delete-waypoint",
        ):
            self.assertIn(f'id="{control_id}"', self.html)

    def test_auto_wheel_uses_the_steering_takeover_path(self) -> None:
        self.assertIn('id="auto-mini-wheel"', self.html)
        self.assertIn(
            'postJson("/api/control/steering-takeover"',
            self.javascript,
        )
        self.assertIn(
            "num(output.steering) ?? steeringFromPwm(",
            self.javascript,
        )


if __name__ == "__main__":
    unittest.main()
