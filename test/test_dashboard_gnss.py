import threading
from types import SimpleNamespace
import unittest

from web_dashboard.gnss_ros import RosGnssReader, empty_gnss_record


class DashboardGnssTests(unittest.TestCase):
    def reader_without_ros(self) -> RosGnssReader:
        reader = RosGnssReader.__new__(RosGnssReader)
        reader.fix_topic = "gnss/fix"
        reader.velocity_topic = "gnss/velocity"
        reader.stale_after_s = 3.0
        reader.log = None
        reader._lock = threading.Lock()
        reader._latest = empty_gnss_record()
        reader._last_fix_at = None
        reader._error = None
        return reader

    def test_fix_and_enu_velocity_form_dashboard_record(self) -> None:
        reader = self.reader_without_ros()
        reader._on_fix(
            SimpleNamespace(
                latitude=43.47,
                longitude=-80.54,
                altitude=320.0,
                status=SimpleNamespace(status=0),
                header=SimpleNamespace(frame_id="gnss_link"),
            )
        )
        reader._on_velocity(
            SimpleNamespace(
                twist=SimpleNamespace(
                    linear=SimpleNamespace(x=2.0, y=0.0),
                )
            )
        )

        health, record = reader.snapshot()
        self.assertEqual(health, "live")
        self.assertEqual(record["fix"], "fix")
        self.assertAlmostEqual(record["speed_mps"], 2.0)
        self.assertAlmostEqual(record["heading_deg"], 90.0)

    def test_diagnostics_supply_satellites_and_hdop(self) -> None:
        reader = self.reader_without_ros()
        reader._on_diagnostics(
            SimpleNamespace(
                status=[
                    SimpleNamespace(
                        name="GNSS",
                        values=[
                            SimpleNamespace(key="satellites", value="12"),
                            SimpleNamespace(key="hdop", value="0.8"),
                        ],
                    )
                ]
            )
        )

        self.assertEqual(reader._latest["satellites"], 12)
        self.assertAlmostEqual(reader._latest["hdop"], 0.8)


if __name__ == "__main__":
    unittest.main()
