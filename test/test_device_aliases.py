from pathlib import Path
import unittest


class StableDeviceAliasTests(unittest.TestCase):
    def test_ros_config_uses_stable_serial_aliases(self) -> None:
        config = Path("config/ros/boat.example.yaml").read_text(
            encoding="utf-8",
        )
        self.assertIn("port: /dev/mtr_gnss", config)
        self.assertIn("port: /dev/mtr_esp32", config)
        self.assertNotIn("port: /dev/ttyACM", config)
        self.assertIn("status_topic: thrusters/status", config)

    def test_legacy_config_uses_stable_serial_aliases(self) -> None:
        config = Path("config/boat.example.json").read_text(
            encoding="utf-8",
        )
        self.assertIn('"port": "/dev/mtr_gnss"', config)
        self.assertIn('"port": "/dev/mtr_esp32"', config)

    def test_rules_match_tested_hardware(self) -> None:
        serial_rules = Path(
            "config/udev/99-mtr-serial.rules",
        ).read_text(encoding="utf-8")
        camera_rules = Path(
            "config/udev/99-mtr-camera.rules",
        ).read_text(encoding="utf-8")

        for expected in (
            'ATTRS{idVendor}=="10c4"',
            'ATTRS{idProduct}=="ea60"',
            'ATTRS{serial}=="0001"',
            'SYMLINK+="mtr_esp32"',
            'ATTRS{idVendor}=="1546"',
            'ATTRS{idProduct}=="01a9"',
            'SYMLINK+="mtr_gnss"',
        ):
            with self.subTest(expected=expected):
                self.assertIn(expected, serial_rules)
        self.assertIn('SYMLINK+="mtr_camera"', camera_rules)


if __name__ == "__main__":
    unittest.main()
