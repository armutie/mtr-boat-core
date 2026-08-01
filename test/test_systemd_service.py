from pathlib import Path
import unittest


class SystemdServiceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.service = Path(
            "config/systemd/mtr-boat.service.in",
        ).read_text(encoding="utf-8")
        cls.runner = Path(
            "scripts/run_boat_service.sh",
        ).read_text(encoding="utf-8")
        cls.installer = Path(
            "scripts/install_systemd_service.sh",
        ).read_text(encoding="utf-8")

    def test_service_restarts_failures_and_stops_launch_cleanly(self) -> None:
        self.assertIn("Restart=on-failure", self.service)
        self.assertIn("KillSignal=SIGINT", self.service)
        self.assertIn("WantedBy=multi-user.target", self.service)

    def test_boot_runtime_starts_serial_owner(self) -> None:
        self.assertIn("enable_thruster:=true", self.runner)
        self.assertIn("initialize in off mode", self.runner)

    def test_runner_allows_ros_setup_optional_variables(self) -> None:
        self.assertIn("set -eo pipefail", self.runner)
        self.assertNotIn("set -euo pipefail", self.runner)

    def test_installer_enables_without_starting_service(self) -> None:
        self.assertIn("systemctl enable mtr-boat.service", self.installer)
        self.assertNotIn(
            "systemctl enable --now mtr-boat.service",
            self.installer,
        )


if __name__ == "__main__":
    unittest.main()
