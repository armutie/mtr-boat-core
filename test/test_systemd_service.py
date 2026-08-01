import ast
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

    def test_optional_local_configs_are_used_when_present(self) -> None:
        self.assertIn("config/ros/boat.local.yaml", self.runner)
        self.assertIn("config/boat.local.json", self.runner)
        self.assertIn('params_file:=${local_params}', self.runner)
        self.assertIn(
            'dashboard_config:=${local_dashboard}',
            self.runner,
        )

    def test_bootstrap_and_service_share_repository_workspace(self) -> None:
        bootstrap = Path(
            "scripts/bootstrap_ros2_workspace.sh",
        ).read_text(encoding="utf-8")

        self.assertIn('workspace_dir="${repo_dir}"', bootstrap)
        self.assertIn(
            'workspace_setup="${repo_root}/install/setup.bash"',
            self.runner,
        )
        self.assertNotIn("mtr_ws", bootstrap)

    def test_runner_allows_ros_setup_optional_variables(self) -> None:
        self.assertIn("set -eo pipefail", self.runner)
        self.assertNotIn("set -euo pipefail", self.runner)

    def test_installer_enables_without_starting_service(self) -> None:
        self.assertIn("systemctl enable mtr-boat.service", self.installer)
        self.assertNotIn(
            "systemctl enable --now mtr-boat.service",
            self.installer,
        )

    def test_critical_ros_processes_respawn(self) -> None:
        expected_nodes = {
            "launch/sensors.launch.py": {
                "gnss_node",
                "imu_node",
                "bno055_node",
                "camera_node",
                "seyond_pointcloud_node",
            },
            "launch/control.launch.py": {
                "control_supervisor_node",
                "thruster_node",
            },
            "launch/boat.launch.py": {
                "radar_uart_node",
                "radar_nav_node",
                "autonomy_node",
            },
        }
        for path, expected in expected_nodes.items():
            source = Path(path).read_text(encoding="utf-8")
            tree = ast.parse(source)
            respawning = set()
            for call in ast.walk(tree):
                if not isinstance(call, ast.Call):
                    continue
                if not isinstance(call.func, ast.Name):
                    continue
                if call.func.id != "Node":
                    continue
                keywords = {
                    keyword.arg: keyword.value
                    for keyword in call.keywords
                }
                executable = keywords.get("executable")
                respawn = keywords.get("respawn")
                if (
                    isinstance(executable, ast.Constant)
                    and isinstance(respawn, ast.Constant)
                    and respawn.value is True
                ):
                    respawning.add(str(executable.value))
            self.assertTrue(
                expected <= respawning,
                f"{path} missing respawn for {expected - respawning}",
            )

        boat_source = Path("launch/boat.launch.py").read_text(
            encoding="utf-8",
        )
        boat_tree = ast.parse(boat_source)
        dashboard_respawns = any(
            isinstance(call, ast.Call)
            and isinstance(call.func, ast.Name)
            and call.func.id == "ExecuteProcess"
            and any(
                keyword.arg == "respawn"
                and isinstance(keyword.value, ast.Constant)
                and keyword.value.value is True
                for keyword in call.keywords
            )
            for call in ast.walk(boat_tree)
        )
        self.assertTrue(dashboard_respawns)


if __name__ == "__main__":
    unittest.main()
