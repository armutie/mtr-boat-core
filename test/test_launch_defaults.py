import ast
from pathlib import Path
import unittest


class SensorLaunchDefaultsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        launch_source = Path("launch/sensors.launch.py").read_text(
            encoding="utf-8",
        )
        cls.launch_tree = ast.parse(launch_source)

    def launch_default(self, argument: str) -> str | None:
        for node in ast.walk(self.launch_tree):
            if not isinstance(node, ast.Call):
                continue
            if not isinstance(node.func, ast.Name):
                continue
            if node.func.id != "DeclareLaunchArgument" or not node.args:
                continue
            if not isinstance(node.args[0], ast.Constant):
                continue
            if node.args[0].value != argument:
                continue
            for keyword in node.keywords:
                if (
                    keyword.arg == "default_value"
                    and isinstance(keyword.value, ast.Constant)
                ):
                    return str(keyword.value.value)
        return None

    def test_bno055_is_the_default_imu(self) -> None:
        self.assertEqual(self.launch_default("imu_driver"), "bno055")

    def test_bno055_fused_orientation_is_enabled(self) -> None:
        self.assertEqual(
            self.launch_default("publish_fused_orientation"),
            "true",
        )

    def test_unmeasured_sensor_transforms_remain_disabled(self) -> None:
        for argument in (
            "publish_imu_tf",
            "publish_camera_tf",
            "publish_lidar_tf",
        ):
            with self.subTest(argument=argument):
                self.assertEqual(self.launch_default(argument), "false")


class ControlLaunchDefaultsTests(unittest.TestCase):
    def test_thruster_is_disabled_by_default(self) -> None:
        launch_source = Path("launch/control.launch.py").read_text(
            encoding="utf-8",
        )
        launch_tree = ast.parse(launch_source)
        for node in ast.walk(launch_tree):
            if not isinstance(node, ast.Call):
                continue
            if not isinstance(node.func, ast.Name):
                continue
            if node.func.id != "DeclareLaunchArgument":
                continue
            if not node.args or not isinstance(node.args[0], ast.Constant):
                continue
            if node.args[0].value != "enable_thruster":
                continue
            default = next(
                (
                    keyword.value.value
                    for keyword in node.keywords
                    if keyword.arg == "default_value"
                    and isinstance(keyword.value, ast.Constant)
                ),
                None,
            )
            self.assertEqual(default, "false")
            return
        self.fail("enable_thruster launch argument not found")


class BoatLaunchDefaultsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        launch_source = Path("launch/boat.launch.py").read_text(
            encoding="utf-8",
        )
        cls.launch_tree = ast.parse(launch_source)

    def launch_default(self, argument: str) -> str | None:
        for node in ast.walk(self.launch_tree):
            if not isinstance(node, ast.Call):
                continue
            if not isinstance(node.func, ast.Name):
                continue
            if node.func.id != "DeclareLaunchArgument" or not node.args:
                continue
            if not isinstance(node.args[0], ast.Constant):
                continue
            if node.args[0].value != argument:
                continue
            for keyword in node.keywords:
                if (
                    keyword.arg == "default_value"
                    and isinstance(keyword.value, ast.Constant)
                ):
                    return str(keyword.value.value)
        return None

    def test_full_runtime_starts_dashboard_and_autonomy(self) -> None:
        self.assertEqual(self.launch_default("enable_dashboard"), "true")
        self.assertEqual(self.launch_default("enable_autonomy"), "true")

    def test_full_runtime_keeps_thrusters_disabled(self) -> None:
        self.assertEqual(self.launch_default("enable_thruster"), "false")

    def test_dashboard_uses_ros_package_executable_path(self) -> None:
        launch_source = Path("launch/boat.launch.py").read_text(
            encoding="utf-8",
        )
        self.assertIn(
            'FindPackagePrefix("mtr_boat_core")',
            launch_source,
        )
        self.assertNotIn(
            'FindExecutable(name="web_dashboard")',
            launch_source,
        )


class Bno055DefaultsTests(unittest.TestCase):
    def test_fused_orientation_is_published_for_the_dashboard(self) -> None:
        config = Path("config/ros/boat.example.yaml").read_text(
            encoding="utf-8",
        )
        self.assertIn("publish_fused_orientation: true", config)


if __name__ == "__main__":
    unittest.main()
