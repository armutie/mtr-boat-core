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

    def test_unmeasured_sensor_transforms_remain_disabled(self) -> None:
        for argument in (
            "publish_imu_tf",
            "publish_camera_tf",
            "publish_lidar_tf",
        ):
            with self.subTest(argument=argument):
                self.assertEqual(self.launch_default(argument), "false")


if __name__ == "__main__":
    unittest.main()
