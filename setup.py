from glob import glob

from setuptools import find_packages, setup


package_name = "mtr_boat_core"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(
        include=[
            "boat_core",
            "boat_core.*",
            "boat_ros",
            "boat_ros.*",
            "gnss",
            "gnss.*",
            "imu",
            "imu.*",
            "radar_nav",
            "radar_nav.*",
            "thruster_control",
            "thruster_control.*",
            "web_dashboard",
            "web_dashboard.*",
        ]
    ),
    package_data={"web_dashboard": ["*.css", "*.html", "*.js"]},
    py_modules=["mmwave_uart"],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.json")),
        (f"share/{package_name}/config", glob("config/ros/*.yaml")),
        (f"share/{package_name}/config/radar", glob("config/radar/*.cfg")),
        (f"share/{package_name}/config/udev", glob("config/udev/*.rules")),
    ],
    install_requires=["pynmea2", "pyserial", "setuptools", "smbus2"],
    zip_safe=True,
    maintainer="mtr_radar",
    maintainer_email="user@example.com",
    description="ROS 2 sensor and navigation foundation for the MTR boat.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "autonomy_node = boat_ros.autonomy_node:main",
            "bno055_node = boat_ros.bno055_node:main",
            "camera_node = boat_ros.camera_node:main",
            "control_supervisor_node = boat_ros.control_supervisor_node:main",
            "gnss_node = boat_ros.gnss_node:main",
            "imu_node = boat_ros.imu_node:main",
            "radar_uart_node = boat_ros.radar_uart_node:main",
            "radar_nav_node = boat_ros.radar_nav_node:main",
            "thruster_node = boat_ros.thruster_node:main",
            "web_dashboard = web_dashboard.server:main",
        ],
    },
)
