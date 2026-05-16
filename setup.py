from setuptools import find_packages, setup


package_name = "radar_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(include=["radar_nav", "radar_nav.*", "radar_ros", "radar_ros.*"]),
    py_modules=["mmwave_uart"],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["pyserial", "setuptools"],
    zip_safe=True,
    maintainer="mtr_radar",
    maintainer_email="user@example.com",
    description="ROS2 bridge for TI mmWave radar parsing and radar_nav obstacle state.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "radar_uart_node = radar_ros.radar_uart_node:main",
            "radar_nav_node = radar_ros.radar_nav_node:main",
        ],
    },
)
