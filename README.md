# mtr_radar

TI mmWave radar parsing, obstacle evidence, and robot navigation experiments.

## ROS2 bridge

This repo can be built as a ROS2 Python package named `radar_nav_ros`. The ROS bridge is split into a sensor node and a navigation node:

- `radar_uart_node`: reads the TI mmWave USB serial stream, parses frames, and publishes decoded point clouds.
- `radar_nav_node`: subscribes to decoded point clouds, runs `RadarNavPipeline`, and publishes filtered points plus navigation state.

Published topics:

- `radar/raw_points` (`sensor_msgs/PointCloud2`): decoded radar points with `x`, `y`, `z`, `doppler`, `snr_raw`, and `noise_raw` fields.
- `radar/filtered_points` (`sensor_msgs/PointCloud2`): points after the current navigation filter.
- `radar/clusters_json` (`std_msgs/String`): JSON list of obstacle clusters.
- `radar/nav_state_json` (`std_msgs/String`): JSON navigation state matching the JSONL logger schema.

### Orange Pi setup

From a ROS2 workspace on the robot:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <your-github-url> mtr_radar
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select radar_nav_ros
source install/setup.bash
```

Install Python serial support if your ROS image does not already provide it:

```bash
python3 -m pip install pyserial
```

Run the live UART publisher:

```bash
ros2 run radar_nav_ros radar_uart_node --ros-args \
  -p data_port:=/dev/ttyUSB1 \
  -p baud:=921600 \
  -p frame_id:=radar
```

If the Orange Pi should also send the TI `.cfg` file at startup:

```bash
ros2 run radar_nav_ros radar_uart_node --ros-args \
  -p cfg_port:=/dev/ttyUSB0 \
  -p cfg_file:=/home/orangepi/radar.cfg \
  -p data_port:=/dev/ttyUSB1
```

In another terminal, run the navigation node:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run radar_nav_ros radar_nav_node
```

Check topics:

```bash
ros2 topic list
ros2 topic echo /radar/nav_state_json
ros2 topic hz /radar/raw_points
```

Run the existing pygame viewer as a ROS2 subscriber instead of reading the UART directly:

```bash
python3 run_nav_live.py --ros --nav-state-topic /radar/nav_state_json
```

Run the browser dashboard from the robot or development laptop:

```bash
python3 web_dashboard/server.py --ros --mmwave-topic radar/nav_state_json
```

Then open:

```text
http://localhost:8080
http://<orange-pi-or-laptop-ip>:8080
```

The dashboard defaults to unavailable sensor states unless `--ros` or `--demo` is provided. `--ros` subscribes to the mmWave navigation JSON topic and keeps GNSS, sonar, and ultrasonic marked unavailable until those feeds are added.

For UI-only testing without connected sensors:

```bash
python3 web_dashboard/server.py --demo
```

The topic names and navigation thresholds are ROS parameters, so a clusterer, controller, or web bridge can subscribe without importing the serial parser directly.

## Basic radar to ESP32 thruster test

The repo includes a minimal serial bridge for a first hardware test:

- laptop reads the radar UART and runs `RadarNavPipeline`
- laptop sends `PWM <microseconds>` or `STOP` lines to an ESP32 over USB serial
- ESP32 firmware should already listen for those serial lines and drive the ESC

Start with dry-run output before connecting/arming the thruster:

```bash
python run_nav_esp32.py --dry-run --esp32-port COM7 --data-port COM5
```

Then run the real serial bridge:

```bash
python run_nav_esp32.py --esp32-port COM7 --data-port COM5
```

If the TI radar also needs its `.cfg` file sent at startup:

```bash
python run_nav_esp32.py --esp32-port COM7 --cfg-port COM6 --cfg-file path/to/radar.cfg --data-port COM5
```

The current defaults use neutral `1500 us`, gentle forward output up to `1600 us`, and a hard clamp of `1350-2000 us`. Tune with `--forward-max-us`, `--forward-min-us`, and `--send-hz` only after the neutral/failsafe behavior is verified.
