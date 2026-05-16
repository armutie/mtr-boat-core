# mtr-boat-core

Orange Pi robot software for the MTR boat experiments.

This repo is meant to become the shared home for the boat's sensor drivers, navigation logic, ROS2 nodes, dashboard, and actuator bridges. Right now the working path is direct Python bench/field testing: radar UART in, ESP32 thruster serial out. ROS2 support exists for the radar path, but full robot control should move there after the hardware behavior is proven.

## Current Capabilities

- Read TI xWR18xx mmWave radar frames over UART.
- Filter and cluster radar points into obstacle evidence.
- Produce simple navigation output: throttle, steering, command, and reason.
- Send gentle ESC PWM commands to an ESP32 over serial for basic thruster tests.
- Run the current hardware test directly from Python scripts.
- Build the radar reader/navigation path as ROS2 nodes for later robot integration.
- Visualize radar/navigation state with pygame or the browser dashboard.

## Layout

- `radar_nav/`: core non-ROS radar/navigation logic. Scripts and ROS nodes both use this.
- `radar_ros/`: ROS2 wrapper nodes around the radar parser and `radar_nav` pipeline.
- `thruster_control/`: ESP32 serial and ESC PWM mapping helpers.
- `scripts/`: runnable robot/development commands.
- `config/radar/`: radar profile files for startup.
- `web_dashboard/`: browser dashboard for demo/ROS sensor state.
- `mmwave_uart.py`: low-level TI mmWave UART parser shared by scripts and ROS nodes.

`radar_nav/` is required for radar navigation in any mode. `radar_ros/` is only needed when running the same radar logic as ROS2 nodes/topics. For now, thruster control is direct serial from scripts; a ROS2 thruster node can be added later.

## Quick Pi Test

Install Python serial support:

```bash
python3 -m pip install pyserial
```

Find serial ports:

```bash
ls /dev/ttyUSB* /dev/ttyACM*
```

Typical ports:

- ESP32: `/dev/ttyACM0`
- radar config UART: `/dev/ttyUSB0`
- radar data UART: `/dev/ttyUSB1`

Create a local robot config:

```bash
cp config/boat.example.json config/boat.local.json
```

Edit `config/boat.local.json` if the ports differ. The local file is ignored by Git, so each machine can keep its own serial port settings.

First test the ESP32/thruster path without radar:

```bash
python3 scripts/run_thruster_ramp.py
```

Then test radar-to-ESP32 in dry-run mode:

```bash
python3 scripts/run_nav_esp32.py --dry-run
```

Run the real bridge with a gentle cap:

```bash
python3 scripts/run_nav_esp32.py --log
```

PWM defaults:

- `1500 us`: neutral / stop
- `1520 us`: minimum forward output
- `1600 us`: default gentle forward cap
- `1350-2000 us`: hard safety clamp accepted by the Python bridge

Use physical power cutoff during thruster tests. `Ctrl+C` sends `STOP`, but hardware power control is the real safety path.

## Live Visualization

Show the pygame view while running the direct radar-to-ESP32 bridge:

```bash
python3 scripts/run_nav_esp32.py --viz --log
```

For visualization only, use dry-run:

```bash
python3 scripts/run_nav_esp32.py --dry-run --viz
```

Control logs are written as JSONL under `logs/` by default and can be replayed with `scripts/run_nav_replay.py`.

Any value from `config/boat.local.json` can still be overridden from the command line, for example:

```bash
python3 scripts/run_nav_esp32.py --forward-max-us 1525 --log
```

## ROS2 Mode

From a ROS2 workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/armutie/mtr-boat-core.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select radar_ros
source install/setup.bash
```

Run the radar UART publisher:

```bash
ros2 run radar_ros radar_uart_node --ros-args \
  -p cfg_port:=/dev/ttyUSB0 \
  -p cfg_file:=config/radar/profile_2d.cfg \
  -p data_port:=/dev/ttyUSB1 \
  -p frame_id:=radar
```

Run the navigation node:

```bash
ros2 run radar_ros radar_nav_node
```

Published topics:

- `radar/raw_points` (`sensor_msgs/PointCloud2`)
- `radar/filtered_points` (`sensor_msgs/PointCloud2`)
- `radar/clusters_json` (`std_msgs/String`)
- `radar/nav_state_json` (`std_msgs/String`)

Check output:

```bash
ros2 topic echo /radar/nav_state_json
ros2 topic hz /radar/raw_points
```

View ROS nav output with pygame:

```bash
python3 scripts/run_nav_live.py --ros --nav-state-topic /radar/nav_state_json
```

## Dashboard

Run the browser dashboard from the robot or development laptop:

```bash
python3 web_dashboard/server.py --ros --mmwave-topic radar/nav_state_json
```

Open:

```text
http://localhost:8080
http://<orange-pi-or-laptop-ip>:8080
```

For UI-only testing without sensors:

```bash
python3 web_dashboard/server.py --demo
```

The dashboard intentionally marks missing GNSS, sonar, and ultrasonic feeds unavailable until those feeds exist.

## Simulation

Run the boat/radar navigation simulation:

```bash
python3 scripts/run_nav_sim.py
```

The sim is for controller development and visualization. It is not required for the basic Pi thruster test.
