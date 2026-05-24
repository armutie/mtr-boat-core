# mtr-boat-core

Orange Pi robot software for the MTR boat experiments.

This repo is meant to become the shared home for the boat's sensor drivers, navigation logic, ROS2 nodes, dashboard, and actuator bridges. Right now the working path is direct Python bench/field testing: radar UART in, ESP32 thruster serial out. ROS2 support exists for the radar path, but full robot control should move there after the hardware behavior is proven.

## Current Capabilities

- Read TI xWR18xx mmWave radar frames over UART.
- Read basic GNSS receivers that output NMEA over serial.
- Read MPU-6050 IMU accel/gyro data over I2C.
- Filter and cluster radar points into obstacle evidence.
- Produce simple navigation output: throttle, steering, command, and reason.
- Send gentle ESC PWM commands to an ESP32 over serial for basic thruster tests.
- Run the current hardware test directly from Python scripts.
- Build the radar reader/navigation path as ROS2 nodes for later robot integration.
- Visualize radar/navigation state with pygame or the browser dashboard.

## Layout

- `radar_nav/`: core non-ROS radar/navigation logic. Scripts and ROS nodes both use this.
- `gnss/`: `pynmea2`-based NMEA parsing for USB/serial GNSS receivers.
- `imu/`: MPU-6050 I2C reader for accel/gyro samples.
- `radar_ros/`: ROS2 wrapper nodes around the radar parser and `radar_nav` pipeline.
- `thruster_control/`: ESP32 serial and ESC PWM mapping helpers.
- `scripts/`: runnable robot/development commands.
- `config/radar/`: radar profile files for startup.
- `web_dashboard/`: browser dashboard for demo/ROS sensor state.
- `mmwave_uart.py`: low-level TI mmWave UART parser shared by scripts and ROS nodes.

`radar_nav/` is required for radar navigation in any mode. `radar_ros/` is only needed when running the same radar logic as ROS2 nodes/topics. For now, thruster control is direct serial from scripts; a ROS2 thruster node can be added later.

## ESP32 Firmware

Flash the ESP32 before running thruster tests:

```text
esp32_thruster/esp32_thruster.ino
```

Arduino IDE setup:

- Board: `ESP32 Dev Module`
- Library: `ESP32Servo`
- ESC signal pin: GPIO `33`
- Serial baud: `115200`

The firmware listens for USB serial lines:

```text
PWM 1650
STOP
```

It returns to neutral if no command arrives for one second.

## Quick Pi Test

Install Python sensor support:

```bash
python3 -m pip install pyserial pynmea2 smbus2
```

Find serial ports:

```bash
ls /dev/ttyUSB* /dev/ttyACM*
```

Typical ports:

- ESP32: `/dev/ttyACM0`
- radar config UART: `/dev/ttyUSB0`
- radar data UART: `/dev/ttyUSB1`
- GNSS: often `/dev/ttyACM2` or another `/dev/ttyACM*`

Create a local robot config:

```bash
cp config/boat.example.json config/boat.local.json
nano config/boat.local.json
```

Edit `config/boat.local.json` before field tests. At minimum, set the serial
ports for the devices actually connected to the Orange Pi:

```json
{
  "radar": {
    "cfg_port": "/dev/ttyUSB0",
    "data_port": "/dev/ttyUSB1"
  },
  "esp32": {
    "port": "/dev/ttyACM0"
  },
  "gnss": {
    "port": "/dev/ttyACM2"
  },
  "imu": {
    "bus": 2,
    "address": "0x68"
  }
}
```

Keep the other values from `boat.example.json` unless you are deliberately
tuning them. `config/boat.local.json` is ignored by Git, so each laptop/Pi can
keep its own serial port settings.

The following commands read `config/boat.local.json` by default:

- `python3 web_dashboard/server.py`
- `python3 scripts/run_gnss_live.py`
- `python3 scripts/run_imu_live.py`
- `python3 scripts/run_thruster_ramp.py`
- `python3 scripts/run_nav_esp32.py`
- `python3 scripts/test_gnss_imu_heading.py`
- `python3 scripts/test_boat_response.py`
- `python3 scripts/test_imu_yaw_drift.py`

If `config/boat.local.json` does not exist, these commands fall back to
`config/boat.example.json`, but that is only a template. For the real boat, copy
the example and set the actual ports.

First test the ESP32/thruster path without radar:

```bash
python3 scripts/run_thruster_ramp.py
```

Then test radar-to-ESP32 in dry-run mode:

```bash
python3 scripts/run_nav_esp32.py --dry-run
```

If a GNSS receiver is connected, test it separately:

```bash
python3 scripts/run_gnss_live.py --log
```

The GNSS script uses `pynmea2` to read NMEA sentences such as GGA/RMC from the serial port in `config/boat.local.json`, prints fix/lat/lon/speed/heading, and writes JSONL logs under `logs/` when `--log` is used.

Replay a saved GNSS log:

```bash
python3 scripts/replay_gnss_log.py logs/gnss_YYYYMMDD_HHMMSS.jsonl
python3 scripts/replay_gnss_log.py logs/gnss_YYYYMMDD_HHMMSS.jsonl --html
python3 scripts/replay_gnss_log.py logs/gnss_YYYYMMDD_HHMMSS.jsonl --map
python3 scripts/replay_gnss_log.py logs/gnss_YYYYMMDD_HHMMSS.jsonl --map --speed
```

If an MPU-6050 is connected over I2C, test it separately:

```bash
python3 scripts/run_imu_live.py
```

Use the visual trace view when a display is available:

```bash
python3 scripts/run_imu_live.py --viz
```

The IMU script reads accel/gyro samples, zero-calibrates gyro drift at startup while the board is still, prints live values with relative yaw from startup, and writes JSONL logs under `logs/` when `--log` is used.

Replay a saved IMU log on a laptop or Pi:

```bash
python3 scripts/replay_imu_log.py logs/imu_YYYYMMDD_HHMMSS.jsonl
python3 scripts/replay_imu_log.py logs/imu_YYYYMMDD_HHMMSS.jsonl --no-viz
```

Run the real bridge with a gentle cap:

```bash
python3 scripts/run_nav_esp32.py --log
```

PWM defaults:

- `1500 us`: neutral / stop
- `1565 us`: minimum forward output observed to start the test motor
- `1650 us`: default forward cap in `config/boat.example.json`
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

## ROS2 Roadmap

The current hardware test path is direct Python:

```text
radar UART -> radar_nav -> thruster_control -> ESP32 serial
```

The intended robot runtime is ROS2 once the radar/ESP32 behavior is proven. The current ROS implementation covers the radar side:

```text
radar_ros/radar_uart_node.py
  -> publishes radar/raw_points

radar_ros/radar_nav_node.py
  -> subscribes radar/raw_points
  -> publishes radar/filtered_points
  -> publishes radar/clusters_json
  -> publishes radar/nav_state_json
```

Missing before ROS2 becomes the main autonomous runtime:

- Add a thruster ROS node that subscribes to nav output and sends ESP32 serial commands.
- Decide whether the thruster node consumes `radar/nav_state_json` directly or a cleaner future command topic.
- Add launch/config files so the robot starts with one command instead of several terminals.
- Move Pi-specific settings from `config/boat.local.json` into ROS launch/YAML when ROS becomes the main runtime.
- Validate `colcon build` and `ros2 run` on the Orange Pi or another sourced ROS2 environment.

## Dashboard

Deploy the latest code from a Windows laptop to the Orange Pi:

```powershell
.\scripts\deploy_pi.ps1 -HostName <orange-pi-ip-or-hostname>
```

Run the browser dashboard from the robot:

```bash
python3 web_dashboard/server.py
```

By default the dashboard binds to `0.0.0.0`, auto-starts the IMU reader, starts
direct mmWave/GNSS readers when their local serial device files are present, and
uses the configured ESP32 port for live motor output when it can open it.
Use `--actuator-dry-run` when you want the dashboard to run without writing
motor commands. Use `--demo` for UI-only testing, `--ros --mmwave-topic
radar/nav_state_json` for a ROS mmWave feed, or `--no-mmwave`, `--no-gnss`,
and `--no-imu` to disable a local reader while debugging.

The dashboard always writes a unified session log unless disabled:

```text
logs/dashboard_session_YYYYMMDD_HHMMSS.jsonl
```

Each session row includes the current mode, manual/auto command, effective
left/right PWM, GNSS, IMU, and radar snapshot. Add `--log` only when you also
want raw per-sensor logs such as `dashboard_gnss_*.jsonl`,
`dashboard_imu_*.jsonl`, and `dashboard_mmwave_*.jsonl`.

Open:

```text
http://localhost:8080
http://<orange-pi-or-laptop-ip>:8080
```

For UI-only testing without sensors:

```bash
python3 web_dashboard/server.py --demo
```

When direct serial sensor mode is enabled, the configured port is tried first. If
it cannot be opened, the dashboard falls back through the common Linux device
names `/dev/ttyACM0`-`/dev/ttyACM2` and `/dev/ttyUSB0`-`/dev/ttyUSB2` as
appropriate, then reports the selected port in the live metadata.

The dashboard intentionally marks missing GNSS, sonar, and ultrasonic feeds unavailable until those feeds exist.

## Simulation

Run the boat/radar navigation simulation:

```bash
python3 scripts/run_nav_sim.py
```

The sim is for controller development and visualization. It is not required for the basic Pi thruster test.
