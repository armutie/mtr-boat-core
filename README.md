# mtr-boat-core

Orange Pi robot software for the MTR boat experiments.

This repo is the shared home for the boat's sensor drivers, navigation logic,
ROS 2 nodes, dashboard, and actuator bridges. ROS 2 is the intended robot
runtime, while the direct Python bench/field paths remain available during the
incremental migration.

## Current Capabilities

- Read TI xWR18xx mmWave radar frames over UART.
- Read basic GNSS receivers that output NMEA over serial.
- Read MPU-6050 IMU accel/gyro data over I2C.
- Read BNO055 9-axis data and fused orientation over I2C.
- Filter and cluster radar points into obstacle evidence.
- Produce simple navigation output: throttle, steering, command, and reason.
- Send gentle ESC PWM commands to an ESP32 over serial for basic thruster tests.
- Run the current hardware test directly from Python scripts.
- Publish GNSS, IMU, and radar data through ROS 2 nodes.
- Publish a UVC camera on `/camera/image_raw` and serve a browser livestream.
- Publish the Seyond D1-R as `/lidar/points` (`PointCloud2`) in `lidar_link`.
- Visualize radar/navigation state with pygame or the browser dashboard.

## Layout

- `radar_nav/`: core non-ROS radar/navigation logic. Scripts and ROS nodes both use this.
- `gnss/`: `pynmea2`-based NMEA parsing for USB/serial GNSS receivers.
- `imu/`: testable MPU-6050 and BNO055 I2C drivers.
- `boat_ros/`: thin ROS 2 wrappers for boat sensors and radar navigation.
- `thruster_control/`: ESP32 serial and ESC PWM mapping helpers.
- `scripts/`: runnable robot/development commands.
- `config/radar/`: radar profile files for startup.
- `web_dashboard/`: browser dashboard for demo/ROS sensor state.
- `mmwave_uart.py`: low-level TI mmWave UART parser shared by scripts and ROS nodes.

`radar_nav/` remains usable without ROS 2. The wrappers in `boat_ros/` keep
hardware and navigation logic testable outside the ROS graph while the robot
runtime is migrated incrementally.

## Camera Livestream

The camera node uses the Arducam's native MJPEG capture mode at 1280x720 and
30 FPS by default. It publishes decoded `sensor_msgs/Image` frames using
sensor-data QoS and serves the latest frames directly to browsers. Depth
estimation is deliberately outside this low-latency path.

Build and launch a camera-only test on the Orange Pi:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch mtr_boat_core sensors.launch.py \
  enable_gnss:=false enable_imu:=false
```

Install the included udev rule once so reconnects and USB enumeration changes
do not move the camera between `/dev/video*` names:

```bash
sudo cp config/udev/99-mtr-camera.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=video4linux
ls -l /dev/mtr_camera
```

The rule identifies the tested Arducam UC684 by vendor, product, serial, and
capture-interface index. A replacement camera with different USB identifiers
needs a corresponding rule update. A stable `/dev/v4l/by-id/` path may also be
set in the ROS YAML when the device provides one.

Open the fullscreen viewer from any device on the same network:

```text
http://<orange-pi-ip>:8081/
```

Useful endpoints are `/stream.mjpg`, `/snapshot.jpg`, and `/health`. The node
keeps running if the USB camera is unplugged and reconnects automatically when
`/dev/mtr_camera` returns. Cached JPEG data and its timestamp are cleared on
disconnect, so the snapshot and stream remain unavailable until a fresh frame
arrives. If a temporary bench setup is sideways, set
`web_rotation_deg` in the ROS YAML to `90`, `180`, or `270`; this rotates only
the browser presentation and leaves the ROS image geometry unchanged.

The web server is optional. Set `enable_web: false` in the ROS YAML to publish
ROS images without opening an HTTP port. A bind failure also disables only the
web viewer; camera capture and `/camera/image_raw` continue.

> The default `0.0.0.0:8081` viewer has no authentication and sends permissive
> CORS headers. Use it only on a trusted boat LAN. Bind to `127.0.0.1`, add a
> firewall/reverse proxy, or disable the web server on an untrusted network.

The supported launch path sets `PYTHONNOUSERSITE=1`, keeping Ubuntu's
`python3-opencv` on its matching system NumPy ABI without modifying
process-global `sys.path`.

Measure the physical camera pose before field use. The mount transform is
disabled by default; publish it only with `publish_camera_tf:=true` plus the
measured `camera_x`, `camera_y`, `camera_z`, `camera_roll`, `camera_pitch`, and
`camera_yaw`. Distances are metres and angles are radians. Camera calibration
is not yet available, so `/camera/camera_info` is intentionally not published
until real intrinsics and distortion coefficients are measured.

## LiDAR Integration

The LiDAR node is provided by the bundled `ros2/seyond_mapping` package.
`dependencies.repos` pins the external Seyond SDK and KISS-ICP inputs to exact
commits. The bootstrap script imports them into one configurable ROS workspace,
builds the original SDK without modifying its demo, then explicitly discovers
and builds both ROS packages in this repository.

Create a reproducible workspace on the Orange Pi:

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-vcstool

MTR_WS=/path/to/mtr_ws
mkdir -p "$MTR_WS/src"
git clone https://github.com/armutie/mtr-boat-core.git \
  "$MTR_WS/src/mtr-boat-core"
"$MTR_WS/src/mtr-boat-core/scripts/bootstrap_ros2_workspace.sh" "$MTR_WS"
source "$MTR_WS/install/setup.bash"
ros2 launch mtr_boat_core sensors.launch.py
```

LiDAR startup is opt-in so a disconnected sensor does not affect the default
boat launch. For a LiDAR-only hardware check:

```bash
ros2 launch mtr_boat_core sensors.launch.py \
  enable_gnss:=false enable_imu:=false enable_lidar:=true
```

The `base_link -> lidar_link` transform is separately opt-in. Measure the real
LiDAR mounting pose before enabling it. For example, a sensor 25 cm forward
and 40 cm above `base_link` would use `publish_lidar_tf:=true lidar_x:=0.25
lidar_z:=0.40`; those numbers are illustrative, not boat measurements.

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

## Field Startup Over Hotspot/WiFi

Use this when you want the laptop/phone dashboard to talk to the Orange Pi over
the same hotspot or WiFi network.

1. Turn on the hotspot or WiFi network.
2. Connect the laptop to that network.
3. Connect the Orange Pi to that network. If needed, plug in a monitor and
   keyboard, log in, and connect WiFi from the desktop/network menu.
4. On the Orange Pi monitor, find its WiFi IP:

```bash
ip addr
```

Look for the `wlan0` address. It will look like `192.168.x.x`. After you have
the IP, the monitor/keyboard are no longer needed.

5. From the laptop, SSH into the Orange Pi:

```bash
ssh uwmtr@ORANGE_PI_WIFI_IP
```

Example:

```bash
ssh uwmtr@192.168.50.23
```

6. Start the dashboard inside `tmux` so it keeps running if SSH or Ethernet
   disconnects:

```bash
cd ~/mtr-boat-core
tmux new -s boat
python3 web_dashboard/server.py --no-mmwave --log
```

For dry-run testing without sending ESP32 motor commands:

```bash
python3 web_dashboard/server.py --no-mmwave --actuator-dry-run --log
```

For live thruster testing:

```bash
python3 web_dashboard/server.py --no-mmwave --actuator-live --log
```

7. Open the dashboard from the laptop or phone on the same network:

```text
http://ORANGE_PI_WIFI_IP:8080
```

8. Detach from `tmux` without stopping the server:

```text
Ctrl+B
D
```

9. Reconnect to the running server output later:

```bash
ssh uwmtr@ORANGE_PI_WIFI_IP
tmux attach -t boat
```

10. Stop the dashboard while attached to `tmux`:

```text
Ctrl+C
```

Useful `tmux` commands:

```bash
tmux ls
tmux new -s boat
tmux attach -t boat
tmux kill-session -t boat
```

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

## ROS 2 Mode

For the complete sensor workspace, use the pinned bootstrap described near the
top of this README. For development without the external LiDAR package, the
boat package alone can still be built from a ROS 2 workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/armutie/mtr-boat-core.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select mtr_boat_core
source install/setup.bash
```

Copy and edit the Orange Pi sensor parameters:

```bash
cp src/mtr-boat-core/config/ros/boat.example.yaml \
  src/mtr-boat-core/config/ros/boat.local.yaml
```

Start the GNSS and default MPU-6050 publishers together:

```bash
ros2 launch mtr_boat_core sensors.launch.py \
  params_file:="$(pwd)/src/mtr-boat-core/config/ros/boat.local.yaml"
```

For a BNO055 bench test, select it explicitly. The ROS parameters match the
current wiring assumption: I2C bus `2`, address `0x29`.

```bash
ros2 launch mtr_boat_core sensors.launch.py \
  params_file:="$(pwd)/src/mtr-boat-core/config/ros/boat.local.yaml" \
  imu_driver:=bno055
```

The node verifies the chip ID, resets stale volatile state, selects NDOF
fusion, and publishes raw acceleration, angular velocity, magnetic field,
temperature, gravity, gravity-removed acceleration, and calibration status.
The reset makes startup deterministic but requires calibration after each node
start. Set `reset_on_start: false` only when intentionally preserving the
sensor's current volatile calibration.

The BNO055 device quaternion uses Bosch's Android-format convention and is not
assumed to be a REP-103 ENU orientation for the installed board. `/imu/data`
therefore remains disabled until `publish_fused_orientation: true` is set in
the local parameter file after a physical axis and heading validation.

Either sensor group can be disabled for bench work:

```bash
ros2 launch mtr_boat_core sensors.launch.py enable_imu:=false
```

After measuring the IMU pose relative to `base_link` in metres and radians,
publish its static transform explicitly:

```bash
ros2 launch mtr_boat_core sensors.launch.py \
  publish_imu_tf:=true \
  imu_x:=0.0 imu_y:=0.0 imu_z:=0.0 \
  imu_roll:=0.0 imu_pitch:=0.0 imu_yaw:=0.0
```

Run the radar UART publisher:

```bash
ros2 run mtr_boat_core radar_uart_node --ros-args \
  -p cfg_port:=/dev/ttyUSB0 \
  -p cfg_file:=config/radar/profile_2d.cfg \
  -p data_port:=/dev/ttyUSB1 \
  -p frame_id:=radar
```

Run the navigation node:

```bash
ros2 run mtr_boat_core radar_nav_node
```

Published topics:

- `gnss/fix` (`sensor_msgs/NavSatFix`)
- `imu/data` (`sensor_msgs/Imu`, optional BNO055 device-fused orientation)
- `imu/data_raw` (`sensor_msgs/Imu`)
- `imu/mag` (`sensor_msgs/MagneticField`)
- `imu/temperature` (`sensor_msgs/Temperature`)
- `imu/linear_acceleration` (`geometry_msgs/Vector3Stamped`, gravity removed)
- `imu/gravity` (`geometry_msgs/Vector3Stamped`)
- `/diagnostics` (`diagnostic_msgs/DiagnosticArray`, BNO055 calibration and system status)
- `/camera/image_raw` (`sensor_msgs/Image`)
- `/lidar/points` (`sensor_msgs/PointCloud2`)
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

GNSS, IMU, and radar point clouds use the ROS sensor-data QoS profile. Check
the sensor publishers:

```bash
ros2 topic hz /gnss/fix
ros2 topic hz /imu/data
ros2 topic hz /imu/data_raw
ros2 topic echo /imu/mag --once
ros2 topic echo /diagnostics
```

The BNO055 calibration values range from `0` (uncalibrated) to `3` (fully
calibrated). Keep the board still briefly for gyro calibration, place it in six
stable orientations for the accelerometer, and move it through figure-eight
paths for the magnetometer. Do not use absolute heading for navigation until
the diagnostic reports all four calibration values at `3`. The default
covariances are deliberately unknown (`0.0`); measure them on the installed
boat before feeding the data into a state estimator.

For a human-readable bench view without ROS, run the live BNO055 test directly
on the Orange Pi:

```bash
python3 scripts/test_bno055_live.py
```

It prints the device quaternion, mounting-dependent roll/pitch/yaw,
acceleration including gravity, gravity-removed linear acceleration, gyro
rate, magnetic field in microtesla, temperature, and all four calibration
levels. Use `--duration-s 30` for a finite run or `--no-reset` to preserve the
current volatile calibration. Treat the displayed orientation as diagnostic
until its axes and heading have been checked against the installed board.

## ROS 2 Roadmap

The current hardware test path is direct Python:

```text
radar UART -> radar_nav -> thruster_control -> ESP32 serial
```

The intended robot runtime is ROS 2. The current package starts with thin
sensor publishers and preserves the proven direct-Python hardware path during
the migration:

```text
boat_ros/gnss_node.py
  -> publishes gnss/fix

boat_ros/imu_node.py
  -> publishes imu/data_raw

boat_ros/bno055_node.py
  -> publishes imu/data_raw, imu/mag, imu/temperature
  -> publishes imu/linear_acceleration, imu/gravity
  -> optionally publishes device-fused orientation on imu/data
  -> publishes BNO055 calibration and health on /diagnostics

boat_ros/radar_uart_node.py
  -> publishes radar/raw_points

boat_ros/radar_nav_node.py
  -> subscribes radar/raw_points
  -> publishes radar/filtered_points
  -> publishes radar/clusters_json
  -> publishes radar/nav_state_json
```

Missing before ROS 2 becomes the main autonomous runtime:

- Define a sensor-independent perception output instead of coupling control to radar.
- Add a safety/command node and a thruster node with exclusive ESP32 ownership.
- Replace radar JSON control messages with typed, sensor-independent messages.
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
