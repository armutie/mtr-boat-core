# Legacy Python tools

ROS 2 is the canonical robot runtime. These direct-Python tools remain useful
for isolated hardware tests, replay, and controller development.

## Setup

```bash
python3 -m pip install pyserial pynmea2 smbus2
cp config/boat.example.json config/boat.local.json
```

Edit `config/boat.local.json` with the real serial ports and I2C addresses. The
local file is ignored by Git.

Common device names:

- ESP32: `/dev/ttyACM0`
- radar configuration: `/dev/ttyUSB0`
- radar data: `/dev/ttyUSB1`
- GNSS: another `/dev/ttyACM*`

## Bench tools

| Purpose | Command |
| --- | --- |
| GNSS | `python3 scripts/run_gnss_live.py --log` |
| MPU-6050 | `python3 scripts/run_imu_live.py` |
| BNO055 | `python3 scripts/test_bno055_live.py` |
| Thruster ramp | `python3 scripts/run_thruster_ramp.py` |
| Radar navigation dry run | `python3 scripts/run_nav_esp32.py --dry-run` |
| Radar simulation | `python3 scripts/run_nav_sim.py` |
| Dashboard demo | `python3 web_dashboard/server.py --demo` |

Replay tools:

```bash
python3 scripts/replay_gnss_log.py logs/gnss_TIMESTAMP.jsonl
python3 scripts/replay_imu_log.py logs/imu_TIMESTAMP.jsonl
python3 scripts/run_nav_replay.py --log LOG_FILE
```

## Dashboard

The normal dashboard is started by `boat.launch.py` and uses ROS sensors and
control. To run it separately:

```bash
web_dashboard
```

Open `http://<orange-pi-ip>:8080` from a device on the same trusted network.
Use `--demo` without hardware, or `--ros --mmwave-topic
radar/nav_state_json` for the ROS radar feed.

Direct hardware access is explicit: `--direct-gnss`, `--direct-mmwave`,
`--imu`, or `--direct-control`. Use `--actuator-dry-run --direct-control` for
a direct-Python test with no motor output.

## Legacy ESP32 test

The single-thruster firmware is `esp32_thruster/esp32_thruster.ino`. It is only
for isolated testing. Normal ROS operation uses
`esp32_thruster_dual/esp32_thruster_dual.ino`.

```text
PWM 1650
STOP
```

It returns to neutral when commands stop for one second.

## Safety

Use a physical motor-power cutoff during every thruster test. `Ctrl+C` and the
ESP32 timeout are safeguards, not substitutes for removing actuator power.
