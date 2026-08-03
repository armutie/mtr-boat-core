# Legacy Python tools

ROS 2 is the canonical robot runtime. These direct-Python tools remain useful
for isolated hardware tests, replay, and controller development.

## Setup

```bash
python3 -m pip install pyserial pynmea2 smbus2
```

The checked-in configuration uses the tested hardware aliases. Copy
`config/boat.example.json` to `config/boat.local.json` only when a bench device
or setting differs; the local file is ignored by Git.

Stable device names after running `sudo ./scripts/install_udev_rules.sh`:

- ESP32: `/dev/mtr_esp32`
- radar configuration: `/dev/ttyUSB0`
- radar data: `/dev/ttyUSB1`
- GNSS: `/dev/mtr_gnss`

## Bench tools

| Purpose | Command |
| --- | --- |
| GNSS | `python3 scripts/run_gnss_live.py --log` |
| MPU-6050 | `python3 scripts/run_imu_live.py` |
| BNO055 | `python3 scripts/test_bno055_live.py` |
| Thruster ramp | `python3 scripts/run_thruster_ramp.py --channel left` |
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

The thruster ramp accepts exact below-neutral values with `--down`, and
`--channel left|right|both` keeps the other channel neutral. Stop
`mtr-boat.service` before running it so the service and test do not compete
for the ESP32 serial port.

```bash
sudo systemctl stop mtr-boat
python3 scripts/run_thruster_ramp.py \
  --config config/boat.example.json \
  --down --channel right \
  --start-us 1495 --end-us 1400 \
  --step-us 5 --hold-s 1.5
sudo systemctl start mtr-boat
```

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
