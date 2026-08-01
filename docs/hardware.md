# Stable hardware device names

Linux assigns names such as `/dev/ttyACM0` and `/dev/ttyUSB0` according to
connection order. MTR uses udev aliases so reconnecting a device or changing a
USB port does not change the ROS configuration.

| Hardware | Stable name | Identification |
| --- | --- | --- |
| Arducam UC684 | `/dev/mtr_camera` | USB `0c45:0261`, serial `UC684`, capture interface |
| ESP32 thruster controller | `/dev/mtr_esp32` | CP2102 `10c4:ea60`, serial `0001` |
| u-blox GNSS receiver | `/dev/mtr_gnss` | USB `1546:01a9` |

The BNO055 is addressed as I2C bus 2, address `0x29`. The Seyond LiDAR is
addressed over Ethernet. Neither requires a udev alias.

## Install

Install or refresh every repository-provided hardware rule with:

```bash
cd /path/to/mtr-boat-core
sudo ./scripts/install_udev_rules.sh
```

The installer reloads udev, triggers connected devices, and reports which
aliases are available. It is safe to rerun after pulling updated rules.

Verify them at any time:

```bash
ls -l /dev/mtr_camera /dev/mtr_esp32 /dev/mtr_gnss
```

An alias appears only while its matching device is connected. If a replacement
device has different USB identifiers, update the corresponding rule in
`config/udev/` and reinstall.

The u-blox receiver used here does not publish a unique USB serial number. The
GNSS rule therefore assumes only one `1546:01a9` receiver is connected.
