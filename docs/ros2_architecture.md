# ROS 2 foundation

The boat uses ROS 2 as a small integration layer around hardware drivers and
testable Python libraries. New nodes and topics should be added only when they
create a real hardware, safety, or computation boundary.

## Initial sensor graph

```text
GNSS driver --------> /gnss/fix          sensor_msgs/NavSatFix
BNO055 driver ------> /imu/data          sensor_msgs/Imu
                   -> /imu/data_raw      sensor_msgs/Imu
                   -> /imu/mag           sensor_msgs/MagneticField
                   -> /imu/temperature   sensor_msgs/Temperature
                   -> /imu/gravity       geometry_msgs/Vector3Stamped
                   -> /diagnostics       diagnostic_msgs/DiagnosticArray
camera driver ------> /camera/image_raw  sensor_msgs/Image
                   -> /camera/camera_info
LiDAR driver -------> /lidar/points      sensor_msgs/PointCloud2
```

GNSS, MPU-6050, and BNO055 nodes are implemented in `boat_ros`. The BNO055 is
the default IMU in `sensors.launch.py`; `imu_driver:=mpu6050` preserves the
original 6-axis bench path.

The BNO055 node owns I2C bus 2/address `0x29`, selects NDOF fusion, and configures
Android/ENU orientation output. Its `imu_link` axes and the static
`base_link -> imu_link` transform must match the physical installation before
orientation is fused into navigation. Calibration status is part of the
runtime health contract rather than being inferred from plausible-looking
orientation values.

## Design rules

- Prefer standard ROS messages over JSON or project-specific messages.
- Keep hardware access in one owning node per device.
- Keep algorithms in ordinary Python modules when they do not need ROS.
- Use topics for continuous data, services for short state changes, and actions
  for missions that take time.
- Use the sensor-data QoS profile for high-rate sensor streams.
- Timestamp data at acquisition and give every physical sensor a frame ID.
- Do not send actuator output directly from the dashboard or perception nodes.
- Preserve the ESP32 timeout-to-neutral behavior and physical power cutoff.

## Expected control boundary

```text
operator/autonomy request
          |
          v
   command + safety node
          |
          v
     thruster driver
          |
          v
         ESP32
```

Only the thruster driver may open the ESP32 serial port. The command and safety
node will eventually reject stale, disarmed, or unhealthy requests before they
reach that driver.

## Orange Pi constraints

- Use a supported 64-bit Ubuntu and matching ROS 2 LTS distribution.
- Install the ROS base variant; run visualization on a laptop.
- Avoid forwarding full-rate image and point-cloud streams over Wi-Fi by
  default.
- Measure CPU load, memory, temperature, and storage use with all sensors
  running before choosing camera resolution or perception rates.
