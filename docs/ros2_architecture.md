# ROS 2 foundation

The boat uses ROS 2 as a small integration layer around hardware drivers and
testable Python libraries. New nodes and topics should be added only when they
create a real hardware, safety, or computation boundary.

## Initial sensor graph

```text
GNSS driver --------> /gnss/fix          sensor_msgs/NavSatFix
IMU driver ---------> /imu/data_raw      sensor_msgs/Imu
BNO055 auxiliary ---> /imu/mag           sensor_msgs/MagneticField
                    -> /imu/temperature  sensor_msgs/Temperature
                    -> /imu/linear_acceleration
                    -> /imu/gravity      geometry_msgs/Vector3Stamped
                    -> /diagnostics      diagnostic_msgs/DiagnosticArray
BNO055 optional ----> /imu/data          sensor_msgs/Imu
camera driver ------> /camera/image_raw  sensor_msgs/Image
LiDAR driver --------> /lidar/points      sensor_msgs/PointCloud2
```

GNSS, MPU-6050, BNO055, and camera nodes are implemented in `boat_ros`. The
LiDAR driver is implemented in the bundled `seyond_mapping` package. The
BNO055 is the default IMU; select the MPU-6050 with
`imu_driver:=mpu6050`.

The BNO055 node owns I2C bus 2/address `0x29`, selects NDOF fusion, and configures
Bosch's Android-format orientation output. Raw acceleration, angular velocity,
and magnetic field data form the default ROS contract. Publishing the
device-fused quaternion on `/imu/data` is disabled by default because its
world-frame and mounting convention must be validated on the installed board
before navigation uses it. Calibration status is part of the runtime health
contract rather than being inferred from plausible-looking orientation values.

The configurable `base_link -> imu_link` transform is also disabled by default.
Measure the IMU pose and validate its axes before setting
`publish_imu_tf:=true`.

## Camera contract

The Arducam UVC integration owns the stable `/dev/mtr_camera` udev alias in
`camera_node` and exposes:

```text
topic:       /camera/image_raw
type:        sensor_msgs/msg/Image
frame:       camera_optical_frame
QoS:         sensor-data (best effort, volatile)
encoding:    bgr8
timestamp:   Orange Pi acquisition time
web viewer:  http://<orange-pi-ip>:8081/
```

The default capture profile is hardware MJPEG at 1280x720 and 30 FPS.
GStreamer sends those native JPEG frames directly to browser clients without a
second encode. A separate latest-frame worker decodes the standard ROS image,
so slow perception or DDS consumers cannot queue stale operator video. Depth
estimation remains a separate consumer so it cannot add latency to the
operator stream.

The browser server can be disabled independently with `enable_web: false`, and
a bind failure does not stop ROS image publication. Its default `0.0.0.0:8081`
endpoint is unauthenticated and uses permissive CORS, so it is for a trusted
boat LAN only.

The image frame follows the ROS optical convention: X right, Y down, Z forward.
`sensors.launch.py` always publishes the fixed
`camera_link -> camera_optical_frame` axis transform while the camera is
enabled. The configurable `base_link -> camera_link` mounting transform is
disabled by default and requires `publish_camera_tf:=true`. Set `camera_x`,
`camera_y`, `camera_z`, `camera_roll`, `camera_pitch`, and `camera_yaw` only
after measuring the installed pose.

No placeholder calibration is published. Add `/camera/camera_info` only after
calibrating the real lens at the selected resolution.

The Seyond D1-R integration uses the `seyond_pointcloud_node` executable from
the `seyond_mapping` package. Its public contract is:

```text
topic:       /lidar/points
type:        sensor_msgs/msg/PointCloud2
frame:       lidar_link
QoS:         sensor-data (best effort, volatile)
fields:      x, y, z, intensity, time
coordinates: REP-103 X=forward, Y=left, Z=up
timestamp:   Orange Pi acquisition time
```

The SDK's native X=up, Y=right, Z=forward coordinates are converted inside the
driver. Per-point `time` remains relative within a frame for scan deskewing.
Until the D1-R is synchronized to the boat clock, the PointCloud2 header uses
the Orange Pi acquisition time so it can be combined with IMU, GNSS, radar,
and camera messages.

LiDAR startup is disabled by default and enabled with `enable_lidar:=true`.
The fixed `base_link -> lidar_link` transform is controlled separately by
`publish_lidar_tf`, which is also false by default. Measure the LiDAR origin
relative to the boat reference point before enabling the transform, then set
`lidar_x`, `lidar_y`, `lidar_z`, `lidar_roll`, `lidar_pitch`, and `lidar_yaw`
at launch. Distances are metres and angles are radians.

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
