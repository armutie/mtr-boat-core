# ROS 2 foundation

The boat uses ROS 2 as a small integration layer around hardware drivers and
testable Python libraries. New nodes and topics should be added only when they
create a real hardware, safety, or computation boundary.

## Initial sensor graph

```text
GNSS driver ------> /gnss/fix          sensor_msgs/NavSatFix
IMU driver -------> /imu/data_raw      sensor_msgs/Imu
camera driver ----> /camera/image_raw  sensor_msgs/Image
                 -> /camera/camera_info
LiDAR driver -----> /lidar/points      sensor_msgs/PointCloud2
```

GNSS and IMU are implemented in `boat_ros`. The camera and LiDAR entries define
the interfaces that their existing drivers should meet when added.

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
