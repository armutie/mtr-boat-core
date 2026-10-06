#pragma once

#include <cmath>
#include <cstring>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "seyond_mapping/point_fields.hpp"
#include "src/sdk_common/inno_lidar_packet_utils.h"

namespace seyond_mapping {

inline sensor_msgs::msg::PointCloud2 make_point_cloud(
    const InnoDataPacket &packet, double min_range, double max_range,
    bool publish_second_return) {
  sensor_msgs::msg::PointCloud2 cloud;
  const bool standard_xyz = packet.type == INNO_ITEM_TYPE_XYZ_POINTCLOUD;
  if (!standard_xyz && !CHECK_EN_XYZ_POINTCLOUD_DATA(packet.type)) return cloud;

  cloud.height = 1;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
      5, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
      "time", 1, sensor_msgs::msg::PointField::FLOAT64);

  // Each input point can produce at most one output point. Allocate that
  // upper bound, filter once, then shrink to the actual count. Counting in
  // native coordinates and writing in ROS coordinates used different float
  // addition orders, overflowing the buffer for points near range cutoffs.
  modifier.resize(packet.item_number);
  size_t kept = 0;
  for (uint32_t i = 0; i < packet.item_number; ++i) {
    float x, y, z, intensity;
    double time;
    bool second_return;
    if (standard_xyz) {
      const auto &p = reinterpret_cast<const InnoXyzPoint *>(packet.payload)[i];
      // Native X=up, Y=right, Z=forward -> ROS X=forward, Y=left, Z=up.
      x = p.z; y = -p.y; z = p.x; intensity = p.refl;
      time = p.ts_10us * 1.0e-5;
      second_return = p.is_2nd_return;
    } else {
      const auto &p = reinterpret_cast<const InnoEnXyzPoint *>(packet.payload)[i];
      x = p.z; y = -p.y; z = p.x;
      intensity = select_enhanced_intensity(
          packet.use_reflectance, p.reflectance, p.intensity);
      time = p.ts_10us * 1.0e-5;
      second_return = p.is_2nd_return;
    }
    const double range = std::sqrt(
        double(x) * x + double(y) * y + double(z) * z);
    if (!std::isfinite(range) || range < min_range || range > max_range ||
        (!publish_second_return && second_return)) continue;

    // memcpy also avoids alignment assumptions about PointCloud2's byte data.
    auto *out = cloud.data.data() + kept * cloud.point_step;
    std::memcpy(out, &x, sizeof(x));
    std::memcpy(out + 4, &y, sizeof(y));
    std::memcpy(out + 8, &z, sizeof(z));
    std::memcpy(out + 12, &intensity, sizeof(intensity));
    std::memcpy(out + 16, &time, sizeof(time));
    ++kept;
  }
  modifier.resize(kept);
  cloud.is_dense = true;
  return cloud;
}

}  // namespace seyond_mapping
