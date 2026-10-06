#include <gtest/gtest.h>

#include <limits>
#include <vector>

#include "seyond_mapping/point_cloud.hpp"

namespace {
template <typename Point>
struct Packet {
  explicit Packet(InnoItemType type, const std::vector<Point> &points)
      : bytes(sizeof(InnoDataPacket) + points.size() * sizeof(Point), 0) {
    auto &p = get();
    p.type = type;
    p.item_number = points.size();
    p.item_size = sizeof(Point);
    p.common.size = bytes.size();
    if (!points.empty()) std::memcpy(p.payload, points.data(), points.size() * sizeof(Point));
  }
  InnoDataPacket &get() { return *reinterpret_cast<InnoDataPacket *>(bytes.data()); }
  std::vector<uint8_t> bytes;
};

template <typename T>
T field(const sensor_msgs::msg::PointCloud2 &cloud, size_t offset) {
  T value;
  std::memcpy(&value, cloud.data.data() + offset, sizeof(value));
  return value;
}
}  // namespace

TEST(PointCloud, RangeBoundaryCannotWritePastCountedBuffer) {
  InnoXyzPoint boundary{};
  boundary.x = 19.56931495666504F;
  boundary.y = -17.337459564208984F;
  boundary.z = 42.6198844909668F;
  // Old first pass rejects this point; its reordered second pass accepts it.
  const float counted = (boundary.x * boundary.x + boundary.y * boundary.y) +
                        boundary.z * boundary.z;
  const float written = (boundary.z * boundary.z + boundary.y * boundary.y) +
                        boundary.x * boundary.x;
  ASSERT_GT(counted, 2500.0F);
  ASSERT_LE(written, 2500.0F);
  InnoXyzPoint valid{};
  valid.z = 5.0F;
  Packet<InnoXyzPoint> packet(INNO_ITEM_TYPE_XYZ_POINTCLOUD,
                            {valid, boundary, boundary, boundary});
  const auto cloud = seyond_mapping::make_point_cloud(packet.get(), 0.15, 50.0, true);
  EXPECT_EQ(cloud.width, 1U);
  EXPECT_EQ(cloud.height, 1U);
  EXPECT_EQ(cloud.point_step, 24U);
  EXPECT_EQ(cloud.row_step, cloud.data.size());
  EXPECT_EQ(cloud.data.size(), cloud.width * cloud.point_step);
  EXPECT_FLOAT_EQ(field<float>(cloud, 0), 5.0F);
}

TEST(PointCloud, FiltersOnceAndPreservesCoordinatesAndTime) {
  InnoXyzPoint valid{};
  valid.x = 1; valid.y = 2; valid.z = 3; valid.refl = 17; valid.ts_10us = 125;
  auto second = valid;
  second.is_2nd_return = true;
  auto invalid = valid;
  invalid.x = std::numeric_limits<float>::quiet_NaN();
  auto distant = valid;
  distant.z = 60;
  Packet<InnoXyzPoint> packet(INNO_ITEM_TYPE_XYZ_POINTCLOUD,
                            {invalid, second, distant, valid, InnoXyzPoint{}});
  const auto cloud = seyond_mapping::make_point_cloud(packet.get(), 0.15, 50, false);
  ASSERT_EQ(cloud.width, 1U);
  EXPECT_FLOAT_EQ(field<float>(cloud, 0), 3.0F);
  EXPECT_FLOAT_EQ(field<float>(cloud, 4), -2.0F);
  EXPECT_FLOAT_EQ(field<float>(cloud, 8), 1.0F);
  EXPECT_FLOAT_EQ(field<float>(cloud, 12), 17.0F);
  EXPECT_DOUBLE_EQ(field<double>(cloud, 16), 125 * 1.0e-5);
  EXPECT_TRUE(cloud.is_dense);
  EXPECT_EQ(seyond_mapping::make_point_cloud(packet.get(), 0.15, 50, true).width, 2U);
}

TEST(PointCloud, EnhancedPacketsRespectIntensityModeAndReturnFilter) {
  InnoEnXyzPoint point{};
  point.z = 2; point.reflectance = 17; point.intensity = 91;
  Packet<InnoEnXyzPoint> packet(INNO_ROBINW_ITEM_TYPE_XYZ_POINTCLOUD, {point});
  packet.get().use_reflectance = true;
  auto cloud = seyond_mapping::make_point_cloud(packet.get(), 0.15, 50, true);
  ASSERT_EQ(cloud.width, 1U);
  EXPECT_FLOAT_EQ(field<float>(cloud, 12), 17.0F);
  packet.get().use_reflectance = false;
  cloud = seyond_mapping::make_point_cloud(packet.get(), 0.15, 50, true);
  EXPECT_FLOAT_EQ(field<float>(cloud, 12), 91.0F);
}

TEST(PointCloud, EmptyAndFullyFilteredPacketsHaveConsistentSizes) {
  Packet<InnoXyzPoint> empty(INNO_ITEM_TYPE_XYZ_POINTCLOUD, {});
  Packet<InnoXyzPoint> filtered(INNO_ITEM_TYPE_XYZ_POINTCLOUD, {InnoXyzPoint{}});
  for (auto *packet : {&empty.get(), &filtered.get()}) {
    const auto cloud = seyond_mapping::make_point_cloud(*packet, 0.15, 50, true);
    EXPECT_EQ(cloud.width, 0U);
    EXPECT_EQ(cloud.height, 1U);
    EXPECT_EQ(cloud.row_step, 0U);
    EXPECT_TRUE(cloud.data.empty());
  }
}
