#include <gtest/gtest.h>

#include "seyond_mapping/point_fields.hpp"

TEST(PointFields, UsesReflectanceWhenPacketRequestsIt) {
  EXPECT_FLOAT_EQ(
      seyond_mapping::select_enhanced_intensity(true, 17, 91), 17.0F);
}

TEST(PointFields, UsesIntensityWhenPacketRequestsIt) {
  EXPECT_FLOAT_EQ(
      seyond_mapping::select_enhanced_intensity(false, 17, 91), 91.0F);
}
