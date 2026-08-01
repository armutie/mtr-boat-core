#pragma once

#include <cstdint>

namespace seyond_mapping {

constexpr float select_enhanced_intensity(
    bool use_reflectance, std::uint16_t reflectance, std::uint16_t intensity) {
  return static_cast<float>(use_reflectance ? reflectance : intensity);
}

}  // namespace seyond_mapping
