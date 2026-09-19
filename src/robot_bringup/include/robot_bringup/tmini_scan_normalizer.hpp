#pragma once

#include <cmath>
#include <limits>

namespace robot_bringup
{

inline float normalize_tmini_range(float range, float range_min, float range_max)
{
  if (!std::isfinite(range) || range < range_min || range > range_max) {
    return std::numeric_limits<float>::infinity();
  }
  return range;
}

}  // namespace robot_bringup
