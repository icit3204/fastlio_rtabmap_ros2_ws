#include <cassert>
#include <cmath>
#include <limits>

#include "robot_bringup/tmini_scan_normalizer.hpp"

int main()
{
  using robot_bringup::normalize_tmini_range;
  assert(std::isinf(normalize_tmini_range(0.0F, 0.03F, 12.0F)));
  assert(std::isinf(normalize_tmini_range(0.02F, 0.03F, 12.0F)));
  assert(std::isinf(normalize_tmini_range(12.1F, 0.03F, 12.0F)));
  assert(std::isinf(normalize_tmini_range(
    std::numeric_limits<float>::quiet_NaN(), 0.03F, 12.0F)));
  assert(std::isinf(normalize_tmini_range(
    std::numeric_limits<float>::infinity(), 0.03F, 12.0F)));
  assert(normalize_tmini_range(0.03F, 0.03F, 12.0F) == 0.03F);
  assert(normalize_tmini_range(1.50F, 0.03F, 12.0F) == 1.50F);
  assert(normalize_tmini_range(12.0F, 0.03F, 12.0F) == 12.0F);
  return 0;
}
