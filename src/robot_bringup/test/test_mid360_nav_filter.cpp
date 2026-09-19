#include <cassert>
#include <cmath>
#include <limits>

#include "robot_bringup/mid360_nav_filter.hpp"

int main()
{
  robot_bringup::Mid360NavFilterBounds bounds;
  assert(bounds.valid());

  assert(bounds.is_obstacle(1.0, 0.0, 0.15));
  assert(bounds.is_obstacle(1.0, 0.0, 1.60));
  assert(!bounds.is_obstacle(1.0, 0.0, 0.149));
  assert(!bounds.is_obstacle(3.01, 0.0, 0.50));
  assert(!bounds.is_obstacle(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.5));

  // Floor/background returns may clear rays but can never mark obstacles.
  assert(bounds.is_clearing_return(1.0, 0.0, 0.0));
  assert(!bounds.is_obstacle(1.0, 0.0, 0.0));
  assert(bounds.is_clearing_return(3.4, 0.0, 0.2));
  assert(!bounds.is_obstacle(3.4, 0.0, 0.2));

  auto invalid = bounds;
  invalid.obstacle_min_height = invalid.obstacle_max_height;
  assert(!invalid.valid());
  invalid = bounds;
  invalid.clearing_max_range = 2.0;
  assert(!invalid.valid());
  return 0;
}
