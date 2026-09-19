#pragma once

#include <cmath>

namespace robot_bringup
{

struct Mid360NavFilterBounds
{
  double obstacle_min_height{0.15};
  double obstacle_max_height{1.60};
  double clearing_min_height{-0.20};
  double clearing_max_height{2.00};
  double obstacle_min_range{0.10};
  double obstacle_max_range{3.00};
  double clearing_min_range{0.10};
  double clearing_max_range{3.50};

  bool valid() const
  {
    return std::isfinite(obstacle_min_height) && std::isfinite(obstacle_max_height) &&
           std::isfinite(clearing_min_height) && std::isfinite(clearing_max_height) &&
           std::isfinite(obstacle_min_range) && std::isfinite(obstacle_max_range) &&
           std::isfinite(clearing_min_range) && std::isfinite(clearing_max_range) &&
           obstacle_min_height < obstacle_max_height &&
           clearing_min_height <= obstacle_min_height &&
           clearing_max_height >= obstacle_max_height &&
           0.0 <= obstacle_min_range && obstacle_min_range < obstacle_max_range &&
           0.0 <= clearing_min_range && clearing_min_range < clearing_max_range &&
           clearing_max_range >= obstacle_max_range;
  }

  bool is_obstacle(double x, double y, double z) const
  {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      return false;
    }
    const double range = std::hypot(x, y);
    return z >= obstacle_min_height && z <= obstacle_max_height &&
           range >= obstacle_min_range && range <= obstacle_max_range;
  }

  bool is_clearing_return(double x, double y, double z) const
  {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      return false;
    }
    const double range = std::hypot(x, y);
    return z >= clearing_min_height && z <= clearing_max_height &&
           range >= clearing_min_range && range <= clearing_max_range;
  }
};

}  // namespace robot_bringup
