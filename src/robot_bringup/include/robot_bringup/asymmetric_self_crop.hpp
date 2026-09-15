#pragma once

#include <cmath>

namespace robot_bringup
{

struct CropBounds
{
  double x_min;
  double x_max;
  double y_min;
  double y_max;
  double z_min;
  double z_max;

  bool valid() const
  {
    return std::isfinite(x_min) && std::isfinite(x_max) &&
           std::isfinite(y_min) && std::isfinite(y_max) &&
           std::isfinite(z_min) && std::isfinite(z_max) &&
           x_min < x_max && y_min < y_max && z_min < z_max;
  }

  bool contains(double x, double y, double z) const
  {
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) &&
           x >= x_min && x <= x_max &&
           y >= y_min && y <= y_max &&
           z >= z_min && z <= z_max;
  }
};

}  // namespace robot_bringup
