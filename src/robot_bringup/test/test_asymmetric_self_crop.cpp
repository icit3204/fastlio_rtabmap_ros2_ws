#include <cmath>
#include <iostream>

#include "robot_bringup/asymmetric_self_crop.hpp"

namespace
{
int failures = 0;

void expect(bool condition, const char * label)
{
  if (!condition) {
    std::cerr << "FAIL: " << label << '\n';
    ++failures;
  }
}
}  // namespace

int main()
{
  const robot_bringup::CropBounds bounds{0.540, 0.670, -0.140, 0.150, 0.530, 0.580};
  expect(bounds.valid(), "authorized bounds valid");
  expect(bounds.contains(0.621691, 0.012759, 0.547948), "known self return removed");
  expect(!bounds.contains(0.539, 0.0, 0.55), "immediately behind retained");
  expect(!bounds.contains(0.671, 0.0, 0.55), "immediately ahead retained");
  expect(!bounds.contains(0.60, -0.141, 0.55), "immediately right retained");
  expect(!bounds.contains(0.60, 0.151, 0.55), "immediately left retained");
  expect(!bounds.contains(0.60, 0.0, 0.581), "immediately above retained");
  expect(!bounds.contains(0.70, 0.0, 0.05), "low external obstacle retained");
  expect(!bounds.contains(0.671, 0.0, 0.55), "thin external obstacle retained");
  expect(!bounds.contains(2.0, 1.0, 0.8), "wall point retained");
  expect(!bounds.contains(NAN, 0.0, 0.55), "non-finite point not classified as self");
  expect(
    !robot_bringup::CropBounds{0.7, 0.6, -0.1, 0.1, 0.5, 0.6}.valid(),
    "inverted bounds rejected");
  return failures == 0 ? 0 : 1;
}
