#include <gtest/gtest.h>

#include <cmath>

#include "wheelchair_controller/mkmini_backend_core.hpp"

using wheelchair_controller::MkminiBackendCore;
using wheelchair_controller::MockTransport;

TEST(MkminiBackendCore, DirectionMatrixAndWireContract)
{
  struct Case {double radius; double velocity; int gear; double steering_sign;};
  const Case cases[] = {
    {10000.0, 250.0, 4, 0.0},
    {-2000.0, 200.0, 4, 1.0},
    {2000.0, 200.0, 4, -1.0},
    {10000.0, -250.0, 2, 0.0},
    {2000.0, -200.0, 2, -1.0},
    {-2000.0, -200.0, 2, 1.0},
    {0.0, 0.0, 3, 0.0},
  };
  double now = 1.0;
  MkminiBackendCore core;
  MockTransport mock;
  for (const auto & item : cases) {
    core.accept_command(item.radius, item.velocity, 0.0, now);
    const auto frame = core.tick(now);
    ASSERT_TRUE(mock.emit(frame));
    EXPECT_EQ(frame.can_id, 0x18C4D2D0U);
    EXPECT_EQ(frame.dlc, 8);
    EXPECT_EQ(frame.gear, item.gear);
    EXPECT_EQ(frame.speed_mmps, std::abs(static_cast<int>(item.velocity)));
    EXPECT_TRUE(wheelchair_controller::checksum_valid(frame));
    EXPECT_TRUE(wheelchair_controller::reserved_bits_zero(frame));
    if (item.steering_sign == 0.0) {
      EXPECT_DOUBLE_EQ(frame.steering_deg, 0.0);
    } else {
      EXPECT_GT(frame.steering_deg * item.steering_sign, 0.0);
    }
    now += 0.02;
  }
  EXPECT_FALSE(mock.opens_socketcan());
  EXPECT_EQ(mock.frames().size(), 7U);
}

TEST(MkminiBackendCore, HeartbeatAndStaleFallbackRemainContinuous)
{
  MkminiBackendCore core;
  MockTransport mock;
  core.accept_command(10000.0, 250.0, 0.0, 5.0);
  for (int i = 0; i < 30; ++i) {
    ASSERT_TRUE(mock.emit(core.tick(5.0 + 0.02 * i)));
  }
  ASSERT_EQ(mock.frames().size(), 30U);
  for (std::size_t i = 0; i < mock.frames().size(); ++i) {
    EXPECT_EQ(mock.frames()[i].alive, i % 16U);
  }
  const auto & stale = mock.frames().back();
  EXPECT_TRUE(stale.stale_fallback);
  EXPECT_EQ(stale.gear, 3);
  EXPECT_EQ(stale.speed_mmps, 0);
  EXPECT_DOUBLE_EQ(stale.steering_deg, 0.0);
}

TEST(MkminiBackendCore, UsesInnerWheelAckermannForMkminiRearAxleRadius)
{
  MkminiBackendCore core;
  const double L = 600.0;
  const double T = 518.0;
  const double degrees = 180.0 / std::acos(-1.0);
  struct Case {double rear_radius_mm;};
  const Case cases[] = {{1750.0}, {1600.0}, {1500.0}, {1400.0}, {1350.0}, {1300.0}};
  double now = 1.0;
  for (const auto & item : cases) {
    // The bridge uses -v/w; invert_radius restores signed rear radius here.
    core.accept_command(-item.rear_radius_mm, 100.0, 0.0, now);
    const auto frame = core.tick(now);
    const double expected = std::atan(L / (item.rear_radius_mm - T / 2.0)) * degrees;
    EXPECT_NEAR(frame.normalized_radius_mm, item.rear_radius_mm, 1e-9);
    EXPECT_NEAR(frame.steering_deg, expected, 0.011);
    EXPECT_LE(frame.steering_deg, 30.0);
    EXPECT_TRUE(wheelchair_controller::checksum_valid(frame));
    now += 0.01;
  }
}

TEST(MkminiBackendCore, SteeringSignAndConservativeClampAreSafe)
{
  MkminiBackendCore core;
  core.accept_command(-1750.0, 100.0, 0.0, 1.0);
  const auto left = core.tick(1.0);
  core.accept_command(1750.0, 100.0, 0.0, 1.1);
  const auto right = core.tick(1.1);
  EXPECT_GT(left.steering_deg, 0.0);
  EXPECT_LT(right.steering_deg, 0.0);
  EXPECT_NEAR(left.steering_deg, -right.steering_deg, 0.011);

  // Direct malformed lower-backend input cannot command beyond the retained
  // 30 degree software limit: it is normalized to the legal rear radius.
  core.accept_command(-1000.0, 100.0, 0.0, 1.2);
  const auto clamped = core.tick(1.2);
  EXPECT_NEAR(clamped.normalized_radius_mm, 1298.230485, 1e-6);
  EXPECT_NEAR(clamped.steering_deg, 30.0, 0.011);
}
