#include <gtest/gtest.h>

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
