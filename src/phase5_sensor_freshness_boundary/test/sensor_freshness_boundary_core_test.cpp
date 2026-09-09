#include <cmath>
#include <limits>

#include <gtest/gtest.h>
#include "phase5_sensor_freshness_boundary/boundary_core.hpp"
#include "phase5_sensor_freshness_boundary/semantic_compare.hpp"

using namespace phase5_sensor_freshness_boundary;

BoundaryCore fresh_core() { return BoundaryCore(BoundaryConfig{}); }
void current_pair(BoundaryCore &b, double t) {
  b.observe_lidar(t, 0.1, 0.1, true, true);
  b.observe_imu(t, 0.01, true, true);
}
void healthy_window(BoundaryCore &b, double start_sec = 0.0) {
  current_pair(b, start_sec);
  for (int i = 1; i <= 400; ++i) {
    const double t = start_sec + i * 0.005;
    b.observe_imu(t, 0.01, true, true);
    if (i % 20 == 0) b.observe_lidar(t, 0.1, 0.1, true, true);
  }
}
BoundaryCore opened_core() {
  BoundaryCore b(BoundaryConfig{});
  healthy_window(b);
  return b;
}

TEST(K4C, T01_initial_closed) { EXPECT_EQ(fresh_core().state(), BoundaryState::CLOSED); }
TEST(K4C, T02_stale_lidar_dropped) { auto b = fresh_core(); EXPECT_EQ(b.observe_lidar(0, 1, 1, true, true), Decision::DROP_CLOSED); EXPECT_EQ(b.counters().closed_lidar_stale, 1u); }
TEST(K4C, T03_stale_imu_dropped) { auto b = fresh_core(); EXPECT_EQ(b.observe_imu(0, 1, true, true), Decision::DROP_CLOSED); EXPECT_EQ(b.counters().closed_imu_stale, 1u); }
TEST(K4C, T04_lidar_alone_cannot_open) { auto b = fresh_core(); for (int i = 0; i < 30; ++i) b.observe_lidar(i * .1, .1, .1, true, true); EXPECT_FALSE(b.open()); }
TEST(K4C, T05_imu_alone_cannot_open) { auto b = fresh_core(); for (int i = 0; i < 30; ++i) b.observe_imu(i * .1, .01, true, true); EXPECT_FALSE(b.open()); }
TEST(K4C, T06_window_short) { auto b = fresh_core(); current_pair(b, 0); current_pair(b, 1.999); EXPECT_FALSE(b.open()); }
TEST(K4C, T07_stale_lidar_resets_window) { auto b = fresh_core(); current_pair(b, 0); b.observe_lidar(1, 1, 1, true, true); EXPECT_EQ(b.counters().stable_window_resets, 1u); EXPECT_FALSE(b.open()); }
TEST(K4C, T08_stale_imu_resets_window) { auto b = fresh_core(); current_pair(b, 0); b.observe_imu(1, 1, true, true); EXPECT_EQ(b.counters().stable_window_resets, 1u); EXPECT_FALSE(b.open()); }
TEST(K4C, T09_reversal_resets_window) { auto b = fresh_core(); current_pair(b, 0); b.observe_lidar(1, .1, .1, true, false); EXPECT_EQ(b.counters().stable_window_resets, 1u); }
TEST(K4C, T10_future_rejected) { auto b = fresh_core(); b.observe_imu(0, -0.001, true, true); EXPECT_EQ(b.counters().closed_imu_stale, 1u); }
TEST(K4C, T11_two_second_window_opens_once) { auto b = fresh_core(); healthy_window(b); EXPECT_TRUE(b.open()); EXPECT_DOUBLE_EQ(b.open_time_sec(), 2.0); auto c = b.counters(); current_pair(b, 3); EXPECT_EQ(b.counters().stable_window_starts, c.stable_window_starts); }
TEST(K4C, T12_closed_messages_discarded) { auto b = fresh_core(); current_pair(b, 0); EXPECT_EQ(b.counters().closed_lidar_dropped, 1u); EXPECT_EQ(b.counters().closed_imu_dropped, 1u); }
TEST(K4C, T13_closed_messages_not_replayed) { auto b = fresh_core(); current_pair(b, 0); b.observe_lidar(1.0, .1, .1, true, true); b.observe_imu(1.0, .01, true, true); EXPECT_FALSE(b.open()); EXPECT_EQ(b.counters().open_lidar_forwarded, 0u); EXPECT_EQ(b.counters().open_imu_forwarded, 0u); }
TEST(K4C, T14_first_post_open_lidar_forwarded) { auto b = opened_core(); EXPECT_EQ(b.observe_lidar(3, 10, 10, true, false), Decision::FORWARD_OPEN); }
TEST(K4C, T15_first_post_open_imu_forwarded) { auto b = opened_core(); EXPECT_EQ(b.observe_imu(3, 10, true, false), Decision::FORWARD_OPEN); }
TEST(K4C, T16_runtime_stale_lidar_forwarded) { auto b = opened_core(); EXPECT_EQ(b.observe_lidar(3, 10, 10, true, false), Decision::FORWARD_OPEN); }
TEST(K4C, T17_runtime_stale_imu_forwarded) { auto b = opened_core(); EXPECT_EQ(b.observe_imu(3, 10, true, false), Decision::FORWARD_OPEN); }
TEST(K4C, T18_runtime_reversal_forwarded) { auto b = opened_core(); EXPECT_EQ(b.observe_lidar(3, 10, 10, false, false), Decision::FORWARD_OPEN); }
TEST(K4C, T19_open_never_closes) { auto b = opened_core(); b.observe_lidar(3, 10, 10, false, false); b.observe_imu(3, 10, false, false); EXPECT_TRUE(b.open()); }
TEST(K4C, T20_open_received_forwarded_equal) { auto b = opened_core(); b.observe_lidar(3, 1, 1, true, true); b.observe_imu(3, 1, true, true); EXPECT_EQ(b.counters().open_lidar_received, b.counters().open_lidar_forwarded); EXPECT_EQ(b.counters().open_imu_received, b.counters().open_imu_forwarded); }
TEST(K4C, T21_canonical_lidar_equality) { livox_ros_driver2::msg::CustomMsg a, b; a.point_num = 1; a.points.resize(1); a.points[0].x = -0.0F; b = a; EXPECT_TRUE(custom_msg_semantically_equal(a, b)); b.points[0].x = 0.0F; EXPECT_FALSE(custom_msg_semantically_equal(a, b)); }
TEST(K4C, T22_canonical_imu_equality) { sensor_msgs::msg::Imu a, b; b = a; EXPECT_TRUE(imu_semantically_equal(a, b)); b.orientation_covariance[4] = 1.0; EXPECT_FALSE(imu_semantically_equal(a, b)); }
TEST(K4C, T23_no_queue) { auto b = fresh_core(); for (int i = 0; i < 100; ++i) b.observe_lidar(i, 1, 1, true, true); EXPECT_EQ(b.counters().open_lidar_received, 0u); }
TEST(K4C, T24_diagnostics_counters) { auto b = fresh_core(); b.observe_lidar(0, 1, 1, true, true); b.observe_imu(0, 1, true, true); EXPECT_EQ(b.counters().closed_lidar_dropped, 1u); EXPECT_EQ(b.counters().closed_imu_dropped, 1u); EXPECT_EQ(b.counters().closed_lidar_stale, 1u); EXPECT_EQ(b.counters().closed_imu_stale, 1u); }
TEST(K4C, T25_boundary_inclusive) { auto b = fresh_core(); b.observe_lidar(0, .5, .5, true, true); b.observe_imu(0, .05, true, true); EXPECT_EQ(b.counters().closed_lidar_stale, 0u); EXPECT_EQ(b.counters().closed_imu_stale, 0u); }
TEST(K4C, T26_disabled_boundary_bypasses) { BoundaryConfig c; c.enabled = false; BoundaryCore b(c); EXPECT_EQ(b.state(), BoundaryState::OPEN); EXPECT_EQ(b.observe_lidar(0, 100, 100, false, false), Decision::FORWARD_OPEN); EXPECT_EQ(b.observe_imu(0, 100, false, false), Decision::FORWARD_OPEN); }

// K4C0 pre-edit reproductions were run against the exact boolean-only
// BoundaryCore before the production edit. This tiny retained model documents
// the old state transition in executable form; the recorded pre-edit run is
// the authoritative reproduction evidence.
struct LegacyBooleanModel {
  bool lidar_current{false};
  bool imu_current{false};
  bool opened{false};
  double stable_start{-1.0};

  void update(double now) {
    if (opened || !lidar_current || !imu_current) return;
    if (stable_start < 0.0) stable_start = now;
    if (now - stable_start >= 2.0) opened = true;
  }
  void lidar(double now) { lidar_current = true; update(now); }
  void imu(double now) { imu_current = true; update(now); }
};

TEST(K4C0_PRE_EDIT, T01_imu_stall_false_open) {
  LegacyBooleanModel b;
  b.lidar(0.0); b.imu(0.0);
  for (int i = 1; i <= 21; ++i) b.lidar(i * 0.1);
  EXPECT_TRUE(b.opened);
}

TEST(K4C0_PRE_EDIT, T02_lidar_stall_false_open) {
  LegacyBooleanModel b;
  b.lidar(0.0); b.imu(0.0);
  for (int i = 1; i <= 21; ++i) b.imu(i * 0.1);
  EXPECT_TRUE(b.opened);
}

TEST(K4C0, T03_imu_stall_remains_closed) {
  auto b = fresh_core();
  current_pair(b, 0.0);
  for (int i = 1; i <= 21; ++i) b.observe_lidar(i * 0.1, 0.1, 0.1, true, true);
  EXPECT_FALSE(b.open());
  EXPECT_FALSE(b.imu_fresh_now(2.1));
}

TEST(K4C0, T04_lidar_stall_remains_closed) {
  auto b = fresh_core();
  current_pair(b, 0.0);
  for (int i = 1; i <= 21; ++i) b.observe_imu(i * 0.1, 0.01, true, true);
  EXPECT_FALSE(b.open());
  EXPECT_FALSE(b.lidar_fresh_now(2.1));
}

TEST(K4C0, T05_imu_expiry_resets_window) {
  auto b = fresh_core();
  current_pair(b, 0.0);
  b.observe_lidar(0.1, 0.1, 0.1, true, true);
  EXPECT_EQ(b.stable_window_start_sec(), -1.0);
  EXPECT_FALSE(b.open());
}

TEST(K4C0, T06_lidar_expiry_resets_window) {
  auto b = fresh_core();
  current_pair(b, 0.0);
  b.observe_imu(0.5, 0.01, true, true);
  EXPECT_EQ(b.stable_window_start_sec(), -1.0);
  EXPECT_FALSE(b.open());
}

TEST(K4C0, T07_stall_near_window_end_cannot_open) {
  auto b = fresh_core();
  current_pair(b, 0.0);
  b.observe_lidar(1.9, 0.1, 0.1, true, true);
  EXPECT_FALSE(b.open());
  b.observe_lidar(2.1, 0.1, 0.1, true, true);
  EXPECT_FALSE(b.open());
}

TEST(K4C0, T08_recovery_requires_new_full_window) {
  auto b = fresh_core();
  current_pair(b, 0.0);
  b.observe_lidar(0.1, 0.1, 0.1, true, true);
  current_pair(b, 1.0);
  for (int i = 1; i <= 399; ++i) {
    const double t = 1.0 + i * 0.005;
    b.observe_imu(t, 0.01, true, true);
    if (i % 20 == 0) b.observe_lidar(t, 0.1, 0.1, true, true);
  }
  EXPECT_FALSE(b.open());
  b.observe_imu(3.0, 0.01, true, true);
  b.observe_lidar(3.0, 0.1, 0.1, true, true);
  EXPECT_TRUE(b.open());
}

TEST(K4C0, T09_imu_expiry_boundary_inclusive) {
  auto b = fresh_core();
  current_pair(b, 0.0);
  EXPECT_TRUE(b.imu_fresh_now(0.04));
}

TEST(K4C0, T10_imu_expiry_strictly_after_is_stale) {
  auto b = fresh_core();
  current_pair(b, 0.0);
  EXPECT_FALSE(b.imu_fresh_now(0.040001));
}

TEST(K4C0, T11_lidar_expiry_boundary_inclusive) {
  auto b = fresh_core();
  current_pair(b, 0.0);
  EXPECT_TRUE(b.lidar_fresh_now(0.4));
}

TEST(K4C0, T12_lidar_expiry_strictly_after_is_stale) {
  auto b = fresh_core();
  current_pair(b, 0.0);
  EXPECT_FALSE(b.lidar_fresh_now(0.400001));
}

TEST(K4C0, T13_invalid_observation_invalidates_prior_evidence) {
  auto b = fresh_core();
  current_pair(b, 0.0);
  b.observe_imu(0.01, 1.0, true, true);
  EXPECT_FALSE(b.imu_fresh_now(0.01));
  EXPECT_EQ(b.stable_window_start_sec(), -1.0);
  b.observe_lidar(0.02, 0.1, 0.1, true, false);
  EXPECT_FALSE(b.lidar_fresh_now(0.02));
}

TEST(K4C0, T14_one_stream_never_seen_never_opens) {
  auto b = fresh_core();
  for (int i = 0; i <= 21; ++i) b.observe_lidar(i * 0.1, 0.1, 0.1, true, true);
  EXPECT_FALSE(b.open());
}

TEST(K4C0, T15_both_streams_stop_never_opens) {
  auto b = fresh_core();
  current_pair(b, 0.0);
  EXPECT_FALSE(b.open());
  EXPECT_FALSE(b.lidar_fresh_now(2.1));
  EXPECT_FALSE(b.imu_fresh_now(2.1));
}

TEST(K4C0, T16_healthy_dual_stream_opens_once) {
  auto b = fresh_core();
  current_pair(b, 0.0);
  for (int i = 1; i <= 420; ++i) {
    const double t = i * 0.005;
    b.observe_imu(t, 0.01, true, true);
    if (i % 20 == 0) b.observe_lidar(t, 0.1, 0.1, true, true);
  }
  EXPECT_TRUE(b.open());
  EXPECT_EQ(b.counters().stable_window_starts, 1u);
}

TEST(K4C0, T17_open_remains_latched_after_stale_reversal) {
  auto b = fresh_core();
  b = opened_core();
  EXPECT_TRUE(b.open());
  b.observe_lidar(2.1, 10.0, 10.0, false, false);
  b.observe_imu(2.1, 10.0, false, false);
  EXPECT_TRUE(b.open());
}

TEST(K4C0, T18_runtime_stale_forwarded_after_open) {
  auto b = fresh_core();
  b = opened_core();
  EXPECT_EQ(b.observe_imu(2.1, 10.0, true, false), Decision::FORWARD_OPEN);
  EXPECT_EQ(b.observe_lidar(2.1, 10.0, 10.0, true, false), Decision::FORWARD_OPEN);
}

TEST(K4C0, T19_closed_no_queue_replay_regression) {
  auto b = fresh_core();
  current_pair(b, 0.0);
  b.observe_lidar(0.1, 1.0, 1.0, true, true);
  b.observe_imu(0.1, 1.0, true, true);
  EXPECT_EQ(b.counters().open_lidar_forwarded, 0u);
  EXPECT_EQ(b.counters().open_imu_forwarded, 0u);
}

TEST(K4C0, T20_lidar_semantic_forwarding_regression) {
  livox_ros_driver2::msg::CustomMsg a, b;
  a.point_num = 1; a.points.resize(1); a.points[0].x = -0.0F;
  b = a;
  EXPECT_TRUE(custom_msg_semantically_equal(a, b));
}

TEST(K4C0, T21_imu_semantic_forwarding_regression) {
  sensor_msgs::msg::Imu a, b;
  a.orientation.x = -0.0; b = a;
  EXPECT_TRUE(imu_semantically_equal(a, b));
}
