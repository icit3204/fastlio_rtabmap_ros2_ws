#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace phase5_sensor_freshness_boundary {

enum class BoundaryState { CLOSED, OPEN };
enum class Decision { DROP_CLOSED, FORWARD_OPEN };

struct BoundaryConfig {
  bool enabled{true};
  double lidar_max_age_sec{0.50};
  double imu_max_age_sec{0.05};
  double stable_window_sec{2.0};
};

struct BoundaryCounters {
  uint64_t closed_lidar_dropped{0};
  uint64_t closed_imu_dropped{0};
  uint64_t closed_lidar_stale{0};
  uint64_t closed_imu_stale{0};
  uint64_t stable_window_starts{0};
  uint64_t stable_window_resets{0};
  uint64_t open_lidar_received{0};
  uint64_t open_imu_received{0};
  uint64_t open_lidar_forwarded{0};
  uint64_t open_imu_forwarded{0};
};

class BoundaryCore final {
public:
  explicit BoundaryCore(BoundaryConfig config)
  : config_(config), state_(config.enabled ? BoundaryState::CLOSED : BoundaryState::OPEN) {}

  Decision observe_lidar(double now_sec, double header_age_sec, double effective_age_sec,
                        bool finite, bool advancing) {
    if (state_ == BoundaryState::OPEN) {
      ++counters_.open_lidar_received;
      ++counters_.open_lidar_forwarded;
      return Decision::FORWARD_OPEN;
    }
    ++counters_.closed_lidar_dropped;
    const bool current = config_.enabled && std::isfinite(now_sec) && finite && advancing &&
      valid_age(header_age_sec, config_.lidar_max_age_sec) &&
      valid_age(effective_age_sec, config_.lidar_max_age_sec);
    if (!current) {
      ++counters_.closed_lidar_stale;
      lidar_valid_until_sec_ = -1.0;
    } else {
      const double header_margin = config_.lidar_max_age_sec - header_age_sec;
      const double effective_margin = config_.lidar_max_age_sec - effective_age_sec;
      lidar_valid_until_sec_ = now_sec + std::min(header_margin, effective_margin);
    }
    update_window(now_sec);
    return Decision::DROP_CLOSED;
  }

  Decision observe_imu(double now_sec, double age_sec, bool finite, bool advancing) {
    if (state_ == BoundaryState::OPEN) {
      ++counters_.open_imu_received;
      ++counters_.open_imu_forwarded;
      return Decision::FORWARD_OPEN;
    }
    ++counters_.closed_imu_dropped;
    const bool current = config_.enabled && std::isfinite(now_sec) && finite && advancing &&
      valid_age(age_sec, config_.imu_max_age_sec);
    if (!current) {
      ++counters_.closed_imu_stale;
      imu_valid_until_sec_ = -1.0;
    } else {
      imu_valid_until_sec_ = now_sec + (config_.imu_max_age_sec - age_sec);
    }
    update_window(now_sec);
    return Decision::DROP_CLOSED;
  }

  BoundaryState state() const { return state_; }
  bool open() const { return state_ == BoundaryState::OPEN; }
  double stable_window_start_sec() const { return stable_start_sec_; }
  double open_time_sec() const { return open_time_sec_; }
  bool lidar_fresh_now(double now_sec) const {
    return !config_.enabled || freshness_until_valid(now_sec, lidar_valid_until_sec_);
  }
  bool imu_fresh_now(double now_sec) const {
    return !config_.enabled || freshness_until_valid(now_sec, imu_valid_until_sec_);
  }
  double lidar_valid_until_sec() const { return lidar_valid_until_sec_; }
  double imu_valid_until_sec() const { return imu_valid_until_sec_; }
  const BoundaryCounters &counters() const { return counters_; }

private:
  static bool valid_age(double age, double limit) {
    return std::isfinite(age) && age >= 0.0 && age <= limit;
  }

  static bool freshness_until_valid(double now_sec, double valid_until_sec) {
    return std::isfinite(now_sec) && std::isfinite(valid_until_sec) && now_sec <= valid_until_sec;
  }

  void update_window(double now_sec) {
    if (state_ == BoundaryState::OPEN || !config_.enabled) return;
    if (!freshness_until_valid(now_sec, lidar_valid_until_sec_) ||
        !freshness_until_valid(now_sec, imu_valid_until_sec_)) {
      if (stable_start_sec_ >= 0.0) {
        stable_start_sec_ = -1.0;
        ++counters_.stable_window_resets;
      }
      return;
    }
    if (stable_start_sec_ < 0.0) {
      stable_start_sec_ = now_sec;
      ++counters_.stable_window_starts;
      return;
    }
    if (now_sec - stable_start_sec_ >= config_.stable_window_sec) {
      state_ = BoundaryState::OPEN;
      open_time_sec_ = now_sec;
    }
  }

  BoundaryConfig config_;
  BoundaryState state_{BoundaryState::CLOSED};
  double lidar_valid_until_sec_{-1.0};
  double imu_valid_until_sec_{-1.0};
  double stable_start_sec_{-1.0};
  double open_time_sec_{-1.0};
  BoundaryCounters counters_{};
};

}  // namespace phase5_sensor_freshness_boundary
