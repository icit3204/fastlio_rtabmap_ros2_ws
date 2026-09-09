#pragma once

#include <array>
#include <cstdint>
#include <vector>

namespace wheelchair_controller
{

struct BackendConfig
{
  uint32_t can_id{0x18C4D2D0U};
  int velocity_limit_mmps{16380};
  double straight_radius_threshold_mm{10000.0};
  double minimum_turn_radius_mm{1000.0};
  bool invert_radius{true};
  double wheelbase_mm{1000.0};
  double maximum_steering_deg{30.0};
  double command_timeout_sec{0.500};
};

struct EncodedFrame
{
  uint32_t can_id{0};
  uint8_t dlc{8};
  std::array<uint8_t, 8> data{};
  int gear{3};
  int speed_mmps{0};
  double steering_deg{0.0};
  double normalized_radius_mm{0.0};
  uint8_t alive{0};
  bool stale_fallback{true};
};

class FrameTransport
{
public:
  virtual ~FrameTransport() = default;
  virtual bool emit(const EncodedFrame & frame) = 0;
  virtual bool opens_socketcan() const noexcept = 0;
};

class MockTransport final : public FrameTransport
{
public:
  bool emit(const EncodedFrame & frame) override;
  bool opens_socketcan() const noexcept override {return false;}
  const std::vector<EncodedFrame> & frames() const noexcept {return frames_;}

private:
  std::vector<EncodedFrame> frames_;
};

class MkminiBackendCore
{
public:
  explicit MkminiBackendCore(BackendConfig config = BackendConfig{});

  void accept_command(
    double radius_mm, double signed_velocity_mmps, double distance_mm,
    double monotonic_now_sec);
  EncodedFrame tick(double monotonic_now_sec, bool paused = false);
  const BackendConfig & config() const noexcept {return config_;}

private:
  double normalize_radius(double radius_mm) const;
  EncodedFrame encode(double radius_mm, double signed_velocity_mmps, bool stale_fallback);

  BackendConfig config_;
  bool has_command_{false};
  double radius_mm_{0.0};
  double signed_velocity_mmps_{0.0};
  double distance_mm_{0.0};
  double last_command_sec_{0.0};
  uint8_t alive_{0};
};

bool checksum_valid(const EncodedFrame & frame) noexcept;
bool reserved_bits_zero(const EncodedFrame & frame) noexcept;

}  // namespace wheelchair_controller
