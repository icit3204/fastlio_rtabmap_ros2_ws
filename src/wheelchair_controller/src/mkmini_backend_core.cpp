#include "wheelchair_controller/mkmini_backend_core.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace wheelchair_controller
{

namespace
{
constexpr double kPi = 3.14159265358979323846;

int clamp_signed(int value, int absolute_limit)
{
  const int limit = std::min(std::abs(absolute_limit), 32767);
  return std::max(-limit, std::min(limit, value));
}
}  // namespace

bool MockTransport::emit(const EncodedFrame & frame)
{
  frames_.push_back(frame);
  return true;
}

MkminiBackendCore::MkminiBackendCore(BackendConfig config)
: config_(config)
{
  if (config_.command_timeout_sec <= 0.0 || config_.wheelbase_mm <= 0.0 ||
    config_.straight_radius_threshold_mm <= 0.0 || config_.minimum_turn_radius_mm <= 0.0)
  {
    throw std::invalid_argument("invalid MK-mini backend configuration");
  }
}

void MkminiBackendCore::accept_command(
  double radius_mm, double signed_velocity_mmps, double distance_mm,
  double monotonic_now_sec)
{
  if (!std::isfinite(radius_mm) || !std::isfinite(signed_velocity_mmps) ||
    !std::isfinite(distance_mm) || !std::isfinite(monotonic_now_sec))
  {
    return;
  }
  radius_mm_ = radius_mm;
  signed_velocity_mmps_ = signed_velocity_mmps;
  distance_mm_ = distance_mm;
  last_command_sec_ = monotonic_now_sec;
  has_command_ = true;
}

EncodedFrame MkminiBackendCore::tick(double monotonic_now_sec, bool paused)
{
  const bool stale = paused || !has_command_ || !std::isfinite(monotonic_now_sec) ||
    monotonic_now_sec<last_command_sec_ ||
      (monotonic_now_sec - last_command_sec_)> config_.command_timeout_sec;
  return stale ? encode(0.0, 0.0, true) :
         encode(radius_mm_, signed_velocity_mmps_, false);
}

double MkminiBackendCore::normalize_radius(double radius_mm) const
{
  if (std::abs(radius_mm) < 1e-6 || radius_mm >= config_.straight_radius_threshold_mm) {
    return radius_mm;
  }
  double normalized = config_.invert_radius ? -radius_mm : radius_mm;
  if (normalized > 0.0 && normalized < config_.minimum_turn_radius_mm) {
    normalized = config_.minimum_turn_radius_mm;
  } else if (normalized < 0.0 && normalized > -config_.minimum_turn_radius_mm) {
    normalized = -config_.minimum_turn_radius_mm;
  }
  return normalized;
}

EncodedFrame MkminiBackendCore::encode(
  double radius_mm, double signed_velocity_mmps, bool stale_fallback)
{
  EncodedFrame frame;
  frame.can_id = config_.can_id;
  frame.alive = alive_;
  frame.stale_fallback = stale_fallback;

  const int velocity = clamp_signed(
    static_cast<int>(std::llround(signed_velocity_mmps)), config_.velocity_limit_mmps);
  frame.gear = velocity > 0 ? 4 : (velocity < 0 ? 2 : 3);  // D, R, N
  frame.speed_mmps = std::abs(velocity);
  frame.normalized_radius_mm = normalize_radius(radius_mm);

  int steering_raw = 0;
  if (std::abs(frame.normalized_radius_mm) >= 1e-6 &&
    std::abs(frame.normalized_radius_mm) < config_.straight_radius_threshold_mm)
  {
    double steering_deg = std::atan(config_.wheelbase_mm / frame.normalized_radius_mm) *
      180.0 / kPi;
    steering_deg = std::clamp(
      steering_deg, -config_.maximum_steering_deg, config_.maximum_steering_deg);
    steering_raw = static_cast<int>(std::llround(steering_deg / 0.01));
  }
  frame.steering_deg = static_cast<double>(steering_raw) * 0.01;

  auto & data = frame.data;
  data.fill(0);
  data[0] = static_cast<uint8_t>(frame.gear & 0x0F);
  data[0] |= static_cast<uint8_t>((frame.speed_mmps & 0x0F) << 4);
  data[1] = static_cast<uint8_t>((frame.speed_mmps >> 4) & 0xFF);
  data[2] = static_cast<uint8_t>((frame.speed_mmps >> 12) & 0x0F);
  const uint16_t steering_u = static_cast<uint16_t>(static_cast<int16_t>(steering_raw));
  data[2] |= static_cast<uint8_t>((steering_u & 0x0F) << 4);
  data[3] = static_cast<uint8_t>((steering_u >> 4) & 0xFF);
  data[4] = static_cast<uint8_t>((steering_u >> 12) & 0x0F);
  data[6] = static_cast<uint8_t>((alive_ & 0x0F) << 4);
  for (int i = 0; i < 7; ++i) {
    data[7] ^= data[i];
  }
  alive_ = static_cast<uint8_t>((alive_ + 1U) & 0x0FU);
  return frame;
}

bool checksum_valid(const EncodedFrame & frame) noexcept
{
  uint8_t checksum = 0;
  for (int i = 0; i < 7; ++i) {
    checksum ^= frame.data[i];
  }
  return checksum == frame.data[7];
}

bool reserved_bits_zero(const EncodedFrame & frame) noexcept
{
  return (frame.data[4] & 0xF0U) == 0U && frame.data[5] == 0U &&
         (frame.data[6] & 0x0FU) == 0U;
}

}  // namespace wheelchair_controller
