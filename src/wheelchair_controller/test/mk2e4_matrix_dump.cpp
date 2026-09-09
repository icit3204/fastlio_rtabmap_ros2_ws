#include <iomanip>
#include <iostream>
#include <string>

#include "wheelchair_controller/mkmini_backend_core.hpp"

int main()
{
  struct Case {const char * name; double radius; double velocity; double time;};
  const Case cases[] = {
    {"forward_straight", 10000.0, 250.0, 1.00},
    {"forward_left", -2000.0, 200.0, 1.02},
    {"forward_right", 2000.0, 200.0, 1.04},
    {"reverse_straight", 10000.0, -250.0, 1.06},
    {"reverse_left", 2000.0, -200.0, 1.08},
    {"reverse_right", -2000.0, -200.0, 1.10},
    {"zero", 0.0, 0.0, 1.12},
  };
  wheelchair_controller::MkminiBackendCore core;
  wheelchair_controller::MockTransport mock;
  std::cout << "case,radius_mm,signed_velocity_mmps,gear,speed_mmps,steering_deg,alive,";
  std::cout << "can_id,dlc,data_hex,reserved_zero,checksum_valid,stale_fallback\n";
  for (const auto & item : cases) {
    core.accept_command(item.radius, item.velocity, 0.0, item.time);
    const auto frame = core.tick(item.time);
    mock.emit(frame);
    std::cout << item.name << ',' << item.radius << ',' << item.velocity << ',' << frame.gear <<
      ',';
    std::cout << frame.speed_mmps << ',' << frame.steering_deg << ',' <<
      static_cast<int>(frame.alive);
    std::cout << ",0x" << std::hex << std::uppercase << frame.can_id << std::dec << ',';
    std::cout << static_cast<int>(frame.dlc) << ',';
    for (const auto byte : frame.data) {
      std::cout << std::hex << std::uppercase << std::setw(2) << std::setfill('0') <<
        static_cast<int>(byte);
    }
    std::cout << std::dec << ',' << wheelchair_controller::reserved_bits_zero(frame) << ',';
    std::cout << wheelchair_controller::checksum_valid(frame) << ',' << frame.stale_fallback <<
      '\n';
  }
  const auto stale = core.tick(1.64);
  mock.emit(stale);
  std::cout << "backend_stale,0,0," << stale.gear << ',' << stale.speed_mmps << ',';
  std::cout << stale.steering_deg << ',' << static_cast<int>(stale.alive) << ",0x";
  std::cout << std::hex << std::uppercase << stale.can_id << std::dec << ',';
  std::cout << static_cast<int>(stale.dlc) << ',';
  for (const auto byte : stale.data) {
    std::cout << std::hex << std::uppercase << std::setw(2) << std::setfill('0') <<
      static_cast<int>(byte);
  }
  std::cout << std::dec << ',' << wheelchair_controller::reserved_bits_zero(stale) << ',';
  std::cout << wheelchair_controller::checksum_valid(stale) << ',' << stale.stale_fallback << '\n';
  return mock.opens_socketcan() ? 1 : 0;
}
