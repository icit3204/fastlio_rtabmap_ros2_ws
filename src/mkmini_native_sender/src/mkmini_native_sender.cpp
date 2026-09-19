#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <sys/timerfd.h>
#include <sys/un.h>
#include <net/if.h>
#include <unistd.h>
#include <fcntl.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <new>
#include <stdexcept>
#include <string>

namespace {
constexpr uint32_t kCtrlCmdId = 0x18C4D2D0U;
constexpr uint32_t kCtrlFbId = 0x18C4D2EFU;
constexpr uint32_t kDiagFbId = 0x18C4EAEFU;
constexpr double kCommandTtlSec = .050;
constexpr double kFeedbackFreshnessSec = .050;
constexpr uint32_t kFeedbackMagic = 0x4D4B5258U;
constexpr uint32_t kFeedbackVersion = 1U;

double mono()
{
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec + ts.tv_nsec * 1e-9;
}

uint8_t checksum(const uint8_t * b)
{
  uint8_t x = 0;
  for (int i = 0; i < 7; ++i) x ^= b[i];
  return x;
}

uint64_t payload_word(const can_frame & frame)
{
  uint64_t word = 0;
  for (int i = 0; i < 7; ++i) word |= uint64_t(frame.data[i]) << (8 * i);
  return word;
}

uint32_t bits(uint64_t word, int start, int length)
{
  return static_cast<uint32_t>((word >> start) & ((uint64_t(1) << length) - 1));
}

int16_t signed16(uint32_t value)
{
  return static_cast<int16_t>(static_cast<uint16_t>(value));
}

struct Command
{
  int gear = 3;
  int speed_mmps = 0;
  int steer_cdeg = 0;
  bool allowed = false;
  double received = 0.;
};

struct SendResult
{
  double stamp = 0.;
  Command command{};
};

struct FeedbackSnapshot
{
  uint32_t magic = kFeedbackMagic;
  uint32_t version = kFeedbackVersion;
  uint64_t sequence = 0;
  double ctrl_stamp = 0.;
  double diag_stamp = 0.;
  uint64_t ctrl_count = 0;
  uint64_t diag_count = 0;
  int32_t ctrl_gear = -1;
  int32_t ctrl_speed_mmps = -1;
  int32_t ctrl_steer_cdeg = 0;
  int32_t ctrl_mode = -1;
  int32_t ctrl_alive = -1;
  int32_t diag_fault_level = -1;
  int32_t diag_auto_can = -1;
  int32_t hard_fault = 1;
  int32_t ctrl_checksum = 0;
  int32_t diag_checksum = 0;
  int32_t ctrl_alive_valid = 0;
  int32_t diag_alive_valid = 0;
  int32_t ctrl_delta = -1;
  int32_t diag_delta = -1;
  double ctrl_interval = -1.;
  double diag_interval = -1.;
};
static_assert(sizeof(FeedbackSnapshot) == 120, "feedback ABI changed");

struct AliveObserver
{
  bool initialized = false;
  bool valid = true;
  int last = -1;
  double stamp = 0.;
  uint64_t count = 0;
  int delta = -1;
  double interval = -1.;

  bool observe(int value, double now, bool checksum_valid)
  {
    interval = initialized ? now - stamp : -1.;
    delta = initialized ? ((value - last) & 0xF) : -1;
    if (!checksum_valid ||
      (initialized && (interval <= 0. || interval > kFeedbackFreshnessSec)) ||
      (initialized && (delta == 0 || delta > 5)))
    {
      valid = false;
      return false;
    }
    initialized = true;
    last = value;
    stamp = now;
    ++count;
    return true;
  }
};

class SharedFeedback
{
public:
  explicit SharedFeedback(const std::string & path) : path_(path)
  {
    unlink(path_.c_str());
    fd_ = open(path_.c_str(), O_RDWR | O_CREAT | O_EXCL | O_CLOEXEC, 0600);
    if (fd_ < 0 || ftruncate(fd_, sizeof(FeedbackSnapshot)) < 0) {
      throw std::runtime_error("feedback mmap file");
    }
    void * address = mmap(
      nullptr, sizeof(FeedbackSnapshot), PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
    if (address == MAP_FAILED) throw std::runtime_error("feedback mmap");
    data_ = static_cast<FeedbackSnapshot *>(address);
    new (data_) FeedbackSnapshot();
  }

  ~SharedFeedback()
  {
    if (data_ != nullptr) munmap(data_, sizeof(FeedbackSnapshot));
    if (fd_ >= 0) close(fd_);
    unlink(path_.c_str());
  }

  FeedbackSnapshot & begin_update()
  {
    __atomic_add_fetch(&data_->sequence, uint64_t(1), __ATOMIC_RELEASE);
    return *data_;
  }

  void end_update()
  {
    __atomic_add_fetch(&data_->sequence, uint64_t(1), __ATOMIC_RELEASE);
  }

private:
  std::string path_;
  int fd_ = -1;
  FeedbackSnapshot * data_ = nullptr;
};

SendResult send_can(int fd, Command command, uint8_t & alive)
{
  if (!command.allowed || mono() - command.received > kCommandTtlSec) command = Command{};
  uint64_t word = (uint64_t(command.gear & 0xF)) |
    (uint64_t(command.speed_mmps & 0xFFFF) << 4) |
    (uint64_t(command.steer_cdeg & 0xFFFF) << 20) |
    (uint64_t(alive++ & 0xF) << 52);
  can_frame frame{};
  frame.can_id = kCtrlCmdId | CAN_EFF_FLAG;
  frame.can_dlc = 8;
  for (int i = 0; i < 7; ++i) frame.data[i] = (word >> (8 * i)) & 0xFF;
  frame.data[7] = checksum(frame.data);
  if (write(fd, &frame, sizeof(frame)) != sizeof(frame)) {
    throw std::runtime_error("SocketCAN TX failed");
  }
  return {mono(), command};
}

int can_socket(const std::string & iface)
{
  int fd = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
  if (fd < 0) throw std::runtime_error("CAN socket");
  can_filter filters[2] = {
    {kCtrlFbId | CAN_EFF_FLAG, CAN_EFF_MASK | CAN_EFF_FLAG},
    {kDiagFbId | CAN_EFF_FLAG, CAN_EFF_MASK | CAN_EFF_FLAG},
  };
  if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, filters, sizeof(filters)) < 0) {
    throw std::runtime_error("CAN filter");
  }
  ifreq ifr{};
  std::strncpy(ifr.ifr_name, iface.c_str(), IFNAMSIZ - 1);
  if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) throw std::runtime_error("CAN interface");
  sockaddr_can address{};
  address.can_family = AF_CAN;
  address.can_ifindex = ifr.ifr_ifindex;
  if (bind(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
    throw std::runtime_error("CAN bind");
  }
  return fd;
}

void receive_feedback(
  int fd, SharedFeedback & shared, AliveObserver & ctrl_alive, AliveObserver & diag_alive)
{
  can_frame frame{};
  while (read(fd, &frame, sizeof(frame)) == sizeof(frame)) {
    const uint32_t id = frame.can_id & CAN_EFF_MASK;
    const double stamp = mono();
    const bool shape_valid = (frame.can_id & CAN_EFF_FLAG) != 0 && frame.can_dlc == 8;
    const bool checksum_valid = shape_valid && frame.data[7] == checksum(frame.data);
    const uint64_t word = payload_word(frame);
    if (id == kCtrlFbId) {
      const int alive = bits(word, 52, 4);
      ctrl_alive.observe(alive, stamp, checksum_valid);
      FeedbackSnapshot & data = shared.begin_update();
      data.ctrl_stamp = stamp;
      data.ctrl_count = ctrl_alive.count;
      data.ctrl_gear = bits(word, 0, 4);
      data.ctrl_speed_mmps = bits(word, 4, 16);
      data.ctrl_steer_cdeg = signed16(bits(word, 20, 16));
      data.ctrl_mode = bits(word, 44, 2);
      data.ctrl_alive = alive;
      data.ctrl_checksum = checksum_valid ? 1 : 0;
      data.ctrl_alive_valid = ctrl_alive.valid ? 1 : 0;
      data.ctrl_delta = ctrl_alive.delta;
      data.ctrl_interval = ctrl_alive.interval;
      shared.end_update();
    } else if (id == kDiagFbId) {
      const int alive = bits(word, 52, 4);
      diag_alive.observe(alive, stamp, checksum_valid);
      const int eps = bits(word, 8, 12);
      const int left_drive = bits(word, 32, 6);
      const int right_drive = bits(word, 38, 6);
      const bool bms_loss = bits(word, 44, 1) != 0;
      const bool estop = bits(word, 45, 1) != 0;
      FeedbackSnapshot & data = shared.begin_update();
      data.diag_stamp = stamp;
      data.diag_count = diag_alive.count;
      data.diag_fault_level = bits(word, 0, 4);
      data.diag_auto_can = bits(word, 4, 1);
      data.hard_fault =
        (eps != 0 || left_drive != 0 || right_drive != 0 || bms_loss || estop) ? 1 : 0;
      data.diag_checksum = checksum_valid ? 1 : 0;
      data.diag_alive_valid = diag_alive.valid ? 1 : 0;
      data.diag_delta = diag_alive.delta;
      data.diag_interval = diag_alive.interval;
      shared.end_update();
    }
  }
  if (errno != EAGAIN && errno != EWOULDBLOCK) {
    throw std::runtime_error("SocketCAN RX failed");
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 5 || std::string(argv[1]) != "--interface") {
    std::cerr << "usage: --interface can0 <control-socket> <feedback-mmap>\n";
    return 2;
  }
  const std::string iface = argv[2];
  const std::string control = argv[3];
  const std::string feedback_path = argv[4];
  unlink(control.c_str());
  try {
    prctl(PR_SET_PDEATHSIG, SIGTERM);
    if (getppid() == 1) return 1;
    SharedFeedback shared(feedback_path);
    AliveObserver ctrl_alive;
    AliveObserver diag_alive;
    int cfd = socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    sockaddr_un control_address{};
    control_address.sun_family = AF_UNIX;
    std::strncpy(
      control_address.sun_path, control.c_str(), sizeof(control_address.sun_path) - 1);
    if (bind(cfd, reinterpret_cast<sockaddr *>(&control_address), sizeof(control_address)) < 0) {
      throw std::runtime_error("control bind");
    }
    int fd = can_socket(iface);
    Command command{};
    uint8_t alive = 0;
    sockaddr_un peer{};
    socklen_t peer_length = sizeof(peer);
    bool have_peer = false;
    double previous = 0., sum = 0., maximum = 0.;
    uint64_t count = 0;
    int tfd = timerfd_create(CLOCK_MONOTONIC, TFD_CLOEXEC);
    itimerspec spec{};
    spec.it_value.tv_nsec = 10000000;
    spec.it_interval.tv_nsec = 10000000;
    timerfd_settime(tfd, 0, &spec, nullptr);
    while (true) {
      fd_set descriptors;
      FD_ZERO(&descriptors);
      FD_SET(cfd, &descriptors);
      FD_SET(tfd, &descriptors);
      FD_SET(fd, &descriptors);
      int selected = select(std::max({cfd, tfd, fd}) + 1, &descriptors, nullptr, nullptr, nullptr);
      if (selected < 0 && errno == EINTR) continue;
      if (selected < 0) throw std::runtime_error("select");
      if (FD_ISSET(fd, &descriptors)) receive_feedback(fd, shared, ctrl_alive, diag_alive);
      if (FD_ISSET(cfd, &descriptors)) {
        char buffer[128]{};
        peer_length = sizeof(peer);
        ssize_t length = recvfrom(
          cfd, buffer, sizeof(buffer) - 1, 0,
          reinterpret_cast<sockaddr *>(&peer), &peer_length);
        if (length > 0) {
          have_peer = true;
          if (std::string(buffer, length) == "STOP") {
            send_can(fd, Command{}, alive);
            break;
          }
          int gear, speed, steer, allowed;
          if (sscanf(buffer, "CMD %d %d %d %d", &gear, &speed, &steer, &allowed) == 4) {
            command = {gear, speed, steer, allowed != 0, mono()};
          }
        }
      }
      if (FD_ISSET(tfd, &descriptors)) {
        uint64_t expirations;
        read(tfd, &expirations, sizeof(expirations));
        double due = mono();
        double interval = previous == 0. ? 0. : due - previous;
        if (previous != 0. && interval > .030) {
          send_can(fd, Command{}, alive);
          throw std::runtime_error("native heartbeat exceeded 30 ms");
        }
        SendResult result = send_can(fd, command, alive);
        double sent = result.stamp;
        interval = previous == 0. ? 0. : sent - previous;
        previous = sent;
        if (interval > 0.) {
          sum += interval;
          maximum = std::max(maximum, interval);
        }
        ++count;
        if (have_peer) {
          char status[256];
          int length = snprintf(
            status, sizeof(status), "STAT %.9f %.9f %llu %.9f %.9f %d %d %d %d",
            sent, interval, static_cast<unsigned long long>(count),
            count > 1 ? sum / (count - 1) : 0., maximum, result.command.gear,
            result.command.speed_mmps, result.command.steer_cdeg,
            result.command.allowed ? 1 : 0);
          sendto(
            cfd, status, length, MSG_DONTWAIT,
            reinterpret_cast<sockaddr *>(&peer), peer_length);
        }
      }
    }
    close(fd);
    close(tfd);
    close(cfd);
    unlink(control.c_str());
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << "\n";
    unlink(control.c_str());
    unlink(feedback_path.c_str());
    return 1;
  }
}
