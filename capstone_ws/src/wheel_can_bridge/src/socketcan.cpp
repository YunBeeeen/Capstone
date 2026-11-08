#include "wheel_can_bridge/socketcan.hpp"

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace wheel_can {

SocketCan::SocketCan(const std::string& ifname) : ifname_(ifname) {
  fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ < 0) throw std::runtime_error(std::string("CAN socket(): ") + std::strerror(errno));

  ifreq ifr{};
  std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
  if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
    const int e = errno;
    ::close(fd_);
    throw std::runtime_error("[" + ifname + "] 인터페이스 없음 (" + std::strerror(e) +
                             ") — USB-CAN 연결, `ip -br link` 확인");
  }
  sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    const int e = errno;
    ::close(fd_);
    throw std::runtime_error("[" + ifname + "] bind 실패: " + std::strerror(e));
  }
  ::fcntl(fd_, F_SETFL, ::fcntl(fd_, F_GETFL) | O_NONBLOCK);
}

SocketCan::~SocketCan() {
  if (fd_ >= 0) ::close(fd_);
}

bool SocketCan::send(const cs::Frame& f) {
  can_frame cf{};
  cf.can_id = f.id & CAN_SFF_MASK;
  if (f.rtr) cf.can_id |= CAN_RTR_FLAG;
  cf.can_dlc = f.len;
  if (!f.rtr) std::memcpy(cf.data, f.data.data(), f.len);
  const ssize_t n = ::write(fd_, &cf, sizeof(cf));
  if (n != static_cast<ssize_t>(sizeof(cf))) {
    last_errno_ = errno;
    return false;
  }
  return true;
}

std::optional<cs::Frame> SocketCan::recv() {
  while (true) {
    can_frame cf{};
    const ssize_t n = ::read(fd_, &cf, sizeof(cf));
    if (n != static_cast<ssize_t>(sizeof(cf))) {
      if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) last_errno_ = errno;
      return std::nullopt;
    }
    if (cf.can_id & (CAN_EFF_FLAG | CAN_ERR_FLAG)) continue;
    cs::Frame f;
    f.id = cf.can_id & CAN_SFF_MASK;
    f.rtr = (cf.can_id & CAN_RTR_FLAG) != 0;
    f.len = cf.can_dlc > 8 ? 8 : cf.can_dlc;
    if (!f.rtr) std::memcpy(f.data.data(), cf.data, f.len);
    return f;
  }
}

}  // namespace wheel_can
