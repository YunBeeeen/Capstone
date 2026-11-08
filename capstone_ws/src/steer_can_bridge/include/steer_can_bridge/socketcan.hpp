// Linux SocketCAN raw 소켓 (논블로킹). 확장 29-bit 프레임만 주고받음 (RS02 private 프로토콜).
#pragma once

#include <optional>
#include <string>

#include "steer_can_bridge/rs02.hpp"

namespace steer_can {

class SocketCan {
 public:
  explicit SocketCan(const std::string& ifname);  // 실패 시 std::runtime_error
  ~SocketCan();
  SocketCan(const SocketCan&) = delete;
  SocketCan& operator=(const SocketCan&) = delete;

  // false = 송신 실패 (errno 는 last_errno). ENOBUFS = 송신 큐 가득 → 버스에서 ACK 하는 노드 없음
  bool send(const rs02::Frame& f);
  // 쌓인 프레임 1개 (없으면 nullopt). 표준/에러/RTR 프레임은 건너뜀
  std::optional<rs02::Frame> recv();

  int last_errno() const { return last_errno_; }
  const std::string& ifname() const { return ifname_; }

 private:
  int fd_ = -1;
  int last_errno_ = 0;
  std::string ifname_;
};

}  // namespace steer_can
