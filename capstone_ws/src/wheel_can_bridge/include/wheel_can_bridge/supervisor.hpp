// 바퀴 4축 상태기계 — ROS·소켓 무관 (시간 now[s] 를 받아 보낼 프레임을 돌려줌 → gtest 로 검증).
//
// 위상(phase)
//   FREE     : 전 축 IDLE, 명령 대기. DRIVE/HOLD 명령이 오고 모든 바퀴 heartbeat 가 살아 있으면 ENTERING
//   ENTERING : 속도 0 으로 CLOSED_LOOP 진입 요청 (retry_period 마다 재요청).
//              enter 시작 이후의 heartbeat 로 전 축 CLOSED_LOOP 확인 → ACTIVE, enter_timeout 넘기면 FAULT
//   ACTIVE   : 매 틱 속도 송신 (DRIVE = 명령, HOLD 또는 명령 끊김(cmd_timeout) = 0). 한 축이라도 폐루프 이탈 → FAULT
//   FAULT    : 원인 래치, 전 축 속도 0 + IDLE. clear_errors() 로만 해제 → FREE (새 명령 필요)
//
// 고장 조건: heartbeat 끊김(hb_timeout) / axis_error ≠ 0 / 폐루프 이탈 / 진입 실패.
// 스워브는 바퀴 하나만 서도 차체가 끌리므로 한 축 고장 = 전 축 정지.
#pragma once

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "wheel_can_bridge/cansimple.hpp"

namespace wheel_can {

enum class Mode : uint8_t { FREE = 0, DRIVE = 1, HOLD = 2 };     // = capstone_msgs/WheelCommand MODE_*
enum class Phase : uint8_t { FREE = 0, ENTERING = 1, ACTIVE = 2, FAULT = 3 };  // = WheelStateArray PHASE_*

struct SupervisorConfig {
  std::vector<int> can_ids{1, 2, 3, 4};
  std::vector<double> sign{1.0, 1.0, 1.0, 1.0};  // 바퀴 기준 → 보드 기준 속도 부호
  double vel_max = 2.0;        // [turn/s] 보드 vel_limit 과 맞춤
  double cmd_timeout = 0.2;    // [s] DRIVE 명령이 이보다 오래되면 속도 0 (HOLD)
  double hb_timeout = 0.5;     // [s] heartbeat 100 ms 의 5배
  double enter_timeout = 3.0;  // [s] CLOSED_LOOP 진입 제한시간
  double retry_period = 0.5;   // [s] 상태 전환 재요청 주기
};

struct WheelData {
  int can_id = 0;
  bool seen = false;
  double hb_time = -std::numeric_limits<double>::infinity();
  uint32_t axis_state = 0;
  uint32_t axis_error = 0;
  // 보드 기준 값 (부호 미적용). 아직 응답 없으면 NaN
  float pos = std::numeric_limits<float>::quiet_NaN();
  float vel = std::numeric_limits<float>::quiet_NaN();
  float iq_set = std::numeric_limits<float>::quiet_NaN();
  float iq_meas = std::numeric_limits<float>::quiet_NaN();
};

class Supervisor {
 public:
  explicit Supervisor(SupervisorConfig cfg);

  // 수신 프레임 반영 (heartbeat / encoder / iq / vbus 응답)
  void on_frame(const cs::Frame& f, double now);

  // 명령 갱신. 잘못된 명령이면 이유 문자열 (무시됨), 정상이면 "".
  // velocity = 바퀴 기준 [turn/s], DRIVE 에서만 길이 = 바퀴 수
  std::string set_command(Mode mode, const std::vector<double>& velocity, double now);

  // 제어 주기마다 호출 → 보낼 프레임
  std::vector<cs::Frame> step(double now);

  // 피드백 RTR 요청 (살아 있는 바퀴의 encoder + Iq, 첫 바퀴 vbus). FREE 에선 Iq 생략 (IDLE = 전류 0)
  std::vector<cs::Frame> feedback_requests(double now) const;

  // 보드 에러 클리어 + FAULT 해제 → FREE
  std::vector<cs::Frame> clear_errors(double now);

  // 종료 시: 전 축 속도 0 + IDLE
  std::vector<cs::Frame> shutdown_frames() const;

  // 지금 실제로 적용 중인 모드 (DRIVE 명령이 cmd_timeout 넘게 끊기면 HOLD)
  Mode effective_mode(double now) const {
    return (mode_ == Mode::DRIVE && now - cmd_time_ > cfg_.cmd_timeout) ? Mode::HOLD : mode_;
  }
  Phase phase() const { return phase_; }
  const std::string& fault() const { return fault_; }
  bool alive(size_t i, double now) const;
  bool ready(double now) const;
  const std::vector<WheelData>& wheels() const { return wheels_; }
  const SupervisorConfig& config() const { return cfg_; }
  float vbus() const { return vbus_; }

 private:
  void latch(const std::string& why, double now, std::vector<cs::Frame>& out);
  void push_all_stop(std::vector<cs::Frame>& out) const;
  int index_of(int can_id) const;

  SupervisorConfig cfg_;
  std::vector<WheelData> wheels_;
  Phase phase_ = Phase::FREE;
  std::string fault_;
  Mode mode_ = Mode::FREE;
  std::vector<double> vel_cmd_;
  double cmd_time_ = -std::numeric_limits<double>::infinity();
  double enter_start_ = 0.0;
  double last_req_ = -std::numeric_limits<double>::infinity();
  double clear_time_ = -std::numeric_limits<double>::infinity();
  float vbus_ = std::numeric_limits<float>::quiet_NaN();
};

}  // namespace wheel_can
