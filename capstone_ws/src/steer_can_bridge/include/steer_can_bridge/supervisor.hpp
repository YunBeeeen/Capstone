// 조향 4축 (RS02, MIT 위치 제어) 상태기계 — ROS·소켓 무관 (시간 now[s] 를 받아 보낼 프레임을 돌려줌 → gtest).
//
// 위상(phase)
//   FREE     : 전 축 토크 끔. free_poll_hz 로 Type04 를 보내 응답(Type02)으로 각도만 읽음.
//              POSITION 명령이 오고 전 축 alive + 각도 확정 + 고장 없음이면 ENTERING
//   ENTERING : (canTimeout 설정 +) Type03 enable, retry_period 마다 재요청. RUN 이 확인된 축은
//              그 순간 실제각을 명령 시작점으로 잡고 MIT 로 유지. 전 축 RUN → ACTIVE, enter_timeout 넘기면 FAULT
//   ACTIVE   : 매 틱 MIT (p = 램프 위치, v = 램프 속도 피드포워드, kp, kd, t = 0).
//              램프 = 목표를 향해 max_rate [rad/s] 로 이동. 명령 끊김(cmd_timeout) = 램프 멈추고 그 자리 유지
//   FAULT    : 원인 래치, 전 축 토크 끔 (Type04 폴링으로 각도는 계속 읽음). clear_errors() 로만 해제 → FREE
//
// 고장 조건 (ENTERING/ACTIVE 에서): 피드백 끊김(fb_timeout) / fault_bits ≠ 0 / Type21 / ACTIVE 중 RUN 이탈
// (밖에서 stop 한 경우 — 다시 enable 하지 않음) / 진입 실패. 한 축 고장 = 전 축 토크 끔.
//
// 각도 확정: RS02 는 켤 때 한 바퀴 안의 각도만 앎 (zero_sta=1 → −π~π). 기구 한계 ±190° 라 실제 +185° 는
// −175° 로 읽힘. 접힌 값은 항상 |θ| ≥ 360° − 190° = 170° 이므로 |읽은 값| < confirm_limit(165°) 면 확실.
// 한 번 확정되면 피드백이 이어지는 동안 유지 (모터가 회전 수를 셈). 피드백이 끊겼다 오면 다시 판정.
#pragma once

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "steer_can_bridge/rs02.hpp"

namespace steer_can {

enum class Mode : uint8_t { FREE = 0, POSITION = 1 };                         // = SteerCommand MODE_*
enum class Phase : uint8_t { FREE = 0, ENTERING = 1, ACTIVE = 2, FAULT = 3 };  // = SteerStateArray PHASE_*

struct SupervisorConfig {
  std::vector<int> can_ids{1, 2, 3, 4};
  std::vector<double> sign{1.0, 1.0, 1.0, 1.0};  // 조향각 = sign × 모터각 (+ = 위에서 보고 반시계)
  uint8_t host_id = 0xFD;
  double steer_limit = 180.0 * M_PI / 180.0;     // [rad] 명령 클램프 (swerve steer_limit_deg 와 같게, 스토퍼 ±190° 까지 여유 10°)
  double confirm_limit = 165.0 * M_PI / 180.0;   // [rad] < 360° − 기구 한계(190°) − 여유
  double kp = 20.0;                              // [Nm/rad]
  double kd = 2.0;                               // [Nm·s/rad]
  double max_rate = 2.0;                         // [rad/s] 램프 속도
  double cmd_timeout = 0.2;                      // [s] POSITION 명령 끊기면 그 자리 유지
  double fb_timeout = 0.15;                      // [s] 피드백 끊김 판정
  double free_poll_period = 0.05;                // [s] FREE/FAULT 에서 Type04 폴링 주기
  double enter_timeout = 1.0;                    // [s]
  double retry_period = 0.2;                     // [s] enable 재요청
  double motor_can_timeout = 0.5;                // [s] 모터 canTimeout (브리지 죽으면 모터가 스스로 disable). 0 = 끔
};

struct MotorData {
  int can_id = 0;
  bool seen = false;
  double fb_time = -std::numeric_limits<double>::infinity();
  bool confirmed = false;
  bool running = false;     // ENTERING 이후 RUN 확인 + 명령 시작점 잡음
  uint8_t mode_status = 0;
  uint8_t fault_bits = 0;
  uint32_t fault21 = 0;     // 마지막 Type21 fault 값
  // 조향 기준 (sign 적용). 아직 응답 없으면 NaN
  double angle = std::numeric_limits<double>::quiet_NaN();
  double vel = std::numeric_limits<double>::quiet_NaN();
  double tor = std::numeric_limits<double>::quiet_NaN();
  double temp = std::numeric_limits<double>::quiet_NaN();
  double angle_cmd = std::numeric_limits<double>::quiet_NaN();  // 램프 위치
};

class Supervisor {
 public:
  explicit Supervisor(SupervisorConfig cfg);

  void on_frame(const rs02::Frame& f, double now);

  // 명령 갱신. 잘못된 명령이면 이유 문자열 (무시됨), 정상이면 "". angle = 조향 기준 [rad], POSITION 에서만 길이 4
  std::string set_command(Mode mode, const std::vector<double>& angle, double now);

  // 제어 주기마다 호출 → 보낼 프레임
  std::vector<rs02::Frame> step(double now);

  // 모터 고장 클리어 (Type04 clear_fault) + FAULT 해제 → FREE
  std::vector<rs02::Frame> clear_errors(double now);

  // 종료 시: 전 축 토크 끔
  std::vector<rs02::Frame> shutdown_frames() const;

  Phase phase() const { return phase_; }
  const std::string& fault() const { return fault_; }
  bool alive(size_t i, double now) const;
  bool ready(double now) const;
  // FREE 에서 POSITION 명령이 왔는데 진입 못 하는 이유 ("" = 없음) — 노드가 경고로 띄움
  const std::string& blocked() const { return blocked_; }
  bool cmd_fresh(double now) const { return now - cmd_time_ <= cfg_.cmd_timeout; }
  const std::vector<MotorData>& motors() const { return motors_; }
  const SupervisorConfig& config() const { return cfg_; }

 private:
  void latch(const std::string& why, std::vector<rs02::Frame>& out);
  void push_all_stop(std::vector<rs02::Frame>& out, bool clear_fault = false) const;
  void push_mit(size_t i, double v_ff, std::vector<rs02::Frame>& out) const;
  std::string unhealthy(double now) const;  // 첫 문제 축 설명 ("" = 전 축 정상)
  int index_of(int can_id) const;

  SupervisorConfig cfg_;
  std::vector<MotorData> motors_;
  Phase phase_ = Phase::FREE;
  std::string fault_;
  std::string blocked_;
  Mode mode_ = Mode::FREE;
  std::vector<double> target_;
  double cmd_time_ = -std::numeric_limits<double>::infinity();
  double enter_start_ = 0.0;
  double last_req_ = -std::numeric_limits<double>::infinity();
  double last_poll_ = -std::numeric_limits<double>::infinity();
  double last_step_ = std::numeric_limits<double>::quiet_NaN();
};

std::string axis_name(size_t i);  // 0 → "1 FR" ...

}  // namespace steer_can
