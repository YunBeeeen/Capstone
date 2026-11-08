// 4모듈 스워브 기구학 — ROS 무관 (gtest 로 검증).
//
// 출처: ksrc 로버 펌웨어 firmware/common/{swerve_kinematics,drive_align_gate}.c 를 C++ 로 옮김.
//       로직·상수·테스트 케이스는 원본과 같음 (float → double 만 다름).
//
// 전제: 조향축이 구동 바퀴 중심(접지점) 바로 위 — 캐스터 오프셋 없음. 그래서 조향각 = 바퀴 속도 방향.
//
// 좌표 (REP-103): 차체 x = 전진, y = 좌측, omega + = 위에서 보고 반시계.
// 조향각 0 = 바퀴 전방, + = 반시계. 바퀴 속도 + = 조향각 방향으로 굴러감.
// 모듈 순서 = [1 FR, 2 RR, 3 RL, 4 FL] (WheelCommand / SteerCommand 와 같음).
#pragma once

#include <array>
#include <cstddef>

namespace swerve {

constexpr std::size_t N = 4;
using Arr = std::array<double, N>;

struct ModulePos {
  double x = 0.0;  // [m] 회전중심 기준, + 전진
  double y = 0.0;  // [m] + 좌측
};
using Modules = std::array<ModulePos, N>;

// 모듈별 이력. 0 으로 시작하거나 seed() 로 실제 조향각에서 시작.
struct ModuleState {
  double last_angle = 0.0;  // IK 가 낸 연속각 (wrap 안 함)
  double last_cmd = 0.0;    // 가동범위로 접어서 마지막으로 내보낸 각
  bool initialized = false;
};
using States = std::array<ModuleState, N>;

struct ModuleCmd {
  double angle = 0.0;  // [rad]
  double speed = 0.0;  // [m/s] 부호 있음 — 180° 조향 대신 부호 반전
};
using Cmds = std::array<ModuleCmd, N>;

struct Twist2D {
  double vx = 0.0, vy = 0.0, omega = 0.0;
};

// 차체 속도 → 모듈별 (연속 조향각, 속도).
//  - 최단경로: 직전 각에서 90° 넘게 돌아야 하면 180° 뒤집고 속도 부호 반전
//  - 데드밴드: 모듈 속도 < 1 mm/s 면 조향각 유지, 속도 0
//  - 저속 조향 유지: 모듈 속도 < min_steer_mps 면 조향각을 바꾸지 않고, 그 속도의 현재 바퀴 방향 성분만 굴림.
//    스틱을 놓을 때 튀는 1~2 cm/s 짜리 잡음 벡터가 조향 목표(최대 90°)가 되는 걸 막음. <= 0 이면 끔 (원본 동작)
//  - desaturation: 어느 바퀴든 max_wheel_mps 초과면 전부 같은 비율로 축소 (운동 모양 보존). <= 0 이면 끔
void ik_compute(double vx, double vy, double omega, const Modules& modules, double max_wheel_mps,
                States& state, Cmds& out, double min_steer_mps = 0.0);

// 조향 가동범위 [lo[i], hi[i]] 안으로 접기 (θ ↔ θ+nπ, n 홀수면 속도 부호 반전). out 제자리 수정.
//  - 직전 명령각에 가장 가까운 등가각 선택. 폭 >= π 면 항상 범위 안, 폭 < π 면 경계로 클램프
//  - unwind: |각 - 범위 중앙| > unwind_rad 이고 |속도| < slow_mps 면 중앙 쪽 등가각으로 미리 넘어감
//    (구동 안 할 때 180° 돌리는 건 공짜 → 이음매(±한계)에서 주행 중 강제 반전을 줄임). unwind_rad <= 0 이면 끔
void fold_to_range(const Arr& lo, const Arr& hi, double unwind_rad, double slow_mps, States& state,
                   Cmds& out);

// 정기구학 (오도메트리용): 모듈별 (조향각, 바퀴 속도) → 차체 속도, 최소제곱.
// 미끄러짐이 없으면 정확, 있으면 8개 식에 가장 잘 맞는 강체 운동.
Twist2D fk_compute(const Modules& modules, const Arr& angle, const Arr& speed);

// 구동 정렬 게이트: 큰 방향 전환 때 조향이 맞을 때까지 구동을 0 으로 보류.
//  1) 목표각이 직전 목표에서 trigger 넘게 바뀌고 구동 명령이 있으면 대기 (첫 구동 명령은 무조건 대기)
//  2) 실측 트리거: 구동 중 |목표 - 실제| > trigger 면 대기 (조금씩 돌려서 1) 을 피한 경우)
//  3) 전 바퀴 |목표 - 실제| <= release 면 해제. 실제각을 모르면(actual_ok=false) 해제 안 함
//  4) max_hold_ticks 넘으면 강제 해제 + timed_out (다음 목표 급변까지 2) 억제). 0 = 무한 대기
// 각도는 wrap 하지 않고 뺌 (+179 → −179 는 조향이 358° 돌아야 하므로).
class AlignGate {
 public:
  AlignGate(double trigger_rad = 0.0, double release_rad = 0.0, unsigned max_hold_ticks = 0);
  void reset();
  bool update_goal(const Arr& goal, const Arr& speed);  // 반환: 대기 중이면 true
  bool allow(const Arr& goal, const Arr& actual, bool actual_ok);  // 반환: 구동 허용
  bool pending() const { return pending_; }
  bool timed_out() const { return timed_out_; }
  unsigned hold_ticks() const { return hold_ticks_; }

 private:
  double trigger_, release_;
  unsigned max_hold_;
  Arr prev_goal_{};
  bool has_prev_ = false, pending_ = false, driving_ = false, timed_out_ = false;
  unsigned hold_ticks_ = 0;
};

// 위를 한 제어주기로 묶음: IK → 가동범위 접기 → (큰 조향 전환이면 먼저 정지) → 정렬 게이트.
//
// 정지 후 조향 (flip_stop): 목표 조향각이 직전에 내보낸 각에서 flip_stop_rad 넘게 바뀌는데 바퀴가 아직 구르면
// (|바퀴 속도| > stop_speed_mps), 조향은 직전 각에 붙잡고 구동만 0 → 바퀴가 실제로 멈추면 새 각으로 돌림.
// 이유: 구동은 보드 램프(0.66 m/s²)로 천천히 서는데 조향은 0.2 s 만에 180° 돌아버림 → 구르는 바퀴가 옆으로
// 끌리거나 반대로 밀어 차체가 넘어짐 (시뮬 0.5 m/s 에서 ±170° 이음매 반전 때 전복 재현, 2026-11-07).
// unwind 도 같은 이유로 바퀴가 실제로 멈췄을 때만.
// 바퀴 속도 피드백이 없으면 est_decel_mps2 로 감속을 추정 (보수적으로 길게 기다림).
//
// 멈춰 있을 때 조향을 돌리지 않음 (2026-11-07):
//  - 명령이 0 (전 바퀴 속도 0) 이면 조향은 지금 보낸 각 그대로, IK 이력도 그 각으로 맞춤 → 정지 대기 중 미뤄 둔
//    목표는 취소. (전엔 스틱 잡음이 남긴 목표를 바퀴가 선 뒤 실행 → 명령 0 인데 0° → 180° 등가각으로 돌았음)
//  - 정지 대기 중엔 매 틱 붙잡은 각을 기준으로 목표를 다시 계산 → 실제 각에 가장 가까운 등가각을 고름.
struct ControllerConfig {
  Modules modules{};
  Arr lo{}, hi{};               // [rad] 조향 운용 한계
  double max_wheel_mps = 0.6;
  double min_steer_mps = 0.0;   // 이보다 느린 모듈은 조향 유지 (ik_compute 참고). <= 0 이면 끔
  double unwind_rad = 0.0;      // <= 0 이면 끔 (운용 기본: 끔 — 멈춰 있는 동안 명령 없이 조향이 돎)
  double slow_mps = 0.0;
  double gate_trigger_rad = 0.0;  // <= 0 이면 게이트 끔
  double gate_release_rad = 0.0;
  unsigned gate_max_hold_ticks = 0;
  double flip_stop_rad = 0.0;     // <= 0 이면 정지 후 조향 끔
  double stop_speed_mps = 0.03;   // 이보다 느리면 "멈춤"
  double est_decel_mps2 = 0.66;   // 피드백 없을 때 바퀴 속도 추정용 (보드 vel_ramp_rate 2 turn/s²)
  double dt = 0.02;               // [s] 제어 주기
  // 조향 명령 회전 속도 제한 [rad/s]. 목표각을 한 번에 보내지 않고 이 속도로 옮겨 감 (<= 0 이면 끔 = 계단 명령).
  // 큰 전환(정지 후 180° 반전, unwind)에서 조향이 "확" 도는 것 방지. 정렬 게이트는 최종 목표 기준이라 다 돌 때까지 구동 보류.
  double max_steer_rate = 0.0;
};

struct StepResult {
  Cmds cmd;              // 내보낼 조향각 + 바퀴 속도 (게이트 적용 전)
  Arr wheel_speed{};     // [m/s] 게이트·정지 적용 후 실제로 보낼 속도
  bool gated = false;    // 구동이 막혔는가 (정렬 대기 또는 정지 중)
  bool stopping = false; // 큰 조향 전환 전에 멈추는 중 (조향은 직전 각 유지)
};

class Controller {
 public:
  explicit Controller(ControllerConfig cfg);
  // 이력을 실제 조향각으로 초기화 (FREE → 활성 전환 때. 손으로 돌려 놓은 각에서 이어가도록)
  void seed(const Arr& actual_angle);
  void reset();  // 이력 0, 게이트 초기화
  // wheel_speed_meas: 바퀴 실제 속도 [m/s] (wheel_state). wheel_ok=false 면 추정값 사용
  StepResult step(const Twist2D& cmd, const Arr& actual_angle, bool actual_ok, const Arr& wheel_speed_meas = Arr{},
                  bool wheel_ok = false);
  const ControllerConfig& config() const { return cfg_; }
  const AlignGate& gate() const { return gate_; }

 private:
  void sync_history_to_sent();  // IK 이력(연속각·접은 각)을 직전에 보낸 각으로
  ControllerConfig cfg_;
  States state_{};
  AlignGate gate_;
  Arr sent_{};              // 직전에 내보낸 조향각
  bool has_sent_ = false;
  bool was_stopping_ = false;  // 직전 틱이 정지 대기였나
  double est_speed_ = 0.0;  // 피드백 없을 때 추정 바퀴 속도 [m/s]
};

}  // namespace swerve
