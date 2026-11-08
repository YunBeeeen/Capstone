// ksrc firmware/common/swerve_kinematics.c + drive_align_gate.c 의 C++ 판. 규칙은 kinematics.hpp 참고.
#include "swerve_controller/kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace swerve {

namespace {
constexpr double PI = 3.14159265358979323846;
constexpr double VEL_EPS_MPS = 0.001;    // 이보다 작은 모듈 속도는 0 으로 봄 → atan2 노이즈를 쫓지 않음
constexpr double FOLD_EPS_RAD = 1e-4;    // 경계를 반올림 수준만 넘은 건 범위 안으로 봄 (옆걸음 ±90 토글 방지)
constexpr double GATE_SPEED_EPS = 1e-6;  // "구동 명령 있음" 판정

double wrap_to_pi(double a) {
  a = std::fmod(a + PI, 2.0 * PI);
  if (a < 0.0) a += 2.0 * PI;
  return a - PI;
}
}  // namespace

void ik_compute(double vx, double vy, double omega, const Modules& modules, double max_wheel_mps,
                States& state, Cmds& out, double min_steer_mps) {
  double max_abs = 0.0;
  for (std::size_t i = 0; i < N; ++i) {
    // 강체 점속도 v_i = v + ω × r_i = (vx − ω·y_i, vy + ω·x_i)
    const double vxi = vx - omega * modules[i].y;
    const double vyi = vy + omega * modules[i].x;
    const double sp = std::hypot(vxi, vyi);

    if (sp < VEL_EPS_MPS) {
      if (!state[i].initialized) {
        state[i].last_angle = 0.0;
        state[i].initialized = true;
      }
      out[i] = {state[i].last_angle, 0.0};
      continue;
    }
    if (sp < min_steer_mps) {  // 저속: 조향 유지, 현재 바퀴 방향 성분만
      if (!state[i].initialized) {
        state[i].last_angle = 0.0;
        state[i].initialized = true;
      }
      const double a = state[i].last_angle;
      const double proj = vxi * std::cos(a) + vyi * std::sin(a);
      out[i] = {a, proj};
      max_abs = std::max(max_abs, std::fabs(proj));
      continue;
    }

    double delta = wrap_to_pi(std::atan2(vyi, vxi) - state[i].last_angle);
    double speed = sp;
    if (std::fabs(delta) > PI / 2) {  // 90° 넘게 돌 바엔 반대로 보고 거꾸로 굴림
      delta -= std::copysign(PI, delta);
      speed = -speed;
    }
    state[i].last_angle += delta;
    state[i].initialized = true;
    out[i] = {state[i].last_angle, speed};
    max_abs = std::max(max_abs, std::fabs(speed));
  }

  if (max_wheel_mps > 0.0 && max_abs > max_wheel_mps) {
    const double k = max_wheel_mps / max_abs;
    for (auto& c : out) c.speed *= k;
  }
}

void fold_to_range(const Arr& lo, const Arr& hi, double unwind_rad, double slow_mps, States& state,
                   Cmds& out) {
  for (std::size_t i = 0; i < N; ++i) {
    const double center = 0.5 * (lo[i] + hi[i]);
    const double th = out[i].angle;
    const double ref = state[i].initialized ? state[i].last_cmd : 0.0;

    // 직전 명령각에 가장 가까운 등가각 th + nπ
    double n = std::floor((ref - th) / PI + 0.5);
    double folded = th + n * PI;
    if (folded > hi[i] + FOLD_EPS_RAD)
      n -= 1.0;
    else if (folded < lo[i] - FOLD_EPS_RAD)
      n += 1.0;
    folded = th + n * PI;
    // 폭 < π: 표현 불가 방향은 경계로 (기구 보호 우선)
    if (folded > hi[i]) folded = hi[i];
    else if (folded < lo[i]) folded = lo[i];

    // 이음매에서 멀어지게 미리 되감기 (그 바퀴가 거의 안 굴 때만)
    if (unwind_rad > 0.0 && std::fabs(folded - center) > unwind_rad &&
        std::fabs(out[i].speed) < slow_mps) {
      const double alt = folded - (folded > center ? PI : -PI);
      if (std::fabs(alt - center) < std::fabs(folded - center) && alt >= lo[i] && alt <= hi[i]) {
        n += (folded > center ? -1.0 : 1.0);
        folded = alt;
      }
    }
    if (static_cast<long long>(std::fabs(n)) % 2 == 1) out[i].speed = -out[i].speed;
    out[i].angle = folded;
    state[i].last_cmd = folded;
  }
}

Twist2D fk_compute(const Modules& modules, const Arr& angle, const Arr& speed) {
  // 식: u_i = vx − ω·y_i,  w_i = vy + ω·x_i   (u, w = 바퀴 속도의 x, y 성분)
  // 정규방정식 AᵀA p = Aᵀb, p = (vx, vy, ω)
  double sx = 0, sy = 0, sxx_yy = 0, su = 0, sw = 0, swx_uy = 0;
  for (std::size_t i = 0; i < N; ++i) {
    const double x = modules[i].x, y = modules[i].y;
    const double u = speed[i] * std::cos(angle[i]);
    const double w = speed[i] * std::sin(angle[i]);
    sx += x;
    sy += y;
    sxx_yy += x * x + y * y;
    su += u;
    sw += w;
    swx_uy += w * x - u * y;
  }
  const double n = static_cast<double>(N);
  // AᵀA = [[n, 0, −Σy], [0, n, Σx], [−Σy, Σx, Σ(x²+y²)]],  Aᵀb = [Σu, Σw, Σ(w·x − u·y)]
  // ω 먼저 소거: vx = (Σu + ω Σy)/n, vy = (Σw − ω Σx)/n
  const double denom = sxx_yy - (sx * sx + sy * sy) / n;
  if (std::fabs(denom) < 1e-12) throw std::invalid_argument("모듈 배치가 퇴화 (모두 한 점)");
  const double omega = (swx_uy - (sw * sx - su * sy) / n) / denom;
  return {(su + omega * sy) / n, (sw - omega * sx) / n, omega};
}

// ---------------------------------------------------------------- AlignGate
AlignGate::AlignGate(double trigger_rad, double release_rad, unsigned max_hold_ticks)
    : trigger_(trigger_rad), release_(release_rad), max_hold_(max_hold_ticks) {}

void AlignGate::reset() {
  prev_goal_.fill(0.0);
  has_prev_ = pending_ = driving_ = timed_out_ = false;
  hold_ticks_ = 0;
}

bool AlignGate::update_goal(const Arr& goal, const Arr& speed) {
  if (trigger_ <= 0.0) {
    pending_ = false;
    return false;
  }
  double max_delta = 0.0;
  bool driving = false;
  for (std::size_t i = 0; i < N; ++i) {
    max_delta = std::max(max_delta, std::fabs(goal[i] - prev_goal_[i]));
    if (std::fabs(speed[i]) > GATE_SPEED_EPS) driving = true;
  }
  driving_ = driving;
  if (driving && (!has_prev_ || max_delta > trigger_)) {
    if (!pending_) hold_ticks_ = 0;
    pending_ = true;
    timed_out_ = false;
  }
  prev_goal_ = goal;
  has_prev_ = true;
  return pending_;
}

bool AlignGate::allow(const Arr& goal, const Arr& actual, bool actual_ok) {
  if (trigger_ <= 0.0) return true;
  double max_err = 0.0;
  if (actual_ok)
    for (std::size_t i = 0; i < N; ++i) max_err = std::max(max_err, std::fabs(goal[i] - actual[i]));

  if (!pending_ && !timed_out_ && driving_ && actual_ok && max_err > trigger_) {
    pending_ = true;
    hold_ticks_ = 0;
  }
  if (!pending_) return true;
  if (actual_ok && max_err <= release_) {
    pending_ = false;
    hold_ticks_ = 0;
    return true;
  }
  ++hold_ticks_;
  if (max_hold_ > 0 && hold_ticks_ >= max_hold_) {
    pending_ = false;
    hold_ticks_ = 0;
    timed_out_ = true;
    return true;
  }
  return false;
}

// ---------------------------------------------------------------- Controller
Controller::Controller(ControllerConfig cfg)
    : cfg_(cfg), gate_(cfg.gate_trigger_rad, cfg.gate_release_rad, cfg.gate_max_hold_ticks) {
  for (std::size_t i = 0; i < N; ++i)
    if (!(cfg_.lo[i] < cfg_.hi[i])) throw std::invalid_argument("조향 한계 lo < hi 아님");
}

void Controller::seed(const Arr& actual_angle) {
  for (std::size_t i = 0; i < N; ++i) {
    state_[i].last_angle = actual_angle[i];
    state_[i].last_cmd = actual_angle[i];
    state_[i].initialized = true;
  }
  gate_.reset();
  sent_ = actual_angle;
  has_sent_ = true;
  est_speed_ = 0.0;
}

void Controller::reset() {
  state_ = States{};
  gate_.reset();
  has_sent_ = false;
  was_stopping_ = false;
  est_speed_ = 0.0;
}

void Controller::sync_history_to_sent() {
  for (std::size_t i = 0; i < N; ++i) {
    state_[i].last_angle = sent_[i];
    state_[i].last_cmd = sent_[i];
    state_[i].initialized = true;
  }
}

StepResult Controller::step(const Twist2D& cmd, const Arr& actual_angle, bool actual_ok, const Arr& wheel_speed_meas,
                            bool wheel_ok) {
  StepResult r;
  // 1) 바퀴가 아직 구르나 (실측, 없으면 추정)
  double meas = est_speed_;
  if (wheel_ok) {
    meas = 0.0;
    for (double v : wheel_speed_meas) meas = std::max(meas, std::fabs(v));
  }
  const bool moving = meas > cfg_.stop_speed_mps;

  // 정지 대기 중이었으면 붙잡은 각에서 목표를 다시 계산 (실제 각에 가장 가까운 등가각)
  if (was_stopping_ && has_sent_) sync_history_to_sent();

  // 2) IK + 접기. 되감기는 바퀴가 실제로 멈췄을 때만
  ik_compute(cmd.vx, cmd.vy, cmd.omega, cfg_.modules, cfg_.max_wheel_mps, state_, r.cmd, cfg_.min_steer_mps);
  fold_to_range(cfg_.lo, cfg_.hi, moving ? 0.0 : cfg_.unwind_rad, cfg_.slow_mps, state_, r.cmd);
  Arr goal{}, speed{};
  for (std::size_t i = 0; i < N; ++i) {
    goal[i] = r.cmd[i].angle;
    speed[i] = r.cmd[i].speed;
  }
  if (!has_sent_) {
    sent_ = goal;
    has_sent_ = true;
  }

  // 명령 0 (전 바퀴 속도 0): 조향을 돌릴 이유가 없음 → 지금 각 유지 + 미뤄 둔 목표 취소
  // (unwind 를 켠 경우만 예외 — 그건 일부러 멈췄을 때 돌리는 기능)
  bool idle = true;
  for (const auto& c : r.cmd) idle = idle && c.speed == 0.0;
  if (idle && cfg_.unwind_rad <= 0.0) {
    sync_history_to_sent();
    for (std::size_t i = 0; i < N; ++i) {
      r.cmd[i] = {sent_[i], 0.0};
      goal[i] = sent_[i];
      speed[i] = 0.0;
    }
  }

  // 3) 큰 조향 전환인데 바퀴가 구르면: 조향은 직전 각 유지, 구동 0 으로 먼저 정지
  double jump = 0.0;
  for (std::size_t i = 0; i < N; ++i) jump = std::max(jump, std::fabs(goal[i] - sent_[i]));
  if (cfg_.flip_stop_rad > 0.0 && moving && jump > cfg_.flip_stop_rad) {
    for (std::size_t i = 0; i < N; ++i) r.cmd[i] = {sent_[i], 0.0};
    r.wheel_speed.fill(0.0);
    r.gated = r.stopping = true;
    gate_.update_goal(sent_, r.wheel_speed);  // 게이트 기준을 붙잡은 각에 맞춰 둠 → 풀릴 때 새 각으로 정렬 대기
    est_speed_ = std::max(0.0, est_speed_ - cfg_.est_decel_mps2 * cfg_.dt);
    was_stopping_ = true;
    return r;
  }
  was_stopping_ = false;

  // 4) 정렬 게이트 (최종 목표각 기준)
  gate_.update_goal(goal, speed);
  r.gated = !gate_.allow(goal, actual_angle, actual_ok);
  double sent_max = 0.0;
  for (std::size_t i = 0; i < N; ++i) {
    r.wheel_speed[i] = r.gated ? 0.0 : speed[i];
    sent_max = std::max(sent_max, std::fabs(r.wheel_speed[i]));
  }
  // 5) 조향 명령 회전 속도 제한: 직전에 보낸 각에서 max_steer_rate·dt 씩만 목표 쪽으로
  if (cfg_.max_steer_rate > 0.0) {
    const double step_a = cfg_.max_steer_rate * cfg_.dt;
    for (std::size_t i = 0; i < N; ++i) r.cmd[i].angle = sent_[i] + std::clamp(goal[i] - sent_[i], -step_a, step_a);
  }
  for (std::size_t i = 0; i < N; ++i) sent_[i] = r.cmd[i].angle;
  // 추정 속도: 보드 램프처럼 명령 쪽으로 est_decel 기울기로만 따라감
  const double step_v = cfg_.est_decel_mps2 * cfg_.dt;
  est_speed_ += std::clamp(sent_max - est_speed_, -step_v, step_v);
  return r;
}

}  // namespace swerve
