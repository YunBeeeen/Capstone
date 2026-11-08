// 1~9, sideways_no_toggle: ksrc firmware/common/tests/test_swerve_kinematics.c 와 같은 케이스
// gate_*: ksrc tests/test_drive_align_gate.c 와 같은 케이스
// 나머지: 이 프로젝트용 추가 (FK, ±170° 운용, unwind, seed, 정지 후 조향, 멈춰 있을 때 조향 유지)
// 모듈 순서는 우리 규약 [FR, RR, RL, FL] — ksrc 원본(FL, FR, RL, RR)에서 FL 은 index 3.
#include <gtest/gtest.h>

#include <cmath>

#include "swerve_controller/kinematics.hpp"

using namespace swerve;

namespace {
constexpr double PI = 3.14159265358979323846;
constexpr double DEG = PI / 180.0;
constexpr int FR = 0, FL = 3;

const Modules SQ = {{{0.12, -0.12}, {-0.12, -0.12}, {-0.12, 0.12}, {0.12, 0.12}}};  // FR RR RL FL

Arr all(double v) { return {v, v, v, v}; }

// 접은 뒤 (각, 속도) 가 요구 지면 속도벡터와 같은지
void expect_ground_velocity(const Modules& m, const Cmds& c, const Twist2D& t, double tol = 1e-9) {
  for (std::size_t i = 0; i < N; ++i) {
    const double want_x = t.vx - t.omega * m[i].y;
    const double want_y = t.vy + t.omega * m[i].x;
    EXPECT_NEAR(c[i].speed * std::cos(c[i].angle), want_x, tol) << "module " << i;
    EXPECT_NEAR(c[i].speed * std::sin(c[i].angle), want_y, tol) << "module " << i;
  }
}
}  // namespace

// ---------------------------------------------------------------- ksrc 1~9
TEST(Ik, ForwardAllZeroAngle) {
  States s{};
  Cmds o{};
  ik_compute(1, 0, 0, SQ, -1, s, o);
  for (auto& c : o) {
    EXPECT_NEAR(c.angle, 0, 1e-12);
    EXPECT_NEAR(c.speed, 1, 1e-12);
  }
}

TEST(Ik, StrafeLeft90) {
  States s{};
  Cmds o{};
  ik_compute(1, 0, 0, SQ, -1, s, o);
  ik_compute(0, 1, 0, SQ, -1, s, o);
  for (auto& c : o) {
    EXPECT_NEAR(c.angle, PI / 2, 1e-12);
    EXPECT_NEAR(c.speed, 1, 1e-12);
  }
}

TEST(Ik, RotateInPlaceShortestPath) {
  // FL 접선 방향은 135° 지만 0° 에서 90° 넘게 돌아야 하므로 −45° + 역회전
  States s{};
  Cmds o{};
  ik_compute(0, 0, 1, SQ, -1, s, o);
  EXPECT_NEAR(o[FL].angle / DEG, -45, 1e-9);
  EXPECT_NEAR(o[FL].speed, -0.12 * std::sqrt(2.0), 1e-12);
  EXPECT_NEAR(o[FR].angle / DEG, 45, 1e-9);  // FR 접선 45° — 그대로
  EXPECT_NEAR(o[FR].speed, 0.12 * std::sqrt(2.0), 1e-12);
}

TEST(Ik, ContinuousThroughSeam) {
  States s{};
  s[0] = {3.0, 0.0, true};
  Cmds o{};
  ik_compute(std::cos(-3.0), std::sin(-3.0), 0, SQ, -1, s, o);
  EXPECT_NEAR(o[0].angle, 3.0 + (2 * PI - 6.0), 1e-9);  // 3.283, −3 으로 튀지 않음
  EXPECT_GT(o[0].speed, 0);
}

TEST(Ik, Opt170BecomesMinus10) {
  States s{};
  Cmds o{};
  ik_compute(std::cos(170 * DEG), std::sin(170 * DEG), 0, SQ, -1, s, o);
  EXPECT_NEAR(o[0].angle / DEG, -10, 1e-9);
  EXPECT_LT(o[0].speed, 0);
}

TEST(Ik, Desaturation) {
  States s{};
  Cmds o{};
  ik_compute(1, 0, 5, SQ, 0.5, s, o);
  double mx = 0;
  for (auto& c : o) mx = std::max(mx, std::fabs(c.speed));
  EXPECT_NEAR(mx, 0.5, 1e-12);
}

TEST(Ik, DeadbandHoldsAngle) {
  States s{};
  s[0] = {1.2345, 0.0, true};
  Cmds o{};
  ik_compute(0, 0, 0, SQ, -1, s, o);
  EXPECT_DOUBLE_EQ(o[0].angle, 1.2345);
  EXPECT_DOUBLE_EQ(o[0].speed, 0);
}

TEST(Fold, AsymmetricRangeMovesToEquivalent) {
  States s{};
  Cmds o{};
  const Arr lo = {0, -PI, -PI, -PI}, hi = all(PI);
  ik_compute(-0.1, 0.1, 0, SQ, -1, s, o);
  EXPECT_NEAR(o[0].angle, -PI / 4, 1e-12);
  fold_to_range(lo, hi, 0, 0, s, o);
  EXPECT_NEAR(o[0].angle, 3 * PI / 4, 1e-12);
  EXPECT_NEAR(o[0].speed * std::cos(o[0].angle), -0.1, 1e-12);
  EXPECT_NEAR(o[0].speed * std::sin(o[0].angle), 0.1, 1e-12);
  EXPECT_NEAR(o[1].angle, -PI / 4, 1e-12);
}

TEST(Fold, NarrowRangeClamps) {
  States s{};
  Cmds o{};
  ik_compute(-0.1, 0.1, 0, SQ, -1, s, o);
  fold_to_range(all(0), all(PI / 4), 0, 0, s, o);
  EXPECT_NEAR(o[0].angle, PI / 4, 1e-12);
}

TEST(Fold, SidewaysNoToggle) {
  States s{};
  Cmds o{};
  for (int k = 0; k < 5; ++k) {
    ik_compute(0.05, 0, 0, SQ, 0.378, s, o);
    fold_to_range(all(-PI / 2), all(PI / 2), 0, 0, s, o);
  }
  int flips = 0;
  double prev = 0;
  for (int k = 0; k < 50; ++k) {
    ik_compute(0, 0.05, 0, SQ, 0.378, s, o);
    fold_to_range(all(-PI / 2), all(PI / 2), 0, 0, s, o);
    if (k > 0 && std::fabs(o[0].angle - prev) > 1.0) ++flips;
    prev = o[0].angle;
  }
  EXPECT_EQ(flips, 0);
  EXPECT_NEAR(o[0].angle, PI / 2, 2e-4);
  EXPECT_NEAR(o[0].speed, 0.05, 1e-9);
}

// ---------------------------------------------------------------- 우리 운용: ±170°
TEST(Fold, Plus170NeverExceededAndGroundVelocityKept) {
  // 차체 회전 없이 이동 방향만 3바퀴 계속 돌림 (가장 나쁜 경우: 조향이 한쪽으로 계속 감김)
  States s{};
  Cmds o{};
  const double lim = 170 * DEG;
  int big_jumps = 0;
  double prev = 0;
  for (int k = 0; k <= 3 * 360; ++k) {
    const Twist2D t{0.3 * std::cos(k * DEG), 0.3 * std::sin(k * DEG), 0};
    ik_compute(t.vx, t.vy, t.omega, SQ, -1, s, o);
    fold_to_range(all(-lim), all(lim), 0, 0, s, o);
    for (auto& c : o) {
      EXPECT_LE(c.angle, lim + 1e-9);
      EXPECT_GE(c.angle, -lim - 1e-9);
    }
    expect_ground_velocity(SQ, o, t);
    if (k > 0 && std::fabs(o[0].angle - prev) > 90 * DEG) ++big_jumps;
    prev = o[0].angle;
  }
  // 이음매는 피할 수 없음: 180° 마다 한 번 꼴 (3바퀴 = 1080° → 약 6 번)
  EXPECT_GE(big_jumps, 4);
  EXPECT_LE(big_jumps, 7);
}

TEST(Fold, UnwindOnlyWhenSlow) {
  const double lim = 170 * DEG;
  // 160° 를 보고 있는 상태
  States s{};
  for (auto& m : s) m = {160 * DEG, 160 * DEG, true};
  Cmds o{};
  // 같은 방향으로 빠르게 → 되감지 않음
  ik_compute(0.3 * std::cos(160 * DEG), 0.3 * std::sin(160 * DEG), 0, SQ, -1, s, o);
  fold_to_range(all(-lim), all(lim), 100 * DEG, 0.02, s, o);
  EXPECT_NEAR(o[0].angle / DEG, 160, 1e-9);
  EXPECT_GT(o[0].speed, 0);
  // 같은 방향으로 아주 느리게 → −20° 로 되감고 부호 반전
  ik_compute(0.01 * std::cos(160 * DEG), 0.01 * std::sin(160 * DEG), 0, SQ, -1, s, o);
  fold_to_range(all(-lim), all(lim), 100 * DEG, 0.02, s, o);
  EXPECT_NEAR(o[0].angle / DEG, -20, 1e-9);
  EXPECT_LT(o[0].speed, 0);
  // 정지(데드밴드) 상태도 되감음 → 이후 그 자리 유지
  ik_compute(0, 0, 0, SQ, -1, s, o);
  fold_to_range(all(-lim), all(lim), 100 * DEG, 0.02, s, o);
  EXPECT_NEAR(o[0].angle / DEG, -20, 1e-9);
}

// ---------------------------------------------------------------- FK
TEST(Fk, RoundTripThroughIkAndFold) {
  const Modules m = {{{0.1048, -0.1556}, {-0.1048, -0.1556}, {-0.1048, 0.1556}, {0.1048, 0.1556}}};
  States s{};
  Cmds o{};
  const Twist2D cases[] = {{0.3, 0, 0}, {0, -0.2, 0}, {0, 0, 1.0}, {0.2, 0.1, -0.7}, {-0.25, 0.15, 0.4}};
  for (const auto& t : cases) {
    ik_compute(t.vx, t.vy, t.omega, m, -1, s, o);
    fold_to_range(all(-170 * DEG), all(170 * DEG), 0, 0, s, o);
    Arr a{}, v{};
    for (std::size_t i = 0; i < N; ++i) {
      a[i] = o[i].angle;
      v[i] = o[i].speed;
    }
    const auto r = fk_compute(m, a, v);
    EXPECT_NEAR(r.vx, t.vx, 1e-9);
    EXPECT_NEAR(r.vy, t.vy, 1e-9);
    EXPECT_NEAR(r.omega, t.omega, 1e-9);
  }
}

TEST(Fk, OneWheelSlipGivesLeastSquares) {
  // 전진 0.3 인데 FR 만 0.4 로 읽힘 → 결과는 전진 ~0.325 + 약간의 회전, 횡속도 0
  const auto r = fk_compute(SQ, all(0), {0.4, 0.3, 0.3, 0.3});
  EXPECT_NEAR(r.vx, 0.325, 1e-9);
  EXPECT_NEAR(r.vy, 0, 1e-9);
  // 오른쪽 바퀴가 빠르면 왼쪽으로 돎 = 반시계(+). Σ(−u·y)/Σ(x²+y²) = 0.012 / 0.1152
  EXPECT_NEAR(r.omega, 0.012 / 0.1152, 1e-9);
}

// ---------------------------------------------------------------- gate (ksrc)
TEST(Gate, KsrcCases) {
  const Arr zero = all(0), fwd = all(0.1);
  AlignGate g(60 * DEG, 10 * DEG, 0);
  // 1) 첫 구동은 대기, 맞으면 즉시 해제
  EXPECT_TRUE(g.update_goal(zero, fwd));
  EXPECT_TRUE(g.allow(zero, zero, true));
  // 2) 30° 변화는 대기 없음
  const Arr g30 = all(30 * DEG);
  EXPECT_FALSE(g.update_goal(g30, fwd));
  EXPECT_TRUE(g.allow(g30, zero, true));
  // 3) 한 바퀴만 90° 바뀌어도 대기
  const Arr goal = {120 * DEG, 30 * DEG, 30 * DEG, 30 * DEG};
  const Arr far = {60 * DEG, 30 * DEG, 30 * DEG, 30 * DEG};
  const Arr near = {112 * DEG, 31 * DEG, 29 * DEG, 30 * DEG};
  EXPECT_TRUE(g.update_goal(goal, fwd));
  EXPECT_FALSE(g.allow(goal, far, true));
  EXPECT_TRUE(g.update_goal(goal, fwd));
  EXPECT_FALSE(g.allow(goal, near, false));  // 실제각 모름 → 해제 안 함
  EXPECT_TRUE(g.allow(goal, near, true));
  // 4) 정지 명령으로 큰 조향 변화는 대기 없음
  EXPECT_FALSE(g.update_goal({-60 * DEG, -60 * DEG, 30 * DEG, 30 * DEG}, zero));
}

TEST(Gate, SeamIsNotWrapped) {
  const Arr fwd = all(0.1);
  const Arr a = {179 * DEG, 0, 0, 0}, b = {-179 * DEG, 0, 0, 0};
  AlignGate g(60 * DEG, 10 * DEG, 0);
  g.update_goal(a, fwd);
  g.allow(a, a, true);
  EXPECT_TRUE(g.update_goal(b, fwd));
  EXPECT_FALSE(g.allow(b, a, true));
}

TEST(Gate, Off) {
  AlignGate g(0, 10 * DEG, 0);
  EXPECT_FALSE(g.update_goal({-179 * DEG, 0, 0, 0}, all(0.1)));
  EXPECT_TRUE(g.allow({-179 * DEG, 0, 0, 0}, all(0), false));
}

TEST(Gate, MeasuredTriggerOnRamp) {
  const Arr fwd = all(0.1);
  AlignGate g(60 * DEG, 10 * DEG, 0);
  g.update_goal(all(0), fwd);
  EXPECT_TRUE(g.allow(all(0), all(0), true));
  g.update_goal(all(40 * DEG), fwd);
  EXPECT_TRUE(g.allow(all(40 * DEG), all(0), true));
  g.update_goal(all(80 * DEG), fwd);
  EXPECT_FALSE(g.allow(all(80 * DEG), all(0), true));
  g.update_goal(all(120 * DEG), fwd);
  EXPECT_FALSE(g.allow(all(120 * DEG), all(40 * DEG), true));
  EXPECT_TRUE(g.allow(all(120 * DEG), all(120 * DEG), true));
}

TEST(Gate, Timeout) {
  const Arr fwd = all(0.1), a0 = all(0), a9 = all(90 * DEG);
  AlignGate g(60 * DEG, 10 * DEG, 5);
  g.update_goal(a0, fwd);
  g.allow(a0, a0, true);
  bool allowed = false;
  for (int k = 0; k < 5; ++k) {
    g.update_goal(a9, fwd);
    allowed = g.allow(a9, a0, true);
  }
  EXPECT_TRUE(allowed);
  EXPECT_TRUE(g.timed_out());
  g.update_goal(a9, fwd);
  EXPECT_TRUE(g.allow(a9, a0, true));
  EXPECT_TRUE(g.update_goal(a0, fwd));
  EXPECT_FALSE(g.timed_out());
}

// ---------------------------------------------------------------- Controller
namespace {
ControllerConfig cfg170() {
  ControllerConfig c;
  c.modules = SQ;
  c.lo = all(-170 * DEG);
  c.hi = all(170 * DEG);
  c.max_wheel_mps = 0.6;
  c.unwind_rad = 100 * DEG;
  c.slow_mps = 0.02;
  c.gate_trigger_rad = 60 * DEG;
  c.gate_release_rad = 10 * DEG;
  return c;
}
}  // namespace

TEST(Controller, GatesUntilSteerAligned) {
  Controller ctl(cfg170());
  const Twist2D left{0, 0.3, 0};
  auto r = ctl.step(left, all(0), true);  // 조향은 아직 0°, 목표 90°
  EXPECT_TRUE(r.gated);
  for (double v : r.wheel_speed) EXPECT_EQ(v, 0.0);
  for (auto& c : r.cmd) EXPECT_NEAR(c.angle, PI / 2, 1e-12);  // 조향 명령은 나감
  r = ctl.step(left, all(85 * DEG), true);
  EXPECT_FALSE(r.gated);
  for (double v : r.wheel_speed) EXPECT_NEAR(v, 0.3, 1e-12);
}

TEST(Controller, NoSteerFeedbackNeverDrives) {
  Controller ctl(cfg170());
  for (int k = 0; k < 100; ++k) {
    const auto r = ctl.step({0.3, 0, 0}, all(0), false);
    EXPECT_TRUE(r.gated);
  }
}

TEST(Controller, SeedContinuesFromHandSetAngle) {
  // FREE 중 손으로 FR 을 150° 에 둠 → 전진: 가까운 등가각 180° 는 범위(±170°) 밖 → 0°, 정회전.
  // 조향이 150° 돌아야 하므로 게이트 대기.
  Controller ctl(cfg170());
  Arr hand = {150 * DEG, 0, 0, 0};
  ctl.seed(hand);
  auto r = ctl.step({0.3, 0, 0}, hand, true);
  EXPECT_NEAR(r.cmd[FR].angle, 0, 1e-12);
  EXPECT_TRUE(r.gated);
  // −100° 에 둔 경우 → 전진: −180°(80° 회전) 는 범위 밖 → 0°(100° 회전), 정회전
  Controller ctl2(cfg170());
  Arr hand2 = {-100 * DEG, 0, 0, 0};
  ctl2.seed(hand2);
  r = ctl2.step({0.3, 0, 0}, hand2, true);
  EXPECT_NEAR(r.cmd[FR].angle, 0, 1e-12);
  EXPECT_GT(r.cmd[FR].speed, 0);
  // 전부 80° 에 둔 경우 → 오른쪽 횡이동: −90°(170° 회전) 대신 +90° 역회전(10° 회전).
  // seed 없으면 이력이 0° 라 −90° 를 골랐을 것
  Controller ctl3(cfg170());
  Arr hand3 = {80 * DEG, 80 * DEG, 80 * DEG, 80 * DEG};
  ctl3.seed(hand3);
  r = ctl3.step({0, -0.3, 0}, hand3, true);
  for (auto& c : r.cmd) {
    EXPECT_NEAR(c.angle, PI / 2, 1e-12);
    EXPECT_LT(c.speed, 0);
  }
}

// ---------------------------------------------------------------- 정지 후 조향 (2026-11-07 전복 대책)
namespace {
ControllerConfig cfg_flip() {
  auto c = cfg170();
  c.flip_stop_rad = 60 * DEG;
  return c;
}
Twist2D dir(double deg, double v = 0.3) { return {v * std::cos(deg * DEG), v * std::sin(deg * DEG), 0}; }
}  // namespace

TEST(FlipStop, SeamFlipWaitsForWheelsToStop) {
  Controller ctl(cfg_flip());
  ctl.seed(all(165 * DEG));
  const Arr moving = all(0.3), stopped = all(0.0);
  // 165° 방향으로 주행 중 (조향 정렬됨)
  auto r = ctl.step(dir(165), all(165 * DEG), true, moving, true);
  EXPECT_FALSE(r.stopping);
  EXPECT_NEAR(r.cmd[0].angle / DEG, 165, 1e-9);
  // 175° 로 → ±170° 넘으므로 −5° 로 반전해야 함. 바퀴가 구르는 동안은 조향 165° 유지 + 구동 0
  for (int k = 0; k < 5; ++k) {
    r = ctl.step(dir(175), all(165 * DEG), true, moving, true);
    EXPECT_TRUE(r.stopping);
    EXPECT_TRUE(r.gated);
    for (std::size_t i = 0; i < N; ++i) {
      EXPECT_NEAR(r.cmd[i].angle / DEG, 165, 1e-9);
      EXPECT_EQ(r.wheel_speed[i], 0.0);
    }
  }
  // 바퀴가 멈추면 새 각(−5°)으로 돌림, 정렬 전까지 구동 보류
  r = ctl.step(dir(175), all(165 * DEG), true, stopped, true);
  EXPECT_FALSE(r.stopping);
  EXPECT_TRUE(r.gated);
  EXPECT_NEAR(r.cmd[0].angle / DEG, -5, 1e-9);
  // 정렬되면 역회전으로 주행 (지면 속도 방향은 175°)
  r = ctl.step(dir(175), all(-5 * DEG), true, stopped, true);
  EXPECT_FALSE(r.gated);
  EXPECT_LT(r.wheel_speed[0], 0);
  EXPECT_NEAR(r.wheel_speed[0] * std::cos(r.cmd[0].angle), 0.3 * std::cos(175 * DEG), 1e-9);
}

TEST(FlipStop, SmallChangeDoesNotStop) {
  Controller ctl(cfg_flip());
  ctl.seed(all(0));
  auto r = ctl.step(dir(0), all(0), true, all(0.3), true);
  r = ctl.step(dir(30), all(0), true, all(0.3), true);
  EXPECT_FALSE(r.stopping);
  EXPECT_NEAR(r.cmd[0].angle / DEG, 30, 1e-9);
}

TEST(FlipStop, UnwindOnlyWhenWheelsStopped) {
  Controller ctl(cfg_flip());
  ctl.seed(all(150 * DEG));
  // 명령 0 (데드밴드) 인데 바퀴가 아직 굴러감 → 되감지 않음
  auto r = ctl.step({0, 0, 0}, all(150 * DEG), true, all(0.2), true);
  EXPECT_NEAR(r.cmd[0].angle / DEG, 150, 1e-9);
  // 바퀴 멈춤 → 중앙 쪽(−30°) 으로 되감기
  r = ctl.step({0, 0, 0}, all(150 * DEG), true, all(0.0), true);
  EXPECT_NEAR(r.cmd[0].angle / DEG, -30, 1e-9);
}

TEST(FlipStop, NoFeedbackUsesEstimatedDecel) {
  auto c = cfg_flip();
  c.dt = 0.02;
  c.est_decel_mps2 = 0.66;
  Controller ctl(c);
  ctl.seed(all(165 * DEG));
  // 피드백 없이 1 s 주행 → 추정 속도 0.3 까지 올라감
  for (int k = 0; k < 50; ++k) ctl.step(dir(165), all(165 * DEG), true);
  // 반전 요구 → (0.3 − 0.03)/0.66 ≈ 0.41 s ≈ 21 주기 동안 정지 대기
  int stop_ticks = 0;
  for (int k = 0; k < 60; ++k) {
    const auto r = ctl.step(dir(175), all(165 * DEG), true);
    if (!r.stopping) break;
    ++stop_ticks;
  }
  EXPECT_GE(stop_ticks, 19);
  EXPECT_LE(stop_ticks, 23);
}

// ---------------------------------------------------------------- 조향 명령 회전 속도 제한
TEST(SteerRate, SlewsTowardGoalAndGatesUntilThere) {
  auto c = cfg_flip();
  c.dt = 0.02;
  c.max_steer_rate = 360 * DEG;  // 주기당 7.2°
  Controller ctl(c);
  ctl.seed(all(0));
  // 정지 상태에서 왼쪽 횡이동 → 목표 90°. 명령은 7.2° 씩, 다 돌 때까지(±10°) 구동 0
  double prev = 0;
  int ticks = 0;
  StepResult r;
  for (; ticks < 40; ++ticks) {
    r = ctl.step(dir(90), all(prev), true, all(0), true);  // 조향이 명령을 바로 따라간다고 가정
    EXPECT_LE(std::fabs(r.cmd[0].angle - prev), 7.2 * DEG + 1e-9);
    prev = r.cmd[0].angle;
    if (!r.gated) break;
  }
  EXPECT_NEAR(prev / DEG, 90, 1e-9);
  EXPECT_GE(ticks, 11);  // 80°(=90−10) 넘을 때까지는 보류 → 최소 12주기
  EXPECT_FALSE(r.gated);
}

// ---------------------------------------------------------------- 멈춰 있을 때 조향 안 돌림 (2026-11-07)
// 운용 설정: ±180°, unwind 끔, 저속 조향 유지 0.03 m/s, 정지 후 조향 60°
namespace {
ControllerConfig cfg_run(double min_steer = 0.03) {
  ControllerConfig c;
  c.modules = SQ;
  c.lo = all(-180 * DEG);
  c.hi = all(180 * DEG);
  c.max_wheel_mps = 0.6;
  c.min_steer_mps = min_steer;
  c.unwind_rad = 0.0;
  c.gate_trigger_rad = 60 * DEG;
  c.gate_release_rad = 10 * DEG;
  c.flip_stop_rad = 60 * DEG;
  return c;
}
}  // namespace

TEST(SteerHold, TinyCommandKeepsAngleAndProjects) {
  States s{};
  Cmds o{};
  ik_compute(0, 0.014, 0, SQ, -1, s, o, 0.03);  // 옆으로 1.4 cm/s (스틱 놓을 때 잡음) → 조향 유지, 옆 성분은 못 굴림
  for (auto& c : o) {
    EXPECT_NEAR(c.angle, 0, 1e-12);
    EXPECT_NEAR(c.speed, 0, 1e-12);
  }
  ik_compute(-0.02, 0, 0, SQ, -1, s, o, 0.03);  // 뒤로 2 cm/s → 조향 유지, 뒤로 굴림
  for (auto& c : o) {
    EXPECT_NEAR(c.angle, 0, 1e-12);
    EXPECT_NEAR(c.speed, -0.02, 1e-12);
  }
  ik_compute(0, 0.3, 0, SQ, -1, s, o, 0.03);  // 문턱 위 → 평소처럼 조향
  for (auto& c : o) EXPECT_NEAR(c.angle, PI / 2, 1e-12);
}

TEST(SteerHold, StickReleaseNoiseDoesNotRotate) {
  // 2026-11-07 시뮬 재현: 전진 중 스틱을 놓으면 오버슈트 + 잡음 → 전엔 바퀴가 선 뒤 0° → 180° 로 돌았음
  Controller ctl(cfg_run());
  ctl.seed(all(0));
  double wheel = 0.25;
  ctl.step({0.25, 0, 0}, all(0), true, all(wheel), true);
  const Twist2D noise[] = {{0.032, 0, 0},  {-0.082, 0, 0}, {-0.047, 0, 0}, {0.019, 0, 0},
                           {0.020, 0.014, 0}, {0, 0, 0},  {-0.016, 0, 0}, {0, -0.014, 0},
                           {0.018, 0, 0},  {-0.017, 0, 0}};
  Arr actual = all(0);
  auto tick = [&](const Twist2D& cmd) {
    const auto r = ctl.step(cmd, actual, true, all(wheel), true);
    for (std::size_t i = 0; i < N; ++i) {
      EXPECT_NEAR(r.cmd[i].angle, 0, 1e-9) << "module " << i << " wheel " << wheel;
      actual[i] = r.cmd[i].angle;
    }
    wheel = std::max(0.0, wheel - 0.0132);
  };
  for (const auto& n : noise) tick(n);
  for (int k = 0; k < 60; ++k) tick({0, 0, 0});  // 바퀴가 완전히 설 때까지
}

TEST(SteerHold, IdleCancelsDeferredTurn) {
  Controller ctl(cfg_run(0.0));  // 저속 유지 없이 — 취소 로직만
  ctl.seed(all(0));
  ctl.step({0.3, 0, 0}, all(0), true, all(0.3), true);
  auto r = ctl.step({0, 0.3, 0}, all(0), true, all(0.3), true);  // 옆으로 → 90° 전환인데 바퀴가 구름 → 대기
  EXPECT_TRUE(r.stopping);
  r = ctl.step({0, 0, 0}, all(0), true, all(0.2), true);  // 서기 전에 명령 0
  for (auto& c : r.cmd) EXPECT_NEAR(c.angle, 0, 1e-9);
  for (int k = 0; k < 10; ++k) {
    r = ctl.step({0, 0, 0}, all(0), true, all(0.0), true);  // 바퀴 멈춤 → 미뤄 둔 90° 를 실행하지 않음
    EXPECT_FALSE(r.stopping);
    for (auto& c : r.cmd) EXPECT_NEAR(c.angle, 0, 1e-9);
  }
  r = ctl.step({0, 0.3, 0}, all(0), true, all(0.0), true);  // 다시 옆 명령 → 이제 정상적으로 90°
  for (auto& c : r.cmd) EXPECT_NEAR(c.angle, PI / 2, 1e-9);
}

TEST(SteerHold, StoppingRecomputesFromHeldAngle) {
  // 정지 대기 중 명령이 바뀌면 붙잡은 각 기준으로 다시 계산 → 등가각(180°)으로 도는 대신 부호 반전
  Controller ctl(cfg_run(0.0));
  ctl.seed(all(0));
  ctl.step({0.3, 0, 0}, all(0), true, all(0.3), true);
  auto r = ctl.step({0, -0.3, 0}, all(0), true, all(0.3), true);  // 오른쪽 → −90° 대기
  EXPECT_TRUE(r.stopping);
  r = ctl.step({-0.3, 0, 0}, all(0), true, all(0.25), true);  // 이어서 후진 → 이력 −90° 기준이면 −180° (0° 의 등가각)
  EXPECT_FALSE(r.stopping);                                    // 붙잡은 0° 기준: 0° 그대로 + 역회전 → 대기 불필요
  for (auto& c : r.cmd) {
    EXPECT_NEAR(c.angle, 0, 1e-9);
    EXPECT_LT(c.speed, 0);
  }
}
