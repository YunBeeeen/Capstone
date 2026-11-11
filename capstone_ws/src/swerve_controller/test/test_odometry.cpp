// SwerveOdometry 시험: IK 로 만든 바퀴 궤적을 적분해서 해석해와 비교 + 거부 조건
#include <gtest/gtest.h>

#include <cmath>

#include "swerve_controller/odometry.hpp"

using namespace swerve;

namespace {
constexpr double PI = 3.14159265358979323846;
constexpr double R = 0.0525;
// 실기 배치 (swerve_controller.yaml)
const Modules ROBOT = {{{0.0911, -0.13605}, {-0.0911, -0.13605}, {-0.0911, 0.13605}, {0.0911, 0.13605}}};

OdomConfig cfg() {
  OdomConfig c;
  c.modules = ROBOT;
  c.wheel_radius = R;
  return c;
}

// 차체 속도 (vx, vy, w) 를 T 초 동안 dt 간격으로 → IK 로 바퀴 각/속도 → 누적 위치로 odom 에 넣음
struct Driver {
  SwerveOdometry odom{cfg()};
  States ik{};
  Arr pos{}, ang{};
  double t = 0.0;
  Driver() { odom.update(t, pos, ang); }
  void drive(double vx, double vy, double w, double T, double dt = 0.05) {
    // 실기처럼 조향 먼저 (정렬 게이트가 다 돌 때까지 구동 보류) — 바퀴는 안 구르고 각만 바뀌는 샘플 하나
    Cmds c0{};
    States peek = ik;
    ik_compute(vx, vy, w, ROBOT, -1.0, peek, c0);
    for (std::size_t i = 0; i < N; ++i) ang[i] = c0[i].angle;
    t += dt;
    ASSERT_EQ(odom.update(t, pos, ang), SwerveOdometry::Result::Ok);
    const int n = static_cast<int>(std::lround(T / dt));
    for (int k = 0; k < n; ++k) {
      Cmds c{};
      ik_compute(vx, vy, w, ROBOT, -1.0, ik, c);
      for (std::size_t i = 0; i < N; ++i) {
        ang[i] = c[i].angle;
        pos[i] += c[i].speed * dt / (2.0 * PI * R);
      }
      t += dt;
      ASSERT_EQ(odom.update(t, pos, ang), SwerveOdometry::Result::Ok);
    }
  }
};
}  // namespace

TEST(Odom, FirstSampleIsInit) {
  SwerveOdometry o(cfg());
  EXPECT_EQ(o.update(0.0, {}, {}), SwerveOdometry::Result::Init);
  EXPECT_EQ(o.update(0.05, {}, {}), SwerveOdometry::Result::Ok);
  EXPECT_NEAR(o.pose().x, 0.0, 1e-12);
}

TEST(Odom, StraightOneTurn) {
  SwerveOdometry o(cfg());
  o.update(0.0, {0, 0, 0, 0}, {0, 0, 0, 0});
  // 1 바퀴를 0.05 s 간격 20 번에 나눠 (1 turn/s)
  for (int k = 1; k <= 20; ++k) {
    const double p = k / 20.0;
    ASSERT_EQ(o.update(0.05 * k, {p, p, p, p}, {0, 0, 0, 0}), SwerveOdometry::Result::Ok);
  }
  EXPECT_NEAR(o.pose().x, 2 * PI * R, 1e-9);
  EXPECT_NEAR(o.pose().y, 0.0, 1e-9);
  EXPECT_NEAR(o.pose().yaw, 0.0, 1e-9);
  EXPECT_NEAR(o.twist().vx, 2 * PI * R, 1e-9);  // 1 turn/s
  EXPECT_NEAR(o.distance(), 2 * PI * R, 1e-9);
}

TEST(Odom, StrafeLeft) {
  Driver d;
  d.drive(0.0, 0.2, 0.0, 2.0);
  EXPECT_NEAR(d.odom.pose().x, 0.0, 1e-9);
  EXPECT_NEAR(d.odom.pose().y, 0.4, 1e-9);
  EXPECT_NEAR(d.odom.pose().yaw, 0.0, 1e-9);
}

TEST(Odom, FlippedModuleSameMotion) {
  // 바퀴 하나(RR)를 180° 돌려 놓고 반대로 굴려도 같은 운동 (IK 의 최단경로 뒤집기와 같은 상황)
  SwerveOdometry o2(cfg());
  o2.update(0.0, {0, 0, 0, 0}, {0, PI, 0, 0});
  o2.update(0.1, {0.1, -0.1, 0.1, 0.1}, {0, PI, 0, 0});
  EXPECT_NEAR(o2.pose().x, 0.1 * 2 * PI * R, 1e-12);
  EXPECT_NEAR(o2.pose().y, 0.0, 1e-12);
  EXPECT_NEAR(o2.pose().yaw, 0.0, 1e-12);
}

TEST(Odom, SpinInPlaceFullTurn) {
  Driver d;
  const double w = 2 * PI / 12.5;  // 12.5 s (= 625 × 0.02 s) 에 정확히 한 바퀴
  d.drive(0.0, 0.0, w, 12.5, 0.02);
  EXPECT_NEAR(d.odom.pose().x, 0.0, 1e-9);
  EXPECT_NEAR(d.odom.pose().y, 0.0, 1e-9);
  EXPECT_NEAR(std::remainder(d.odom.pose().yaw, 2 * PI), 0.0, 1e-9);  // 2π → 0 (wrap)
}

TEST(Odom, ArcMatchesAnalytic) {
  Driver d;
  const double v = 0.3, w = 0.4, T = 5.0;
  d.drive(v, 0.0, w, T, 0.05);
  // 원호: x = v/w sin(wT), y = v/w (1 − cos(wT))
  EXPECT_NEAR(d.odom.pose().x, v / w * std::sin(w * T), 1e-3);
  EXPECT_NEAR(d.odom.pose().y, v / w * (1 - std::cos(w * T)), 1e-3);
  EXPECT_NEAR(d.odom.pose().yaw, w * T, 1e-6);
}

TEST(Odom, HolonomicArc) {
  // 옆으로 가면서 회전 — 차체 기준 vy, 월드에서 원호
  Driver d;
  const double vy = 0.2, w = -0.3, T = 4.0;
  d.drive(0.0, vy, w, T, 0.02);
  // 차체 기준 (0, vy) 를 방위 wt 로 돌려 적분: x = ∫ −vy sin(wt) dt, y = ∫ vy cos(wt) dt
  const double x = vy / w * (std::cos(w * T) - 1), y = vy / w * std::sin(w * T);
  EXPECT_NEAR(d.odom.pose().x, x, 1e-3);
  EXPECT_NEAR(d.odom.pose().y, y, 1e-3);
  EXPECT_NEAR(d.odom.pose().yaw, w * T, 1e-6);
}

TEST(Odom, SquareClosesLoop) {
  Driver d;
  d.drive(0.25, 0, 0, 4.0);
  d.drive(0, 0.25, 0, 4.0);
  d.drive(-0.25, 0, 0, 4.0);
  d.drive(0, -0.25, 0, 4.0);
  EXPECT_NEAR(d.odom.pose().x, 0.0, 1e-9);
  EXPECT_NEAR(d.odom.pose().y, 0.0, 1e-9);
  EXPECT_NEAR(d.odom.distance(), 4.0, 1e-9);
}

TEST(Odom, RejectsBadDtGapJump) {
  SwerveOdometry o(cfg());
  o.update(1.0, {0, 0, 0, 0}, {0, 0, 0, 0});
  EXPECT_EQ(o.update(1.0, {0.1, 0.1, 0.1, 0.1}, {0, 0, 0, 0}), SwerveOdometry::Result::BadDt);
  EXPECT_EQ(o.update(0.9, {0.1, 0.1, 0.1, 0.1}, {0, 0, 0, 0}), SwerveOdometry::Result::BadDt);
  // 간격 1 s > max_dt 0.5 → 버리고 새 기준
  EXPECT_EQ(o.update(2.0, {0.1, 0.1, 0.1, 0.1}, {0, 0, 0, 0}), SwerveOdometry::Result::Gap);
  EXPECT_NEAR(o.pose().x, 0.0, 1e-12);
  // 0.05 s 에 바퀴 하나가 10 바퀴 (보드 재부팅 등) → 점프
  EXPECT_EQ(o.update(2.05, {10.1, 0.1, 0.1, 0.1}, {0, 0, 0, 0}), SwerveOdometry::Result::Jump);
  EXPECT_NEAR(o.pose().x, 0.0, 1e-12);
  // 그 뒤는 정상 이어감
  EXPECT_EQ(o.update(2.10, {10.2, 0.2, 0.2, 0.2}, {0, 0, 0, 0}), SwerveOdometry::Result::Ok);
  EXPECT_NEAR(o.pose().x, 0.1 * 2 * PI * R, 1e-9);
}

TEST(Odom, ResyncKeepsPose) {
  SwerveOdometry o(cfg());
  o.update(0.0, {0, 0, 0, 0}, {0, 0, 0, 0});
  o.update(0.1, {0.1, 0.1, 0.1, 0.1}, {0, 0, 0, 0});
  const double x = o.pose().x;
  o.resync();
  // 끊긴 사이 바퀴가 돌았어도 (0.1 → 5.0) 다시 들어온 샘플은 Init — 포즈 유지
  EXPECT_EQ(o.update(0.2, {5, 5, 5, 5}, {0, 0, 0, 0}), SwerveOdometry::Result::Init);
  EXPECT_NEAR(o.pose().x, x, 1e-12);
  o.reset();
  EXPECT_NEAR(o.pose().x, 0.0, 1e-12);
}
