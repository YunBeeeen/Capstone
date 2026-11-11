// 스워브 바퀴 오도메트리 — ROS 무관 (gtest 로 검증).
//
// 입력: 바퀴 누적 위치 [turn] (wheel_state.position, 바퀴 기준 부호) + 조향각 [rad] (steer_state.angle)
// 출력: odom 프레임 차체 포즈 (x, y, yaw) + 차체 속도 (body 프레임)
//
// 계산 (매 wheel_state 샘플):
//   1) 모듈별 이동거리 d_i = Δposition_i × 2πr     — 속도 × dt 가 아니라 위치 차분 (Hall 60 cnt/rev 라
//      저속 velocity 가 거칠고, RTR 응답 타이밍이 흔들려도 누적 카운트는 안 틀어짐)
//   2) 그 구간 조향각 = (직전 샘플 각 + 지금 각) / 2
//   3) fk_compute(모듈, 각, d) → 차체 변위 (dx, dy, dθ)  — FK 가 선형이라 속도 대신 변위를 넣어도 됨
//   4) 중간 방위(yaw + dθ/2) 로 odom 프레임에 누적
//
// 거부 (적분 안 하고 이번 샘플을 새 기준으로):
//   - 첫 샘플, dt <= 0 (시간 역행/중복), dt > max_dt (피드백 끊김 — 그 사이 조향각을 모름)
//   - 어느 바퀴든 |d_i| > max_wheel_mps·dt + jump_margin_m (보드 재부팅으로 위치 리셋 등)
#pragma once

#include "swerve_controller/kinematics.hpp"

namespace swerve {

struct Pose2D {
  double x = 0.0, y = 0.0, yaw = 0.0;  // [m], [m], [rad] (−π, π]
};

struct OdomConfig {
  Modules modules{};
  double wheel_radius = 0.0525;  // [m]
  double max_wheel_mps = 1.0;    // [m/s] 이보다 빠른 변위 = 점프로 보고 버림 (보드 vel_limit 2 turn/s = 0.66 m/s)
  double jump_margin_m = 0.02;   // [m] 위 판정 여유 (Hall 1 카운트 ≈ 5.5 mm)
  double max_dt = 0.5;           // [s] 샘플 간격이 이보다 길면 버림
};

class SwerveOdometry {
 public:
  enum class Result { Init, Ok, BadDt, Gap, Jump };

  explicit SwerveOdometry(OdomConfig cfg);
  void reset(const Pose2D& p = {});  // 포즈를 p 로, 다음 샘플은 Init
  void resync();                     // 포즈는 유지, 다음 샘플은 Init (센서 끊겼을 때)

  // t [s], wheel_pos [turn], steer_angle [rad]
  Result update(double t, const Arr& wheel_pos, const Arr& steer_angle);

  const Pose2D& pose() const { return pose_; }
  const Twist2D& twist() const { return twist_; }  // body 프레임, 마지막 Ok 구간 평균
  double distance() const { return dist_; }        // 누적 주행 거리 [m] (차체 병진, 진단용)
  const OdomConfig& config() const { return cfg_; }

 private:
  void store(double t, const Arr& pos, const Arr& ang);
  OdomConfig cfg_;
  Pose2D pose_{};
  Twist2D twist_{};
  double dist_ = 0.0;
  bool has_prev_ = false;
  double prev_t_ = 0.0;
  Arr prev_pos_{}, prev_ang_{};
};

const char* to_string(SwerveOdometry::Result r);

}  // namespace swerve
