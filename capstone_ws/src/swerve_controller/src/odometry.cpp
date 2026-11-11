#include "swerve_controller/odometry.hpp"

#include <cmath>

namespace swerve {

namespace {
constexpr double PI = 3.14159265358979323846;
double wrap(double a) {
  a = std::fmod(a + PI, 2.0 * PI);
  if (a <= 0.0) a += 2.0 * PI;
  return a - PI;
}
}  // namespace

SwerveOdometry::SwerveOdometry(OdomConfig cfg) : cfg_(cfg) {}

void SwerveOdometry::reset(const Pose2D& p) {
  pose_ = p;
  pose_.yaw = wrap(p.yaw);
  twist_ = {};
  dist_ = 0.0;
  has_prev_ = false;
}

void SwerveOdometry::resync() {
  twist_ = {};
  has_prev_ = false;
}

void SwerveOdometry::store(double t, const Arr& pos, const Arr& ang) {
  prev_t_ = t;
  prev_pos_ = pos;
  prev_ang_ = ang;
  has_prev_ = true;
}

SwerveOdometry::Result SwerveOdometry::update(double t, const Arr& wheel_pos, const Arr& steer_angle) {
  if (!has_prev_) {
    store(t, wheel_pos, steer_angle);
    return Result::Init;
  }
  const double dt = t - prev_t_;
  if (dt <= 0.0) return Result::BadDt;  // 기준 유지 (중복/역행 샘플만 무시)
  if (dt > cfg_.max_dt) {
    twist_ = {};
    store(t, wheel_pos, steer_angle);
    return Result::Gap;
  }

  const double k = 2.0 * PI * cfg_.wheel_radius;
  const double limit = cfg_.max_wheel_mps * dt + cfg_.jump_margin_m;
  Arr d{}, mid{};
  for (std::size_t i = 0; i < N; ++i) {
    d[i] = (wheel_pos[i] - prev_pos_[i]) * k;
    // 조향각은 wrap 안 된 연속각 (±190° 범위) 이라 그냥 평균
    mid[i] = 0.5 * (prev_ang_[i] + steer_angle[i]);
    if (!std::isfinite(d[i]) || !std::isfinite(mid[i]) || std::fabs(d[i]) > limit) {
      twist_ = {};
      store(t, wheel_pos, steer_angle);
      return Result::Jump;
    }
  }

  const Twist2D delta = fk_compute(cfg_.modules, mid, d);  // body 프레임 변위 (dx, dy, dθ)
  const double h = pose_.yaw + 0.5 * delta.omega;           // 중간 방위
  const double c = std::cos(h), s = std::sin(h);
  pose_.x += c * delta.vx - s * delta.vy;
  pose_.y += s * delta.vx + c * delta.vy;
  pose_.yaw = wrap(pose_.yaw + delta.omega);
  dist_ += std::hypot(delta.vx, delta.vy);
  twist_ = {delta.vx / dt, delta.vy / dt, delta.omega / dt};
  store(t, wheel_pos, steer_angle);
  return Result::Ok;
}

const char* to_string(SwerveOdometry::Result r) {
  switch (r) {
    case SwerveOdometry::Result::Init: return "init";
    case SwerveOdometry::Result::Ok: return "ok";
    case SwerveOdometry::Result::BadDt: return "bad_dt";
    case SwerveOdometry::Result::Gap: return "gap";
    case SwerveOdometry::Result::Jump: return "jump";
  }
  return "?";
}

}  // namespace swerve
