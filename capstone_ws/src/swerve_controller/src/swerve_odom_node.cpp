// swerve_odom — 바퀴 엔코더 + 조향각 → 오도메트리
//
//   sub  wheel_state  capstone_msgs/WheelStateArray  (position [turn], 바퀴 기준 부호 — wheel_can_bridge 가 wheel_sign 적용)
//   sub  steer_state  capstone_msgs/SteerStateArray  (angle [rad])
//   pub  odom         nav_msgs/Odometry              (frame odom_frame → child base_frame, twist 는 child 기준)
//   tf   odom_frame → base_frame                     (publish_tf=true 일 때. EKF 를 쓰면 false — TF 는 EKF 가 냄)
//   srv  ~/reset      std_srvs/Trigger               (포즈 0 으로)
//
// wheel_state 가 올 때마다 적분 (실기 20 Hz, FREE 5 Hz — 손으로 밀어도 추적). 계산은 odometry.hpp.
// base_frame 기본값 'world' = 지금 URDF 의 루트 링크 (바닥 기준점, base_footprint 역할). URDF 를 바꾸면 같이 바꿀 것.
#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "capstone_msgs/msg/steer_state_array.hpp"
#include "capstone_msgs/msg/wheel_state_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "swerve_controller/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"

using capstone_msgs::msg::SteerStateArray;
using capstone_msgs::msg::WheelStateArray;

namespace swerve {

class SwerveOdomNode : public rclcpp::Node {
 public:
  SwerveOdomNode() : Node("swerve_odom") {
    const auto mx = declare_parameter<std::vector<double>>("module_x", {0.0911, -0.0911, -0.0911, 0.0911});
    const auto my = declare_parameter<std::vector<double>>("module_y", {-0.13605, -0.13605, 0.13605, 0.13605});
    if (mx.size() != N || my.size() != N) throw std::invalid_argument("module_x / module_y 길이 != 4");
    OdomConfig c;
    for (std::size_t i = 0; i < N; ++i) c.modules[i] = {mx[i], my[i]};
    c.wheel_radius = declare_parameter<double>("wheel_radius", 0.0525);
    c.max_wheel_mps = declare_parameter<double>("max_wheel_mps_jump", 1.0);
    c.jump_margin_m = declare_parameter<double>("jump_margin_m", 0.02);
    c.max_dt = declare_parameter<double>("max_dt", 0.5);
    odom_ = std::make_unique<SwerveOdometry>(c);

    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "world");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    steer_timeout_ = declare_parameter<double>("steer_state_timeout", 0.2);
    pose_cov_ = declare_parameter<std::vector<double>>("pose_covariance_diagonal", {1e-3, 1e-3, 1e-2});
    twist_cov_ = declare_parameter<std::vector<double>>("twist_covariance_diagonal", {1e-3, 1e-3, 1e-2});
    if (pose_cov_.size() != 3 || twist_cov_.size() != 3)
      throw std::invalid_argument("*_covariance_diagonal 길이 != 3 (x, y, yaw)");

    pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 20);
    if (publish_tf_) tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    steer_sub_ = create_subscription<SteerStateArray>(
        "steer_state", 20, [this](const SteerStateArray& m) { on_steer(m); });
    wheel_sub_ = create_subscription<WheelStateArray>(
        "wheel_state", 20, [this](const WheelStateArray& m) { on_wheel(m); });
    reset_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/reset", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          odom_->reset();
          res->success = true;
          res->message = "포즈 0 으로";
          RCLCPP_INFO(get_logger(), "reset — 포즈 0");
        });

    RCLCPP_INFO(get_logger(), "시작: %s → %s, TF %s, 바퀴 반지름 %.4f m", odom_frame_.c_str(), base_frame_.c_str(),
                publish_tf_ ? "발행" : "안 함 (EKF 등이 냄)", c.wheel_radius);
  }

 private:
  void on_steer(const SteerStateArray& m) {
    if (m.angle.size() != N) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "steer_state angle 길이 %zu != 4", m.angle.size());
      steer_ok_ = false;
      return;
    }
    // 실기 브리지: 모터별 alive + confirmed (켤 때 각도 확정). 시뮬은 motors 없이 ready 만 줌.
    bool ok = true;
    if (m.motors.size() == N) {
      for (const auto& s : m.motors) ok = ok && s.alive && s.confirmed;
    } else {
      ok = m.ready;
    }
    for (std::size_t i = 0; i < N; ++i) {
      steer_[i] = m.angle[i];
      ok = ok && std::isfinite(steer_[i]);
    }
    steer_ok_ = ok;
    steer_time_ = now().seconds();
  }

  void on_wheel(const WheelStateArray& m) {
    const double wall = now().seconds();
    std::string why;
    if (m.wheels.size() != N) why = "wheel_state 바퀴 수 != 4";
    else if (!steer_ok_) why = "조향각 모름 (steer_state 없음 / 미확정 / 통신 끊김)";
    else if (wall - steer_time_ > steer_timeout_) why = "steer_state 오래됨";
    Arr pos{};
    if (why.empty()) {
      for (std::size_t i = 0; i < N; ++i) {
        if (!m.wheels[i].alive || !std::isfinite(m.wheels[i].position)) {
          why = "바퀴 " + std::to_string(m.wheels[i].can_id) + " heartbeat 없음";
          break;
        }
        pos[i] = m.wheels[i].position;
      }
    }
    if (!why.empty()) {
      odom_->resync();  // 포즈는 유지, 다시 들어오면 그 시점부터 이어서
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "적분 멈춤: %s", why.c_str());
      return;
    }

    const rclcpp::Time stamp = (m.header.stamp.sec == 0 && m.header.stamp.nanosec == 0)
                                   ? now()
                                   : rclcpp::Time(m.header.stamp, get_clock()->get_clock_type());
    const auto r = odom_->update(stamp.seconds(), pos, steer_);
    switch (r) {
      case SwerveOdometry::Result::Init:
        RCLCPP_INFO(get_logger(), "적분 시작 (포즈 x=%.3f y=%.3f yaw=%.1f°)", odom_->pose().x, odom_->pose().y,
                    odom_->pose().yaw * 180.0 / 3.14159265358979323846);
        break;
      case SwerveOdometry::Result::BadDt:
        return;  // 같은/이전 시각 샘플 — 무시
      case SwerveOdometry::Result::Gap:
        RCLCPP_WARN(get_logger(), "wheel_state 간격 > max_dt — 이 구간 버림");
        break;
      case SwerveOdometry::Result::Jump:
        RCLCPP_WARN(get_logger(), "바퀴 위치 점프 (보드 재부팅?) — 이 구간 버림");
        break;
      case SwerveOdometry::Result::Ok:
        break;
    }
    publish(stamp);
  }

  void publish(const rclcpp::Time& stamp) {
    const auto& p = odom_->pose();
    const auto& v = odom_->twist();
    nav_msgs::msg::Odometry o;
    o.header.stamp = stamp;
    o.header.frame_id = odom_frame_;
    o.child_frame_id = base_frame_;
    o.pose.pose.position.x = p.x;
    o.pose.pose.position.y = p.y;
    o.pose.pose.orientation.z = std::sin(0.5 * p.yaw);
    o.pose.pose.orientation.w = std::cos(0.5 * p.yaw);
    o.twist.twist.linear.x = v.vx;
    o.twist.twist.linear.y = v.vy;
    o.twist.twist.angular.z = v.omega;
    // 6×6 행 우선 (x y z roll pitch yaw). 2D 라 z/roll/pitch 는 크게 (모름)
    constexpr std::array<int, 3> IDX = {0, 7, 35};
    for (int k : {14, 21, 28}) o.pose.covariance[k] = o.twist.covariance[k] = 1e6;
    for (std::size_t i = 0; i < 3; ++i) {
      o.pose.covariance[IDX[i]] = pose_cov_[i];
      o.twist.covariance[IDX[i]] = twist_cov_[i];
    }
    pub_->publish(o);

    if (tf_) {
      geometry_msgs::msg::TransformStamped t;
      t.header = o.header;
      t.child_frame_id = base_frame_;
      t.transform.translation.x = p.x;
      t.transform.translation.y = p.y;
      t.transform.rotation = o.pose.pose.orientation;
      tf_->sendTransform(t);
    }
  }

  std::unique_ptr<SwerveOdometry> odom_;
  std::string odom_frame_, base_frame_;
  bool publish_tf_ = true;
  double steer_timeout_ = 0.2;
  std::vector<double> pose_cov_, twist_cov_;
  Arr steer_{};
  bool steer_ok_ = false;
  double steer_time_ = -1e18;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::Subscription<SteerStateArray>::SharedPtr steer_sub_;
  rclcpp::Subscription<WheelStateArray>::SharedPtr wheel_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
};

}  // namespace swerve

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<swerve::SwerveOdomNode>());
  rclcpp::shutdown();
  return 0;
}
