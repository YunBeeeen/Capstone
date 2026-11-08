// swerve_controller — 차체 속도(cmd_vel) → 조향 4 + 구동 4 명령
//
//   sub  cmd_vel      geometry_msgs/Twist            (x, y [m/s], angular.z [rad/s], REP-103)
//   sub  steer_state  capstone_msgs/SteerStateArray  (실제 조향각 — 정렬 게이트용)
//   sub  wheel_state  capstone_msgs/WheelStateArray  (실제 바퀴 속도 — 큰 조향 전환 전 정지 확인, 되감기 조건)
//   pub  steer_cmd    capstone_msgs/SteerCommand     (FREE / POSITION, [rad])
//   pub  wheel_cmd    capstone_msgs/WheelCommand     (FREE / DRIVE / HOLD, [turn/s]) → wheel_can_bridge
//   srv  ~/set_free   std_srvs/SetBool               (true = 전부 FREE 고정, false = 해제)
//
// 상태
//   FREE   : 시작 상태. 조향 토크 끔 + 구동 IDLE. 신선한 cmd_vel 이 오면 ACTIVE (set_free(true) 로 고정했으면 안 감)
//   ACTIVE : 매 주기 IK → ±한계 접기 → 정렬 게이트. cmd_vel 이 cmd_timeout 넘게 끊기면 구동 HOLD + 조향 각 유지
// 계산은 kinematics.hpp (ksrc 로버 펌웨어 포팅). 이 파일은 ROS 연결만.
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "capstone_msgs/msg/steer_command.hpp"
#include "capstone_msgs/msg/steer_state_array.hpp"
#include "capstone_msgs/msg/wheel_state_array.hpp"
#include "capstone_msgs/msg/wheel_command.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "swerve_controller/kinematics.hpp"

using capstone_msgs::msg::SteerCommand;
using capstone_msgs::msg::SteerStateArray;
using capstone_msgs::msg::WheelStateArray;
using capstone_msgs::msg::WheelCommand;
using geometry_msgs::msg::Twist;

namespace swerve {

namespace {
constexpr double PI = 3.14159265358979323846;
double mono_now() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace

class SwerveNode : public rclcpp::Node {
 public:
  SwerveNode() : Node("swerve_controller") {
    const auto mx = declare_parameter<std::vector<double>>("module_x", {0.0911, -0.0911, -0.0911, 0.0911});
    const auto my = declare_parameter<std::vector<double>>("module_y", {-0.13605, -0.13605, 0.13605, 0.13605});
    if (mx.size() != N || my.size() != N) throw std::invalid_argument("module_x / module_y 길이 != 4");
    wheel_radius_ = declare_parameter<double>("wheel_radius", 0.0525);
    const double limit = declare_parameter<double>("steer_limit_deg", 180.0) * PI / 180.0;
    require_steer_ = declare_parameter<bool>("require_steer_feedback", true);
    cmd_timeout_ = declare_parameter<double>("cmd_timeout", 0.3);
    steer_timeout_ = declare_parameter<double>("steer_state_timeout", 0.2);
    const double rate = declare_parameter<double>("control_rate_hz", 50.0);
    gate_warn_s_ = declare_parameter<double>("gate_warn_s", 1.5);

    ControllerConfig c;
    for (std::size_t i = 0; i < N; ++i) {
      c.modules[i] = {mx[i], my[i]};
      c.lo[i] = -limit;
      c.hi[i] = limit;
    }
    c.max_wheel_mps = declare_parameter<double>("max_wheel_mps", 0.6);
    c.min_steer_mps = declare_parameter<double>("steer_min_mps", 0.03);
    c.unwind_rad = declare_parameter<double>("unwind_deg", 0.0) * PI / 180.0;
    c.slow_mps = declare_parameter<double>("unwind_slow_mps", 0.02);
    // 조향 피드백 없이 돌리는 시험(require_steer_feedback=false)이면 게이트를 끔 — 실제각을 모르면 영영 안 풀리므로
    c.gate_trigger_rad = require_steer_ ? declare_parameter<double>("gate_trigger_deg", 60.0) * PI / 180.0 : 0.0;
    c.gate_release_rad = declare_parameter<double>("gate_release_deg", 10.0) * PI / 180.0;
    const double max_hold_s = declare_parameter<double>("gate_max_hold_s", 0.0);
    c.gate_max_hold_ticks = static_cast<unsigned>(std::lround(max_hold_s * rate));
    // 큰 조향 전환(±170° 이음매 반전 등) 전에 바퀴를 먼저 세움 — 구르는 채로 조향이 돌면 차체가 넘어짐
    c.flip_stop_rad = declare_parameter<double>("flip_stop_deg", 60.0) * PI / 180.0;
    c.stop_speed_mps = declare_parameter<double>("stop_speed_mps", 0.03);
    c.est_decel_mps2 = declare_parameter<double>("est_decel_mps2", 0.66);
    c.dt = 1.0 / rate;
    c.max_steer_rate = declare_parameter<double>("max_steer_rate_deg_s", 360.0) * PI / 180.0;
    wheel_timeout_ = declare_parameter<double>("wheel_state_timeout", 0.3);
    ctl_ = std::make_unique<Controller>(c);

    steer_pub_ = create_publisher<SteerCommand>("steer_cmd", 10);
    wheel_pub_ = create_publisher<WheelCommand>("wheel_cmd", 10);
    cmd_sub_ = create_subscription<Twist>("cmd_vel", 10, [this](const Twist& m) { on_cmd(m); });
    wheel_sub_ = create_subscription<WheelStateArray>(
        "wheel_state", 10, [this](const WheelStateArray& m) { on_wheel(m); });
    steer_sub_ = create_subscription<SteerStateArray>(
        "steer_state", 10, [this](const SteerStateArray& m) { on_steer(m); });
    free_srv_ = create_service<std_srvs::srv::SetBool>(
        "~/set_free", [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                             std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
          free_latched_ = req->data;
          res->success = true;
          res->message = free_latched_ ? "FREE 고정" : "FREE 고정 해제 — cmd_vel 오면 ACTIVE";
          RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
        });
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this] { tick(); });

    RCLCPP_INFO(get_logger(), "시작: FREE. 조향 한계 ±%.0f°, 바퀴 반지름 %.4f m, 조향 피드백 %s", limit * 180 / PI,
                wheel_radius_, require_steer_ ? "필요(게이트 ON)" : "불필요(게이트 OFF — 시험용)");
  }

 private:
  void on_cmd(const Twist& m) {
    if (!std::isfinite(m.linear.x) || !std::isfinite(m.linear.y) || !std::isfinite(m.angular.z)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "cmd_vel 에 NaN/inf — 무시");
      return;
    }
    cmd_ = {m.linear.x, m.linear.y, m.angular.z};
    cmd_time_ = mono_now();
  }

  void on_wheel(const WheelStateArray& m) {
    if (m.wheels.size() != N) return;
    bool alive = true;
    for (std::size_t i = 0; i < N; ++i) {
      wheel_[i] = m.wheels[i].velocity * 2.0 * PI * wheel_radius_;  // turn/s → m/s
      alive = alive && m.wheels[i].alive && std::isfinite(m.wheels[i].velocity);
    }
    wheel_alive_ = alive;
    wheel_time_ = mono_now();
  }

  void on_steer(const SteerStateArray& m) {
    if (m.angle.size() != N) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "steer_state angle 길이 %zu != 4", m.angle.size());
      return;
    }
    for (std::size_t i = 0; i < N; ++i) steer_[i] = m.angle[i];
    steer_ready_ = m.ready;
    steer_time_ = mono_now();
  }

  bool steer_ok(double now) const { return steer_ready_ && now - steer_time_ <= steer_timeout_; }

  void tick() {
    const double now = mono_now();
    const bool fresh = now - cmd_time_ <= cmd_timeout_;
    const bool s_ok = steer_ok(now);

    if (free_latched_ || (!active_ && !fresh)) {
      if (active_) RCLCPP_INFO(get_logger(), "→ FREE");
      active_ = false;
      publish_free();
      return;
    }
    if (!active_) {
      // 손으로 돌려 둔 조향각에서 이어가도록 이력을 실제각으로 맞춤
      if (s_ok) ctl_->seed(steer_);
      else ctl_->reset();
      active_ = true;
      gated_since_ = -1.0;
      RCLCPP_INFO(get_logger(), "→ ACTIVE (조향 이력 %s)", s_ok ? "실제각에서 시작" : "0° 에서 시작 — steer_state 없음");
    }

    const Twist2D t = fresh ? cmd_ : Twist2D{};
    const bool w_ok = wheel_alive_ && now - wheel_time_ <= wheel_timeout_;
    const auto r = ctl_->step(t, steer_, s_ok, wheel_, w_ok);
    if (r.stopping && !was_stopping_)
      RCLCPP_INFO(get_logger(), "큰 조향 전환 → 바퀴 먼저 정지 (%s)", w_ok ? "wheel_state 로 확인" : "피드백 없음 — 감속 추정");
    was_stopping_ = r.stopping;

    SteerCommand sc;
    sc.header.stamp = now_stamp();
    sc.mode = SteerCommand::MODE_POSITION;
    for (auto& c : r.cmd) sc.angle.push_back(static_cast<float>(c.angle));
    steer_pub_->publish(sc);

    WheelCommand wc;
    wc.header.stamp = sc.header.stamp;
    if (fresh) {
      wc.mode = WheelCommand::MODE_DRIVE;
      for (double v : r.wheel_speed) wc.velocity.push_back(static_cast<float>(v / (2.0 * PI * wheel_radius_)));
    } else {
      wc.mode = WheelCommand::MODE_HOLD;  // 명령 끊김 → 제자리 버팀 (조향은 마지막 각 유지)
    }
    wheel_pub_->publish(wc);

    // 게이트에 오래 걸려 있으면 알림 (조향 브리지 꺼짐 / 기구 간섭 등)
    if (r.gated && fresh) {
      if (gated_since_ < 0) gated_since_ = now;
      if (now - gated_since_ > gate_warn_s_)
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "조향 정렬 대기 %.1f s — %s", now - gated_since_,
                             s_ok ? "조향이 목표에 안 닿음 (간섭/토크 부족?)" : "steer_state 없음/준비 안 됨");
    } else {
      gated_since_ = -1.0;
    }
  }

  void publish_free() {
    SteerCommand sc;
    sc.header.stamp = now_stamp();
    sc.mode = SteerCommand::MODE_FREE;
    steer_pub_->publish(sc);
    WheelCommand wc;
    wc.header.stamp = sc.header.stamp;
    wc.mode = WheelCommand::MODE_FREE;
    wheel_pub_->publish(wc);
  }

  builtin_interfaces::msg::Time now_stamp() { return get_clock()->now(); }

  std::unique_ptr<Controller> ctl_;
  double wheel_radius_, cmd_timeout_, steer_timeout_, gate_warn_s_;
  bool require_steer_;
  Twist2D cmd_{};
  double cmd_time_ = -1e18;
  Arr steer_{};
  bool steer_ready_ = false;
  double steer_time_ = -1e18;
  Arr wheel_{};
  bool wheel_alive_ = false, was_stopping_ = false;
  double wheel_time_ = -1e18, wheel_timeout_ = 0.3;
  bool active_ = false, free_latched_ = false;
  double gated_since_ = -1.0;

  rclcpp::Publisher<SteerCommand>::SharedPtr steer_pub_;
  rclcpp::Publisher<WheelCommand>::SharedPtr wheel_pub_;
  rclcpp::Subscription<Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<SteerStateArray>::SharedPtr steer_sub_;
  rclcpp::Subscription<WheelStateArray>::SharedPtr wheel_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr free_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace swerve

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<swerve::SwerveNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
