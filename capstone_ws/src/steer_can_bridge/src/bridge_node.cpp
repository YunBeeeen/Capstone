// steer_can_bridge — 조향 4축 (RobStride 02, MIT 위치 제어, 1 Mbps) ↔ ROS2
//
//   sub  steer_cmd    capstone_msgs/SteerCommand     (FREE / POSITION, 조향 기준 [rad])
//   pub  steer_state  capstone_msgs/SteerStateArray  (phase, ready, fault, 실제 조향각, 축별 상태)
//   srv  ~/clear_errors  std_srvs/Trigger            (모터 고장 클리어 + FAULT 해제)
//
// 상태기계는 supervisor.hpp. 이 파일은 소켓 ↔ 상태기계 ↔ ROS 연결만.
// 종료(Ctrl+C) 시 전 축 토크 끔 (Type04).
#include <cerrno>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "capstone_msgs/msg/steer_command.hpp"
#include "capstone_msgs/msg/steer_state_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "steer_can_bridge/socketcan.hpp"
#include "steer_can_bridge/supervisor.hpp"

using capstone_msgs::msg::SteerCommand;
using capstone_msgs::msg::SteerStateArray;

namespace steer_can {

namespace {
double mono_now() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
const char* phase_name(Phase p) {
  switch (p) {
    case Phase::FREE: return "FREE";
    case Phase::ENTERING: return "ENTERING";
    case Phase::ACTIVE: return "ACTIVE";
    case Phase::FAULT: return "FAULT";
  }
  return "?";
}
constexpr double kDeg = M_PI / 180.0;
}  // namespace

class BridgeNode : public rclcpp::Node {
 public:
  BridgeNode() : Node("steer_can_bridge") {
    const auto ifname = declare_parameter<std::string>("can_interface", "can_steer");
    SupervisorConfig cfg;
    const auto ids = declare_parameter<std::vector<int64_t>>("can_ids", {1, 2, 3, 4});
    cfg.can_ids.assign(ids.begin(), ids.end());
    cfg.sign = declare_parameter<std::vector<double>>("steer_sign", std::vector<double>(ids.size(), 1.0));
    cfg.host_id = static_cast<uint8_t>(declare_parameter<int64_t>("host_id", 0xFD));
    cfg.steer_limit = declare_parameter<double>("steer_limit_deg", 180.0) * kDeg;
    cfg.confirm_limit = declare_parameter<double>("confirm_limit_deg", 165.0) * kDeg;
    cfg.kp = declare_parameter<double>("kp", 20.0);
    cfg.kd = declare_parameter<double>("kd", 2.0);
    cfg.max_rate = declare_parameter<double>("max_rate", 2.0);
    cfg.cmd_timeout = declare_parameter<double>("cmd_timeout", 0.2);
    cfg.fb_timeout = declare_parameter<double>("fb_timeout", 0.15);
    cfg.free_poll_period = 1.0 / declare_parameter<double>("free_poll_hz", 20.0);
    cfg.enter_timeout = declare_parameter<double>("enter_timeout", 1.0);
    cfg.retry_period = declare_parameter<double>("retry_period", 0.2);
    cfg.motor_can_timeout = declare_parameter<double>("motor_can_timeout", 0.5);
    const double control_hz = declare_parameter<double>("control_rate_hz", 200.0);
    state_period_ = 1.0 / declare_parameter<double>("state_rate_hz", 50.0);
    temp_warn_ = declare_parameter<double>("temp_warn", 70.0);

    sup_ = std::make_unique<Supervisor>(cfg);
    can_ = std::make_unique<SocketCan>(ifname);

    pub_ = create_publisher<SteerStateArray>("steer_state", 10);
    sub_ = create_subscription<SteerCommand>("steer_cmd", 10, [this](const SteerCommand& m) { on_cmd(m); });
    srv_ = create_service<std_srvs::srv::Trigger>(
        "~/clear_errors", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                 std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          const std::string was = sup_->fault();
          send_all(sup_->clear_errors(mono_now()));
          res->success = true;
          res->message = was.empty() ? "클리어 송신 (FAULT 아니었음)" : "FAULT 해제: " + was;
          RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
        });
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / control_hz), [this] { tick(); });

    std::string idstr;
    for (int id : cfg.can_ids) idstr += " " + std::to_string(id);
    RCLCPP_INFO(get_logger(), "[%s] can_ids:%s  MIT kp %.1f kd %.2f  램프 %.2f rad/s  제어 %.0f Hz  한계 ±%.0f° (확정 ±%.0f°)",
                ifname.c_str(), idstr.c_str(), cfg.kp, cfg.kd, cfg.max_rate, control_hz, cfg.steer_limit / kDeg,
                cfg.confirm_limit / kDeg);
  }

  void stop_all() {
    send_all(sup_->shutdown_frames());
    RCLCPP_INFO(get_logger(), "종료: 전 축 토크 끔 (Type04) 송신");
  }

 private:
  void on_cmd(const SteerCommand& m) {
    std::vector<double> a(m.angle.begin(), m.angle.end());
    const auto err = sup_->set_command(static_cast<Mode>(m.mode), a, mono_now());
    if (!err.empty()) RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "명령 무시: %s", err.c_str());
  }

  void tick() {
    const double now = mono_now();
    while (auto f = can_->recv()) sup_->on_frame(*f, now);

    const Phase before = sup_->phase();
    send_all(sup_->step(now));
    const Phase after = sup_->phase();
    if (after != before) {
      if (after == Phase::FAULT)
        RCLCPP_ERROR(get_logger(), "%s → FAULT: %s  (해제: ros2 service call /steer_can_bridge/clear_errors "
                     "std_srvs/srv/Trigger)", phase_name(before), sup_->fault().c_str());
      else
        RCLCPP_INFO(get_logger(), "%s → %s", phase_name(before), phase_name(after));
    }
    if (!sup_->blocked().empty())
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "POSITION 명령 대기 중 — %s", sup_->blocked().c_str());
    if (after == Phase::ACTIVE) {
      const bool fresh = sup_->cmd_fresh(now);
      if (was_fresh_ && !fresh) RCLCPP_INFO(get_logger(), "steer_cmd 끊김 — 그 자리 유지");
      was_fresh_ = fresh;
    } else {
      was_fresh_ = true;
    }
    const bool rdy = sup_->ready(now);
    if (!was_ready_ && rdy) RCLCPP_INFO(get_logger(), "전 축 응답 + 각도 확정 — 명령 대기");
    was_ready_ = rdy;

    if (now >= next_state_) {
      publish_state(now);
      next_state_ = now + state_period_;
    }
  }

  void send_all(const std::vector<rs02::Frame>& frames) {
    for (const auto& f : frames) {
      if (!can_->send(f)) {
        const int e = can_->last_errno();
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "CAN 송신 실패 (errno %d) — %s", e,
                             e == ENOBUFS ? "송신 큐 가득: 버스에 ACK 없음(모터 전원/배선/1 Mbps) 이거나 txqueuelen 부족"
                                          : "모터 전원 / CANH·CANL / 커넥터 확인");
        return;
      }
    }
  }

  void publish_state(double now) {
    SteerStateArray msg;
    msg.header.stamp = get_clock()->now();
    msg.phase = static_cast<uint8_t>(sup_->phase());
    msg.ready = sup_->ready(now);
    msg.fault = sup_->fault();
    const auto& ms = sup_->motors();
    for (size_t i = 0; i < ms.size(); ++i) {
      const auto& m = ms[i];
      capstone_msgs::msg::SteerState s;
      s.can_id = static_cast<uint8_t>(m.can_id);
      s.alive = sup_->alive(i, now);
      s.confirmed = m.confirmed;
      s.mode_status = m.mode_status;
      s.fault_bits = m.fault_bits;
      s.angle = static_cast<float>(m.angle);
      s.velocity = static_cast<float>(m.vel);
      s.torque = static_cast<float>(m.tor);
      s.temperature = static_cast<float>(m.temp);
      s.angle_cmd = static_cast<float>(m.angle_cmd);
      msg.angle.push_back(s.angle);
      msg.motors.push_back(s);
      if (std::isfinite(m.temp) && m.temp > temp_warn_)
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s 온도 %.1f °C (> %.0f)", axis_name(i).c_str(),
                             m.temp, temp_warn_);
    }
    pub_->publish(msg);
  }

  std::unique_ptr<Supervisor> sup_;
  std::unique_ptr<SocketCan> can_;
  rclcpp::Publisher<SteerStateArray>::SharedPtr pub_;
  rclcpp::Subscription<SteerCommand>::SharedPtr sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  double state_period_ = 0.02;
  double next_state_ = 0.0;
  double temp_warn_ = 70.0;
  bool was_ready_ = false;
  bool was_fresh_ = true;
};

}  // namespace steer_can

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<steer_can::BridgeNode> node;
  try {
    node = std::make_shared<steer_can::BridgeNode>();
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("steer_can_bridge"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(node);
  node->stop_all();
  rclcpp::shutdown();
  return 0;
}
