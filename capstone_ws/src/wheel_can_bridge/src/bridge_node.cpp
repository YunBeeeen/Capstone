// wheel_can_bridge — 구동 바퀴 4축 (XDrive ×2, ODrive fw 0.5.1 CANSimple) ↔ ROS2
//
//   sub  wheel_cmd    capstone_msgs/WheelCommand     (FREE / DRIVE / HOLD, 바퀴 기준 turn/s)
//   pub  wheel_state  capstone_msgs/WheelStateArray  (phase, fault, 바퀴별 상태·속도·Iq)
//   srv  ~/clear_errors  std_srvs/Trigger            (보드 에러 클리어 + FAULT 해제)
//
// 상태기계는 supervisor.hpp. 이 파일은 소켓 ↔ 상태기계 ↔ ROS 연결만.
// 종료(Ctrl+C) 시 전 축 속도 0 + IDLE.
#include <cerrno>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "capstone_msgs/msg/wheel_command.hpp"
#include "capstone_msgs/msg/wheel_state_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "wheel_can_bridge/socketcan.hpp"
#include "wheel_can_bridge/supervisor.hpp"

using namespace std::chrono_literals;
using capstone_msgs::msg::WheelCommand;
using capstone_msgs::msg::WheelStateArray;

namespace wheel_can {

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
}  // namespace

class BridgeNode : public rclcpp::Node {
 public:
  BridgeNode() : Node("wheel_can_bridge") {
    const auto ifname = declare_parameter<std::string>("can_interface", "can_drive");
    SupervisorConfig cfg;
    const auto ids = declare_parameter<std::vector<int64_t>>("can_ids", {1, 2, 3, 4});
    cfg.can_ids.assign(ids.begin(), ids.end());
    cfg.sign = declare_parameter<std::vector<double>>("wheel_sign", std::vector<double>(ids.size(), 1.0));
    cfg.vel_max = declare_parameter<double>("vel_max", 2.0);
    cfg.cmd_timeout = declare_parameter<double>("cmd_timeout", 0.2);
    cfg.hb_timeout = declare_parameter<double>("hb_timeout", 0.5);
    cfg.enter_timeout = declare_parameter<double>("enter_timeout", 3.0);
    cfg.retry_period = declare_parameter<double>("retry_period", 0.5);
    cfg.encoder_recover = declare_parameter<bool>("encoder_recover", true);
    cfg.recover_max = static_cast<int>(declare_parameter<int64_t>("recover_max", 10));
    cfg.recover_window = declare_parameter<double>("recover_window", 10.0);
    cfg.recover_timeout = declare_parameter<double>("recover_timeout", 1.0);
    cfg.recover_retry = declare_parameter<double>("recover_retry", 0.2);
    cfg.clear_on_start = declare_parameter<bool>("clear_on_start", true);
    const double control_hz = declare_parameter<double>("control_rate_hz", 50.0);
    feedback_period_ = 1.0 / declare_parameter<double>("feedback_rate_hz", 20.0);
    feedback_free_period_ = 1.0 / declare_parameter<double>("feedback_rate_free_hz", 5.0);
    max_rtr_per_tick_ = static_cast<size_t>(declare_parameter<int64_t>("max_rtr_per_tick", 4));
    vbus_warn_ = declare_parameter<double>("vbus_warn", 22.5);

    sup_ = std::make_unique<Supervisor>(cfg);
    can_ = std::make_unique<SocketCan>(ifname);

    pub_ = create_publisher<WheelStateArray>("wheel_state", 10);
    sub_ = create_subscription<WheelCommand>(
        "wheel_cmd", 10, [this](const WheelCommand& m) { on_cmd(m); });
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
    RCLCPP_INFO(get_logger(), "[%s] can_ids:%s  제어 %.0f Hz / 피드백 %.0f Hz (FREE %.0f Hz)  vel_max %.2f turn/s",
                ifname.c_str(), idstr.c_str(), control_hz, 1.0 / feedback_period_, 1.0 / feedback_free_period_,
                cfg.vel_max);
  }

  // 종료 시 호출 (rclcpp 가 내려간 뒤에도 소켓은 살아 있음)
  void stop_all() {
    send_all(sup_->shutdown_frames());
    RCLCPP_INFO(get_logger(), "종료: 전 축 속도 0 + IDLE 송신");
  }

 private:
  void on_cmd(const WheelCommand& m) {
    std::vector<double> v(m.velocity.begin(), m.velocity.end());
    const auto err = sup_->set_command(static_cast<Mode>(m.mode), v, mono_now());
    if (!err.empty()) RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "명령 무시: %s", err.c_str());
  }

  void tick() {
    const double now = mono_now();
    while (auto f = can_->recv()) sup_->on_frame(*f, now);

    const Phase before = sup_->phase();
    const auto ctrl = sup_->step(now);
    send_all(ctrl);
    const Phase after = sup_->phase();
    if (sup_->start_clears() != last_start_clears_) {
      last_start_clears_ = sup_->start_clears();
      RCLCPP_WARN(get_logger(), "%s", sup_->last_start_clear().c_str());
    }
    if (sup_->recoveries() != last_recoveries_) {
      last_recoveries_ = sup_->recoveries();
      rec_start_ = now;
      RCLCPP_WARN(get_logger(), "%s  [누적 %d회]", sup_->last_recovery().c_str(), last_recoveries_);
    }
    const bool rec = sup_->recovering();
    if (was_recovering_ && !rec && after == Phase::ACTIVE)
      RCLCPP_INFO(get_logger(), "자동 복구 완료 (%.0f ms) — 주행 재개", (now - rec_start_) * 1000.0);
    was_recovering_ = rec;
    if (after != before) {
      if (after == Phase::FAULT)
        RCLCPP_ERROR(get_logger(), "%s → FAULT: %s  (해제: ros2 service call /wheel_can_bridge/clear_errors "
                     "std_srvs/srv/Trigger)", phase_name(before), sup_->fault().c_str());
      else
        RCLCPP_INFO(get_logger(), "%s → %s", phase_name(before), phase_name(after));
    }
    // ACTIVE 안에서의 DRIVE ↔ HOLD 전환은 phase 가 안 바뀌므로 따로 로그
    const Mode mode = sup_->effective_mode(now);
    if (after == Phase::ACTIVE && mode != last_mode_) {
      if (mode == Mode::HOLD)
        RCLCPP_INFO(get_logger(), "HOLD — 속도 0 유지 (%s)",
                    last_mode_ == Mode::DRIVE ? "wheel_cmd 끊김 0.2 s" : "HOLD 명령");
      else if (mode == Mode::DRIVE)
        RCLCPP_INFO(get_logger(), "DRIVE — 속도 명령 반영");
    }
    last_mode_ = (after == Phase::ACTIVE) ? mode : Mode::FREE;
    if (!was_ready_ && sup_->ready(now)) RCLCPP_INFO(get_logger(), "모든 바퀴 heartbeat 확인 — 명령 대기");
    was_ready_ = sup_->ready(now);

    // 피드백: RTR 묶음을 한 번에 쏘지 않고 틱당 max_rtr_per_tick 개씩 나눠 보냄.
    // (속도 4 + RTR 9 를 한 틱에 몰면 SocketCAN 송신 큐(txqueuelen 기본 10)가 넘쳐 ENOBUFS)
    // FREE 에선 느리게 (바퀴를 손으로 밀어도 위치는 계속 추적)
    if (pending_rtr_.empty() && now >= next_feedback_) {
      for (const auto& f : sup_->feedback_requests(now)) pending_rtr_.push_back(f);
      publish_state(now);
      next_feedback_ = now + (sup_->phase() == Phase::FREE ? feedback_free_period_ : feedback_period_);
    }
    std::vector<cs::Frame> batch;
    // 한 틱 총 송신 ≤ kMaxFramesPerTick (진입/정지 틱은 속도 4 + 상태 4 = 8 → 그 틱엔 RTR 쉼)
    const size_t room = ctrl.size() >= kMaxFramesPerTick ? 0 : kMaxFramesPerTick - ctrl.size();
    const size_t limit = std::min(max_rtr_per_tick_, room);
    while (!pending_rtr_.empty() && batch.size() < limit) {
      batch.push_back(pending_rtr_.front());
      pending_rtr_.pop_front();
    }
    send_all(batch);
  }

  void send_all(const std::vector<cs::Frame>& frames) {
    for (const auto& f : frames) {
      if (!can_->send(f)) {
        const int e = can_->last_errno();
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "CAN 송신 실패 (errno %d) — %s", e,
                             e == ENOBUFS ? "송신 큐 가득: 버스에 ACK 없음(전원/배선) 이거나 txqueuelen 부족"
                                          : "보드 전원 / CANH·CANL / 커넥터 확인");
        return;
      }
    }
  }

  void publish_state(double now) {
    const float vb = sup_->vbus();
    if (std::isfinite(vb) && vb < vbus_warn_)
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                           "배터리 낮음 %.2f V (< %.1f V) — 충전 필요 (보드 저전압 트립 21 V)", vb, vbus_warn_);
    WheelStateArray msg;
    msg.header.stamp = get_clock()->now();
    msg.phase = static_cast<uint8_t>(sup_->phase());
    msg.ready = sup_->ready(now);
    msg.fault = sup_->fault();
    msg.vbus = sup_->vbus();
    const auto& ws = sup_->wheels();
    const auto& sign = sup_->config().sign;
    for (size_t i = 0; i < ws.size(); ++i) {
      capstone_msgs::msg::WheelState s;
      s.can_id = static_cast<uint8_t>(ws[i].can_id);
      s.alive = sup_->alive(i, now);
      s.axis_state = ws[i].axis_state;
      s.axis_error = ws[i].axis_error;
      s.position = static_cast<float>(sign[i]) * ws[i].pos;  // 바퀴 기준 부호
      s.velocity = static_cast<float>(sign[i]) * ws[i].vel;
      s.iq_setpoint = static_cast<float>(sign[i]) * ws[i].iq_set;
      s.iq_measured = static_cast<float>(sign[i]) * ws[i].iq_meas;
      msg.wheels.push_back(s);
    }
    pub_->publish(msg);
  }

  std::unique_ptr<Supervisor> sup_;
  std::unique_ptr<SocketCan> can_;
  rclcpp::Publisher<WheelStateArray>::SharedPtr pub_;
  rclcpp::Subscription<WheelCommand>::SharedPtr sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  double feedback_period_ = 0.05;
  double feedback_free_period_ = 0.2;
  double vbus_warn_ = 22.5;
  size_t max_rtr_per_tick_ = 4;
  static constexpr size_t kMaxFramesPerTick = 8;
  std::deque<cs::Frame> pending_rtr_;
  double next_feedback_ = 0.0;
  bool was_ready_ = false;
  int last_recoveries_ = 0;
  int last_start_clears_ = 0;
  bool was_recovering_ = false;
  double rec_start_ = 0.0;
  Mode last_mode_ = Mode::FREE;
};

}  // namespace wheel_can

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<wheel_can::BridgeNode> node;
  try {
    node = std::make_shared<wheel_can::BridgeNode>();
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("wheel_can_bridge"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(node);
  node->stop_all();
  rclcpp::shutdown();
  return 0;
}
