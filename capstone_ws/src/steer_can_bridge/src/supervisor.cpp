#include "steer_can_bridge/supervisor.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace steer_can {

namespace {
std::string deg(double rad) {
  char b[32];
  std::snprintf(b, sizeof(b), "%+.1f°", rad * 180.0 / M_PI);
  return b;
}
}  // namespace

std::string axis_name(size_t i) {
  static const char* names[] = {"FR", "RR", "RL", "FL"};
  return std::to_string(i + 1) + " " + (i < 4 ? names[i] : "?");
}

Supervisor::Supervisor(SupervisorConfig cfg) : cfg_(std::move(cfg)) {
  if (cfg_.can_ids.empty()) throw std::invalid_argument("can_ids 비어 있음");
  if (cfg_.sign.size() != cfg_.can_ids.size()) throw std::invalid_argument("sign 길이 != can_ids 길이");
  for (double s : cfg_.sign)
    if (s != 1.0 && s != -1.0) throw std::invalid_argument("sign 은 1 또는 -1");
  if (cfg_.confirm_limit <= 0.0 || cfg_.confirm_limit > M_PI) throw std::invalid_argument("confirm_limit 은 (0, π]");
  if (cfg_.max_rate <= 0.0) throw std::invalid_argument("max_rate > 0");
  motors_.resize(cfg_.can_ids.size());
  for (size_t i = 0; i < motors_.size(); ++i) motors_[i].can_id = cfg_.can_ids[i];
  target_.assign(motors_.size(), 0.0);
}

int Supervisor::index_of(int can_id) const {
  for (size_t i = 0; i < cfg_.can_ids.size(); ++i)
    if (cfg_.can_ids[i] == can_id) return static_cast<int>(i);
  return -1;
}

bool Supervisor::alive(size_t i, double now) const { return now - motors_[i].fb_time <= cfg_.fb_timeout; }

bool Supervisor::ready(double now) const { return phase_ != Phase::FAULT && unhealthy(now).empty(); }

std::string Supervisor::unhealthy(double now) const {
  for (size_t i = 0; i < motors_.size(); ++i) {
    const auto& m = motors_[i];
    if (!alive(i, now)) return axis_name(i) + (m.seen ? " 피드백 끊김" : " 무응답 (전원/배선/CAN ID)");
    if (m.fault_bits) return axis_name(i) + " 모터 고장 비트 " + std::to_string(m.fault_bits);
    if (m.fault21) return axis_name(i) + " Type21 고장 " + std::to_string(m.fault21);
    if (!m.confirmed)
      return axis_name(i) + " 각도 미확정 (" + deg(m.angle) + ", |θ| < " + deg(cfg_.confirm_limit).substr(1) +
             " 이 되게 손으로 정면 쪽으로 돌릴 것)";
  }
  return "";
}

void Supervisor::on_frame(const rs02::Frame& f, double now) {
  if (auto fb = rs02::dec_feedback(f)) {
    const int i = index_of(fb->motor_id);
    if (i < 0) return;
    auto& m = motors_[i];
    // 피드백이 끊겼다 다시 오면 (모터 전원 재투입일 수 있음) 각도 확정부터 다시
    if (now - m.fb_time > cfg_.fb_timeout) m.confirmed = false;
    const double s = cfg_.sign[i];
    m.seen = true;
    m.fb_time = now;
    m.mode_status = fb->mode_status;
    m.fault_bits = fb->fault_bits;
    m.angle = s * fb->pos;
    m.vel = s * fb->vel;
    m.tor = s * fb->tor;
    m.temp = fb->temp;
    if (!m.confirmed && std::fabs(m.angle) < cfg_.confirm_limit) m.confirmed = true;
    return;
  }
  if (auto ff = rs02::dec_fault(f)) {
    const int i = index_of(ff->motor_id);
    if (i >= 0) motors_[i].fault21 = ff->fault;
  }
}

std::string Supervisor::set_command(Mode mode, const std::vector<double>& angle, double now) {
  if (mode != Mode::FREE && mode != Mode::POSITION) return "알 수 없는 mode " + std::to_string(int(mode));
  if (mode == Mode::POSITION) {
    if (angle.size() != motors_.size())
      return "angle 길이 " + std::to_string(angle.size()) + " != " + std::to_string(motors_.size());
    for (double a : angle)
      if (!std::isfinite(a)) return "angle 에 NaN/inf";
    for (size_t i = 0; i < angle.size(); ++i)
      target_[i] = std::clamp(angle[i], -cfg_.steer_limit, cfg_.steer_limit);
  }
  mode_ = mode;
  cmd_time_ = now;
  return "";
}

void Supervisor::push_all_stop(std::vector<rs02::Frame>& out, bool clear_fault) const {
  for (int id : cfg_.can_ids) out.push_back(rs02::enc_stop(cfg_.host_id, static_cast<uint8_t>(id), clear_fault));
}

void Supervisor::push_mit(size_t i, double v_ff, std::vector<rs02::Frame>& out) const {
  const double s = cfg_.sign[i];
  out.push_back(rs02::enc_mit(static_cast<uint8_t>(cfg_.can_ids[i]), s * motors_[i].angle_cmd, s * v_ff, cfg_.kp,
                              cfg_.kd, 0.0));
}

void Supervisor::latch(const std::string& why, std::vector<rs02::Frame>& out) {
  phase_ = Phase::FAULT;
  fault_ = why;
  for (auto& m : motors_) m.running = false;
  push_all_stop(out);
  last_poll_ = -std::numeric_limits<double>::infinity();
}

std::vector<rs02::Frame> Supervisor::step(double now) {
  std::vector<rs02::Frame> out;
  const double dt = std::isnan(last_step_) ? 0.0 : std::clamp(now - last_step_, 0.0, 0.05);
  last_step_ = now;
  const bool want_pos = mode_ == Mode::POSITION && cmd_fresh(now);

  // ENTERING/ACTIVE 공통 고장 판정
  if (phase_ == Phase::ENTERING || phase_ == Phase::ACTIVE) {
    for (size_t i = 0; i < motors_.size(); ++i) {
      const auto& m = motors_[i];
      std::string why;
      // RUN 전인 축은 enable 재요청(retry_period)에만 응답하므로 끊김 판정 안 함 → enter_timeout 이 잡음
      if (m.running && !alive(i, now)) why = " 피드백 끊김";
      else if (m.fault_bits) why = " 모터 고장 비트 " + std::to_string(m.fault_bits);
      else if (m.fault21) why = " Type21 고장 " + std::to_string(m.fault21);
      else if (phase_ == Phase::ACTIVE && m.mode_status != rs02::MODE_RUN) why = " RUN 이탈 (밖에서 stop?)";
      if (!why.empty()) {
        latch(axis_name(i) + why, out);
        return out;
      }
    }
    if (mode_ == Mode::FREE) {  // FREE 명령 → 토크 끔
      phase_ = Phase::FREE;
      for (auto& m : motors_) m.running = false;
      push_all_stop(out);
      last_poll_ = now;
      return out;
    }
  }

  switch (phase_) {
    case Phase::FREE:
    case Phase::FAULT: {
      if (now - last_poll_ >= cfg_.free_poll_period) {
        push_all_stop(out);
        last_poll_ = now;
      }
      if (phase_ == Phase::FAULT) break;
      blocked_.clear();
      if (!want_pos) break;
      blocked_ = unhealthy(now);
      if (!blocked_.empty()) break;
      phase_ = Phase::ENTERING;
      enter_start_ = now;
      last_req_ = now;
      out.clear();
      for (size_t i = 0; i < motors_.size(); ++i) {
        const auto id = static_cast<uint8_t>(cfg_.can_ids[i]);
        if (cfg_.motor_can_timeout > 0.0)
          out.push_back(rs02::enc_write_u32(cfg_.host_id, id, rs02::PARAM_CAN_TIMEOUT,
                                            static_cast<uint32_t>(std::lround(cfg_.motor_can_timeout * 20000.0))));
        out.push_back(rs02::enc_enable(cfg_.host_id, id));
      }
      break;
    }

    case Phase::ENTERING: {
      bool all = true;
      for (size_t i = 0; i < motors_.size(); ++i) {
        auto& m = motors_[i];
        if (!m.running && m.fb_time >= enter_start_ && m.mode_status == rs02::MODE_RUN) {
          m.running = true;
          m.angle_cmd = m.angle;  // 지금 있는 자리에서 시작 → 튀지 않음
        }
        if (m.running) push_mit(i, 0.0, out);
        all = all && m.running;
      }
      if (all) {
        phase_ = Phase::ACTIVE;
        break;
      }
      if (now - enter_start_ > cfg_.enter_timeout) {
        std::string who;
        for (size_t i = 0; i < motors_.size(); ++i)
          if (!motors_[i].running) who += (who.empty() ? "" : ", ") + axis_name(i);
        out.clear();
        latch("enable 실패 (RUN 미확인: " + who + ")", out);
        break;
      }
      if (now - last_req_ >= cfg_.retry_period) {
        last_req_ = now;
        for (size_t i = 0; i < motors_.size(); ++i)
          if (!motors_[i].running) out.push_back(rs02::enc_enable(cfg_.host_id, static_cast<uint8_t>(cfg_.can_ids[i])));
      }
      break;
    }

    case Phase::ACTIVE: {
      const double max_step = cfg_.max_rate * dt;
      for (size_t i = 0; i < motors_.size(); ++i) {
        auto& m = motors_[i];
        const double tgt = want_pos ? target_[i] : m.angle_cmd;  // 명령 끊김 → 그 자리
        const double d = std::clamp(tgt - m.angle_cmd, -max_step, max_step);
        m.angle_cmd += d;
        push_mit(i, dt > 0.0 ? d / dt : 0.0, out);
      }
      break;
    }
  }
  return out;
}

std::vector<rs02::Frame> Supervisor::clear_errors(double now) {
  std::vector<rs02::Frame> out;
  push_all_stop(out, true);
  for (auto& m : motors_) {
    m.fault21 = 0;
    m.running = false;
  }
  phase_ = Phase::FREE;
  fault_.clear();
  mode_ = Mode::FREE;  // 새 명령 필요
  last_poll_ = now;
  return out;
}

std::vector<rs02::Frame> Supervisor::shutdown_frames() const {
  std::vector<rs02::Frame> out;
  push_all_stop(out);
  return out;
}

}  // namespace steer_can
