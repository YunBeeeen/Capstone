#include "wheel_can_bridge/supervisor.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace wheel_can {

namespace {
constexpr double CLEAR_GRACE = 0.05;  // [s] 클리어 직후 이 시간 안에 온 heartbeat 의 에러는 클리어 전 값일 수 있어 무시
constexpr uint32_t AXIS_ERROR_ENCODER_FAILED = 0x100;  // ODrive 0.5.x AxisError (Hall ILLEGAL_HALL_STATE 등)

std::string hex(uint32_t v) {
  char buf[16];
  std::snprintf(buf, sizeof(buf), "0x%X", v);
  return buf;
}
}  // namespace

Supervisor::Supervisor(SupervisorConfig cfg) : cfg_(std::move(cfg)) {
  if (cfg_.can_ids.empty()) throw std::invalid_argument("can_ids 비어 있음");
  if (cfg_.sign.size() != cfg_.can_ids.size())
    throw std::invalid_argument("sign 길이 != can_ids 길이");
  for (double s : cfg_.sign)
    if (s != 1.0 && s != -1.0) throw std::invalid_argument("sign 은 +1 / -1 만");
  for (int id : cfg_.can_ids) {
    cs::arb_id(id, 0);  // 범위 검사
    WheelData w;
    w.can_id = id;
    wheels_.push_back(w);
  }
  vel_cmd_.assign(wheels_.size(), 0.0);
  rec_.assign(wheels_.size(), false);
  rec_start_.assign(wheels_.size(), 0.0);
  rec_req_.assign(wheels_.size(), 0.0);
}

bool Supervisor::recovering() const {
  for (bool r : rec_)
    if (r) return true;
  return false;
}

void Supervisor::reset_recover() { rec_.assign(wheels_.size(), false); }

bool Supervisor::can_recover(double now) {
  if (!cfg_.encoder_recover) return false;
  while (!rec_times_.empty() && now - rec_times_.front() > cfg_.recover_window) rec_times_.pop_front();
  return static_cast<int>(rec_times_.size()) < cfg_.recover_max;
}

void Supervisor::start_recover(size_t i, double now, std::vector<cs::Frame>& out) {
  auto& w = wheels_[i];
  rec_times_.push_back(now);
  rec_[i] = true;
  rec_start_[i] = now;
  rec_req_[i] = now;
  ++n_recover_;
  last_recovery_ = "id" + std::to_string(w.can_id) + " axis_error " + hex(w.axis_error) + " (순간 엔코더 에러) → 자동 복구, " +
                   std::to_string(rec_times_.size()) + "/" + std::to_string(cfg_.recover_max) + " (" +
                   std::to_string(static_cast<int>(cfg_.recover_window)) + " s 안)";
  w.axis_error = 0;
  clear_time_ = now;  // grace 동안 온 heartbeat 의 옛 에러 무시
  // 속도 0 은 이번 틱 끝에서 전 축에 보냄 (프레임 수를 줄여 송신 큐 10 안에)
  out.push_back(cs::clear_errors(w.can_id));
  out.push_back(cs::axis_state(w.can_id, cs::CLOSED_LOOP_CONTROL));
}

int Supervisor::index_of(int can_id) const {
  for (size_t i = 0; i < wheels_.size(); ++i)
    if (wheels_[i].can_id == can_id) return static_cast<int>(i);
  return -1;
}

bool Supervisor::alive(size_t i, double now) const {
  return wheels_[i].seen && now - wheels_[i].hb_time <= cfg_.hb_timeout;
}

bool Supervisor::ready(double now) const {
  for (size_t i = 0; i < wheels_.size(); ++i)
    if (!alive(i, now)) return false;
  return true;
}

void Supervisor::on_frame(const cs::Frame& f, double now) {
  if (f.rtr) return;
  const int i = index_of(cs::id_of(f.id));
  if (i < 0) return;
  WheelData& w = wheels_[i];
  switch (cs::cmd_of(f.id)) {
    case cs::HEARTBEAT: {
      const auto hb = cs::dec_heartbeat(f);
      w.seen = true;
      w.hb_time = now;
      w.axis_state = hb.axis_state;
      // 클리어 직후 grace 안의 heartbeat 는 클리어 전에 찍혔을 수 있음 → 에러만 무시
      if (now >= clear_time_ + CLEAR_GRACE || hb.axis_error == 0) w.axis_error = hb.axis_error;
      break;
    }
    case cs::GET_ENCODER_ESTIMATES: {
      const auto p = cs::dec_f32_pair(f);
      w.pos = p.a;
      w.vel = p.b;
      break;
    }
    case cs::GET_IQ: {
      const auto p = cs::dec_f32_pair(f);
      w.iq_set = p.a;
      w.iq_meas = p.b;
      break;
    }
    case cs::GET_VBUS:
      if (i == 0) vbus_ = cs::dec_vbus(f);
      break;
    default:
      break;
  }
}

std::string Supervisor::set_command(Mode mode, const std::vector<double>& velocity, double now) {
  if (mode != Mode::FREE && mode != Mode::DRIVE && mode != Mode::HOLD) return "알 수 없는 mode";
  std::vector<double> v(wheels_.size(), 0.0);
  if (mode == Mode::DRIVE) {
    if (velocity.size() != wheels_.size())
      return "DRIVE velocity 길이 " + std::to_string(velocity.size()) + " != 바퀴 수 " +
             std::to_string(wheels_.size());
    for (size_t i = 0; i < v.size(); ++i) {
      if (!std::isfinite(velocity[i])) return "velocity 에 NaN/inf";
      v[i] = std::clamp(velocity[i], -cfg_.vel_max, cfg_.vel_max);
    }
  }
  mode_ = mode;
  vel_cmd_ = v;
  cmd_time_ = now;
  return "";
}

void Supervisor::push_all_stop(std::vector<cs::Frame>& out) const {
  for (const auto& w : wheels_) out.push_back(cs::input_vel(w.can_id, 0.0f));
  for (const auto& w : wheels_) out.push_back(cs::axis_state(w.can_id, cs::IDLE));
}

void Supervisor::latch(const std::string& why, double now, std::vector<cs::Frame>& out) {
  reset_recover();
  phase_ = Phase::FAULT;
  fault_ = why;
  mode_ = Mode::FREE;  // 해제 후엔 새 명령이 와야 다시 움직임
  last_req_ = now;
  push_all_stop(out);
}

std::vector<cs::Frame> Supervisor::step(double now) {
  std::vector<cs::Frame> out;

  // 1) 고장 감지
  if (phase_ != Phase::FAULT) {
    for (size_t i = 0; i < wheels_.size(); ++i) {
      const auto& w = wheels_[i];
      if (!w.seen) continue;  // 한 번도 안 보인 바퀴는 ready() 로 막음
      std::string why;
      if (!alive(i, now)) {
        why = "id" + std::to_string(w.can_id) + " heartbeat 끊김";
      } else if (w.axis_error != 0) {
        if (rec_[i]) continue;  // 복구 중: 폐루프로 돌아올 때까지 기다림 (아래 ACTIVE 에서 시간 제한)
        if (phase_ == Phase::ACTIVE && w.axis_error == AXIS_ERROR_ENCODER_FAILED && can_recover(now)) {
          start_recover(i, now, out);
          continue;
        }
        why = "id" + std::to_string(w.can_id) + " axis_error " + hex(w.axis_error);
        if (w.axis_error == AXIS_ERROR_ENCODER_FAILED && cfg_.encoder_recover && phase_ == Phase::ACTIVE)
          why += " (자동 복구 " + std::to_string(cfg_.recover_max) + "회/" +
                 std::to_string(static_cast<int>(cfg_.recover_window)) + " s 초과)";
      }
      if (!why.empty()) {
        latch(why, now, out);
        return out;
      }
    }
  }

  // 2) FAULT: IDLE 이 아닌 살아 있는 축에 IDLE 재요청
  if (phase_ == Phase::FAULT) {
    if (now - last_req_ >= cfg_.retry_period) {
      for (size_t i = 0; i < wheels_.size(); ++i) {
        if (alive(i, now) && wheels_[i].axis_state != cs::IDLE) {
          out.push_back(cs::input_vel(wheels_[i].can_id, 0.0f));
          out.push_back(cs::axis_state(wheels_[i].can_id, cs::IDLE));
          last_req_ = now;
        }
      }
    }
    return out;
  }

  // 3) 유효 모드: DRIVE 명령이 끊기면 HOLD(속도 0)
  const Mode eff = effective_mode(now);

  if (eff == Mode::FREE) {
    reset_recover();
    if (phase_ == Phase::ENTERING || phase_ == Phase::ACTIVE) {
      push_all_stop(out);
      phase_ = Phase::FREE;
      last_req_ = now;
    }
    return out;
  }

  // DRIVE / HOLD
  if (phase_ == Phase::FREE) {
    if (!ready(now)) return out;  // 모든 바퀴가 보일 때까지 대기
    phase_ = Phase::ENTERING;
    enter_start_ = now;
    last_req_ = -std::numeric_limits<double>::infinity();
  }

  if (phase_ == Phase::ENTERING) {
    for (const auto& w : wheels_) out.push_back(cs::input_vel(w.can_id, 0.0f));
    bool all_in = true;
    std::string missing;
    for (const auto& w : wheels_) {
      // enter 시작 이후에 온 heartbeat 로만 확인 (이전 폐루프 기록에 속지 않게)
      const bool in = w.hb_time > enter_start_ && w.axis_state == cs::CLOSED_LOOP_CONTROL;
      if (!in) {
        all_in = false;
        missing += " id" + std::to_string(w.can_id);
      }
    }
    if (all_in) {
      phase_ = Phase::ACTIVE;
      return out;  // 이번 틱은 속도 0, 다음 틱부터 명령
    }
    if (now - enter_start_ > cfg_.enter_timeout) {
      latch("CLOSED_LOOP 진입 실패:" + missing, now, out);
      return out;
    }
    if (now - last_req_ >= cfg_.retry_period) {
      for (const auto& w : wheels_)
        if (!(w.hb_time > enter_start_ && w.axis_state == cs::CLOSED_LOOP_CONTROL))
          out.push_back(cs::axis_state(w.can_id, cs::CLOSED_LOOP_CONTROL));
      last_req_ = now;
    }
    return out;
  }

  // ACTIVE — 자동 복구 진행
  for (size_t i = 0; i < wheels_.size(); ++i) {
    if (!rec_[i]) continue;
    const auto& w = wheels_[i];
    if (w.hb_time > rec_start_[i] + CLEAR_GRACE && w.axis_state == cs::CLOSED_LOOP_CONTROL && w.axis_error == 0) {
      rec_[i] = false;  // 복귀
      continue;
    }
    if (now - rec_start_[i] > cfg_.recover_timeout) {
      latch("id" + std::to_string(w.can_id) + " 순간 엔코더 에러 자동 복구 실패 (" +
                std::to_string(cfg_.recover_timeout).substr(0, 3) + " s 안에 폐루프 복귀 못 함)",
            now, out);
      return out;
    }
    if (now - rec_req_[i] >= cfg_.recover_retry) {
      out.push_back(cs::clear_errors(w.can_id));
      out.push_back(cs::axis_state(w.can_id, cs::CLOSED_LOOP_CONTROL));
      rec_req_[i] = now;
      clear_time_ = now;
    }
  }
  for (size_t i = 0; i < wheels_.size(); ++i) {
    const auto& w = wheels_[i];
    if (!rec_[i] && w.axis_state != cs::CLOSED_LOOP_CONTROL) {
      latch("id" + std::to_string(w.can_id) + " 폐루프 이탈 (state " + std::to_string(w.axis_state) + ")",
            now, out);
      return out;
    }
  }
  const bool hold_all = recovering();  // 복구 중엔 전 축 0 — 한 바퀴만 힘 빠진 채 나머지가 끌고 가지 않게
  for (size_t i = 0; i < wheels_.size(); ++i) {
    const double v = (eff == Mode::DRIVE && !hold_all) ? vel_cmd_[i] : 0.0;
    out.push_back(cs::input_vel(wheels_[i].can_id, static_cast<float>(cfg_.sign[i] * v)));
  }
  return out;
}

std::vector<cs::Frame> Supervisor::feedback_requests(double now) const {
  std::vector<cs::Frame> out;
  for (size_t i = 0; i < wheels_.size(); ++i) {
    if (!alive(i, now)) continue;
    out.push_back(cs::rtr(wheels_[i].can_id, cs::GET_ENCODER_ESTIMATES));
    if (phase_ != Phase::FREE) out.push_back(cs::rtr(wheels_[i].can_id, cs::GET_IQ));  // IDLE 에선 Iq = 0
  }
  if (alive(0, now)) out.push_back(cs::rtr(wheels_[0].can_id, cs::GET_VBUS));
  return out;
}

std::vector<cs::Frame> Supervisor::clear_errors(double now) {
  std::vector<cs::Frame> out;
  reset_recover();
  for (auto& w : wheels_) {
    out.push_back(cs::clear_errors(w.can_id));
    w.axis_error = 0;
  }
  clear_time_ = now;
  if (phase_ == Phase::FAULT) {
    phase_ = Phase::FREE;
    fault_.clear();
  }
  mode_ = Mode::FREE;
  return out;
}

std::vector<cs::Frame> Supervisor::shutdown_frames() const {
  std::vector<cs::Frame> out;
  push_all_stop(out);
  return out;
}

}  // namespace wheel_can
