// 상태기계 시나리오 검증 — 가짜 heartbeat 를 넣고 보낼 프레임을 확인
#include <gtest/gtest.h>

#include <cmath>
#include <map>

#include "wheel_can_bridge/supervisor.hpp"

using namespace wheel_can;

namespace {
cs::Frame hb(int id, uint32_t state, uint32_t err = 0) {
  cs::Frame f;
  f.id = cs::arb_id(id, cs::HEARTBEAT);
  f.len = 8;
  cs::put_u32(f.data.data(), err);
  cs::put_u32(f.data.data() + 4, state);
  return f;
}

struct Sent {
  std::map<int, float> vel;       // id → 마지막 input_vel
  std::map<int, uint32_t> state;  // id → 마지막 axis_state 요청
  int clears = 0;
};

Sent parse(const std::vector<cs::Frame>& frames) {
  Sent s;
  for (const auto& f : frames) {
    const int id = cs::id_of(f.id);
    switch (cs::cmd_of(f.id)) {
      case cs::SET_INPUT_VEL: s.vel[id] = cs::get_f32(f.data.data()); break;
      case cs::SET_AXIS_STATE: s.state[id] = cs::get_u32(f.data.data()); break;
      case cs::CLEAR_ERRORS: ++s.clears; break;
      default: break;
    }
  }
  return s;
}

class SupTest : public ::testing::Test {
 protected:
  SupTest() : sup(make_cfg()) {}
  static SupervisorConfig make_cfg() {
    SupervisorConfig c;
    c.can_ids = {1, 2, 3, 4};
    c.sign = {1.0, -1.0, 1.0, 1.0};
    return c;
  }
  void beat_all(uint32_t state, double now) {
    for (int id : {1, 2, 3, 4}) sup.on_frame(hb(id, state), now);
  }
  // FREE → ACTIVE 까지 진행, 끝난 시각 반환
  double bring_up(double t) {
    beat_all(cs::IDLE, t);
    EXPECT_EQ(sup.set_command(Mode::DRIVE, {0.5, 0.5, 0.5, 0.5}, t), "");
    sup.step(t);
    EXPECT_EQ(sup.phase(), Phase::ENTERING);
    t += 0.1;
    beat_all(cs::CLOSED_LOOP_CONTROL, t);
    EXPECT_EQ(sup.set_command(Mode::DRIVE, {0.5, 0.5, 0.5, 0.5}, t), "");
    sup.step(t);
    EXPECT_EQ(sup.phase(), Phase::ACTIVE);
    return t;
  }
  Supervisor sup;
};
}  // namespace

TEST_F(SupTest, WaitsUntilAllWheelsSeen) {
  sup.on_frame(hb(1, cs::IDLE), 0.0);
  sup.on_frame(hb(2, cs::IDLE), 0.0);
  sup.set_command(Mode::DRIVE, {0.1, 0.1, 0.1, 0.1}, 0.0);
  EXPECT_TRUE(sup.step(0.0).empty());  // 3, 4 안 보임 → 아무것도 안 보냄
  EXPECT_EQ(sup.phase(), Phase::FREE);
  EXPECT_FALSE(sup.ready(0.0));
}

TEST_F(SupTest, EnterSendsZeroAndClosedLoop) {
  beat_all(cs::IDLE, 0.0);
  sup.set_command(Mode::DRIVE, {0.5, 0.5, 0.5, 0.5}, 0.0);
  const auto s = parse(sup.step(0.0));
  EXPECT_EQ(sup.phase(), Phase::ENTERING);
  for (int id : {1, 2, 3, 4}) {
    EXPECT_FLOAT_EQ(s.vel.at(id), 0.0f);  // 진입 중엔 속도 0
    EXPECT_EQ(s.state.at(id), cs::CLOSED_LOOP_CONTROL);
  }
}

TEST_F(SupTest, StaleClosedLoopHeartbeatIsNotTrusted) {
  beat_all(cs::CLOSED_LOOP_CONTROL, 0.0);  // 진입 요청 전 기록
  sup.set_command(Mode::DRIVE, {0.5, 0.5, 0.5, 0.5}, 0.01);
  sup.step(0.01);
  EXPECT_EQ(sup.phase(), Phase::ENTERING);
  sup.step(0.02);
  EXPECT_EQ(sup.phase(), Phase::ENTERING);  // 새 heartbeat 전엔 ACTIVE 아님
}

TEST_F(SupTest, ActiveSendsSignedClampedVelocity) {
  double t = bring_up(0.0);
  t += 0.02;
  beat_all(cs::CLOSED_LOOP_CONTROL, t);
  sup.set_command(Mode::DRIVE, {0.5, 0.5, 5.0, -0.3}, t);
  const auto s = parse(sup.step(t));
  EXPECT_FLOAT_EQ(s.vel.at(1), 0.5f);
  EXPECT_FLOAT_EQ(s.vel.at(2), -0.5f);  // sign -1
  EXPECT_FLOAT_EQ(s.vel.at(3), 2.0f);   // vel_max 로 자름
  EXPECT_FLOAT_EQ(s.vel.at(4), -0.3f);
  EXPECT_TRUE(s.state.empty());
}

TEST_F(SupTest, CommandTimeoutHoldsZero) {
  double t = bring_up(0.0);
  t += 0.3;  // cmd_timeout 0.2 초과
  beat_all(cs::CLOSED_LOOP_CONTROL, t);
  const auto s = parse(sup.step(t));
  EXPECT_EQ(sup.phase(), Phase::ACTIVE);  // 폐루프 유지 (HOLD)
  for (int id : {1, 2, 3, 4}) EXPECT_FLOAT_EQ(s.vel.at(id), 0.0f);
}

TEST_F(SupTest, HoldModeKeepsClosedLoopAtZero) {
  double t = bring_up(0.0);
  t += 0.02;
  beat_all(cs::CLOSED_LOOP_CONTROL, t);
  sup.set_command(Mode::HOLD, {}, t);
  const auto s = parse(sup.step(t));
  EXPECT_EQ(sup.phase(), Phase::ACTIVE);
  for (int id : {1, 2, 3, 4}) EXPECT_FLOAT_EQ(s.vel.at(id), 0.0f);
}

TEST_F(SupTest, FreeModeSendsIdle) {
  double t = bring_up(0.0);
  t += 0.02;
  sup.set_command(Mode::FREE, {}, t);
  const auto s = parse(sup.step(t));
  EXPECT_EQ(sup.phase(), Phase::FREE);
  for (int id : {1, 2, 3, 4}) {
    EXPECT_FLOAT_EQ(s.vel.at(id), 0.0f);
    EXPECT_EQ(s.state.at(id), cs::IDLE);
  }
  EXPECT_TRUE(sup.step(t + 0.02).empty());  // FREE 에선 조용히
}

TEST_F(SupTest, HeartbeatLossFaultsAllWheels) {
  double t = bring_up(0.0);
  t += 0.6;  // id4 만 heartbeat 없음
  for (int id : {1, 2, 3}) sup.on_frame(hb(id, cs::CLOSED_LOOP_CONTROL), t);
  sup.set_command(Mode::DRIVE, {0.5, 0.5, 0.5, 0.5}, t);
  const auto s = parse(sup.step(t));
  EXPECT_EQ(sup.phase(), Phase::FAULT);
  EXPECT_NE(sup.fault().find("id4"), std::string::npos);
  for (int id : {1, 2, 3, 4}) EXPECT_EQ(s.state.at(id), cs::IDLE);
}

TEST_F(SupTest, AxisErrorFaultsThenClearRecovers) {
  double t = bring_up(0.0);
  t += 0.02;
  sup.on_frame(hb(3, cs::IDLE, cs::AXIS_ERROR_WATCHDOG_TIMER_EXPIRED), t);
  sup.step(t);
  EXPECT_EQ(sup.phase(), Phase::FAULT);
  EXPECT_NE(sup.fault().find("0x800"), std::string::npos);

  // FAULT 중 명령은 무시
  sup.set_command(Mode::DRIVE, {0.5, 0.5, 0.5, 0.5}, t);
  sup.step(t + 0.01);
  EXPECT_EQ(sup.phase(), Phase::FAULT);

  t += 0.1;
  const auto c = parse(sup.clear_errors(t));
  EXPECT_EQ(c.clears, 4);
  EXPECT_EQ(sup.phase(), Phase::FREE);
  // 클리어 직후(grace 안) 옛 에러가 담긴 heartbeat → 무시
  sup.on_frame(hb(3, cs::IDLE, cs::AXIS_ERROR_WATCHDOG_TIMER_EXPIRED), t + 0.01);
  sup.step(t + 0.01);
  EXPECT_EQ(sup.phase(), Phase::FREE);
  // 해제 후엔 새 명령 전까지 안 움직임
  beat_all(cs::IDLE, t + 0.1);
  EXPECT_TRUE(sup.step(t + 0.1).empty());
}

TEST_F(SupTest, PersistentErrorRefaultsAfterClear) {
  beat_all(cs::IDLE, 0.0);
  sup.clear_errors(0.0);
  sup.on_frame(hb(2, cs::IDLE, 0x100), 0.2);  // grace 지난 뒤에도 에러 → 진짜
  sup.step(0.2);
  EXPECT_EQ(sup.phase(), Phase::FAULT);
}

TEST_F(SupTest, WheelDroppingOutOfClosedLoopFaults) {
  double t = bring_up(0.0);
  t += 0.02;
  beat_all(cs::CLOSED_LOOP_CONTROL, t);
  sup.on_frame(hb(2, cs::IDLE), t);  // 외부에서 stop.py 등으로 IDLE
  sup.set_command(Mode::DRIVE, {0.5, 0.5, 0.5, 0.5}, t);
  sup.step(t);
  EXPECT_EQ(sup.phase(), Phase::FAULT);  // 자동 재진입하지 않음
  EXPECT_NE(sup.fault().find("id2"), std::string::npos);
}

TEST_F(SupTest, EnterTimeoutFaults) {
  beat_all(cs::IDLE, 0.0);
  double t = 0.0;
  while (t < 3.5) {
    sup.set_command(Mode::HOLD, {}, t);
    beat_all(cs::IDLE, t);  // 계속 IDLE (예: 캘리브레이션 안 됨)
    sup.step(t);
    if (sup.phase() == Phase::FAULT) break;
    t += 0.1;
  }
  EXPECT_EQ(sup.phase(), Phase::FAULT);
  EXPECT_NE(sup.fault().find("진입 실패"), std::string::npos);
}

TEST_F(SupTest, RejectsBadCommands) {
  EXPECT_NE(sup.set_command(Mode::DRIVE, {0.1, 0.1}, 0.0), "");
  EXPECT_NE(sup.set_command(Mode::DRIVE, {0.1, NAN, 0.1, 0.1}, 0.0), "");
  EXPECT_NE(sup.set_command(static_cast<Mode>(7), {}, 0.0), "");
  EXPECT_EQ(sup.set_command(Mode::HOLD, {}, 0.0), "");
}

TEST_F(SupTest, FeedbackRequestsOnlyAliveWheels) {
  sup.on_frame(hb(1, cs::IDLE), 0.0);
  sup.on_frame(hb(3, cs::IDLE), 0.0);
  const auto req = sup.feedback_requests(0.0);
  EXPECT_EQ(req.size(), 2u + 1u);  // FREE: enc × 2 + vbus (Iq 생략)
  for (const auto& f : req) EXPECT_TRUE(f.rtr);
}

TEST_F(SupTest, FeedbackIncludesIqWhenActive) {
  bring_up(0.0);
  EXPECT_EQ(sup.feedback_requests(0.1).size(), 4u * 2u + 1u);  // (enc + iq) × 4 + vbus
}

TEST_F(SupTest, ShutdownStopsAll) {
  const auto s = parse(sup.shutdown_frames());
  for (int id : {1, 2, 3, 4}) {
    EXPECT_FLOAT_EQ(s.vel.at(id), 0.0f);
    EXPECT_EQ(s.state.at(id), cs::IDLE);
  }
}

TEST(SupConfig, RejectsBadSign) {
  SupervisorConfig c;
  c.sign = {1.0, 1.0, 0.5, 1.0};
  EXPECT_THROW(Supervisor{c}, std::invalid_argument);
}
