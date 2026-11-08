// 상태기계 시나리오 검증 — 가짜 Type02 피드백을 넣고 보낼 프레임을 확인
#include <gtest/gtest.h>

#include <cmath>
#include <map>

#include "steer_can_bridge/supervisor.hpp"

using namespace steer_can;

namespace {
constexpr double DEG = M_PI / 180.0;
constexpr double DT = 0.005;  // 200 Hz

rs02::Frame fb(int id, double pos, uint8_t mode = rs02::MODE_RESET, uint8_t fault = 0) {
  rs02::Frame f;
  const uint8_t flags = static_cast<uint8_t>((mode << 6) | (fault & 0x3F));
  f.id = rs02::pack_id(rs02::FEEDBACK, static_cast<uint16_t>((flags << 8) | id), 0xFD);
  rs02::put_be16(&f.data[0], rs02::float_to_u16(pos, rs02::P_MIN, rs02::P_MAX));
  rs02::put_be16(&f.data[2], rs02::float_to_u16(0.0, rs02::V_MIN, rs02::V_MAX));
  rs02::put_be16(&f.data[4], rs02::float_to_u16(0.0, rs02::T_MIN, rs02::T_MAX));
  rs02::put_be16(&f.data[6], 300);
  return f;
}

struct Mit {
  double p, v, kp, kd;
};
struct Sent {
  std::map<int, Mit> mit;
  std::map<int, int> enable, stop, stop_clear, can_timeout;
  size_t total = 0;
};

Sent parse(const std::vector<rs02::Frame>& frames) {
  Sent s;
  s.total = frames.size();
  for (const auto& f : frames) {
    const int id = f.id & 0xFF;
    switch (rs02::type_of(f.id)) {
      case rs02::MIT_CONTROL:
        s.mit[id] = {rs02::u16_to_float(rs02::get_be16(&f.data[0]), rs02::P_MIN, rs02::P_MAX),
                     rs02::u16_to_float(rs02::get_be16(&f.data[2]), rs02::V_MIN, rs02::V_MAX),
                     rs02::u16_to_float(rs02::get_be16(&f.data[4]), rs02::KP_MIN, rs02::KP_MAX),
                     rs02::u16_to_float(rs02::get_be16(&f.data[6]), rs02::KD_MIN, rs02::KD_MAX)};
        break;
      case rs02::ENABLE: ++s.enable[id]; break;
      case rs02::STOP: ++(f.data[0] ? s.stop_clear : s.stop)[id]; break;
      case rs02::PARAM_WRITE:
        if ((f.data[0] | (f.data[1] << 8)) == rs02::PARAM_CAN_TIMEOUT)
          s.can_timeout[id] = f.data[4] | (f.data[5] << 8) | (f.data[6] << 16) | (f.data[7] << 24);
        break;
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
    c.sign = {1.0, -1.0, 1.0, 1.0};  // 2번만 반대로 달렸다고 가정
    return c;
  }
  // 모터각(raw) 으로 피드백. 조향각 = sign × raw
  void feed(const std::map<int, double>& raw, double now, uint8_t mode = rs02::MODE_RESET) {
    for (const auto& [id, p] : raw) sup.on_frame(fb(id, p, mode), now);
  }
  void feed_all(double raw, double now, uint8_t mode = rs02::MODE_RESET) {
    feed({{1, raw}, {2, raw}, {3, raw}, {4, raw}}, now, mode);
  }
  // FREE → ACTIVE. 시작 조향각 0. 끝난 시각 반환
  double bring_up(double t, const std::vector<double>& target = {0, 0, 0, 0}) {
    feed_all(0.0, t);
    EXPECT_TRUE(sup.ready(t));
    EXPECT_EQ(sup.set_command(Mode::POSITION, target, t), "");
    const auto s = parse(sup.step(t));
    EXPECT_EQ(sup.phase(), Phase::ENTERING);
    EXPECT_EQ(s.enable.size(), 4u);
    t += DT;
    feed_all(0.0, t, rs02::MODE_RUN);
    sup.step(t);
    EXPECT_EQ(sup.phase(), Phase::ACTIVE);
    return t;
  }
  // ACTIVE 에서 n 틱: 매 틱 명령 갱신 + RUN 피드백
  Sent run(double& t, int n, const std::vector<double>& target) {
    Sent s;
    for (int k = 0; k < n; ++k) {
      t += DT;
      sup.set_command(Mode::POSITION, target, t);
      feed_all(0.0, t, rs02::MODE_RUN);
      s = parse(sup.step(t));
    }
    return s;
  }
  Supervisor sup;
};
}  // namespace

TEST_F(SupTest, FreePollsStopAndReadsAngle) {
  feed({{1, 0.3}, {2, 0.3}, {3, 0.3}, {4, 0.3}}, 0.0);
  const auto s = parse(sup.step(0.0));
  EXPECT_EQ(s.stop.size(), 4u);  // Type04 폴링 (토크 끔 + 응답으로 각도)
  EXPECT_EQ(sup.phase(), Phase::FREE);
  EXPECT_TRUE(sup.ready(0.0));
  EXPECT_NEAR(sup.motors()[0].angle, 0.3, 1e-3);
  EXPECT_NEAR(sup.motors()[1].angle, -0.3, 1e-3);  // sign −1
  EXPECT_EQ(parse(sup.step(0.01)).total, 0u);       // 폴링 주기 50 ms 전엔 안 보냄
  EXPECT_EQ(parse(sup.step(0.06)).stop.size(), 4u);
}

TEST_F(SupTest, NoFeedbackNotReady) {
  EXPECT_FALSE(sup.ready(0.0));
  sup.set_command(Mode::POSITION, {0, 0, 0, 0}, 0.0);
  sup.step(0.0);
  EXPECT_EQ(sup.phase(), Phase::FREE);
  EXPECT_NE(sup.blocked().find("무응답"), std::string::npos);
}

TEST_F(SupTest, UnconfirmedAngleBlocksUntilTurnedByHand) {
  feed({{1, 0.0}, {2, 0.0}, {3, -175 * DEG}, {4, 0.0}}, 0.0);  // 3번: 켤 때 −175° (실제 +185° 일 수 있음)
  EXPECT_FALSE(sup.ready(0.0));
  sup.set_command(Mode::POSITION, {0, 0, 0, 0}, 0.0);
  sup.step(0.0);
  EXPECT_EQ(sup.phase(), Phase::FREE);
  EXPECT_NE(sup.blocked().find("3 RL 각도 미확정"), std::string::npos) << sup.blocked();
  // 정면 쪽으로 돌렸는데 값이 커지면(접힌 상태) 여전히 미확정
  feed({{1, 0.0}, {2, 0.0}, {3, -200 * DEG}, {4, 0.0}}, 0.05);
  EXPECT_FALSE(sup.motors()[2].confirmed);
  // 반대로 돌려 |θ| < 165° → 확정
  feed({{1, 0.0}, {2, 0.0}, {3, -160 * DEG}, {4, 0.0}}, 0.1);
  EXPECT_TRUE(sup.motors()[2].confirmed);
  // 확정 후엔 165° 넘어가도 유지 (모터가 회전 수를 셈)
  feed({{1, 0.0}, {2, 0.0}, {3, -168 * DEG}, {4, 0.0}}, 0.15);
  EXPECT_TRUE(sup.motors()[2].confirmed);
  sup.set_command(Mode::POSITION, {0, 0, 0, 0}, 0.15);
  sup.step(0.15);
  EXPECT_EQ(sup.phase(), Phase::ENTERING);
}

TEST_F(SupTest, FeedbackGapResetsConfirmation) {
  feed_all(170 * DEG, 0.0);  // 확정 안 됨
  feed_all(100 * DEG, 0.05);
  EXPECT_TRUE(sup.motors()[0].confirmed);
  feed_all(170 * DEG, 0.10);
  EXPECT_TRUE(sup.motors()[0].confirmed);
  // 1 s 끊김 (전원 재투입?) 후 170° 로 다시 나타남 → 다시 미확정
  feed_all(170 * DEG, 1.10);
  EXPECT_FALSE(sup.motors()[0].confirmed);
}

TEST_F(SupTest, EnteringWritesCanTimeoutAndStartsFromActualAngle) {
  feed({{1, 0.5}, {2, 0.5}, {3, 0.5}, {4, 0.5}}, 0.0);
  sup.set_command(Mode::POSITION, {0, 0, 0, 0}, 0.0);
  const auto s0 = parse(sup.step(0.0));
  EXPECT_EQ(sup.phase(), Phase::ENTERING);
  EXPECT_EQ(s0.can_timeout.size(), 4u);
  EXPECT_EQ(s0.can_timeout.at(1), 10000);  // 0.5 s × 20000
  EXPECT_EQ(s0.stop.size(), 0u);
  // 1, 2 번만 RUN → 그 둘만 MIT 로 현재각 유지
  feed({{1, 0.5}, {2, 0.5}}, DT, rs02::MODE_RUN);
  const auto s1 = parse(sup.step(DT));
  EXPECT_EQ(sup.phase(), Phase::ENTERING);
  ASSERT_EQ(s1.mit.size(), 2u);
  EXPECT_NEAR(s1.mit.at(1).p, 0.5, 1e-3);
  EXPECT_NEAR(s1.mit.at(2).p, 0.5, 1e-3);  // 조향 −0.5 → 모터 +0.5
  EXPECT_NEAR(s1.mit.at(1).v, 0.0, 1e-3);
  // RUN 전인 3, 4 번은 재요청 사이(> fb_timeout)에도 끊김 FAULT 아님 → 0.2 s 에 재요청
  for (double t = 2 * DT; t < 0.21; t += DT) {
    feed({{1, 0.5}, {2, 0.5}}, t, rs02::MODE_RUN);
    sup.set_command(Mode::POSITION, {0, 0, 0, 0}, t);
    const auto s = parse(sup.step(t));
    EXPECT_NE(sup.phase(), Phase::FAULT) << sup.fault() << " t=" << t;
    if (s.enable.size()) {
      EXPECT_EQ(s.enable.count(3), 1u);
      EXPECT_EQ(s.enable.count(1), 0u);
    }
  }
  feed({{1, 0.5}, {2, 0.5}, {3, 0.5}, {4, 0.5}}, 0.22, rs02::MODE_RUN);
  sup.step(0.22);
  EXPECT_EQ(sup.phase(), Phase::ACTIVE);
}

TEST_F(SupTest, RampAtMaxRateWithVelocityFeedforward) {
  double t = bring_up(0.0);
  const std::vector<double> tgt = {90 * DEG, 90 * DEG, -30 * DEG, 0.0};
  auto s = run(t, 1, tgt);
  EXPECT_NEAR(s.mit.at(1).p, 2.0 * DT, 2e-3);
  EXPECT_NEAR(s.mit.at(1).v, 2.0, 2e-3);    // 램프 속도 = 피드포워드
  EXPECT_NEAR(s.mit.at(2).p, -2.0 * DT, 2e-3);  // sign −1
  EXPECT_NEAR(s.mit.at(2).v, -2.0, 2e-3);
  EXPECT_NEAR(s.mit.at(3).v, -2.0, 2e-3);
  // 4번 목표 0: 피드백 양자화(0 → 0.00019 rad)만큼 첫 틱에 되돌림 → |v| ≤ 0.00019/DT ≈ 0.04
  EXPECT_NEAR(s.mit.at(4).v, 0.0, 0.05);
  EXPECT_NEAR(s.mit.at(1).kp, 20.0, 0.01);
  EXPECT_NEAR(s.mit.at(1).kd, 2.0, 1e-3);
  s = run(t, 200, tgt);  // 1 s 뒤 — 90° 는 0.79 s 에 도착
  EXPECT_NEAR(s.mit.at(1).p, 90 * DEG, 2e-3);
  EXPECT_NEAR(s.mit.at(1).v, 0.0, 2e-3);
  EXPECT_NEAR(s.mit.at(3).p, -30 * DEG, 2e-3);
}

TEST_F(SupTest, TargetClampedToLimit) {
  double t = bring_up(0.0);
  auto s = run(t, 400, {200 * DEG, -200 * DEG, 0, 0});
  EXPECT_NEAR(sup.motors()[0].angle_cmd, 180 * DEG, 1e-9);
  EXPECT_NEAR(sup.motors()[1].angle_cmd, -180 * DEG, 1e-9);
  EXPECT_NEAR(s.mit.at(2).p, 180 * DEG, 2e-3);  // 모터 기준 (sign −1)
}

TEST_F(SupTest, CommandTimeoutHoldsCurrentRampPosition) {
  double t = bring_up(0.0);
  run(t, 20, {90 * DEG, 0, 0, 0});  // 0.1 s → 0.2 rad
  const double held = sup.motors()[0].angle_cmd;
  EXPECT_NEAR(held, 0.2, 1e-3);  // 시작점 = 양자화된 피드백 (±0.0002 rad)
  // 명령 끊김: 0.2 s 까지는 계속 램프, 그 뒤 멈춤
  Sent s;
  for (int k = 0; k < 100; ++k) {
    t += DT;
    feed_all(0.0, t, rs02::MODE_RUN);
    s = parse(sup.step(t));
  }
  EXPECT_EQ(sup.phase(), Phase::ACTIVE);
  const double stopped = sup.motors()[0].angle_cmd;
  EXPECT_NEAR(stopped, held + 2.0 * 0.2, 0.02);
  EXPECT_NEAR(s.mit.at(1).v, 0.0, 2e-3);
  EXPECT_NEAR(s.mit.at(1).p, stopped, 2e-3);
}

TEST_F(SupTest, FreeCommandDisables) {
  double t = bring_up(0.0);
  t += DT;
  sup.set_command(Mode::FREE, {}, t);
  feed_all(0.0, t, rs02::MODE_RUN);
  const auto s = parse(sup.step(t));
  EXPECT_EQ(sup.phase(), Phase::FREE);
  EXPECT_EQ(s.stop.size(), 4u);
  EXPECT_EQ(s.mit.size(), 0u);
}

TEST_F(SupTest, FeedbackLossFaultsAndClearReturnsToFree) {
  double t = bring_up(0.0);
  run(t, 5, {0, 0, 0, 0});
  // 4번 피드백 끊김
  Sent s;
  for (int k = 0; k < 40 && sup.phase() != Phase::FAULT; ++k) {
    t += DT;
    sup.set_command(Mode::POSITION, {0, 0, 0, 0}, t);
    feed({{1, 0.0}, {2, 0.0}, {3, 0.0}}, t, rs02::MODE_RUN);
    s = parse(sup.step(t));
  }
  ASSERT_EQ(sup.phase(), Phase::FAULT);
  EXPECT_NE(sup.fault().find("4 FL 피드백 끊김"), std::string::npos) << sup.fault();
  EXPECT_EQ(s.stop.size(), 4u);
  EXPECT_FALSE(sup.ready(t));
  // FAULT 에선 명령이 와도 재진입 안 함
  t += 0.1;
  feed_all(0.0, t);
  sup.set_command(Mode::POSITION, {0, 0, 0, 0}, t);
  sup.step(t);
  EXPECT_EQ(sup.phase(), Phase::FAULT);
  const auto c = parse(sup.clear_errors(t));
  EXPECT_EQ(c.stop_clear.size(), 4u);
  EXPECT_EQ(sup.phase(), Phase::FREE);
  EXPECT_TRUE(sup.fault().empty());
}

TEST_F(SupTest, RunLostFaults) {
  double t = bring_up(0.0);
  t += DT;
  sup.set_command(Mode::POSITION, {0, 0, 0, 0}, t);
  feed({{1, 0.0}, {2, 0.0}, {3, 0.0}}, t, rs02::MODE_RUN);
  feed({{4, 0.0}}, t, rs02::MODE_RESET);  // 밖에서 stop.py
  sup.step(t);
  EXPECT_EQ(sup.phase(), Phase::FAULT);
  EXPECT_NE(sup.fault().find("RUN 이탈"), std::string::npos);
}

TEST_F(SupTest, MotorFaultBitsFault) {
  double t = bring_up(0.0);
  t += DT;
  sup.set_command(Mode::POSITION, {0, 0, 0, 0}, t);
  feed({{1, 0.0}, {2, 0.0}, {3, 0.0}}, t, rs02::MODE_RUN);
  sup.on_frame(fb(4, 0.0, rs02::MODE_RUN, 0b000100), t);  // 과열
  sup.step(t);
  EXPECT_EQ(sup.phase(), Phase::FAULT);
  EXPECT_NE(sup.fault().find("4 FL 모터 고장"), std::string::npos);
}

TEST_F(SupTest, EnterTimeoutFaults) {
  feed_all(0.0, 0.0);
  sup.set_command(Mode::POSITION, {0, 0, 0, 0}, 0.0);
  sup.step(0.0);
  double t = 0.0;
  while (t < 1.2 && sup.phase() == Phase::ENTERING) {
    t += DT;
    sup.set_command(Mode::POSITION, {0, 0, 0, 0}, t);
    feed({{1, 0.0}, {2, 0.0}, {3, 0.0}}, t, rs02::MODE_RUN);
    feed({{4, 0.0}}, t, rs02::MODE_RESET);  // 4번은 enable 이 안 먹음
    sup.step(t);
  }
  EXPECT_EQ(sup.phase(), Phase::FAULT);
  EXPECT_NE(sup.fault().find("enable 실패 (RUN 미확인: 4 FL)"), std::string::npos) << sup.fault();
}

TEST_F(SupTest, RejectsBadCommands) {
  EXPECT_NE(sup.set_command(Mode::POSITION, {0, 0, 0}, 0.0), "");
  EXPECT_NE(sup.set_command(Mode::POSITION, {0, NAN, 0, 0}, 0.0), "");
  EXPECT_NE(sup.set_command(static_cast<Mode>(7), {}, 0.0), "");
  EXPECT_EQ(sup.set_command(Mode::FREE, {}, 0.0), "");
}

TEST(SupConfig, RejectsBadConfig) {
  SupervisorConfig c;
  c.sign = {1, 1, 1};
  EXPECT_THROW(Supervisor{c}, std::invalid_argument);
  c.sign = {1, 1, 1, 0.5};
  EXPECT_THROW(Supervisor{c}, std::invalid_argument);
}
