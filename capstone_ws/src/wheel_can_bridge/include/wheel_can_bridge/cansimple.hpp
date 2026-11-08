// ODrive CANSimple (fw 0.5.1) 프레임 인코드/디코드 — ROS·소켓 무관한 순수 함수.
// 기준 구현: ~/capstone/capstone_ws/Wheel_motor/wheel_can/lib/cansimple.py
// 공용 테스트 벡터: ~/capstone/capstone_ws/Wheel_motor/wheel_can/tests/cansimple_vectors.csv (Python 과 같은 파일로 검증)
//
// 프레임 ID(11-bit) = can_id << 5 | cmd   (can_id = 보드의 axis.config.can_node_id)
// 데이터는 little-endian. Get_* 은 RTR 요청 → 같은 ID 로 응답 (0.5.1 은 heartbeat 만 자동 송신).
#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

namespace wheel_can::cs {

enum Cmd : uint8_t {
  HEARTBEAT = 0x001,
  ESTOP = 0x002,
  GET_MOTOR_ERROR = 0x003,
  GET_ENCODER_ERROR = 0x004,
  SET_AXIS_STATE = 0x007,
  GET_ENCODER_ESTIMATES = 0x009,
  SET_CONTROLLER_MODE = 0x00B,
  SET_INPUT_VEL = 0x00D,
  GET_IQ = 0x014,
  GET_VBUS = 0x017,
  CLEAR_ERRORS = 0x018,
};

enum AxisState : uint32_t {
  UNDEFINED = 0,  // "요청 없음" — 끄기 아님
  IDLE = 1,
  CLOSED_LOOP_CONTROL = 8,
};

constexpr uint32_t AXIS_ERROR_WATCHDOG_TIMER_EXPIRED = 0x800;

struct Frame {
  uint32_t id = 0;
  uint8_t len = 0;
  bool rtr = false;
  std::array<uint8_t, 8> data{};
};

inline uint32_t arb_id(int can_id, uint8_t cmd) {
  if (can_id < 0 || can_id > 0x3F) throw std::out_of_range("can_id 0..63: " + std::to_string(can_id));
  return (static_cast<uint32_t>(can_id) << 5) | (cmd & 0x1F);
}
inline int id_of(uint32_t arb) { return static_cast<int>(arb >> 5); }
inline uint8_t cmd_of(uint32_t arb) { return static_cast<uint8_t>(arb & 0x1F); }

// ---- little-endian 바이트 변환 (호스트 엔디언과 무관하게 명시적으로)
inline void put_u32(uint8_t* p, uint32_t v) {
  for (int i = 0; i < 4; ++i) p[i] = static_cast<uint8_t>(v >> (8 * i));
}
inline uint32_t get_u32(const uint8_t* p) {
  uint32_t v = 0;
  for (int i = 0; i < 4; ++i) v |= static_cast<uint32_t>(p[i]) << (8 * i);
  return v;
}
inline void put_f32(uint8_t* p, float f) {
  uint32_t u;
  std::memcpy(&u, &f, 4);
  put_u32(p, u);
}
inline float get_f32(const uint8_t* p) {
  uint32_t u = get_u32(p);
  float f;
  std::memcpy(&f, &u, 4);
  return f;
}

// ---- 인코드 (Pi → 보드)
inline Frame axis_state(int can_id, uint32_t state) {
  Frame f;
  f.id = arb_id(can_id, SET_AXIS_STATE);
  f.len = 4;
  put_u32(f.data.data(), state);
  return f;
}

inline Frame input_vel(int can_id, float vel, float torque_ff = 0.0f) {
  Frame f;
  f.id = arb_id(can_id, SET_INPUT_VEL);
  f.len = 8;
  put_f32(f.data.data(), vel);
  put_f32(f.data.data() + 4, torque_ff);
  return f;
}

inline Frame controller_mode(int can_id, int32_t control_mode, int32_t input_mode) {
  Frame f;
  f.id = arb_id(can_id, SET_CONTROLLER_MODE);
  f.len = 8;
  put_u32(f.data.data(), static_cast<uint32_t>(control_mode));
  put_u32(f.data.data() + 4, static_cast<uint32_t>(input_mode));
  return f;
}

inline Frame clear_errors(int can_id) {
  Frame f;
  f.id = arb_id(can_id, CLEAR_ERRORS);
  f.len = 0;
  return f;
}

// RTR 요청. fw 0.5.1 실기에서 DLC 8 로 보낸 요청에 응답 확인 (Python bus.request 와 동일)
inline Frame rtr(int can_id, uint8_t cmd) {
  Frame f;
  f.id = arb_id(can_id, cmd);
  f.len = 8;
  f.rtr = true;
  return f;
}

// ---- 디코드 (보드 → Pi)
struct Heartbeat {
  uint32_t axis_error;
  uint32_t axis_state;
};
inline Heartbeat dec_heartbeat(const Frame& f) { return {get_u32(f.data.data()), get_u32(f.data.data() + 4)}; }

struct F32Pair {
  float a;  // encoder: pos [turn] / iq: setpoint [A]
  float b;  // encoder: vel [turn/s] / iq: measured [A]
};
inline F32Pair dec_f32_pair(const Frame& f) { return {get_f32(f.data.data()), get_f32(f.data.data() + 4)}; }
inline float dec_vbus(const Frame& f) { return get_f32(f.data.data()); }

}  // namespace wheel_can::cs
