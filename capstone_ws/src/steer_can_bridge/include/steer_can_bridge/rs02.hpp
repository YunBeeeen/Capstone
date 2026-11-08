// RobStride 02 private CAN 프로토콜 (1 Mbps, 확장 29-bit ID) — 인코드/디코드만, 상태 없음.
// 기준 구현: ~/capstone/capstone_ws/Steer_motor/steer_can/lib/{frames,parse}.py (idle_ws 에서 실기 검증)
// 공용 테스트 벡터: ~/capstone/capstone_ws/Steer_motor/steer_can/tests/rs02_vectors.csv
//
// ID = [28:24] 통신 타입 | [23:8] data2 | [7:0] data1
// Type 1·3·4·18 의 응답은 전부 Type 2 피드백 → 토크 끈 상태(Type 4)에서도 위치를 읽을 수 있음.
#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <optional>

namespace rs02 {

struct Frame {
  uint32_t id = 0;  // 29-bit
  uint8_t len = 8;
  std::array<uint8_t, 8> data{};
};

enum Type : uint8_t {
  GET_DEVICE_ID = 0x00,
  MIT_CONTROL = 0x01,
  FEEDBACK = 0x02,
  ENABLE = 0x03,
  STOP = 0x04,
  SET_ZERO = 0x06,
  PARAM_READ = 0x11,
  PARAM_WRITE = 0x12,
  FAULT_FEEDBACK = 0x15,
  SAVE = 0x16,
};

enum ModeStatus : uint8_t { MODE_RESET = 0, MODE_CALI = 1, MODE_RUN = 2 };

// 파라미터 인덱스 (매뉴얼 4.2, RAM 쓰기 — 저장은 SAVE)
constexpr uint16_t PARAM_CAN_TIMEOUT = 0x7028;  // uint32, 20000 = 1 s, 0 = 끔. 넘기면 모터가 스스로 disable

// MIT 범위 (매뉴얼 4.1.2, 위치 ±4π = 회전 수 셈)
constexpr double P_MIN = -4.0 * M_PI, P_MAX = 4.0 * M_PI;
constexpr double V_MIN = -44.0, V_MAX = 44.0;
constexpr double T_MIN = -17.0, T_MAX = 17.0;
constexpr double KP_MIN = 0.0, KP_MAX = 500.0;
constexpr double KD_MIN = 0.0, KD_MAX = 5.0;

inline uint32_t pack_id(uint8_t type, uint16_t data2, uint8_t data1) {
  return (static_cast<uint32_t>(type & 0x1F) << 24) | (static_cast<uint32_t>(data2) << 8) | data1;
}
inline uint8_t type_of(uint32_t id) { return static_cast<uint8_t>((id >> 24) & 0x1F); }
inline uint16_t data2_of(uint32_t id) { return static_cast<uint16_t>((id >> 8) & 0xFFFF); }

// Python lib.common.float_to_u16 과 비트 단위로 같게: 같은 순서의 double 연산 + 짝수 쪽 반올림(nearbyint).
// (p=0, v=0, t=0 이 정확히 32767.5 → 32768. lround 는 x.5 에서 Python 과 갈릴 수 있음)
inline uint16_t float_to_u16(double x, double lo, double hi) {
  x = x < lo ? lo : (x > hi ? hi : x);
  return static_cast<uint16_t>(std::nearbyint((x - lo) * 65535.0 / (hi - lo)));
}
inline double u16_to_float(uint16_t u, double lo, double hi) { return lo + (u / 65535.0) * (hi - lo); }

inline void put_be16(uint8_t* p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v >> 8);
  p[1] = static_cast<uint8_t>(v);
}
inline uint16_t get_be16(const uint8_t* p) { return static_cast<uint16_t>((p[0] << 8) | p[1]); }

inline Frame enc_enable(uint8_t host, uint8_t id) {
  Frame f;
  f.id = pack_id(ENABLE, host, id);
  return f;
}
inline Frame enc_stop(uint8_t host, uint8_t id, bool clear_fault = false) {
  Frame f;
  f.id = pack_id(STOP, host, id);
  f.data[0] = clear_fault ? 1 : 0;
  return f;
}
// MIT: 모터 안에서 t_ref = kp·(p − p_actual) + kd·(v − v_actual) + t
inline Frame enc_mit(uint8_t id, double p, double v, double kp, double kd, double t) {
  Frame f;
  f.id = pack_id(MIT_CONTROL, float_to_u16(t, T_MIN, T_MAX), id);
  put_be16(&f.data[0], float_to_u16(p, P_MIN, P_MAX));
  put_be16(&f.data[2], float_to_u16(v, V_MIN, V_MAX));
  put_be16(&f.data[4], float_to_u16(kp, KP_MIN, KP_MAX));
  put_be16(&f.data[6], float_to_u16(kd, KD_MIN, KD_MAX));
  return f;
}
inline Frame enc_write_u32(uint8_t host, uint8_t id, uint16_t index, uint32_t value) {
  Frame f;
  f.id = pack_id(PARAM_WRITE, host, id);
  f.data[0] = static_cast<uint8_t>(index);
  f.data[1] = static_cast<uint8_t>(index >> 8);
  for (int k = 0; k < 4; ++k) f.data[4 + k] = static_cast<uint8_t>(value >> (8 * k));
  return f;
}
inline Frame enc_write_f32(uint8_t host, uint8_t id, uint16_t index, float value) {
  uint32_t u;
  std::memcpy(&u, &value, 4);
  return enc_write_u32(host, id, index, u);
}

struct Feedback {
  uint8_t motor_id = 0;
  uint8_t mode_status = 0;
  uint8_t fault_bits = 0;  // ID bit21~16
  double pos = 0.0;        // [rad] 출력축. 켤 때 zero_sta=1 이면 −π~π, 이후 ±4π 안에서 회전 수 셈
  double vel = 0.0;        // [rad/s]
  double tor = 0.0;        // [Nm]
  double temp = 0.0;       // [°C]
};

inline std::optional<Feedback> dec_feedback(const Frame& f) {
  if (type_of(f.id) != FEEDBACK || f.len != 8) return std::nullopt;
  const uint16_t d2 = data2_of(f.id);
  const uint8_t flags = static_cast<uint8_t>(d2 >> 8);
  Feedback fb;
  fb.motor_id = static_cast<uint8_t>(d2 & 0xFF);
  fb.mode_status = (flags >> 6) & 0x03;
  fb.fault_bits = flags & 0x3F;
  fb.pos = u16_to_float(get_be16(&f.data[0]), P_MIN, P_MAX);
  fb.vel = u16_to_float(get_be16(&f.data[2]), V_MIN, V_MAX);
  fb.tor = u16_to_float(get_be16(&f.data[4]), T_MIN, T_MAX);
  fb.temp = get_be16(&f.data[6]) / 10.0;
  return fb;
}

// Type 21 → (motor_id, fault u32). 바이트 순서 LE 가정 (매뉴얼 미명시 — 0/비0 판단에만 씀)
struct FaultFrame {
  uint8_t motor_id = 0;
  uint32_t fault = 0;
  uint32_t warning = 0;
};
inline std::optional<FaultFrame> dec_fault(const Frame& f) {
  if (type_of(f.id) != FAULT_FEEDBACK || f.len != 8) return std::nullopt;
  FaultFrame r;
  r.motor_id = static_cast<uint8_t>(data2_of(f.id) & 0xFF);
  for (int k = 0; k < 4; ++k) {
    r.fault |= static_cast<uint32_t>(f.data[k]) << (8 * k);
    r.warning |= static_cast<uint32_t>(f.data[4 + k]) << (8 * k);
  }
  return r;
}

}  // namespace rs02
