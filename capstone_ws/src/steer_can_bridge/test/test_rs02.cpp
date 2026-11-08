// Python steer_can 과 같은 벡터 파일(rs02_vectors.csv)로 인코드/디코드 검증
#include <gtest/gtest.h>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "steer_can_bridge/rs02.hpp"

namespace {
std::vector<std::string> split(const std::string& s) {
  std::vector<std::string> out;
  std::stringstream ss(s);
  std::string tok;
  while (std::getline(ss, tok, ',')) out.push_back(tok);
  while (out.size() < 9) out.emplace_back();
  return out;
}

rs02::Frame frame_of(const std::string& id_hex, const std::string& data_hex) {
  rs02::Frame f;
  f.id = static_cast<uint32_t>(std::stoul(id_hex, nullptr, 16));
  f.len = static_cast<uint8_t>(data_hex.size() / 2);
  for (size_t i = 0; i < f.len; ++i) f.data[i] = static_cast<uint8_t>(std::stoul(data_hex.substr(2 * i, 2), nullptr, 16));
  return f;
}

void expect_same(const rs02::Frame& got, const rs02::Frame& want, const std::string& ctx) {
  EXPECT_EQ(got.id, want.id) << ctx;
  EXPECT_EQ(got.len, want.len) << ctx;
  EXPECT_EQ(got.data, want.data) << ctx;
}
}  // namespace

TEST(Rs02, SharedVectors) {
  std::ifstream in(VECTORS_CSV);
  ASSERT_TRUE(in.good()) << "벡터 파일 없음: " << VECTORS_CSV;
  constexpr uint8_t HOST = 0xFD;
  int n = 0;
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') continue;
    const auto c = split(line);
    const std::string& kind = c[0];
    const auto id = static_cast<uint8_t>(std::stoi(c[1]));
    const auto want = frame_of(c[7], c[8]);
    const std::string ctx = line;
    ++n;
    if (kind == "enable") {
      expect_same(rs02::enc_enable(HOST, id), want, ctx);
    } else if (kind == "stop") {
      expect_same(rs02::enc_stop(HOST, id, std::stoi(c[2]) != 0), want, ctx);
    } else if (kind == "mit") {
      expect_same(rs02::enc_mit(id, std::stod(c[2]), std::stod(c[3]), std::stod(c[4]), std::stod(c[5]), std::stod(c[6])),
                  want, ctx);
    } else if (kind == "write_f32") {
      expect_same(rs02::enc_write_f32(HOST, id, static_cast<uint16_t>(std::stoul(c[2], nullptr, 16)), std::stof(c[3])),
                  want, ctx);
    } else if (kind == "feedback") {
      const auto fb = rs02::dec_feedback(want);
      ASSERT_TRUE(fb.has_value()) << ctx;
      EXPECT_EQ(fb->motor_id, id) << ctx;
      EXPECT_EQ((fb->mode_status << 6) | fb->fault_bits, std::stoi(c[6])) << ctx;
      EXPECT_NEAR(fb->pos, std::stod(c[2]), 1e-9) << ctx;
      EXPECT_NEAR(fb->vel, std::stod(c[3]), 1e-9) << ctx;
      EXPECT_NEAR(fb->tor, std::stod(c[4]), 1e-9) << ctx;
      EXPECT_NEAR(fb->temp, std::stod(c[5]), 1e-9) << ctx;
    } else {
      ADD_FAILURE() << "unknown kind: " << ctx;
    }
  }
  EXPECT_GT(n, 15);
}

TEST(Rs02, HalfwayRoundsToEvenLikePython) {
  // (0 − (−4π))·65535/(8π) = 32767.5 → 32768 (짝수). lround 로 짜면 여기선 같지만 x.5 의 짝수 쪽 규칙은 nearbyint 만 보장
  EXPECT_EQ(rs02::float_to_u16(0.0, rs02::P_MIN, rs02::P_MAX), 32768);
  EXPECT_EQ(rs02::float_to_u16(2.5, 0.0, 65535.0 / 1.0 * 1.0), 2);  // 2.5 → 2 (Python round(2.5) == 2)
}

TEST(Rs02, RejectsOtherTypes) {
  rs02::Frame f;
  f.id = rs02::pack_id(rs02::ENABLE, 0x0201, 0xFD);
  EXPECT_FALSE(rs02::dec_feedback(f).has_value());
  f.id = rs02::pack_id(rs02::FAULT_FEEDBACK, 0xFD02, 0xFD);
  f.data = {4, 0, 0, 0, 1, 0, 0, 0};
  const auto ff = rs02::dec_fault(f);
  ASSERT_TRUE(ff.has_value());
  EXPECT_EQ(ff->motor_id, 2);
  EXPECT_EQ(ff->fault, 4u);
  EXPECT_EQ(ff->warning, 1u);
}
