// Python wheel_can 과 같은 벡터 파일(cansimple_vectors.csv)로 인코드/디코드 검증
#include <gtest/gtest.h>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "wheel_can_bridge/cansimple.hpp"

using namespace wheel_can;

namespace {
std::vector<std::string> split(const std::string& s) {
  std::vector<std::string> out;
  std::stringstream ss(s);
  std::string tok;
  while (std::getline(ss, tok, ',')) out.push_back(tok);
  while (out.size() < 6) out.emplace_back();
  return out;
}

std::vector<uint8_t> unhex(const std::string& h) {
  std::vector<uint8_t> out;
  for (size_t i = 0; i + 1 < h.size(); i += 2) out.push_back(static_cast<uint8_t>(std::stoul(h.substr(i, 2), nullptr, 16)));
  return out;
}

cs::Frame frame_of(uint32_t id, const std::vector<uint8_t>& d) {
  cs::Frame f;
  f.id = id;
  f.len = static_cast<uint8_t>(d.size());
  std::copy(d.begin(), d.end(), f.data.begin());
  return f;
}
}  // namespace

TEST(CanSimple, SharedVectors) {
  std::ifstream in(VECTORS_CSV);
  ASSERT_TRUE(in.good()) << "벡터 파일 없음: " << VECTORS_CSV;
  int n = 0;
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') continue;
    const auto c = split(line);
    const std::string& kind = c[0];
    const int id = std::stoi(c[1]);
    const uint32_t arb = static_cast<uint32_t>(std::stoul(c[4], nullptr, 16));
    const auto data = unhex(c[5]);
    SCOPED_TRACE(line);
    ++n;

    cs::Frame enc;
    bool is_enc = true;
    if (kind == "axis_state") enc = cs::axis_state(id, static_cast<uint32_t>(std::stoul(c[2])));
    else if (kind == "input_vel") enc = cs::input_vel(id, std::stof(c[2]), std::stof(c[3]));
    else if (kind == "controller_mode") enc = cs::controller_mode(id, std::stoi(c[2]), std::stoi(c[3]));
    else if (kind == "clear_errors") enc = cs::clear_errors(id);
    else if (kind == "rtr_encoder") enc = cs::rtr(id, cs::GET_ENCODER_ESTIMATES);
    else if (kind == "rtr_iq") enc = cs::rtr(id, cs::GET_IQ);
    else is_enc = false;

    if (is_enc) {
      EXPECT_EQ(enc.id, arb);
      if (enc.rtr) {
        EXPECT_TRUE(data.empty());
      } else {
        ASSERT_EQ(enc.len, data.size());
        for (size_t i = 0; i < data.size(); ++i) EXPECT_EQ(enc.data[i], data[i]) << "byte " << i;
      }
      continue;
    }

    const auto f = frame_of(arb, data);
    if (kind == "heartbeat") {
      EXPECT_EQ(arb, cs::arb_id(id, cs::HEARTBEAT));
      const auto hb = cs::dec_heartbeat(f);
      EXPECT_EQ(hb.axis_error, std::stoul(c[2]));
      EXPECT_EQ(hb.axis_state, std::stoul(c[3]));
    } else if (kind == "encoder_estimates" || kind == "iq") {
      EXPECT_EQ(arb, cs::arb_id(id, kind == "iq" ? cs::GET_IQ : cs::GET_ENCODER_ESTIMATES));
      const auto p = cs::dec_f32_pair(f);
      EXPECT_NEAR(p.a, std::stod(c[2]), 1e-4);
      EXPECT_NEAR(p.b, std::stod(c[3]), 1e-4);
    } else if (kind == "vbus") {
      EXPECT_EQ(arb, cs::arb_id(id, cs::GET_VBUS));
      EXPECT_NEAR(cs::dec_vbus(f), std::stod(c[2]), 1e-3);
    } else {
      ADD_FAILURE() << "모르는 kind: " << kind;
    }
  }
  EXPECT_GT(n, 10);
}

TEST(CanSimple, IdRange) {
  EXPECT_EQ(cs::arb_id(16, cs::HEARTBEAT), 0x201u);  // XDrive 공장 axis0
  EXPECT_EQ(cs::id_of(0x081), 4);
  EXPECT_EQ(cs::cmd_of(0x081), cs::HEARTBEAT);
  EXPECT_THROW(cs::arb_id(64, cs::HEARTBEAT), std::out_of_range);
}
