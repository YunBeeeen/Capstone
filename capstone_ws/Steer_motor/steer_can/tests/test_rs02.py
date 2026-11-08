"""RS02 프레임 인코드/디코드 + C++ 와 공용 벡터(rs02_vectors.csv).

    cd ~/capstone/capstone_ws/Steer_motor/steer_can && python3 -m unittest discover -s tests -t .
"""

import math
import unittest
from pathlib import Path

from lib.common import float_to_u16, pack_ext_id, unpack_ext_id
from lib.frames import (frame_type01_mit, frame_type03_enable, frame_type04_stop, frame_type18_write_f32,
                        MIT_RS02)
from lib.parse import fault_names, parse_feedback_any, parse_feedback_like_type2, parse_type21_fault

VECTORS = Path(__file__).with_name("rs02_vectors.csv")
HOST = 0xFD


class TestFrames(unittest.TestCase):
    def test_ext_id_layout(self):
        arb = pack_ext_id(0x03, data2=HOST, data1=1)
        self.assertEqual(arb, 0x0300FD01)
        self.assertEqual(unpack_ext_id(arb), (0x03, HOST, 1))

    def test_mit_zero_is_midscale(self):
        arb, data = frame_type01_mit(1, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.assertEqual((arb >> 8) & 0xFFFF, 0x8000)          # t=0 → 32767.5 → 32768 (짝수 쪽)
        self.assertEqual(data[0:4], bytes.fromhex("80008000"))  # p=0, v=0
        self.assertEqual(data[4:8], bytes(4))                    # kp=kd=0

    def test_mit_clamps(self):
        _, data = frame_type01_mit(1, 100.0, -100.0, 1e4, 1e4, 0.0)
        self.assertEqual(data, bytes.fromhex("FFFF0000FFFFFFFF"))

    def test_type02_parse(self):
        flags = (2 << 6) | 0b000100       # RUN + overtemp
        arb = pack_ext_id(0x02, data2=(flags << 8) | 3, data1=HOST)
        fb = parse_feedback_like_type2(arb, bytes.fromhex("8000800080000190"))
        self.assertEqual((fb.motor_id, fb.mode_status, fb.fault_bits), (3, 2, 0b000100))
        self.assertAlmostEqual(fb.pos, 0.0, places=3)
        self.assertAlmostEqual(fb.temp_c, 40.0)
        self.assertEqual(fault_names(fb.fault_bits), "overtemp")
        self.assertIsNone(parse_feedback_like_type2(pack_ext_id(0x18, (flags << 8) | 3, HOST), bytes(8)))

    def test_type21_parse(self):
        arb = pack_ext_id(0x15, data2=(HOST << 8) | 2, data1=HOST)
        self.assertEqual(parse_type21_fault(arb, bytes.fromhex("0400000001000000")), (2, 4, 1))


class TestSharedVectors(unittest.TestCase):
    def test_vectors(self):
        rows = [l.split(",") for l in VECTORS.read_text().splitlines() if l and not l.startswith("#")]
        self.assertGreater(len(rows), 15)
        for kind, cid, a, b, c, d, e, arb_hex, data_hex in rows:
            with self.subTest(kind=kind, can_id=cid, a=a):
                cid_i, arb, data = int(cid), int(arb_hex, 16), bytes.fromhex(data_hex)
                if kind == "enable":
                    self.assertEqual(frame_type03_enable(HOST, cid_i), (arb, data))
                elif kind == "stop":
                    self.assertEqual(frame_type04_stop(HOST, cid_i, bool(int(a))), (arb, data))
                elif kind == "mit":
                    self.assertEqual(frame_type01_mit(cid_i, *map(float, (a, b, c, d, e))), (arb, data))
                elif kind == "write_f32":
                    self.assertEqual(frame_type18_write_f32(HOST, cid_i, int(a, 16), float(b)), (arb, data))
                elif kind == "feedback":
                    fb = parse_feedback_like_type2(arb, data)
                    self.assertEqual(fb.motor_id, cid_i)
                    self.assertEqual((fb.mode_status << 6) | fb.fault_bits, int(e))
                    for got, want in ((fb.pos, a), (fb.vel, b), (fb.tor, c), (fb.temp_c, d)):
                        self.assertAlmostEqual(got, float(want), places=9)
                else:
                    self.fail(f"unknown kind {kind}")


class TestActiveReport(unittest.TestCase):
    def test_type24_frame_parsed_like_type02(self):
        # 2026-11-08 실기 candump (active report 켠 상태, 토크 끔)
        fb = parse_feedback_any(0x180001FD, bytes.fromhex("9BFE80147FFF00FA"))
        self.assertEqual((fb.motor_id, fb.host_id, fb.mode_status, fb.fault_bits), (1, 0xFD, 0, 0))
        self.assertAlmostEqual(math.degrees(fb.pos), 157.47, places=1)
        self.assertAlmostEqual(fb.temp_c, 25.0)
        self.assertAlmostEqual(fb.tor, 0.0, places=2)
        # Type02 전용 파서는 여전히 0x18 을 거부
        self.assertIsNone(parse_feedback_like_type2(0x180001FD, bytes(8)))


if __name__ == "__main__":
    unittest.main()
