#!/usr/bin/env python3
"""노트북 패드 ↔ 라파 UDP 패킷 (v1 호환 + v2 리프트)."""
import struct
import unittest

import udp_packet as up


class TestUdpPacket(unittest.TestCase):
    def test_v2_roundtrip(self):
        d = up.encode(7, 0.1, -0.2, 0.3, -50, up.LIFT_PARK | up.LIFT_CLEAR)
        self.assertEqual(len(d), 22)
        seq, vx, vy, w, pwm, flags = up.decode(d)
        self.assertEqual((seq, pwm, flags), (7, -50, up.LIFT_PARK | up.LIFT_CLEAR))
        self.assertAlmostEqual(vy, -0.2, places=6)

    def test_v1_still_accepted(self):
        d = struct.pack("<4sIfff", b"CAPS", 3, 0.1, 0.0, 0.0)
        self.assertEqual(up.decode(d)[4:], (0, 0))  # 리프트 입력 없음

    def test_pwm_clamped_on_encode(self):
        self.assertEqual(up.decode(up.encode(1, 0, 0, 0, 150, 0))[4], 100)
        self.assertEqual(up.decode(up.encode(1, 0, 0, 0, -150, 0))[4], -100)

    def test_rejects_bad(self):
        self.assertIsNone(up.decode(b"CAPS" + b"\0" * 10))
        self.assertIsNone(up.decode(struct.pack("<4sIfffbB", b"XXXX", 1, 0, 0, 0, 0, 0)))
        self.assertIsNone(up.decode(struct.pack("<4sIfffbB", b"CAPS", 1, float("nan"), 0, 0, 0, 0)))
        self.assertIsNone(up.decode(struct.pack("<4sIfffbB", b"CAPS", 1, 0, 0, 0, 101, 0)))


class TestLiftButtons(unittest.TestCase):
    def test_lift_pwm(self):
        from virtual_pad import lift_pwm
        f = lambda *b: lift_pwm(*b, 100, 50)  # noqa: E731
        self.assertEqual(f(1, 0, 0, 0), 100)    # Y 빠른 위
        self.assertEqual(f(0, 1, 0, 0), -100)   # A 빠른 아래
        self.assertEqual(f(0, 0, 1, 0), 50)     # X 느린 위
        self.assertEqual(f(0, 0, 0, 1), -50)    # B 느린 아래
        self.assertEqual(f(1, 0, 1, 0), 100)    # 빠른 우선
        self.assertEqual(f(1, 1, 0, 0), 0)      # 위·아래 같이 = 0
        self.assertEqual(f(0, 0, 0, 0), 0)


class TestSpeedLimits(unittest.TestCase):
    def test_dpad_xy(self):
        from virtual_pad import adjust_limits
        self.assertEqual(adjust_limits(0.15, 0.5, 0, 1), (0.2, 0.5))     # 위 = 선속도 +
        self.assertEqual(adjust_limits(0.15, 0.5, 0, -1), (0.1, 0.5))    # 아래 = 선속도 −
        self.assertEqual(adjust_limits(0.15, 0.5, 1, 0), (0.15, 0.6))    # 오른쪽 = 각속도 +
        self.assertEqual(adjust_limits(0.15, 0.5, -1, 0), (0.15, 0.4))   # 왼쪽 = 각속도 −
        self.assertEqual(adjust_limits(0.05, 0.1, -1, -1), (0.05, 0.1))  # 하한
        self.assertEqual(adjust_limits(0.6, 2.0, 1, 1), (0.6, 2.0))      # 상한


if __name__ == "__main__":
    unittest.main()
