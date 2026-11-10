#!/usr/bin/env python3
"""노트북 패드 ↔ 라파 UDP 패킷 (v1 호환 + v2 리프트)."""
import struct
import unittest

import udp_packet as up


class TestUdpPacket(unittest.TestCase):
    def test_v2_roundtrip(self):
        d = up.encode(7, 0.1, -0.2, 0.3, -1, up.LIFT_PARK | up.LIFT_CLEAR)
        self.assertEqual(len(d), 22)
        seq, vx, vy, w, jog, flags = up.decode(d)
        self.assertEqual((seq, jog, flags), (7, -1, up.LIFT_PARK | up.LIFT_CLEAR))
        self.assertAlmostEqual(vy, -0.2, places=6)

    def test_v1_still_accepted(self):
        d = struct.pack("<4sIfff", b"CAPS", 3, 0.1, 0.0, 0.0)
        self.assertEqual(up.decode(d)[4:], (0, 0))  # 리프트 입력 없음

    def test_jog_clamped_on_encode(self):
        self.assertEqual(up.decode(up.encode(1, 0, 0, 0, 5, 0))[4], 1)

    def test_rejects_bad(self):
        self.assertIsNone(up.decode(b"CAPS" + b"\0" * 10))
        self.assertIsNone(up.decode(struct.pack("<4sIfffbB", b"XXXX", 1, 0, 0, 0, 0, 0)))
        self.assertIsNone(up.decode(struct.pack("<4sIfffbB", b"CAPS", 1, float("nan"), 0, 0, 0, 0)))
        self.assertIsNone(up.decode(struct.pack("<4sIfffbB", b"CAPS", 1, 0, 0, 0, 3, 0)))


if __name__ == "__main__":
    unittest.main()
