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


class TestNavUdp(unittest.TestCase):
    def test_nav_roundtrip_and_not_mixed_with_pad(self):
        import udp_packet as up
        d = up.encode_nav(7, 0.1, -0.05, 0.3)
        seq, vx, vy, w = up.decode_nav(d)
        self.assertEqual(seq, 7)
        self.assertAlmostEqual(vx, 0.1, places=6)
        self.assertAlmostEqual(w, 0.3, places=6)
        self.assertIsNone(up.decode(d))                       # 라파 브리지가 Nav 패킷을 패드 명령으로 받지 않음
        self.assertIsNone(up.decode_nav(up.encode(1, 0.1, 0, 0)))
        self.assertIsNone(up.decode_nav(up.encode_nav(1, float("nan"), 0, 0)))

    def test_source_fresh_stale_and_reorder(self):
        import socket
        import time
        import udp_packet as up
        from virtual_pad import NavUdpSource
        src = NavUdpSource(0)
        port = src.sock.getsockname()[1]
        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        for seq, vx in ((5, 0.1), (4, 0.9), (6, 0.2)):        # 4 = 순서 뒤집힌 옛 패킷 → 버림
            tx.sendto(up.encode_nav(seq, vx, 0.0, 0.0), ("127.0.0.1", port))
        time.sleep(0.05)
        src.poll()
        self.assertAlmostEqual(src.latest(0.5)[0], 0.2, places=6)
        time.sleep(0.12)
        self.assertIsNone(src.latest(0.1))                    # 오래되면 None (패드가 0 을 냄)
        src.close()
        tx.close()

    def test_boost_min(self):
        from virtual_pad import boost_min
        self.assertEqual(boost_min(0.02, 0, 0, 0.08, 0.25)[0], 0.08)        # 작으면 최소값까지
        self.assertEqual(boost_min(0.003, 0, 0, 0.08, 0.25)[0], 0.003)      # 0 근처는 그대로 (서 있을 때)
        self.assertEqual(boost_min(0.2, 0, 0, 0.08, 0.25)[0], 0.2)
        self.assertEqual(boost_min(0, 0, -0.05, 0.08, 0.25)[2], -0.25)      # 제자리 돌기
        self.assertEqual(boost_min(0.05, 0, 0.1, 0.08, 0.25)[2], 0.1)       # 움직이는 중 회전은 그대로
        self.assertEqual(boost_min(0.02, 0, 0, 0.0, 0.0)[0], 0.02)          # 끔

    def test_estop_states_and_ack(self):
        import socket
        import time
        import udp_packet as up
        from virtual_pad import NavUdpSource
        self.assertEqual(up.decode_estop(up.encode_estop(3, True)), (3, True))
        self.assertIsNone(up.decode_nav(up.encode_estop(3, True)))
        self.assertIsNone(up.decode_estop(up.encode_nav(3, 0, 0, 0)))
        src = NavUdpSource(0, allow_from="127.0.0.2")          # Jetson 주소만 받는 설정이어도 노트북 정지는 받음
        port = src.sock.getsockname()[1]
        self.assertIsNone(src.estop())                           # 안 썼으면 None (패드 기준)
        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        tx.settimeout(0.5)
        tx.sendto(up.encode_estop(1, True), ("127.0.0.1", port))
        time.sleep(0.05)
        src.poll()
        self.assertEqual(src.estop(), "run")
        self.assertEqual(up.decode_estop_ack(tx.recvfrom(64)[0]), 1)
        tx.sendto(up.encode_estop(2, False), ("127.0.0.1", port))
        time.sleep(0.05)
        src.poll()
        self.assertEqual(src.estop(), "stop")
        tx.sendto(up.encode_estop(3, True), ("127.0.0.1", port))
        time.sleep(0.05)
        src.poll()
        time.sleep(0.06)
        self.assertEqual(src.estop(max_age=0.03), "lost")        # 끊기면 정지
        self.assertEqual(src.estop(), "stop")                   # 끊긴 뒤엔 진행 신호가 다시 와야 감
        src.close()
        tx.close()


if __name__ == "__main__":
    unittest.main()
