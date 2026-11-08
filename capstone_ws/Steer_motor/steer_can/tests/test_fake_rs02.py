"""fake_rs02 응답 로직 (vcan 없이): enable → RUN, MIT 로 따라감, stop → RESET, 켤 때 각도 접힘."""

import math
import unittest

from lib.frames import frame_type01_mit, frame_type03_enable, frame_type04_stop, frame_type18_write_u32
from lib.parse import MODE_RESET, MODE_RUN, parse_feedback_like_type2
from tests.fake_rs02 import FakeMotor


class TestFakeRs02(unittest.TestCase):
    def test_power_on_angle_folds(self):
        m = FakeMotor(1, start_deg=185.0)
        fb = parse_feedback_like_type2(*m.handle(*frame_type04_stop(0xFD, 1), 0.0)[0])
        self.assertAlmostEqual(math.degrees(fb.pos), -175.0, delta=0.05)
        self.assertEqual(fb.mode_status, MODE_RESET)

    def test_enable_mit_track_stop(self):
        m = FakeMotor(2)
        fb = parse_feedback_like_type2(*m.handle(*frame_type03_enable(0xFD, 2), 0.0)[0])
        self.assertEqual(fb.mode_status, MODE_RUN)
        t = 0.0
        for _ in range(200):
            t += 0.005
            m.handle(*frame_type01_mit(2, 1.0, 0.0, 20.0, 2.0, 0.0), t)
            m.tick(0.005, t)
        self.assertAlmostEqual(m.pos, 1.0, places=3)
        fb = parse_feedback_like_type2(*m.handle(*frame_type04_stop(0xFD, 2), t)[0])
        self.assertEqual(fb.mode_status, MODE_RESET)
        self.assertEqual(m.handle(*frame_type04_stop(0xFD, 3), t), [])  # 다른 ID 는 무시

    def test_can_timeout_disables(self):
        m = FakeMotor(1)
        m.handle(*frame_type18_write_u32(0xFD, 1, 0x7028, 10000), 0.0)
        m.handle(*frame_type03_enable(0xFD, 1), 0.0)
        m.tick(0.01, 0.4)
        self.assertEqual(m.mode, MODE_RUN)
        m.tick(0.01, 0.6)
        self.assertEqual(m.mode, MODE_RESET)


if __name__ == "__main__":
    unittest.main()
