"""Python 바인딩 시험 — C++ gtest 와 같은 값이 나오는지 (몇 개만, 바인딩 연결 확인용)."""
import math
import unittest

import swerve_kinematics as sk

DEG = math.pi / 180
SQ = [(0.12, -0.12), (-0.12, -0.12), (-0.12, 0.12), (0.12, 0.12)]  # FR RR RL FL


def make(gate=True):
    c = sk.ControllerConfig()
    c.modules = SQ
    c.lo, c.hi = [-170 * DEG] * 4, [170 * DEG] * 4
    c.max_wheel_mps = 0.6
    c.unwind_rad, c.slow_mps = 100 * DEG, 0.02
    if gate:
        c.gate_trigger_rad, c.gate_release_rad = 60 * DEG, 10 * DEG
    return sk.Controller(c)


class TestBindings(unittest.TestCase):
    def test_config_roundtrip(self):
        c = sk.ControllerConfig()
        c.modules = SQ
        self.assertEqual([tuple(m) for m in c.modules], SQ)

    def test_forward(self):
        r = make(gate=False).step(0.3, 0, 0)
        for a, v in zip(r.angle, r.wheel_speed):
            self.assertAlmostEqual(a, 0)
            self.assertAlmostEqual(v, 0.3)

    def test_rotate_fl_flips(self):
        r = make(gate=False).step(0, 0, 1.0)
        self.assertAlmostEqual(r.angle[3] / DEG, -45, places=9)  # FL
        self.assertAlmostEqual(r.speed[3], -0.12 * math.sqrt(2))

    def test_gate_and_seed(self):
        ctl = make()
        r = ctl.step(0, 0.3, 0, [0.0] * 4, True)
        self.assertTrue(r.gated)
        self.assertEqual(list(r.wheel_speed), [0.0] * 4)
        r = ctl.step(0, 0.3, 0, [85 * DEG] * 4, True)
        self.assertFalse(r.gated)
        ctl.seed([80 * DEG] * 4)
        r = ctl.step(0, -0.3, 0, [80 * DEG] * 4, True)
        for a, v in zip(r.angle, r.speed):
            self.assertAlmostEqual(a, math.pi / 2)
            self.assertLess(v, 0)

    def test_fk_roundtrip(self):
        ctl = make(gate=False)
        for t in [(0.3, 0, 0), (0.2, 0.1, -0.7), (-0.25, 0.15, 0.4)]:
            r = ctl.step(*t)
            got = sk.fk(SQ, r.angle, r.speed)
            for g, w in zip(got, t):
                self.assertAlmostEqual(g, w, places=9)


if __name__ == "__main__":
    unittest.main()
