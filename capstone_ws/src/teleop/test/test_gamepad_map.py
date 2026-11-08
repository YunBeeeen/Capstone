#!/usr/bin/env python3
"""gamepad_map 유닛테스트. 실행: PYTHONPATH=../scripts python3 test_gamepad_map.py"""
import math
import unittest

from gamepad_map import joy_to_cmd, stick_vector

CFG = {"require_enable_button": True, "enable_button": 5, "axis_linear_x": 1, "axis_linear_y": 0,
       "axis_angular": 3, "scale_linear_x": 0.25, "scale_linear_y": 0.25, "scale_angular": 0.5,
       "deadzone": 0.15, "deadzone_angular": 0.15}
RB = [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0]


def axes(lx=0.0, ly=0.0, rx=0.0):
    a = [0.0] * 8
    a[0], a[1], a[3] = lx, ly, rx
    return a


class TestGamepadMap(unittest.TestCase):
    def test_deadman(self):
        self.assertEqual(joy_to_cmd(axes(ly=1.0), [0] * 11, CFG), (False, 0.0, 0.0, 0.0))
        en, vx, vy, w = joy_to_cmd(axes(ly=1.0), RB, CFG)
        self.assertTrue(en)
        self.assertAlmostEqual(vx, 0.25)

    def test_release_noise_is_zero(self):
        # 2026-11-07 시뮬에서 바퀴를 돌린 잡음: 축별 0.05 데드존이면 (−0.06, 0) → "오른쪽 1.4 cm/s"
        for lx, ly in [(-0.06, -0.00), (0.06, 0.08), (0.04, -0.07), (0.1, 0.1)]:
            _, vx, vy, _ = joy_to_cmd(axes(lx, ly), RB, CFG)
            self.assertEqual((vx, vy), (0.0, 0.0), (lx, ly))

    def test_direction_kept_and_rescaled_from_edge(self):
        # 데드존 경계 바로 밖은 거의 0 (튀지 않음), 방향은 그대로
        sx, sy = stick_vector(0.16 * math.cos(0.5), 0.16 * math.sin(0.5), 0.15)
        self.assertAlmostEqual(math.atan2(sy, sx), 0.5)
        self.assertLess(math.hypot(sx, sy), 0.02)
        # 끝까지 밀면 1
        sx, sy = stick_vector(0.0, 1.0, 0.15)
        self.assertAlmostEqual(sy, 1.0)

    def test_diagonal_not_faster(self):
        _, vx, vy, _ = joy_to_cmd(axes(1.0, 1.0), RB, CFG)  # 모서리 (크기 1.41) → 1 로 클램프
        self.assertAlmostEqual(math.hypot(vx, vy), 0.25)

    def test_angular_deadzone(self):
        self.assertEqual(joy_to_cmd(axes(rx=0.1), RB, CFG)[3], 0.0)
        self.assertAlmostEqual(joy_to_cmd(axes(rx=-1.0), RB, CFG)[3], -0.5)

    def test_short_axes_safe(self):
        en, vx, vy, w = joy_to_cmd([0.5], RB, CFG)  # 없는 축은 0 (IndexError 안 남)
        self.assertEqual((vx, w), (0.0, 0.0))
        self.assertGreater(vy, 0.0)
        self.assertEqual(joy_to_cmd([], [], CFG), (False, 0.0, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
