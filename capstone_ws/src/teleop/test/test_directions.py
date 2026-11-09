#!/usr/bin/env python3
"""세 조종 경로가 같은 의도 → 같은 cmd_vel 방향인지 (실제 config/shaks_s0.yaml 을 읽어서).

  가상 패드 (마우스)            : joystick_math.left_pad_to_linear / right_pad_to_angular (화면 위 = −y 픽셀)
  게임패드 → 라파 joy_node      : gamepad_map.joy_to_cmd, ROS joy 규약 (스틱 왼쪽·위 = +)
  게임패드 → 노트북 pygame       : pygame 축은 왼쪽·위 = − → virtual_pad.Gamepad 가 뒤집어서 joy_to_cmd

기대 (REP-103): 위/전진 = +vx, 왼쪽 = +vy, 회전 스틱·패드 왼쪽 = 반시계 = +ω.
2026-11-08: yaml 의 scale_linear_y / scale_angular 가 −로 되돌아가 있었는데 기존 테스트는 scale 을 코드에 박아 둬서 못 잡음.
"""
import unittest
from pathlib import Path

import yaml

from gamepad_map import joy_to_cmd
from joystick_math import left_pad_to_linear, right_pad_to_angular

CFG = yaml.safe_load((Path(__file__).resolve().parents[1] / "config" / "shaks_s0.yaml").read_text())[
    "gamepad_teleop"]["ros__parameters"]
DEADMAN = [0] * 11
DEADMAN[CFG["enable_button"]] = 1


def joy_axes(lx=0.0, ly=0.0, rx=0.0):
    """ROS joy 규약 축 배열 (왼쪽·위 = +)."""
    a = [0.0] * 8
    a[CFG["axis_linear_y"]], a[CFG["axis_linear_x"]], a[CFG["axis_angular"]] = lx, ly, rx
    return a


def pygame_to_joy(raw):
    """virtual_pad.Gamepad.read 와 같은 변환."""
    return [-v for v in raw]


def sign(v, eps=1e-9):
    return 0 if abs(v) < eps else (1 if v > 0 else -1)


class TestDirections(unittest.TestCase):
    # (이름, 가상 패드 (nx, ny, rnx), joy 축 (lx, ly, rx), 기대 부호 (vx, vy, ω))
    CASES = [
        ("전진", (0.0, -1.0, 0.0), (0.0, 1.0, 0.0), (1, 0, 0)),
        ("후진", (0.0, 1.0, 0.0), (0.0, -1.0, 0.0), (-1, 0, 0)),
        ("좌 횡이동", (-1.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0, 1, 0)),
        ("우 횡이동", (1.0, 0.0, 0.0), (-1.0, 0.0, 0.0), (0, -1, 0)),
        ("반시계 회전", (0.0, 0.0, -1.0), (0.0, 0.0, 1.0), (0, 0, 1)),
        ("시계 회전", (0.0, 0.0, 1.0), (0.0, 0.0, -1.0), (0, 0, -1)),
    ]

    def test_virtual_pad(self):
        for name, (nx, ny, rnx), _, want in self.CASES:
            vx, vy = left_pad_to_linear(nx, ny, 0.3)
            w = right_pad_to_angular(rnx, 1.0)
            self.assertEqual((sign(vx), sign(vy), sign(w)), want, f"가상 패드 {name}")

    def test_gamepad_ros_joy(self):
        for name, _, (lx, ly, rx), want in self.CASES:
            on, vx, vy, w = joy_to_cmd(joy_axes(lx, ly, rx), DEADMAN, CFG)
            self.assertTrue(on)
            self.assertEqual((sign(vx), sign(vy), sign(w)), want, f"게임패드(joy) {name} — yaml scale 부호 확인")

    def test_gamepad_laptop_pygame(self):
        for name, _, (lx, ly, rx), want in self.CASES:
            raw = [-v for v in joy_axes(lx, ly, rx)]  # pygame 이 실제로 주는 값 (왼쪽·위 = −)
            on, vx, vy, w = joy_to_cmd(pygame_to_joy(raw), DEADMAN, CFG)
            self.assertEqual((sign(vx), sign(vy), sign(w)), want, f"게임패드(pygame) {name}")

    def test_all_scales_positive(self):
        for k in ("scale_linear_x", "scale_linear_y", "scale_angular"):
            self.assertGreater(CFG[k], 0, f"{k} 는 + 여야 함 (joy 규약 기준)")


if __name__ == "__main__":
    unittest.main()
