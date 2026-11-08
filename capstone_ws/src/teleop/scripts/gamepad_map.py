"""게임패드 /joy → 차체 속도 (순수 함수 — rclpy 없이 테스트). gamepad_teleop.py 가 씀.

왼쪽 스틱 (x=axes[axis_linear_y], y=axes[axis_linear_x]) 을 하나의 벡터로 보고:
  1) 단위 원으로 클램프 — 대각선이 직선보다 빠르지 않게 (패드 스틱은 모서리에서 크기 ~1.2 까지 나옴)
  2) 원형 데드존 (joystick_math.apply_deadzone) — 크기 < deadzone 면 (0, 0), 밖은 방향 유지 + 0~1 로 다시 늘림
     축별 데드존은 한 축만 살아남아 "정확히 옆/앞" 잡음 벡터를 만듦 → 스틱 놓을 때 바퀴가 홱 돎 (2026-11-07 시뮬)
  3) 축별 scale 곱 (teleop_twist_joy 와 같은 의미: cmd = axis × scale)
회전 (axes[axis_angular]) 은 1차원 데드존 + 다시 늘림.
"""
import math

from joystick_math import apply_deadzone


def _axis(axes, i):
    return float(axes[i]) if 0 <= i < len(axes) else 0.0


def _button(buttons, i):
    return 0 <= i < len(buttons) and bool(buttons[i])


def stick_vector(sx: float, sy: float, deadzone: float):
    """(sx, sy) 스틱 → 단위 원 클램프 + 원형 데드존. 반환 (sx, sy)."""
    mag = math.hypot(sx, sy)
    if mag > 1.0:
        sx, sy = sx / mag, sy / mag
    return apply_deadzone(sx, sy, deadzone)


def joy_to_cmd(axes, buttons, cfg: dict):
    """반환 (enabled, vx, vy, omega). enabled = deadman 버튼 눌림 (require_enable_button=False 면 항상 True)."""
    enabled = (not cfg["require_enable_button"]) or _button(buttons, cfg["enable_button"])
    if not enabled:
        return False, 0.0, 0.0, 0.0
    ly = _axis(axes, cfg["axis_linear_x"])   # 스틱 위/아래 → vx
    lx = _axis(axes, cfg["axis_linear_y"])   # 스틱 좌/우 → vy
    lx, ly = stick_vector(lx, ly, cfg["deadzone"])
    r, _ = apply_deadzone(max(-1.0, min(1.0, _axis(axes, cfg["axis_angular"]))), 0.0, cfg["deadzone_angular"])
    return True, ly * cfg["scale_linear_x"], lx * cfg["scale_linear_y"], r * cfg["scale_angular"]
