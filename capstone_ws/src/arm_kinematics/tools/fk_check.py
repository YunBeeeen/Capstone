#!/usr/bin/env python3
"""모터 각도 → 손끝 위치 (FK). 실기 팔의 손끝을 줄자로 잰 값과 비교해 모델(장착 45°·관절 방향·0점)을 확인한다.

  python3 tools/fk_check.py --zed-height 0.885                         # 0점 자세 (play_plan --pose 가 0 근처일 때)
  python3 tools/fk_check.py --zed-height 0.885 --motors=5.1,-60.2,-40.0,10.3,0,0   # play_plan --pose 출력 그대로

손끝 = IK 가 맞추는 점 (right_arm_tool0) = **두 손가락 끝의 가운데** (손가락 몸통은 이 점 뒤 0~9 cm).
좌표 = 로봇 바닥 기준: 원점은 로봇 중심 아래 바닥, x 앞 (ZED 방향), y 왼쪽, z 위. 오른팔이라 손끝 y 는 보통 음수.
"""
import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from arm_kinematics.b601 import B601Kinematics  # noqa: E402

ZED_HEIGHT_AT_LIFT0 = 0.675


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--motors", default="0,0,0,0,0,0", help="모터 1~6 [도], 쉼표. 음수로 시작하면 --motors=-5,...")
    ap.add_argument("--zed-height", type=float, required=True, help="바닥 → ZED 삼각대 나사 높이 실측 [m] (리프트 = 이 값 − 0.675)")
    args = ap.parse_args()
    m = [float(v) for v in args.motors.split(",")]
    if len(m) != 6:
        ap.error("--motors 는 6개")
    lift_total = args.zed_height - ZED_HEIGHT_AT_LIFT0
    kin = B601Kinematics()
    if not 0.0 <= lift_total <= 2 * kin.lift_stage_max + 1e-6:
        ap.error(f"--zed-height → 리프트 {1000 * lift_total:.0f} mm, 범위 0~{2000 * kin.lift_stage_max:.0f}")
    T = kin.fk(m, lift_total / 2)
    p, R = T.translation, T.rotation
    print(f"모터 1~6 {m} 도, 리프트 {1000 * lift_total:.0f} mm")
    print(f"손끝 (손가락 끝 가운데): x {100 * p[0]:6.1f} cm (앞)  y {100 * p[1]:6.1f} cm (왼쪽 +)  z {100 * p[2]:6.1f} cm (바닥에서)")
    print(f"손가락이 향하는 방향 (접근축): {np.round(R[:, 0], 2).tolist()}  — 수평에서 아래로 "
          f"{np.degrees(np.arcsin(-R[2, 0])) + 0.0:.0f}°")
    print(f"손가락이 벌어지는 방향:        {np.round(R[:, 1], 2).tolist()}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
