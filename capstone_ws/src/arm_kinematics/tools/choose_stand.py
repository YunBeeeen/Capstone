#!/usr/bin/env python3
"""가구 앞 설 자리 고르기 (정지 오차에 강한 자리 + 리프트 높이).

  python3 tools/choose_stand.py --target shelf3 --along=-0.10
  python3 tools/choose_stand.py --target desk --along 0.0 --nav-err 0.05,0.05,5

출력: 가구 앞면까지 거리, 가구 가운데의 좌우 위치(로봇 기준), 리프트 높이, 정지 오차 통과 수.
place_mark.py 의 설 자리를 이 값으로 잡으면 된다 (앞면 거리 = 로봇 중심 → 가구 앞면, 차체 앞 끝은 0.195 m).
"""
import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from arm_kinematics import B601Kinematics  # noqa: E402
from arm_kinematics.gravity import conservative_limits  # noqa: E402
from arm_kinematics.scene import ALONG, BODY_FRONT_X, PRESETS  # noqa: E402
from arm_kinematics.stand import choose_stand  # noqa: E402

ZED_HEIGHT_AT_LIFT0 = 0.675


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--target", choices=PRESETS, default="shelf3")
    ap.add_argument("--along", type=float, default=ALONG, help="물체의 가구 폭 방향 위치 [m] (− = 로봇 오른쪽). 음수는 --along=-0.1")
    ap.add_argument("--fronts", default="0.35,0.40,0.45,0.50,0.55")
    ap.add_argument("--laterals", default="0,-0.05,0.05,-0.10,0.10", help="가구 가운데 y 후보 [m] (ZED 가 가구를 보도록 작게)")
    ap.add_argument("--nav-err", default="0.03,0.03,3", help="정지 오차: 앞뒤 m, 좌우 m, 방향 도")
    ap.add_argument("--zed-height", type=float, default=0.885, help="지금 ZED 높이 [m] (리프트 이동 비용 기준)")
    ap.add_argument("--start", help="시작 자세 모터 1~6 [도] (기본 0점)")
    ap.add_argument("--limits", choices=["conservative", "collector"], default="conservative")
    args = ap.parse_args()

    kin = B601Kinematics()
    if args.limits == "conservative":
        kin.set_motor_limits_deg(conservative_limits(kin))
    start = [float(v) for v in args.start.split(",")] if args.start else [0.0] * 6
    start_lift = min(max((args.zed_height - ZED_HEIGHT_AT_LIFT0) / 2, 0.0), kin.lift_stage_max)
    t0 = time.time()
    print(f"{args.target}, 물체 폭 위치 {args.along:+.2f} m, 지금 리프트 {2000 * start_lift:.0f} mm, 정지 오차 ±{args.nav_err}")
    best, _ = choose_stand(args.target, args.along, kin,
                           fronts=[float(v) for v in args.fronts.split(",")],
                           laterals=[float(v) for v in args.laterals.split(",")],
                           start_deg=start, start_lift=start_lift,
                           nav_err=tuple(float(v) for v in args.nav_err.split(",")))
    print(f"\n({time.time() - t0:.0f} s)")
    if best is None:
        print("어느 자리에서도 계획 불가")
        return 1
    print(f"추천 설 자리: 가구 앞면까지 {best.front:.2f} m (차체 앞에서 {100 * (best.front - BODY_FRONT_X):.0f} cm), "
          f"가구 가운데 y {best.lateral:+.2f} m, 리프트 {best.lift_mm:.0f} mm (ZED {ZED_HEIGHT_AT_LIFT0 + best.lift_mm / 1000:.3f} m), "
          f"cost {best.total_cost:.2f}, 정지 오차 {best.robust}/{best.perturbations} 통과")
    return 0


if __name__ == "__main__":
    sys.exit(main())
