#!/usr/bin/env python3
"""설 자리 찾기: 가구 가운데를 ZED 정면에 두고, 앞면 거리 × 물체의 폭 방향 위치마다 집을 수 있는지 훑는다.

  python3 tools/stand_sweep.py                          # 책상 + 선반 1~4층, 기본 격자
  python3 tools/stand_sweep.py --targets shelf3,shelf4 --front 0.3,0.35,0.4 --along=-0.2,-0.1,0

각 칸: heading 모드 IK (그리퍼가 가구 안쪽, 기울기 −10~45°), 리프트 0/210/420/630 mm,
충돌 검사 (팔 ↔ 몸체·가구), 그다음 실제 경로 계획(시작 0점 자세 → 접근점 → 직선 삽입)까지 통과해야 ✓.
칸 값 = 고른 후보의 cost (작을수록 좋음), × = 불가.
가구 앞면이 차체 앞 끝(ZED, 0.195 m)보다 5 cm 이상 떨어진 거리만 본다.
"""
import argparse
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from arm_kinematics import B601Kinematics  # noqa: E402
from arm_kinematics.collision import CollisionChecker  # noqa: E402
from arm_kinematics.ik import B601IK  # noqa: E402
from arm_kinematics.motion import MotionPlanner  # noqa: E402
from arm_kinematics.scene import BODY_FRONT_X, PRESETS, env_for, preset_target  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--targets", default=",".join(PRESETS))
    ap.add_argument("--front", default="0.25,0.30,0.35,0.40,0.45,0.50,0.55,0.60")
    ap.add_argument("--lateral", type=float, default=0.0, help="가구 가운데 y [m] (ZED 정면 = 0)")
    ap.add_argument("--along", default="0.2,0.1,0,-0.1,-0.2", help="물체의 가구 폭 방향 위치 [m] (− = 로봇 오른쪽)")
    ap.add_argument("--tilt", default="-10,45")
    ap.add_argument("--seeds", type=int, default=16)
    args = ap.parse_args()
    kin = B601Kinematics()
    fronts = [float(v) for v in args.front.split(",") if float(v) >= BODY_FRONT_X + 0.05]
    alongs = [float(v) for v in args.along.split(",")]
    lat = args.lateral
    tilt = tuple(float(v) for v in args.tilt.split(","))
    lifts = [0.0, 0.105, 0.21, 0.315]
    best_overall = {}
    for name in args.targets.split(","):
        t0 = time.time()
        rows = {}
        for al in alongs:
            for fr in fronts:
                target, into, _ = preset_target(name, front=fr, lateral=lat, along=al)
                ik = B601IK(kin, collision=CollisionChecker(kin, env_for(name, fr, lat)))
                cands = ik.solve(target, lifts, heading=into, tilt_deg=tilt, n_random=args.seeds)
                plan = MotionPlanner(ik).plan({i: 0.0 for i in range(1, 7)}, target, cands, heading=into, max_tries=4)
                ok = plan is not None and plan.candidate is not None
                rows[(al, fr)] = (plan.candidate.cost, plan.candidate.lift_stage_m) if ok else None
                if ok and (name not in best_overall or plan.candidate.cost < best_overall[name][0]):
                    best_overall[name] = (plan.candidate.cost, fr, al, plan.candidate.lift_stage_m, plan.candidate.load)
        print(f"\n== {name}  (목표 = 앞면 안쪽, 면 위 5 cm · {time.time() - t0:.0f}s)")
        print("  along \\ front " + "".join(f"{fr:>8.2f}" for fr in fronts))
        for al in alongs:
            cells = []
            for fr in fronts:
                r = rows[(al, fr)]
                cells.append(f"{'×':>8}" if r is None else f"{r[0]:>5.2f}/{int(2000 * r[1] / 210)}")
            print(f"{al:>+15.2f} " + "".join(f"{c:>8}" for c in cells))
        print("  칸 = cost/리프트 단계(0~3 = 0/210/420/630 mm), × = 불가")
    print("\n== 추천 (cost 최소)")
    for name, (cost, fr, al, lift, load) in best_overall.items():
        print(f"  {name:7s} 앞면까지 {fr:.2f} m (차체 앞에서 {100 * (fr - BODY_FRONT_X):.0f} cm), "
              f"물체 폭 위치 {al:+.2f} m, 리프트 {2000 * lift:.0f} mm, cost {cost:.2f}, 부하 {load:.2f}")
    missing = [n for n in args.targets.split(",") if n not in best_overall]
    if missing:
        print("  어디서도 안 됨:", ", ".join(missing))


if __name__ == "__main__":
    main()
