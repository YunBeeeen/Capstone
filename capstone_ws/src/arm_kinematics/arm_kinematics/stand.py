"""설 자리 고르기: 가구(선반·책상) 앞 어디에, 리프트 몇 mm 로 서야 물체를 잘 집고·둘 수 있나.

내비게이션은 ZED(로봇 정면 +x)가 가구를 보며 선다. 후보 = 앞면 거리 × 가구 가운데의 좌우 위치(작게, ZED 가
가구를 봐야 하므로). 리프트는 IK 가 같이 고른다 (lift_free). 각 후보에서:
  1) 공칭 자리에서 계획 (IK → 경로 → 경로 비용) — 자세 + 경로 비용 합
  2) 정지 오차 (앞뒤·좌우 ±d, 방향 ±yaw) 를 하나씩 넣고 다시 계획 — 도착 후 ZED 로 가구를 다시 재서
     계획하는 실제 흐름과 같다. 모든 오차에서 계획이 되면 '강건'.
강건한 후보 중 공칭 비용이 가장 작은 자리를 고른다 (없으면 통과한 오차 수가 많은 순).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field


from .b601 import B601Kinematics
from .collision import CollisionChecker
from .ik import B601IK
from .motion import MotionPlanner
from .scene import Placement, env_for, preset_target, room_to_robot, robot_to_room, to_robot


@dataclass
class StandResult:
    front: float
    lateral: float
    ok: bool
    total_cost: float = float("inf")
    lift_mm: float = float("nan")
    reason: str = ""
    robust: int = 0              # 통과한 정지 오차 수
    perturbations: int = 0
    failures: list[str] = field(default_factory=list)


def plan_from(kin, name, front, lateral, along, robot: Placement, start_deg, start_lift, height=0.05,
              tilt=(-10.0, 45.0), margin=0.02, n_random=12):
    """robot 자리(방 좌표, 공칭 = 원점 +x)에서 계획. 반환 Plan 또는 None."""
    tgt_room, into_room, _ = preset_target(name, height, front=front, lateral=lateral, along=along)
    target = room_to_robot(tgt_room, robot)
    into = room_to_robot(robot_to_room([0, 0, 0], robot) + into_room, robot)
    checker = CollisionChecker(kin, to_robot(env_for(name, front, lateral), robot), margin=margin)
    ik = B601IK(kin, collision=checker)
    cands = ik.solve(target, [start_lift], heading=into, tilt_deg=tilt, n_random=n_random,
                     seed=dict(zip(range(1, 7), start_deg)), lift_free=True, start_lift_stage_m=start_lift)
    plan = MotionPlanner(ik).plan(dict(zip(range(1, 7), start_deg)), target, cands, heading=into,
                                  start_lift_stage_m=start_lift, max_tries=6, select_top_k=3)
    return plan if plan is not None and plan.candidate is not None else None, plan


def choose_stand(name: str, along: float, kin: B601Kinematics | None = None,
                 fronts=(0.35, 0.40, 0.45, 0.50, 0.55), laterals=(0.0, -0.05, 0.05, -0.10, 0.10),
                 start_deg=(0.0,) * 6, start_lift=0.105, nav_err=(0.03, 0.03, 3.0), top=4, log=print):
    """반환 (최선 StandResult 또는 None, 전체 결과 목록). nav_err = (앞뒤 m, 좌우 m, 방향 도)."""
    kin = kin or B601Kinematics()
    results = []
    for fr in fronts:
        for lat in laterals:
            plan, raw = plan_from(kin, name, fr, lat, along, Placement(0, 0, 0), start_deg, start_lift)
            if plan is None:
                why = raw.tried[-1] if raw is not None and raw.tried else "IK 해 없음"
                results.append(StandResult(fr, lat, False, reason=why))
            else:
                results.append(StandResult(fr, lat, True, plan.total_cost, 2000 * plan.candidate.lift_stage_m))
            log(f"  앞면 {fr:.2f} m, 가구 가운데 y {lat:+.2f} m: "
                + (f"cost {results[-1].total_cost:.2f}, 리프트 {results[-1].lift_mm:.0f} mm" if results[-1].ok else "불가"))
    ok = sorted([r for r in results if r.ok], key=lambda r: r.total_cost)
    dx, dy, dyaw = nav_err
    perturb = [(dx, 0, 0), (-dx, 0, 0), (0, dy, 0), (0, -dy, 0), (0, 0, dyaw), (0, 0, -dyaw)]
    for r in ok[:top]:
        r.perturbations = len(perturb)
        for px, py, pyaw in perturb:
            plan, raw = plan_from(kin, name, r.front, r.lateral, along, Placement(px, py, math.radians(pyaw)),
                                  start_deg, start_lift)
            if plan is not None:
                r.robust += 1
            else:
                r.failures.append(f"({px:+.2f} m, {py:+.2f} m, {pyaw:+.0f}°)")
        log(f"  정지 오차 시험 앞면 {r.front:.2f}, y {r.lateral:+.2f}: {r.robust}/{r.perturbations} 통과"
            + (f" — 실패 {', '.join(r.failures)}" if r.failures else ""))
    tested = [r for r in ok[:top]]
    if not tested:
        return None, results
    best = sorted(tested, key=lambda r: (-(r.robust == r.perturbations), -r.robust, r.total_cost))[0]
    return best, results
