"""IK 후보 → 실행 가능한 관절 경로.

순서: (1) 리프트를 후보 높이로 (팔은 시작 자세) → (2) 관절 보간으로 접근점(목표에서 접근축 반대로 d 만큼)
      → (3) 접근점 → 목표를 직선(작업 공간)으로, 작은 걸음마다 IK (이전 해를 시작점) → 그리퍼 닫기 자리.
비용 낮은 후보부터 시도하고, 경로 전체가 한계 안·충돌 없음·관절 튐 없음이면 그 후보를 고른다.
속도: 관절 최대 각속도로 시간을 매긴다 (기본 30°/s, 리프트 50 mm/s).
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .b601 import ARM_JOINTS, MOTORS, public_to_q
from .ik import B601IK, IKCandidate, load_penalty
from .timeline import Timeline

TRAJ_WEIGHTS = {
    "time": 1.0,            # 재생 시간 / 20 s
    "load_path": 1.0,       # 경로 중 최대 중력 부하의 포화 벌점 (0.8 까지 0)
    "singular_path": 1.0,   # 0.02 / 정밀 구간(접근점 → 목표) 최소 σmin
    "clearance": 1.0,       # 이동 구간 팔↔환경 여유 5 cm 미만부터 0→1 (상자 margin 위에서)
}


@dataclass
class Waypoint:
    t: float                      # 시작부터 시간 [s]
    motors_deg: dict[int, float]  # 모터 1~6 [도]
    lift_stage_m: float
    phase: str                    # lift | approach | insert


@dataclass
class Plan:
    candidate: IKCandidate
    pregrasp: dict[int, float]
    waypoints: list[Waypoint]
    tried: list[str] = field(default_factory=list)   # 앞 후보들이 떨어진 이유

    @property
    def duration(self) -> float:
        return self.waypoints[-1].t if self.waypoints else 0.0


def _deg_dict(q):
    return {n: float(v) for n, v in zip(MOTORS, np.degrees(q))}


class MotionPlanner:
    def __init__(self, ik: B601IK, limits_deg: dict[int, tuple[float, float]] | None = None,
                 max_joint_speed_deg: float = 30.0, lift_speed_m: float = 0.05, check_step_deg: float = 2.0):
        self.ik, self.kin, self.checker = ik, ik.kin, ik.collision
        lim = limits_deg or self.kin.motor_limits_deg()
        self.lo = np.radians([lim[n][0] for n in MOTORS])
        self.hi = np.radians([lim[n][1] for n in MOTORS])
        self.vmax = np.radians(max_joint_speed_deg)
        self.lift_speed = lift_speed_m
        self.step = np.radians(check_step_deg)

    via_back = (0.20, 0.30)   # 경유점: 목표에서 접근축 반대로 [m]

    def _near_solution(self, xyz, lift, kw, q_ref, max_dev_deg=45.0):
        """xyz 에 대한 IK 해 중 q_ref 에 가장 가까운 것 (방향 조건 kw). 시작점 하나 → 실패 시 여럿."""
        for n_random in (0, 24):
            sols = [s for s in self.ik.solve(xyz, lift, seed=_deg_dict(q_ref), n_random=n_random, **kw) if s.converged]
            best, dev = None, None
            for s in sols:
                q = public_to_q(s.motors_deg)
                d = float(np.max(np.abs(q - q_ref)))
                if d <= np.radians(max_dev_deg) and not self._ok(q, lift) and (dev is None or d < dev):
                    best, dev = q, d
            if best is not None:
                return best
        return None

    def _ok(self, q, lift) -> str | None:
        if np.any(q < self.lo - 1e-6) or np.any(q > self.hi + 1e-6):
            return "관절 한계(실행 범위) 밖"
        if self.checker is not None and self.checker.in_collision(list(np.degrees(q)), lift):
            pairs = self.checker.colliding_pairs(list(np.degrees(q)), lift)
            return f"충돌 {pairs[:2]}"
        return None

    def _segment(self, qa, qb, lift):
        n = max(1, int(np.ceil(np.max(np.abs(qb - qa)) / self.step)))
        for k in range(1, n + 1):
            q = qa + (qb - qa) * k / n
            why = self._ok(q, lift)
            if why:
                return f"이동 중 {why}"
        return None

    def trajectory_cost(self, plan: "Plan", n: int = 30) -> tuple[float, dict[str, float], dict[str, float]]:
        """경로 비용 (합, 항목, 측정값). 자세 비용(후보 cost)과 더해 후보를 고른다.

        구간을 나눠 본다:
          load_path     리프트 뒤 경로 전체의 최대 중력 부하 (보상 포화 벌점, 0.8 까지 0)
          clearance     이동 구간(시작 → 접근점)의 팔↔환경 최소 여유 — 기둥·판에 붙어 지나가는 것.
                        삽입 구간은 목표가 판 바로 위라 원래 가까우므로 보지 않는다.
          singular_path 정밀 구간(접근점 → 목표)의 최소 σmin. 출발(접힌 자세)은 모든 경로가 같아 빼다.
          time          재생 시간 / 20 s
        """
        wps = [{"t": w.t, "phase": w.phase, "motors_deg": {str(k): v for k, v in w.motors_deg.items()}}
               for w in plan.waypoints]
        tl = Timeline(wps, v_limit_deg=float(np.degrees(self.vmax)))
        lift = plan.waypoints[-1].lift_stage_m
        t0, t_app, t_end = tl.phase_end("lift"), tl.phase_end("approach"), tl.duration
        transit = [list(tl.sample(float(t))[0]) for t in np.linspace(t0, t_app, n)]
        precise = [list(tl.sample(float(t))[0]) for t in np.linspace(t_app, t_end, max(n // 3, 5))]
        loads = [self.ik.gravity.load(q) for q in transit + precise]
        sigmas = [self.kin.manipulability(q, lift)["sigma_min"] for q in precise]
        clears = [self.checker.clearance(q, lift) for q in transit] if self.checker is not None else []
        clr = min(clears) if clears else float("inf")
        meas = {"duration_s": round(t_end, 2), "max_load": round(max(loads), 3),
                "min_sigma_insert": round(min(sigmas), 5),
                "min_clearance_transit_m": None if clr == float("inf") else clr}
        terms = {
            "time": TRAJ_WEIGHTS["time"] * t_end / 20.0,
            "load_path": TRAJ_WEIGHTS["load_path"] * load_penalty(max(loads)),
            "singular_path": TRAJ_WEIGHTS["singular_path"] * 0.02 / max(min(sigmas), 1e-4),
            "clearance": 0.0 if clr == float("inf") else TRAJ_WEIGHTS["clearance"] * float(np.clip((0.05 - clr) / 0.05, 0, 1)),
        }
        terms = {k: round(v, 4) for k, v in terms.items()}
        return round(sum(terms.values()), 4), terms, meas

    def plan(self, start_deg, target, candidates: list[IKCandidate], approach=None, heading=None,
             start_lift_stage_m: float = 0.0,
             standoffs: tuple[float, ...] = (0.15, 0.12, 0.10, 0.07, 0.05),
             insert_step: float = 0.01, max_tries: int = 10, start_clip_deg: float = 2.0,
             select_top_k: int = 4) -> Plan | None:
        """standoffs: 접근점 거리 후보 [m] — 앞에서부터 시도 (가구 앞이 몸에 가까우면 짧게).

        heading 을 주면 (IK 도 heading 으로 푼 후보): 접근점은 목표에서 heading 반대로 수평으로 물러난 곳,
        삽입은 heading 방향 수평 직선, 그 동안 그리퍼 자세(후보의 회전)를 그대로 유지한다.
        """
        q0 = public_to_q(start_deg)
        q0c = np.clip(q0, self.lo, self.hi)        # 쉬는 자세가 한계를 살짝 넘을 수 있음 (예: 2번 +0.4°)
        if np.max(np.abs(q0c - q0)) > np.radians(start_clip_deg):
            return Plan(None, {}, [], [f"시작 자세가 실행 한계에서 {np.degrees(np.max(np.abs(q0c - q0))):.1f}° 밖"])
        q0 = q0c
        approach = None if approach is None else np.asarray(approach, float) / np.linalg.norm(approach)
        if heading is not None:
            heading = np.array(heading, float)
            heading[2] = 0.0
            heading /= np.linalg.norm(heading)
        tried, feasible = [], []
        for c in [c for c in candidates if c.ok][:max_tries]:
            if len(feasible) >= select_top_k:
                break
            lift = c.lift_stage_m
            q_goal = public_to_q(c.motors_deg)
            label = f"cost {c.cost:.2f} 리프트 {2000 * lift:.0f}mm"
            why = self._ok(q_goal, lift)
            if why:
                tried.append(f"{label}: 목표 자세 {why}")
                continue
            # 물러나고 들어가는 방향: heading(수평) > 지정 접근축 > 후보의 그리퍼 접근축
            R_c = self.kin.fk(c.motors_deg, lift).rotation
            a = heading if heading is not None else approach if approach is not None else R_c[:, 0]
            kw = dict(approach=approach) if (approach is not None and heading is None) else dict(rotation=R_c)
            q_pre = None
            for standoff in standoffs:
                pre_xyz = np.asarray(target, float) - standoff * a
                q_pre = self._near_solution(pre_xyz, lift, kw, q_goal)
                if q_pre is not None:
                    break
            if q_pre is None:
                tried.append(f"{label}: 접근점 IK 실패 ({', '.join(f'{100 * d:.0f}' for d in standoffs)} cm 모두, 목표 자세 45° 안에 충돌 없는 해 없음)")
                continue
            # 직선 삽입: 접근점 → 목표, 이전 해를 시작점으로
            path, q, ok = [q_pre], q_pre, True
            n = max(1, int(np.ceil(standoff / insert_step)))
            for k in range(1, n + 1):
                p = pre_xyz + (np.asarray(target) - pre_xyz) * k / n
                sol = self.ik.solve(p, lift, seed=_deg_dict(q), n_random=0, **kw)
                if not sol or not sol[0].converged:
                    tried.append(f"{label}: 직선 삽입 {k}/{n} IK 실패"); ok = False; break
                qn = public_to_q(sol[0].motors_deg)
                if np.max(np.abs(qn - q)) > np.radians(10):
                    tried.append(f"{label}: 직선 삽입 중 관절 튐 {np.degrees(np.max(np.abs(qn - q))):.0f}°"); ok = False; break
                why = self._ok(qn, lift)
                if why:
                    tried.append(f"{label}: 직선 삽입 {k}/{n} {why}"); ok = False; break
                path.append(qn); q = qn
            if not ok:
                continue
            # 리프트 이동 (시작 자세 유지) + 시작 → 접근점. 직접 가면 부딪히면 더 뒤(바깥) 경유점을 거친다.
            vias = []
            why = None
            for sl in np.linspace(start_lift_stage_m, lift, 6):   # 리프트가 움직이는 동안 (팔은 시작 자세)
                why = why or self._ok(q0, float(sl))
            why = why or self._segment(q0, q_pre, lift)
            if why:
                fixed = None
                for back in self.via_back:
                    q_via = self._near_solution(np.asarray(target, float) - back * a, lift, kw, q_pre)
                    if q_via is None:
                        continue
                    if not (self._ok(q_via, lift) or self._segment(q0, q_via, lift) or self._segment(q_via, q_pre, lift)):
                        fixed = q_via
                        break
                if fixed is None:
                    tried.append(f"{label}: 시작→접근점 {why} (경유점으로도 안 됨)")
                    continue
                vias = [fixed]
            wps, t, s0 = [], 0.0, start_lift_stage_m
            wps.append(Waypoint(0.0, _deg_dict(q0), s0, "lift"))
            t += abs(2 * lift - 2 * s0) / self.lift_speed + 0.5
            wps.append(Waypoint(t, _deg_dict(q0), lift, "lift"))
            prev = q0
            for qv in vias + [q_pre]:
                t += float(np.max(np.abs(qv - prev)) / self.vmax) + 0.5
                wps.append(Waypoint(t, _deg_dict(qv), lift, "approach"))
                prev = qv
            prev = q_pre
            for qn in path[1:]:
                t += max(float(np.max(np.abs(qn - prev)) / self.vmax), insert_step / 0.03)   # ≤ 3 cm/s
                wps.append(Waypoint(t, _deg_dict(qn), lift, "insert"))
                prev = qn
            plan = Plan(c, _deg_dict(q_pre), wps, tried)
            plan.standoff = standoff
            feasible.append(plan)
        if not feasible:
            return Plan(None, {}, [], tried) if tried else None
        # 충돌 없는 경로 (최대 select_top_k 개) 중 자세 비용 + 경로 비용 이 가장 작은 것
        scored = []
        for p in feasible:
            tc, terms, meas = self.trajectory_cost(p)
            p.trajectory_cost, p.trajectory_terms, p.trajectory_meas = tc, terms, meas
            p.total_cost = round(p.candidate.cost + tc, 4)
            scored.append(p)
        scored.sort(key=lambda p: p.total_cost)
        best = scored[0]
        best.tried = tried
        best.alternatives = [(p.total_cost, p.candidate.cost, p.trajectory_cost, p.candidate.lift_stage_m) for p in scored]
        return best


def sample(plan: Plan, t: float) -> tuple[np.ndarray, float, str]:
    """시간 t 의 관절[rad, 1~6번]·리프트 단 이동량. 구간 선형 보간."""
    w = plan.waypoints
    if t <= w[0].t:
        return public_to_q(w[0].motors_deg), w[0].lift_stage_m, w[0].phase
    for a, b in zip(w, w[1:]):
        if t <= b.t:
            u = (t - a.t) / max(b.t - a.t, 1e-9)
            qa, qb = public_to_q(a.motors_deg), public_to_q(b.motors_deg)
            return qa + u * (qb - qa), a.lift_stage_m + u * (b.lift_stage_m - a.lift_stage_m), b.phase
    return public_to_q(w[-1].motors_deg), w[-1].lift_stage_m, w[-1].phase


__all__ = ["MotionPlanner", "Plan", "Waypoint", "sample", "ARM_JOINTS"]
