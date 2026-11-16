"""B601 팔 IK — 여러 시작점 DLS(감쇠 최소제곱)로 후보를 모으고 비용으로 정렬한다.

목표는 로봇 바닥 기준(world: x 앞, y 왼쪽, z 바닥에서 위) 그리퍼 끝(tool0) 위치.
방향 조건은 세 가지:
  approach=None, rotation=None  위치만 (방향 자유, 비용으로 고름)
  approach=(ax, ay, az)          그리퍼 접근축(tool0 x축)만 지정, 그 축 둘레 회전은 자유
  rotation=3×3                   자세 전체 지정
리프트는 고정값(또는 여러 값을 주면 각 높이에서 풀어 합친다). 팔 한계는 실기 한계(B601Kinematics).
"""
from __future__ import annotations

from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass, field

import numpy as np
import pinocchio as pin

from .b601 import ARM_JOINTS, BASE_FRAME, MOTORS, TOOL_FRAME, B601Kinematics, public_to_q
from .gravity import ArmGravity

DEFAULT_WEIGHTS = {
    "singular": 1.0,   # 0.02 / σmin  (σmin = 0.02 이면 1)
    "load": 1.0,       # 중력 보상 포화: 부하(최대 |τ|/보상 상한) 0.8 까지 0, 0.8→1.0 에서 0→1 (뻗는 것 자체는 벌점 없음)
    "limit": 1.0,      # 한계 여유 10° 미만부터 0→1
    "move": 0.3,       # 시작 자세에서 움직인 양 [rad 합] / π
    "lift": 0.5,       # 리프트 이동량 (총) / 630 mm — 리프트는 느리므로 팔로 되면 덜 쓰게 (lift_free 일 때)
}


@dataclass
class IKCandidate:
    motors_deg: dict[int, float]        # 모터 1~6 → 공개 각도[도]
    lift_stage_m: float
    pos_err_mm: float
    ang_err_deg: float                   # 지정한 방향 조건과의 차이 (위치만이면 0)
    converged: bool
    sigma_min: float
    load: float
    limit_margin_deg: float
    move_rad: float
    collision: bool | None = None        # 검사기를 안 주면 None
    collision_pairs: list[tuple[str, str]] = field(default_factory=list)
    cost: float = 0.0
    cost_terms: dict[str, float] = field(default_factory=dict)

    @property
    def ok(self) -> bool:
        return self.converged and self.collision is not True

    def arm(self) -> dict[int, float]:
        return dict(self.motors_deg)


LOAD_FREE = 0.8   # 팀 중력 보상기 상한(1~3번 10 N·m, 4~6번 3 N·m)의 80% 까지는 벌점 없음


def load_penalty(load: float) -> float:
    """보상 토크가 상한에서 잘리면 팔이 처져 정확도가 떨어진다. 그 근처(80~100%)만 0→1 로 벌점."""
    return float(np.clip((load - LOAD_FREE) / (1.0 - LOAD_FREE), 0.0, 1.0))


def _limit_margin_deg(q, lo, hi) -> float:
    return float(np.degrees(np.min(np.minimum(q - lo, hi - q))))


class B601IK:
    def __init__(self, kin: B601Kinematics | None = None, gravity: ArmGravity | None = None,
                 collision=None, frame: str = TOOL_FRAME, reference: str = BASE_FRAME):
        self.kin = kin or B601Kinematics()
        self.gravity = gravity or ArmGravity()
        self.collision = collision
        self.frame, self.reference = frame, reference
        self.model, self.data = self.kin.model, self.kin.model.createData()
        self._fid = self.kin._frame(frame)
        self._rid = self.kin._frame(reference)

    # ---- single solve -------------------------------------------------------------------------
    def _pose(self, q_full):
        pin.framesForwardKinematics(self.model, self.data, q_full)
        ref = self.data.oMf[self._rid]
        return ref.actInv(self.data.oMf[self._fid]), ref.rotation

    @staticmethod
    def _orient(T, Jw, orient):
        """방향 조건 → (오차 벡터, 자코비안 행, 각도 오차[도]). orient = (종류, 값)."""
        kind, val = orient
        a = T.rotation[:, 0]
        if kind == "rotation":
            e = pin.log3(val @ T.rotation.T)
            return e, Jw, float(np.degrees(np.linalg.norm(e)))
        if kind == "approach":
            e = np.cross(a, val)
            P = np.eye(3) - np.outer(a, a)          # 접근축 둘레 회전은 자유
            return e, P @ Jw, float(np.degrees(np.arccos(np.clip(a @ val, -1, 1))))
        if kind == "heading":
            h, (t_lo, t_hi) = val                    # h: 수평 단위벡터, 기울기 범위 [도] (아래 +)
            z = np.array([0.0, 0.0, 1.0])
            lat_axis = np.cross(z, h)
            lat = float(a @ lat_axis)
            pitch = float(np.degrees(np.arcsin(np.clip(-a[2], -1, 1))))
            if a @ h > 0 and t_lo <= pitch <= t_hi:
                # 옆 성분만 0 으로: d(a·l)/dt = ω·(a×l)
                return np.array([-lat]), (np.cross(a, lat_axis) @ Jw)[None, :], float(np.degrees(np.arcsin(min(1.0, abs(lat)))))
            pc = np.radians(np.clip(pitch, t_lo, t_hi) if a @ h > 0 else 0.5 * (t_lo + t_hi))
            a_des = np.cos(pc) * h - np.sin(pc) * z
            P = np.eye(3) - np.outer(a, a)
            return np.cross(a, a_des), P @ Jw, float(np.degrees(np.arccos(np.clip(a @ a_des, -1, 1))))
        return None, None, 0.0

    LIFT_COL = 0.5   # 리프트 열 가중 (작을수록 DLS 가 팔을 먼저 쓴다)

    def _solve_one(self, q0, lift, target, orient, iters, tol_mm, tol_deg, free_lift=False):
        """반환: (팔 q[rad], 리프트 단 이동량 s, 위치 오차 mm, 각도 오차 °, 수렴).
        free_lift: 리프트를 7 번째 변수로 같이 푼다 (두 단이 같이 움직여 위치 열 = 기준축 z × 2)."""
        kin = self.kin
        lo, hi = kin.arm_lower, kin.arm_upper
        q = np.clip(q0.copy(), lo, hi)
        s = float(np.clip(lift, 0.0, kin.lift_stage_max))
        q_full = kin.configuration(list(np.degrees(q)), s)
        for _ in range(iters):
            q_full[kin._arm_q] = q
            q_full[kin._lift_q] = s
            T, R_ref = self._pose(q_full)
            e_pos = target - T.translation
            pin.computeJointJacobians(self.model, self.data, q_full)
            J6 = pin.getFrameJacobian(self.model, self.data, self._fid, pin.LOCAL_WORLD_ALIGNED)[:, kin._arm_v]
            Jv, Jw = R_ref.T @ J6[:3], R_ref.T @ J6[3:]
            e_rot, J_rot, ang = self._orient(T, Jw, orient)
            e = e_pos if e_rot is None else np.concatenate((e_pos, e_rot))
            J = Jv if J_rot is None else np.vstack((Jv, J_rot))
            if free_lift:
                col = np.zeros((J.shape[0], 1))
                col[:3, 0] = R_ref.T @ np.array([0.0, 0.0, 2.0]) * self.LIFT_COL
                J = np.hstack((J, col))
            err_mm = float(np.linalg.norm(e_pos) * 1000)
            if err_mm < tol_mm and ang < tol_deg:
                return q, s, err_mm, ang, True
            lam = 0.02 + 0.2 * min(1.0, np.linalg.norm(e))
            dx = J.T @ np.linalg.solve(J @ J.T + lam ** 2 * np.eye(len(e)), e)
            q = np.clip(q + np.clip(dx[:6], -0.2, 0.2), lo, hi)
            if free_lift:
                s = float(np.clip(s + np.clip(dx[6] * self.LIFT_COL, -0.05, 0.05), 0.0, kin.lift_stage_max))
        q_full[kin._arm_q] = q
        q_full[kin._lift_q] = s
        T, R_ref = self._pose(q_full)
        err_mm = float(np.linalg.norm(target - T.translation) * 1000)
        _, _, ang = self._orient(T, np.zeros((3, 6)), orient)
        return q, s, err_mm, ang, err_mm < tol_mm and ang < tol_deg

    # ---- candidates ---------------------------------------------------------------------------
    def solve(self, target_xyz: Sequence[float], lift_stage_m: float | Iterable[float] = 0.0,
              approach: Sequence[float] | None = None, rotation=None,
              heading: Sequence[float] | None = None, tilt_deg: tuple[float, float] = (-10.0, 45.0),
              seed: Mapping | Sequence[float] | None = None, n_random: int = 24, iters: int = 150,
              tol_mm: float = 1.0, tol_deg: float = 1.0, weights: Mapping[str, float] | None = None,
              dedupe_deg: float = 3.0, rng: np.random.Generator | None = None,
              lift_free: bool = False, start_lift_stage_m: float | None = None) -> list[IKCandidate]:
        """후보 목록을 비용 순으로 돌려준다 (수렴·충돌 없음 → 먼저).

        lift_stage_m: 리프트 단 이동량 (하나 또는 여러 개). lift_free=False 면 그 높이들에 고정해서 푼다.
        lift_free:    리프트도 같이 고른다. 0 ~ 최대를 13 단계(총 52.5 mm 간격) + 주어진 높이로 고정해 풀고,
                      각 높이에서 리프트를 7 번째 변수로 한 번 더 풀어 격자 사이 높이도 찾는다.
                      (연속 변수만으로는 DLS 가 리프트를 범위 끝으로 미는 경우가 많아 해를 놓침)
        start_lift_stage_m: 지금 리프트 높이 — lift 비용(이동량) 기준. None 이면 0 비용.

        heading: 수평 방향 (예: 선반 안쪽). 그리퍼 접근축이 이 방향의 수직면 안에서 앞을 보고,
                 수평에서 tilt_deg (아래 +, 위 −) 범위로만 기울어진다.
        """
        target = np.asarray(target_xyz, dtype=float)
        if sum(v is not None for v in (approach, rotation, heading)) > 1:
            raise ValueError("give only one of approach, rotation, heading")
        orient = ("none", None)
        if approach is not None:
            approach = np.asarray(approach, dtype=float)
            orient = ("approach", approach / np.linalg.norm(approach))
        if rotation is not None:
            orient = ("rotation", np.asarray(rotation, dtype=float))
        if heading is not None:
            h = np.asarray(heading, dtype=float).copy()
            h[2] = 0.0
            if np.linalg.norm(h) < 1e-9:
                raise ValueError("heading must have a horizontal component")
            orient = ("heading", (h / np.linalg.norm(h), (float(tilt_deg[0]), float(tilt_deg[1]))))
        w = {**DEFAULT_WEIGHTS, **(weights or {})}
        rng = rng or np.random.default_rng(0)
        lo, hi = self.kin.arm_lower, self.kin.arm_upper
        q_seed = public_to_q(seed) if seed is not None else None
        lifts = [float(lift_stage_m)] if np.isscalar(lift_stage_m) else [float(s) for s in lift_stage_m]

        starts = []
        if q_seed is not None:
            starts.append(q_seed)
        starts += [rng.uniform(lo, hi) for _ in range(n_random)]
        out: list[IKCandidate] = []
        if lift_free:
            grid = np.linspace(0.0, self.kin.lift_stage_max, 13)
            lifts = sorted({round(float(v), 5) for v in list(lifts) + list(grid)})
            per_level = max(8, n_random // 2)
            starts = starts[:1 + per_level] if q_seed is not None else starts[:per_level]
        jobs = [(lift, q0, False) for lift in lifts for q0 in starts]
        if lift_free:
            jobs += [(lift, starts[0], True) for lift in lifts]          # 격자 사이 높이
        found: list[tuple[np.ndarray, float]] = []
        last_lift = None
        for lift, q0, free in jobs:
            if not lift_free and lift != last_lift:
                found = []                       # 고정 높이마다 따로 중복 제거
            last_lift = lift
            q, s, err_mm, ang, conv = self._solve_one(q0, lift, target, orient, iters, tol_mm, tol_deg,
                                                      free_lift=free)
            if not conv and err_mm > 20:
                continue
            # 같은 해: 관절 3° 안 + 리프트 총 높이 3 cm 안
            if any(np.max(np.abs(np.degrees(q - f))) < dedupe_deg and abs(2 * (s - fs)) < 0.03 for f, fs in found):
                continue
            found.append((q, s))
            out.append(self._score(q, s, err_mm, ang, conv, q_seed, w, start_lift_stage_m))
        out.sort(key=lambda c: (not c.ok, c.cost))
        return out

    def _score(self, q, lift, err_mm, ang, conv, q_seed, w, start_lift=None) -> IKCandidate:
        deg = list(np.degrees(q))
        mm = self.kin.manipulability(deg, lift, self.frame)
        load = self.gravity.load(deg)
        margin = _limit_margin_deg(q, self.kin.arm_lower, self.kin.arm_upper)
        move = float(np.sum(np.abs(q - q_seed))) if q_seed is not None else 0.0
        terms = {
            "singular": w["singular"] * 0.02 / max(mm["sigma_min"], 1e-4),
            "load": w["load"] * load_penalty(load),
            "limit": w["limit"] * float(np.clip((10.0 - margin) / 10.0, 0.0, 1.0)),
            "move": w["move"] * move / np.pi,
        }
        if start_lift is not None:
            terms["lift"] = w["lift"] * abs(2 * (lift - start_lift)) / (2 * self.kin.lift_stage_max)
        cand = IKCandidate(
            motors_deg={n: round(float(d), 2) for n, d in zip(MOTORS, deg)}, lift_stage_m=lift,
            pos_err_mm=round(err_mm, 3), ang_err_deg=round(ang, 3), converged=conv,
            sigma_min=round(mm["sigma_min"], 5), load=round(load, 3), limit_margin_deg=round(margin, 2),
            move_rad=round(move, 3), cost=round(sum(terms.values()), 4),
            cost_terms={k: round(v, 4) for k, v in terms.items()})
        if self.collision is not None:
            pairs = self.collision.colliding_pairs(deg, lift)
            cand.collision, cand.collision_pairs = bool(pairs), pairs
        return cand


def describe(c: IKCandidate) -> str:
    m = ", ".join(f"{n}:{c.motors_deg[n]:.1f}" for n in sorted(c.motors_deg))
    col = {None: "-", True: "충돌", False: "OK"}[c.collision]
    terms = " + ".join(f"{k} {v:.2f}" for k, v in c.cost_terms.items())
    return (f"[{'OK' if c.ok else '--'}] cost {c.cost:.3f} = {terms}\n"
            f"      리프트 {2000 * c.lift_stage_m:.0f}mm  오차 {c.pos_err_mm:.2f}mm/{c.ang_err_deg:.1f}°  "
            f"σmin {c.sigma_min:.4f}  부하 {c.load:.2f}  한계여유 {c.limit_margin_deg:.1f}°  충돌 {col}\n      모터 {m}")


__all__ = ["B601IK", "IKCandidate", "DEFAULT_WEIGHTS", "describe", "ARM_JOINTS"]
