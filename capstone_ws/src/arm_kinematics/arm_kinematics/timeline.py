"""경로점 → 시간 궤적 (numpy 만 씀: 라즈베리파이 실행기에서 이 파일만 import).

경로점을 단계(phase)로 묶고, 단계마다 관절 공간 경로 길이를 사다리꼴 속도로 진행한다: a_limit 로 가속 → 일정 속도 →
a_limit 로 감속. 단계 경계에서 속도 0, 단계 안 경로점에서는 멈추지 않는다 (직선 삽입의 1 cm 걸음에서 덜컥거리지 않게).
단계 시간 = 최고 관절 속도 v_limit·가속 a_limit 로 가는 최소 시간. 경로점에 계획 시각 "t" 가 있으면 최고 속도가 계획
평균 속도를 넘지 않게 (계획기의 리프트 50 mm/s·삽입 3 cm/s 를 최고 속도 기준으로 지킴) 시간을 늘린다.
(2026-11-17 까지는 5차 최소 저크 곡선 — 최고 속도 = 평균 × 1.875 라 같은 속도 한계에서 거의 두 배 느렸다.
 팔로워에도 30°/s·120°/s² 제한기가 있어 사다리꼴 모서리는 거기서 다듬어진다.)
단계 안 경로점의 꺾임(직선 삽입 1 cm 걸음)은 속도 방향이 순간 바뀌지만 그 크기가 작아 팔로워 제한기가 흡수한다.

waypoints: [{"t": 초, "phase": str, "motors_deg": {"1": 도, ... "6": 도}, ...}]  (plan.json 과 같은 모양)
"""
from __future__ import annotations

import numpy as np

PEAK = 1.875  # (옛 5차 곡선) max ds/dτ — 호환용
ACC_PEAK = 10.0 / 3 ** 0.5  # (옛 5차 곡선) max d²s/dτ²
EPS_S = 1e-3  # 움직임 없는 단계의 길이 [s]


def _quintic(tau):
    """5차 최소 저크 곡선 (리프트 보간 등 다른 곳에서 씀)."""
    tau = min(max(tau, 0.0), 1.0)
    return tau ** 3 * (10 - 15 * tau + 6 * tau * tau)


def min_time(d: float, v: float, a: float) -> float:
    """거리 d 를 최고 속도 v·가속 a 사다리꼴로 가는 최소 시간."""
    if d <= 0:
        return 0.0
    if d >= v * v / a:
        return d / v + v / a
    return 2.0 * (d / a) ** 0.5


def time_for_peak(d: float, v_peak: float, a: float) -> float:
    """최고 속도가 정확히 v_peak 인 사다리꼴 시간 (계획 평균 속도를 최고 속도로 지킬 때)."""
    return min_time(d, v_peak, a) if d > 0 and v_peak > 0 else 0.0


def trapezoid_s(t: float, d: float, T: float, a: float) -> float:
    """시간 T 동안 거리 d 를 가속 a 사다리꼴로: 시각 t 의 진행 거리. T 가 최소 시간보다 길면 일정 속도를 낮춘다."""
    if d <= 0 or T <= 0:
        return d
    t = min(max(t, 0.0), T)
    disc = (a * T) ** 2 - 4.0 * a * d
    vc = (a * T - max(disc, 0.0) ** 0.5) / 2.0          # 일정 구간 속도 (두 근 중 작은 쪽)
    ta = vc / a
    if t < ta:
        return 0.5 * a * t * t
    if t > T - ta:
        return d - 0.5 * a * (T - t) ** 2
    return vc * (t - ta / 2.0)


def _motors(w) -> np.ndarray:
    m = w["motors_deg"]
    return np.array([float(m[str(i)] if str(i) in m else m[i]) for i in range(1, 7)])


class Timeline:
    def __init__(self, waypoints, v_limit_deg: float = 30.0, speed: float = 1.0, min_phase_s: float = 0.5,
                 phase_min_s: dict | None = None, a_limit_deg: float = 120.0):
        """phase_min_s: 단계별 최소 시간 (예: 리프트가 움직이는 'lift' 단계)."""
        if not waypoints:
            raise ValueError("no waypoints")
        if not 0 < speed <= 1.0:
            raise ValueError("speed must be in (0, 1]")
        q = [_motors(w) for w in waypoints]
        phases = [w.get("phase", "") for w in waypoints]
        times = [w.get("t") for w in waypoints]
        # 단계 = 같은 phase 이름이 이어지는 경로점들. 각 단계는 앞 단계의 마지막 점에서 시작한다.
        self.segments = []  # (phase, points[k×6], cumulative arc[k], duration)
        start = 0
        for i in range(1, len(q) + 1):
            if i == len(q) or phases[i] != phases[start]:
                pts = np.array([q[max(start - 1, 0)]] + q[start:i]) if start > 0 else np.array(q[start:i])
                step = np.max(np.abs(np.diff(pts, axis=0)), axis=1) if len(pts) > 1 else np.zeros(0)
                arc = np.concatenate([[0.0], np.cumsum(step)])
                d = float(arc[-1])
                floor = (phase_min_s or {}).get(phases[start], 0.0)
                if d > 0:
                    floor = max(floor, min_phase_s)
                if times[i - 1] is not None and times[max(start - 1, 0)] is not None:
                    tp = float(times[i - 1]) - float(times[max(start - 1, 0)])      # 계획 시간
                    if tp > 0:                    # 최고 속도 ≤ 계획 평균 속도 (움직임 없는 단계는 계획 시간 그대로 — 리프트)
                        floor = max(floor, time_for_peak(d, d / tp, a_limit_deg) if d > 0 else tp)
                dur = max(floor, min_time(d, v_limit_deg, a_limit_deg), EPS_S) / speed
                self.segments.append((phases[start], pts, arc, dur))
                start = i
        self.starts = np.concatenate([[0.0], np.cumsum([s[3] for s in self.segments])])
        self.duration = float(self.starts[-1])
        self.v_limit = v_limit_deg
        self.a_limit = a_limit_deg
        self.acc = a_limit_deg * speed * speed      # 시간을 1/speed 로 늘리면 가속은 speed² 배

    def phase_end(self, name: str) -> float:
        """name 단계가 (마지막으로) 끝나는 시각."""
        ends = [self.starts[k + 1] for k, s in enumerate(self.segments) if s[0] == name]
        if not ends:
            raise KeyError(f"no phase {name!r}; phases: {[s[0] for s in self.segments]}")
        return float(ends[-1])

    def sample(self, t: float) -> tuple[np.ndarray, str]:
        """시각 t 의 모터 1~6 목표 [도] 와 단계 이름."""
        t = min(max(t, 0.0), self.duration)
        k = min(int(np.searchsorted(self.starts, t, side="right")) - 1, len(self.segments) - 1)
        phase, pts, arc, dur = self.segments[k]
        tau = (t - self.starts[k]) / dur
        if len(pts) == 1 or arc[-1] <= 0 or tau >= 1.0:
            return pts[-1].copy(), phase
        s = trapezoid_s(t - self.starts[k], float(arc[-1]), dur, self.acc)
        j = min(int(np.searchsorted(arc, s, side="right")) - 1, len(arc) - 2)
        u = (s - arc[j]) / max(arc[j + 1] - arc[j], 1e-12)
        return pts[j] + u * (pts[j + 1] - pts[j]), phase

    def reversed(self) -> "Timeline":
        """같은 길을 거꾸로 (끝 → 시작). 복귀용."""
        rev = Timeline.__new__(Timeline)
        rev.segments = [(f"back_{p}", pts[::-1].copy(), arc[-1] - arc[::-1], dur) for p, pts, arc, dur in reversed(self.segments)]
        rev.starts = np.concatenate([[0.0], np.cumsum([s[3] for s in rev.segments])])
        rev.duration = self.duration
        rev.v_limit = self.v_limit
        rev.a_limit = self.a_limit
        rev.acc = self.acc                          # 사다리꼴은 대칭이라 거꾸로도 같은 가속
        return rev
