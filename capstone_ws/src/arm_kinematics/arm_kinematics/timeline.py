"""경로점 → 부드러운 시간 궤적 (numpy 만 씀: 라즈베리파이 실행기에서 이 파일만 import).

경로점을 단계(phase)로 묶고, 단계마다 관절 공간 경로 길이를 5차 최소 저크 곡선
s(τ) = 10τ³ − 15τ⁴ + 6τ⁵ 로 진행한다. 단계 경계에서 속도·가속도 0, 단계 안 경로점에서는 멈추지 않는다
(직선 삽입의 1 cm 걸음에서 덜컥거리지 않게). 단계 시간은 최고 관절 속도가 v_limit 을 넘지 않게 정한다
(5차 곡선 최고 속도 = 평균 × 1.875). 경로점에 계획 시각 "t" 가 있으면 단계마다 계획 시간 × 1.875 보다 짧게 하지 않는다
(계획기가 정한 리프트 50 mm/s·삽입 3 cm/s 같은 제한을 최고 속도 기준으로 지킨다).

waypoints: [{"t": 초, "phase": str, "motors_deg": {"1": 도, ... "6": 도}, ...}]  (plan.json 과 같은 모양)
"""
from __future__ import annotations

import numpy as np

PEAK = 1.875  # max ds/dτ of the quintic


def _quintic(tau):
    tau = min(max(tau, 0.0), 1.0)
    return tau ** 3 * (10 - 15 * tau + 6 * tau * tau)


def _motors(w) -> np.ndarray:
    m = w["motors_deg"]
    return np.array([float(m[str(i)] if str(i) in m else m[i]) for i in range(1, 7)])


class Timeline:
    def __init__(self, waypoints, v_limit_deg: float = 30.0, speed: float = 1.0, min_phase_s: float = 0.5,
                 phase_min_s: dict | None = None):
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
                floor = max(min_phase_s, (phase_min_s or {}).get(phases[start], 0.0))
                if times[i - 1] is not None and times[max(start - 1, 0)] is not None:
                    floor = max(floor, PEAK * (float(times[i - 1]) - float(times[max(start - 1, 0)])))
                dur = max(floor, PEAK * float(arc[-1]) / v_limit_deg) / speed
                self.segments.append((phases[start], pts, arc, dur))
                start = i
        self.starts = np.concatenate([[0.0], np.cumsum([s[3] for s in self.segments])])
        self.duration = float(self.starts[-1])
        self.v_limit = v_limit_deg

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
        s = _quintic(tau) * arc[-1]
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
        return rev
