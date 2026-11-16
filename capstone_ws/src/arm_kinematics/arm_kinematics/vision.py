"""카메라 한 장 (컬러 + 깊이) 의 상자·마스크 → 로봇 바닥 기준 물체 3D. numpy 만 씀 (ROS·GPU 불필요).

좌표: 로봇 바닥 기준 (x 앞, y 왼쪽, z 위) = arm_kinematics 의 'world' = navigation 의 base_footprint.
카메라: 광학 프레임 (x 오른쪽, y 아래, z 앞), K = (fx, fy, cx, cy), T = 로봇 ← 카메라 광학 4×4.

물체 점 고르기: 마스크(SAM) 가 있으면 그 픽셀, 없으면 상자 안 픽셀. 거기서 받침면(책상) 높이를 상자 둘레에서 구하고
그보다 1 cm 넘게 높은 점만 남긴다 (상자 안 책상·뒤 배경 제거). 카메라는 물체의 앞·윗면만 보므로 보이는 점의 중앙값은
카메라 쪽으로 치우친다 → 중심은 수평으로는 주축 방향 범위의 가운데, 높이는 받침면과 윗면의 가운데로 잡는다.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class ObjectEstimate:
    center: np.ndarray           # 로봇 바닥 기준 [m]
    top_z: float                 # 윗면 높이 [m]
    support_z: float | None      # 받침면 높이 [m] (못 구하면 None)
    size: tuple                  # (긴 축, 짧은 축, 높이) [m] — 보이는 부분 기준이라 실제보다 작을 수 있음
    yaw: float                   # 긴 축 방향 [rad] (로봇 x 축에서, −π/2~π/2)
    n_points: int

    def __str__(self):
        c = self.center
        return (f"중심 ({100 * c[0]:.1f}, {100 * c[1]:.1f}, {100 * c[2]:.1f}) cm, 크기 {100 * self.size[0]:.1f}×"
                f"{100 * self.size[1]:.1f}×{100 * self.size[2]:.1f} cm, 긴 축 {np.degrees(self.yaw):+.0f}°, 점 {self.n_points}")


def pixels_to_points(depth, K, T, us, vs):
    """픽셀들 → 로봇 기준 점 N×3 (깊이가 없거나 이상한 픽셀은 뺌)."""
    fx, fy, cx, cy = K
    us, vs = np.asarray(us), np.asarray(vs)
    z = depth[vs, us].astype(float)
    ok = np.isfinite(z) & (z > 0.05) & (z < 10.0)
    us, vs, z = us[ok], vs[ok], z[ok]
    p = np.stack([(us - cx) / fx * z, (vs - cy) / fy * z, z, np.ones_like(z)], axis=1)
    return (p @ np.asarray(T).T)[:, :3]


def box_pixels(box, shape, step=1, shrink=0.0):
    """상자 (x0, y0, x1, y1) 안 픽셀 좌표. shrink = 가장자리를 비율만큼 깎음."""
    h, w = shape[:2]
    x0, y0, x1, y1 = box[:4]
    dx, dy = (x1 - x0) * shrink, (y1 - y0) * shrink
    xs = np.arange(int(max(0, x0 + dx)), int(min(w - 1, x1 - dx)) + 1, step)
    ys = np.arange(int(max(0, y0 + dy)), int(min(h - 1, y1 - dy)) + 1, step)
    vs, us = np.meshgrid(ys, xs, indexing="ij")
    return us.ravel(), vs.ravel()


def support_height(depth, K, T, box, ring=0.35, step=2, below=None):
    """상자 둘레 띠 (상자 크기의 ring 배만큼 바깥) 의 점들로 받침면 높이 [m]: 수평면이 가장 많은 높이 (1 cm 칸 최빈값).
    below 를 주면 그보다 낮은 점만 (물체 윗면보다 낮은 곳). 못 구하면 None."""
    h, w = depth.shape[:2]
    x0, y0, x1, y1 = box[:4]
    mx, my = (x1 - x0) * ring, (y1 - y0) * ring
    big = (max(0, x0 - mx), max(0, y0 - my), min(w - 1, x1 + mx), min(h - 1, y1 + my))
    us, vs = box_pixels(big, depth.shape, step)
    inside = (us >= x0) & (us <= x1) & (vs >= y0) & (vs <= y1)
    p = pixels_to_points(depth, K, T, us[~inside], vs[~inside])
    if below is not None:
        p = p[p[:, 2] < below]
    if len(p) < 30:
        return None
    # 가장 많은 높이가 아니라, 점이 충분히 모인 높이 중 가장 높은 면 (물체 바로 아래 책상).
    # 책상 끝 근처 물체는 둘레에 책상 너머 바닥이 더 많이 보여 최빈값이 바닥이 됨 (시뮬 확인 2026-11-17)
    z = p[:, 2]
    bins = np.floor(z / 0.01).astype(int)
    vals, counts = np.unique(bins, return_counts=True)
    dense = vals[counts >= max(15, 0.15 * counts.max())]
    k = dense.max()
    sel = z[(bins >= k - 1) & (bins <= k + 1)]
    return float(np.median(sel)) if len(sel) >= 15 else None


def estimate_object(depth, K, T, box, mask=None, min_above=0.01, step=1) -> ObjectEstimate | None:
    """상자(+마스크) → 물체 3D. 점이 너무 적으면 None."""
    if mask is not None:
        vs, us = np.nonzero(mask)
        if step > 1:
            us, vs = us[::step], vs[::step]
    else:
        us, vs = box_pixels(box, depth.shape, step, shrink=0.05)
    p = pixels_to_points(depth, K, T, us, vs)
    if len(p) < 20:
        return None
    # 상자 안에서 가장 가까운 덩어리만 (뒤 배경 제거): 카메라까지 거리의 아래쪽 무리
    cam = np.asarray(T)[:3, 3]
    r = np.linalg.norm(p - cam, axis=1)
    r0 = np.percentile(r, 10)
    p = p[r < r0 + 0.25]
    top = float(np.percentile(p[:, 2], 95))
    sup = support_height(depth, K, T, box, below=top - 0.005)
    if sup is not None:
        p = p[p[:, 2] > sup + min_above]
    if len(p) < 20:
        return None
    xy = p[:, :2]
    mu = xy.mean(axis=0)
    cov = np.cov((xy - mu).T)
    evals, evecs = np.linalg.eigh(cov)
    a = evecs[:, 1]                                       # 긴 축
    if a[0] < 0:
        a = -a
    b = np.array([-a[1], a[0]])
    pa, pb = (xy - mu) @ a, (xy - mu) @ b
    lo_a, hi_a = np.percentile(pa, [2, 98])
    lo_b, hi_b = np.percentile(pb, [2, 98])
    c_xy = mu + a * (lo_a + hi_a) / 2 + b * (lo_b + hi_b) / 2
    top = float(np.percentile(p[:, 2], 95))
    base = sup if sup is not None else float(np.percentile(p[:, 2], 5))
    yaw = float(np.arctan2(a[1], a[0]))
    yaw = (yaw + np.pi / 2) % np.pi - np.pi / 2
    return ObjectEstimate(np.array([c_xy[0], c_xy[1], (base + top) / 2]), top, sup,
                          (float(hi_a - lo_a), float(hi_b - lo_b), float(top - base)), yaw, len(p))
