import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from arm_kinematics.vision import estimate_object, support_height  # noqa: E402

W, H = 640, 360
DESK_TOP, DESK_X = 0.88, (0.35, 0.85)          # 책상 윗면 높이, 앞·뒤 모서리 x (그 뒤는 바닥)


def camera(height=1.23, pitch_deg=21.6):
    p = math.radians(pitch_deg)
    f = np.array([math.cos(p), 0.0, -math.sin(p)])
    x = np.array([0.0, -1.0, 0.0])
    T = np.eye(4)
    T[:3, :3] = np.column_stack([x, np.cross(f, x), f])
    T[:3, 3] = [0.167, 0.06, height]
    fy = (H / 2) / math.tan(math.radians(69) / 2)
    return (fy, fy, W / 2, H / 2), T


def render_depth(K, T, box_c, box_size, yaw, extra=()):
    """책상(윗면 DESK_TOP, x 범위 DESK_X) + 그 뒤 바닥 + yaw 돌린 상자 하나 (+ extra 상자들 [(c, size, yaw)]) 의 깊이 (광축 방향 거리)."""
    fx, fy, cx, cy = K
    vs, us = np.mgrid[0:H, 0:W]
    d_cam = np.stack([(us - cx) / fx, (vs - cy) / fy, np.ones_like(us, float)], axis=-1)   # 광축 거리 1 당 방향
    R, o = T[:3, :3], T[:3, 3]
    d = d_cam @ R.T                                                                       # 로봇 기준
    best = np.full((H, W), np.inf)
    # 책상 윗면 / 바닥
    for z0, xr in ((DESK_TOP, DESK_X), (0.0, (DESK_X[1], 9.0))):
        t = (z0 - o[2]) / d[..., 2]
        px = o[0] + t * d[..., 0]
        ok = (t > 0) & (px >= xr[0]) & (px <= xr[1])
        best = np.where(ok & (t < best), t, best)
    # 상자들 (상자 좌표로 돌려 slab 교차)
    for bc, bs, by in [(np.asarray(box_c), box_size, yaw), *[(np.asarray(c_), s_, y_) for c_, s_, y_ in extra]]:
        c, s = math.cos(-by), math.sin(-by)
        Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        ol = Rz @ (o - bc)
        dl = d @ Rz.T
        half = np.array(bs) / 2
        with np.errstate(divide="ignore", invalid="ignore"):
            t1 = (-half - ol) / dl
            t2 = (half - ol) / dl
        tn = np.nanmax(np.minimum(t1, t2), axis=-1)
        tf = np.nanmin(np.maximum(t1, t2), axis=-1)
        hit = (tn <= tf) & (tn > 0)
        best = np.where(hit & (tn < best), tn, best)
    best[~np.isfinite(best)] = 0.0
    return best                                                                           # t = 광축 거리 (d_cam z = 1)


def box_of(K, T, c, size, yaw, pad=4):
    fx, fy, cx, cy = K
    half = np.array(size) / 2
    cz, sz = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    pts = np.array([c + Rz @ (half * s) for s in np.array(np.meshgrid([-1, 1], [-1, 1], [-1, 1])).T.reshape(-1, 3)])
    q = (np.linalg.inv(T) @ np.c_[pts, np.ones(8)].T).T
    u, v = fx * q[:, 0] / q[:, 2] + cx, fy * q[:, 1] / q[:, 2] + cy
    return [u.min() - pad, v.min() - pad, u.max() + pad, v.max() + pad, 1.0, "x"]


class VisionTests(unittest.TestCase):
    def check(self, c_xy, size, yaw, tol_xy=0.015, tol_z=0.01):
        K, T = camera()
        c = np.array([*c_xy, DESK_TOP + size[2] / 2])
        depth = render_depth(K, T, c, size, yaw)
        est = estimate_object(depth, K, T, box_of(K, T, c, size, yaw))
        self.assertIsNotNone(est)
        self.assertLess(np.linalg.norm(est.center[:2] - c[:2]), tol_xy, f"{est}")
        self.assertLess(abs(est.center[2] - c[2]), tol_z, f"{est}")
        self.assertAlmostEqual(est.support_z, DESK_TOP, delta=0.006)
        return est

    def test_small_object_in_middle_of_desk(self):
        self.check((0.50, -0.20), (0.06, 0.06, 0.06), 0.0)

    def test_long_object_axis_and_size(self):
        est = self.check((0.60, 0.15), (0.18, 0.04, 0.035), math.radians(60))
        self.assertAlmostEqual(est.yaw, math.radians(60), delta=math.radians(8))
        self.assertGreater(est.size[0], 0.14)

    def test_object_near_back_edge_uses_desk_not_floor(self):
        # 책상 뒤 끝 근처: 상자 둘레에 바닥이 더 많이 보여도 받침면은 책상 (시뮬에서 40 cm 틀렸던 경우)
        self.check((0.76, -0.40), (0.075, 0.075, 0.08), 0.0)

    def test_floor_height_floor_only_and_behind_desk(self):
        global DESK_X
        from arm_kinematics.vision import camera_pose, floor_height
        T_zero = camera_pose(0, math.radians(21.6), [0.167, 0.06, 0.0])
        saved = DESK_X
        try:
            for desk, h in (((-1.0, -0.9), 0.76), ((-1.0, -0.9), 1.23), ((0.35, 0.85), 1.23)):
                DESK_X = desk                     # (-1, -0.9) = 책상이 카메라 뒤 → 앞은 바닥만
                globals()["DESK_X"] = desk
                K, T = camera(height=h)
                d = render_depth(K, T, np.array([0.5, 0.0, DESK_TOP + 0.03]), (0.06, 0.06, 0.06), 0.0)
                self.assertAlmostEqual(floor_height(d, K, T_zero), h, delta=0.01, msg=f"책상 {desk}, 높이 {h}")
        finally:
            globals()["DESK_X"] = saved

    def test_floor_height_ignores_glossy_reflections(self):
        # 반짝이는 바닥: 바닥 픽셀 20 % 의 깊이를 5~60 % 멀게 (바닥 아래로 흩어진 반사 점) → 그래도 바닥 높이
        from arm_kinematics.vision import camera_pose, floor_height
        saved = globals()["DESK_X"]
        try:
            globals()["DESK_X"] = (-1.0, -0.9)
            K, T = camera(height=0.93)
            d = render_depth(K, T, np.array([5.0, 5.0, 0.03]), (0.01, 0.01, 0.01), 0.0)
            rng = np.random.default_rng(0)
            pick = (d > 0) & (rng.random(d.shape) < 0.2)
            d[pick] *= rng.uniform(1.05, 1.6, pick.sum())
            h = floor_height(d, K, camera_pose(0, math.radians(21.6), [0.167, 0.06, 0.0]))
            self.assertAlmostEqual(h, 0.93, delta=0.01)
        finally:
            globals()["DESK_X"] = saved

    def test_cluttered_neighbor_not_taken_as_support(self):
        # 작은 물체 (높이 3.5 cm) 바로 옆에 조금 낮은 넓적한 물체 (3 cm): 둘레 띠만 쓰면 이웃 윗면을 받침면으로 잡음 (실기 바나나·옥수수)
        from arm_kinematics.vision import horizontal_planes
        K, T = camera()
        c = np.array([0.55, 0.0, DESK_TOP + 0.0175])
        size = (0.12, 0.035, 0.035)
        extra = [((0.55, 0.06, DESK_TOP + 0.015), (0.14, 0.06, 0.03), 0.0)]
        depth = render_depth(K, T, c, size, 0.0, extra)
        planes = horizontal_planes(depth, K, T)
        est = estimate_object(depth, K, T, box_of(K, T, c, size, 0.0), planes=planes)
        self.assertIsNotNone(est)
        self.assertAlmostEqual(est.support_z, DESK_TOP, delta=0.006)
        self.assertLess(abs(est.center[2] - c[2]), 0.01)

    def test_support_height_alone(self):
        K, T = camera()
        c = np.array([0.78, 0.0, DESK_TOP + 0.03])
        depth = render_depth(K, T, c, (0.06, 0.06, 0.06), 0.0)
        self.assertAlmostEqual(support_height(depth, K, T, box_of(K, T, c, (0.06, 0.06, 0.06), 0.0)), DESK_TOP, delta=0.006)


if __name__ == "__main__":
    unittest.main()
