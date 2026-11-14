#!/usr/bin/env python3
"""선반 확인 (선반 기억 2 단계) — 저장해 둔 "보는 자리" 사진·depth 와 지금 화면을 비교해서
선반이 있는지, 저장 때보다 얼마나 옮겨졌는지 (= 설 자리가 지금 어디인지) 를 잼. 로봇은 움직이지 않음.

  python3 src/navigation/tools/shelf_check.py shelf_A            # 지금 자리에서 재서 출력
  python3 src/navigation/tools/shelf_check.py shelf_A -n 20      # 20 번 재서 흩어짐(표준편차) 보기
먼저: place_mark.py shelf_A (설 자리에서) → 0.8~1.2 m 뒤로 물러나 place_mark.py shelf_A --view (보는 자리에서)

방법 (둘 다 "설 자리 좌표계 S" 가 지금 차체 기준 어디인지를 x, y, yaw 로 구함):
  특징점: 저장 사진의 선반 부분 SIFT 점(저장 depth 로 3D) ↔ 지금 사진 SIFT 점(지금 depth 로 3D) → 평면 강체 변환 RANSAC
  모양:   저장 depth 의 선반 점구름 ↔ 지금 점구름 ICP (x, y, yaw 만). 특징점 결과가 있으면 거기서, 없으면 기억 좌표에서 시작
  일치율은 가려진 점(더 가까운 게 앞에 있음)을 빼고 셈 → 사람이 일부 가려도 보이는 부분만으로 판단. 35 % 도 안 보이면 판단 불가
  전체 모양이 안 맞으면 판(선반 폭을 다 덮는 수평면)만으로 다시 맞춤 → 물건이 바뀌어도 틀로 확인
  뼈대만 있는 선반 (앞 기둥이 튀어나와 보이는 것) 은 앞 기둥 두 개로도 맞춤 (find_posts) — 물건이 다 바뀌어도 남는 부분
판정: 사진·모양 둘 다 맞고 서로 4 cm·4° 안 → 확실 / 기둥·모양·판만 맞음 → 보통 / 특징점만 → 낮음 / 아무것도 → 없음
"선반 부분" = 설 자리 앞 gate (place_mark --view 가 자동으로 잡아 저장: 앞면부터 선반 깊이까지, 좌우 폭). 뒤 벽·옆 물건은 뺌.
"""
import argparse
import math
import os
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

sys.path.insert(0, str(Path(__file__).resolve().parent))
import places  # noqa: E402

ZED = "/zed/zed_node/"
Z_MIN, Z_MAX = 0.04, 1.7      # 선반 점으로 볼 높이 [m] (바닥 제외)
VOX = 0.015                   # ICP 점 간격 [m]
FEAT_MIN = 12                 # 특징점 일치 최소 개수
FIT_MIN = 0.60                # ICP: 저장 점 중 2.5 cm 안에 짝이 있는 비율
AGREE = (0.04, math.radians(4.0))


# ------------------------------------------------------------------ 영상 받기
def _img(m):
    ch = {"bgra8": 4, "rgba8": 4, "bgr8": 3, "rgb8": 3}.get(m.encoding)
    if not ch:
        return None
    a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step // ch, ch)[:, :m.width, :3]
    return np.ascontiguousarray(a[:, :, ::-1] if m.encoding.startswith("rgb") else a)


def _depth(m):
    if m.encoding != "32FC1":
        return None
    d = np.frombuffer(m.data, np.float32).reshape(m.height, m.step // 4)[:, :m.width].copy()
    d[~np.isfinite(d) | (d <= 0)] = 0.0
    return d


def tf_matrix(buf, node, parent, child, timeout=3.0):
    """TF parent←child 4×4. 없으면 None."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.05)
        try:
            t = buf.lookup_transform(parent, child, rclpy.time.Time()).transform
        except Exception:
            continue
        x, y, z, w = t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w
        m = np.eye(4)
        m[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
        m[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
        return m
    return None


def grab(node, mc, timeout=4.0):
    """새 영상 한 벌: {bgr, depth[m], K(fx,fy,cx,cy), T(base←카메라 광학 4×4)}. 못 받으면 None."""
    got = {}
    subs = [node.create_subscription(Image, ZED + "rgb/color/rect/image", lambda m: got.__setitem__("rgb", m), qos_profile_sensor_data),
            node.create_subscription(Image, ZED + "depth/depth_registered", lambda m: got.__setitem__("depth", m), qos_profile_sensor_data),
            node.create_subscription(CameraInfo, ZED + "rgb/color/rect/camera_info", lambda m: got.__setitem__("info", m), qos_profile_sensor_data)]
    t0 = time.time()
    while time.time() - t0 < timeout and len(got) < 3:
        rclpy.spin_once(node, timeout_sec=0.05)
    for s in subs:
        node.destroy_subscription(s)
    if len(got) < 3:
        return None
    bgr, d = _img(got["rgb"]), _depth(got["depth"])
    T = tf_matrix(mc.tf, node, "base_footprint", got["depth"].header.frame_id)
    if bgr is None or d is None or T is None:
        return None
    k = got["info"].k
    return {"bgr": bgr, "depth": d, "K": (k[0], k[4], k[2], k[5]), "T": T}


# ------------------------------------------------------------------ 기하
def pix_to_base(depth, K, T, us, vs):
    """픽셀 (us, vs) → base 좌표 N×3 과 유효 여부."""
    fx, fy, cx, cy = K
    z = depth[vs, us]
    p = np.stack([(us - cx) / fx * z, (vs - cy) / fy * z, z, np.ones_like(z)], axis=1)
    return (p @ T.T)[:, :3], z > 0


def cloud(depth, K, T, step=2):
    """depth 전체 → base 좌표 점 N×3 (step 픽셀마다)."""
    vs, us = np.mgrid[0:depth.shape[0]:step, 0:depth.shape[1]:step]
    p, ok = pix_to_base(depth, K, T, us.ravel(), vs.ravel())
    return p[ok]


def se2(pose):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return np.array([[c, -s], [s, c]]), np.array(pose[:2])


def apply(pose, p):
    """S 좌표 점 → pose(부모 기준 S 의 x, y, yaw) 의 부모 좌표."""
    R, t = se2(pose)
    out = p.copy()
    out[:, :2] = p[:, :2] @ R.T + t
    return out


def apply_inv(pose, p):
    R, t = se2(pose)
    out = p.copy()
    out[:, :2] = (p[:, :2] - t) @ R
    return out


def fit2d(a, b):
    """a(N×2) → b(N×2) 로 옮기는 평면 강체 변환 (x, y, yaw)."""
    ca, cb = a.mean(0), b.mean(0)
    h = (a - ca).T @ (b - cb)
    yaw = math.atan2(h[0, 1] - h[1, 0], h[0, 0] + h[1, 1])
    R, _ = se2((0, 0, yaw))
    t = cb - R @ ca
    return (float(t[0]), float(t[1]), yaw)


def voxel(p, size=VOX):
    if len(p) == 0:
        return p
    _, i = np.unique(np.floor(p / size).astype(np.int64), axis=0, return_index=True)
    return p[i]


def in_gate(p, gate, margin=0.0):
    return ((p[:, 0] > gate[0] - margin) & (p[:, 0] < gate[1] + margin) & (p[:, 1] > gate[2] - margin)
            & (p[:, 1] < gate[3] + margin) & (p[:, 2] > Z_MIN) & (p[:, 2] < Z_MAX))


def find_gate(pts_s, width, shelf_depth, tall=0.8):
    """설 자리 좌표 점들에서 선반 영역 gate [x0, x1, y0, y1] 찾기. 못 찾으면 None.
    선반 = 설 자리 앞에서 가장 가까운, 폭이 width 쯤 되는 '키 큰(tall 넘는 높이까지 있는)' 덩어리.
    좌우 3 cm 칸마다 키 큰 점 중 가장 앞 x (앞 윤곽) 를 구하고, width 만 한 구간 중 윤곽이 고르고 가장 가까운 곳을 고름.
    예전엔 '설 자리 정면 ±폭/2 에서 가장 가까운 면' 으로 잡았는데, 설 자리가 선반 가운데가 아니거나 옆의 낮은 상자가
    앞으로 나와 있으면 선반을 놓쳤음 (실기 2026-11-13: 선반은 0.67 m 앞·오른쪽 20 cm 인데 왼쪽 상자 모서리 0.34 m 를 앞면으로 잡음).
    gate = 앞면 3 cm 앞 ~ 뒷면 5 cm 전 (뒤 벽을 빼려고), 좌우 끝 + 3 cm."""
    m = (pts_s[:, 0] > 0.15) & (pts_s[:, 0] < 1.6) & (np.abs(pts_s[:, 1]) < 0.9) & (pts_s[:, 2] > tall) & (pts_s[:, 2] < Z_MAX)
    p = pts_s[m]
    if len(p) < 100:
        return None
    cw = 0.03
    col = np.floor((p[:, 1] + 0.9) / cw).astype(int)
    ncol = int(1.8 / cw)
    front = np.full(ncol, np.nan)
    for c in np.unique(col):
        xs = p[col == c, 0]
        if len(xs) >= 3:
            front[c] = np.percentile(xs, 5)
    k = max(3, int(round(width / cw)))
    best = None
    for c0 in range(0, ncol - k + 1):
        w = front[c0:c0 + k]
        ok = np.isfinite(w)
        if ok.mean() < 0.6:
            continue
        xf = float(np.median(w[ok]))
        spread = float(np.median(np.abs(w[ok] - xf)))
        if spread > 0.04:
            continue
        yc = (c0 + k / 2) * cw - 0.9
        key = (round(xf / 0.05), -round(float(ok.mean()), 1), spread, abs(yc))   # 가까운 것 → 빈 칸 적은 것 → 윤곽 고른 것 → 정면
        if best is None or key < best[0]:
            best = (key, c0, xf)
    cand = []                                                     # (앞면 x, y0, y1)
    if best is not None:
        _, c0, xf = best
        inl = np.where(np.isfinite(front) & (np.abs(front - xf) < 0.08))[0]  # 앞면이 같은 칸들 중 고른 구간과 이어진 것
        lo, hi = c0, c0 + k - 1
        while lo - 1 in inl and (hi - lo + 1) * cw < width + 0.09:
            lo -= 1
        while hi + 1 in inl and (hi - lo + 1) * cw < width + 0.09:
            hi += 1
        cand.append((xf, lo * cw - 0.9, (hi + 1) * cw - 0.9))
    # 뼈대만 있는 선반 (판 사이로 뒤 벽이 보임): 앞 윤곽이 고르지 않으므로 앞 기둥 두 개를 찾음 —
    #  주변 칸보다 12 cm 넘게 앞으로 튀어나온 가는 세로 구조 둘이 선반 폭만큼 떨어져 있는 것
    #  (실기 2026-11-13: 윤곽이 기둥 0.66·0.70, 그 사이는 0.70~0.94 로 들쭉날쭉 → 고른 구간 찾기는 뒤 벽을 골랐음)
    posts = []
    for c in range(ncol):
        if not np.isfinite(front[c]):
            continue
        nb = np.r_[front[max(0, c - 4):max(0, c - 1)], front[c + 2:c + 5]]
        nb = nb[np.isfinite(nb)]
        if len(nb) >= 3 and front[c] < np.median(nb) - 0.12:
            if posts and c - posts[-1][2] <= 1:                   # 옆 칸과 이어진 기둥은 하나로
                posts[-1] = (min(posts[-1][0], front[c]), posts[-1][1], c)
            else:
                posts.append((front[c], c, c))
    pairs = [(min(a[0], b[0]), a[1] * cw - 0.9, (b[2] + 1) * cw - 0.9) for i, a in enumerate(posts) for b in posts[i + 1:]
             if abs(((b[1] + b[2]) - (a[1] + a[2])) / 2 * cw - width) < 0.09 and abs(a[0] - b[0]) < 0.10]
    cand += pairs
    if not cand:
        return None
    xf, y0, y1 = min(cand, key=lambda t: (round(t[0] / 0.05), abs((t[1] + t[2]) / 2)))    # 가까운 것, 비슷하면 정면에 가까운 것
    # 앞면: 키 큰 부분보다 아래 판이 더 앞으로 나온 선반 (판마다 깊이가 다름) → 그 폭 안의 낮은 점도 봐서 12 cm 까지 당김
    low = pts_s[(pts_s[:, 1] > y0) & (pts_s[:, 1] < y1) & (pts_s[:, 2] > Z_MIN + 0.04) & (pts_s[:, 2] <= tall)
                & (pts_s[:, 0] > xf - 0.12) & (pts_s[:, 0] < xf + 0.02)]
    if len(low) > 50:
        xf = min(xf, float(np.percentile(low[:, 0], 5)))
    # 뒷면: 선반 깊이까지. 그 안에 벽(그 폭 전체에 걸친 세로 면)이 있으면 벽 4 cm 전까지
    x1 = xf + shelf_depth - 0.05
    back = pts_s[(pts_s[:, 1] > y0) & (pts_s[:, 1] < y1) & (pts_s[:, 2] > 0.3) & (pts_s[:, 2] < Z_MAX)
                 & (pts_s[:, 0] > xf + 0.15) & (pts_s[:, 0] < xf + shelf_depth + 0.15)]
    if len(back) > 200:
        h, e = np.histogram(back[:, 0], bins=np.arange(xf + 0.15, xf + shelf_depth + 0.16, 0.02))
        i = int(np.argmax(h))
        if h[i] > 0.25 * len(back):                                 # 한 면에 몰려 있음 = 벽
            x1 = min(x1, float(e[i]) - 0.04)
    return [round(xf - 0.03, 3), round(x1, 3), round(float(y0) - 0.03, 3), round(float(y1) + 0.03, 3)]


ROBOT_FRONT, ROBOT_LEFT, ROBOT_RIGHT = 0.26, 0.22, 0.33     # 차체 중심 → 외곽 [m] (Nav2 footprint: 오른쪽이 팔 때문에 넓음)
PATH_Y = (-(ROBOT_RIGHT + 0.05), ROBOT_LEFT + 0.05)         # 접근 통로 좌우 범위 (차체 외곽 + 5 cm)


def path_blocked(cur, s_pose, x_end, min_pts=25):
    """지금 화면에서, 차체와 목표(선반 면) 사이 접근 통로에 뭔가 있는가 (사람·물건). s_pose = 설 자리의 지금 차체 기준 (x, y, yaw).
    통로 = 설 자리 좌표로 차체 앞 0.3 m 부터 x_end 까지, 좌우 PATH_Y, 높이 8 cm~1.5 m.
    x_end = 선반 면 7 cm 전 (정면이면 gate[0] − 0.04). 반환 (막힘 여부, 점 개수, 어디에 있는지 설명)."""
    p = apply_inv(s_pose, cloud(cur["depth"], cur["K"], cur["T"]))
    c, s = math.cos(s_pose[2]), math.sin(s_pose[2])
    xr = -(c * s_pose[0] + s * s_pose[1])                      # 설 자리 좌표에서 차체 x
    m = (p[:, 0] > xr + 0.30) & (p[:, 0] < x_end) & (p[:, 1] > PATH_Y[0]) & (p[:, 1] < PATH_Y[1]) & (p[:, 2] > 0.08) & (p[:, 2] < 1.5)
    q = voxel(p[m])
    info = ""
    if len(q) >= min_pts:
        info = (f"설 자리 기준 앞 {q[:, 0].min():.2f}~{q[:, 0].max():.2f} m (선반 면 {x_end + 0.07:.2f}), "
                f"좌우 {q[:, 1].min():+.2f}~{q[:, 1].max():+.2f}, 높이 {q[:, 2].min():.2f}~{q[:, 2].max():.2f} m")
    return len(q) >= min_pts, len(q), info


def shelf_box(gate):
    """gate → 선반 상자 (설 자리 좌표): (앞면 x, 깊이, 폭). 가운데의 좌우 위치는 shelf_yc."""
    return gate[0] + 0.03, gate[1] - gate[0] + 0.02, (gate[3] - gate[2]) - 0.06


def shelf_yc(gate):
    """선반 가운데가 설 자리 축에서 왼쪽으로 얼마나 치우쳤나 [m] (설 자리가 선반 정중앙이 아닐 수 있음)."""
    return (gate[2] + gate[3]) / 2


def side_stands(gate, wall_margin=0.15):
    """선반 옆면에 서는 자리 (설 자리 좌표 S 기준 (x, y, yaw)) — 정면이 막혔을 때 씀. {"left": ..., "right": ...}
    선반 = 벽에 붙은 상자라고 봄: 앞면·양 옆면으로만 갈 수 있음. 옆면에서는 차체가 벽에 닿지 않게 벽에서 떨어져 섬
    (벽 쪽 외곽 + wall_margin). 그래서 차체 중심이 선반 옆면 가운데가 아니라 앞쪽으로 치우침. 선반과의 간격은 정면과 같게.
      left  = 로봇이 정면을 볼 때 왼쪽 옆면. 거기 서면 벽이 차체 왼쪽 (좁은 쪽)
      right = 오른쪽 옆면. 벽이 차체 오른쪽 (팔 쪽) → 더 떨어져 섬"""
    xf, depth, width = shelf_box(gate)
    gap = xf - ROBOT_FRONT                                     # 정면에서 차체 앞 ↔ 선반 앞면 간격
    wall = xf + depth + 0.02                                   # 벽 (선반 뒤 2 cm 라고 봄)
    reach = width / 2 + gap + ROBOT_FRONT
    yc = shelf_yc(gate)
    return {"left": (wall - (ROBOT_LEFT + wall_margin), yc + reach, -math.pi / 2),
            "right": (wall - (ROBOT_RIGHT + wall_margin), yc - reach, math.pi / 2)}


SIDE_INSET = 0.025            # 옆에서 보면 판 모서리가 기둥 바깥면보다 이만큼 안쪽 (기둥 두께). 앞·뒤 기둥에 가려 길이도 그만큼씩 짧게 보임


def side_refine(ref, cur, target, name, iters=3):
    """옆면 보는 자리에서 다시 맞추기: 선반 판의 옆 모서리(길이 = 선반 깊이)를 depth 에서 찾아 옆면 설 자리를 고침.
    기준 사진 없이 선반 크기(gate)와 판 높이만 씀. target = 옆면 설 자리의 지금 차체 기준 (x, y, yaw), name = left/right.
    반환 (고친 target, 쓴 판 개수). 판을 2 장 이상 못 찾으면 그대로 돌려줌."""
    xf, depth, _ = shelf_box(ref["gate"])
    sp = side_stands(ref["gate"])[name]
    x_face = ROBOT_FRONT + (xf - ROBOT_FRONT)                      # 옆면 설 자리 좌표에서 선반 옆면까지
    yc = (xf + depth / 2 - sp[0]) * (1 if name == "left" else -1)  # 옆 모서리 가운데
    scene = cloud(cur["depth"], cur["K"], cur["T"])
    used = 0
    for _ in range(iters):
        s = apply_inv(target, scene)
        s = s[(s[:, 0] > x_face - 0.25) & (s[:, 0] < x_face + 0.15) & (np.abs(s[:, 1] - yc) < depth / 2 + 0.25)]
        dth, dxy = [], []
        for z0 in ref["levels"]:
            c = front_edge(s, z0, length=depth - 2 * SIDE_INSET)
            if c is None:
                continue
            R, _ = se2((0, 0, c[1]))
            dth.append(c[1])
            dxy.append(np.array(c[0]) - R @ np.array([x_face + SIDE_INSET, yc]))
        used = len(dth)
        if used < 2:
            break
        t = np.median(np.array(dxy), axis=0)
        target = places.compose(target, (float(t[0]), float(t[1]), float(np.median(dth))))
    return target, used


class DepthWatch:
    """depth 를 계속 받아 두고 필요할 때 최신 한 장을 줌 (접근 중 통로 감시용)."""

    def __init__(self, node, mc):
        self.n, self.mc, self.d, self.k, self.T = node, mc, None, None, None
        self.subs = [node.create_subscription(Image, ZED + "depth/depth_registered", lambda m: setattr(self, "d", m), qos_profile_sensor_data),
                     node.create_subscription(CameraInfo, ZED + "rgb/color/rect/camera_info", lambda m: setattr(self, "k", m), qos_profile_sensor_data)]

    def frame(self):
        if self.d is None or self.k is None:
            return None
        if self.T is None:
            self.T = tf_matrix(self.mc.tf, self.n, "base_footprint", self.d.header.frame_id, timeout=1.0)
            if self.T is None:
                return None
        k = self.k.k
        return {"depth": _depth(self.d), "K": (k[0], k[4], k[2], k[5]), "T": self.T}


# ------------------------------------------------------------------ 기준(저장본)
def _sift(bgr):
    import cv2
    return cv2.SIFT_create(nfeatures=3000, contrastThreshold=0.02).detectAndCompute(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), None)


def feats(bgr, depth, K, T):
    """SIFT 점 → (base 좌표 N×3, descriptor N×128). depth 없는 점은 뺌."""
    kp, des = _sift(bgr)
    if des is None or not len(kp):
        return np.zeros((0, 3)), np.zeros((0, 128), np.float32)
    uv = np.array([k.pt for k in kp])
    us = np.clip(np.round(uv[:, 0]).astype(int), 0, depth.shape[1] - 1)
    vs = np.clip(np.round(uv[:, 1]).astype(int), 0, depth.shape[0] - 1)
    p, ok = pix_to_base(depth, K, T, us, vs)
    return p[ok], des[ok]


def load_ref(place, yaml_path):
    """places.yaml 의 view 항목 → 기준 모델 (모두 설 자리 좌표계 S). 없으면 None."""
    import cv2
    v = place.get("view")
    if not v or "gate" not in v:
        return None
    base = Path(yaml_path).parent
    bgr = cv2.imread(str(base / v["image"]))
    mm = cv2.imread(str(base / v["depth"]), cv2.IMREAD_UNCHANGED)
    if bgr is None or mm is None:
        return None
    depth = mm.astype(np.float32) / 1000.0
    T = np.array(v["T_base_cam"]).reshape(4, 4)
    siv = v["stand_in_view"]
    siv = (siv[0], siv[1], math.radians(siv[2]))          # 보는 자리 차체 기준 설 자리
    gate = v["gate"]
    pts = apply_inv(siv, cloud(depth, v["K"], T))
    fp, fd = feats(bgr, depth, v["K"], T)
    fp = apply_inv(siv, fp)
    m = in_gate(fp, gate)
    model = voxel(pts[in_gate(pts, gate)])
    levels = board_levels(model, gate, (v.get("shelf") or [None])[0])
    width = (v.get("shelf") or [shelf_box(gate)[2]])[0]
    near = pts[(pts[:, 0] > gate[0] - 0.10) & (pts[:, 0] < gate[1] + 0.15) & (pts[:, 1] > gate[2] - 0.25) & (pts[:, 1] < gate[3] + 0.25)]
    saved = {"bgr": bgr, "depth": depth, "K": v["K"], "T": T}
    posts_img = find_posts_img(saved, siv, gate, width)
    return {"pts": model, "levels": levels, "boards": model[board_mask(model, levels)], "fp": fp[m], "fd": fd[m],
            "gate": gate, "siv": siv, "width": width, "posts": posts_img or find_posts(near, width), "posts_img": posts_img is not None}


def find_posts(pts, width, tall=0.8):
    """앞 기둥 두 개 찾기 → [(x, y), (x, y)] (y 작은 것 먼저) 또는 None. pts 는 선반 근처만 잘라서 줄 것.
    기둥 = 키 큰(tall 넘는) 점 중, 좌우 3 cm 칸에서 주변 칸보다 12 cm 넘게 앞으로 튀어나온 가는 세로 구조.
    둘 사이가 선반 폭 ±9 cm 이고 앞뒤 차가 10 cm 안인 쌍 중 가장 가까운 것. 위치는 그 기둥 점들의 평균 (칸 크기보다 정밀).
    물건이 다 바뀌어도 남는 부분이라, 사진·모양이 안 맞을 때 이것만으로 선반 위치·방향을 구함 (두 점이면 평면 자세가 정해짐)."""
    p = pts[(pts[:, 2] > tall) & (pts[:, 2] < Z_MAX)]
    if len(p) < 50:
        return None
    cw = 0.03
    y_lo = float(p[:, 1].min())
    col = np.floor((p[:, 1] - y_lo) / cw).astype(int)
    ncol = int(col.max()) + 1
    front = np.full(ncol, np.nan)
    for c in np.unique(col):
        xs = p[col == c, 0]
        if len(xs) >= 3:
            front[c] = np.percentile(xs, 5)
    posts = []
    for c in range(ncol):
        if not np.isfinite(front[c]):
            continue
        nb = np.r_[front[max(0, c - 4):max(0, c - 1)], front[c + 2:c + 5]]
        nb = nb[np.isfinite(nb)]
        if len(nb) >= 2 and front[c] < np.median(nb) - 0.12:
            if posts and c - posts[-1][2] <= 1:
                posts[-1] = (min(posts[-1][0], front[c]), posts[-1][1], c)
            else:
                posts.append((front[c], c, c))

    def centre(post):
        f, c0, c1 = post
        q = p[(col >= c0) & (col <= c1) & (p[:, 0] < f + 0.05)]
        return float(np.median(q[:, 0])), float(np.median(q[:, 1]))
    best = None
    for i, a in enumerate(posts):
        for b in posts[i + 1:]:
            ca, cb = centre(a), centre(b)
            if abs(math.hypot(cb[0] - ca[0], cb[1] - ca[1]) - width) < 0.09 and abs(ca[0] - cb[0]) < 0.12:
                key = min(ca[0], cb[0])
                if best is None or key < best[0]:
                    best = (key, [ca, cb])
    return best[1] if best else None


def find_posts_img(cur, s_pose, gate, width, loose=False):
    """사진에서 앞 기둥 두 개 찾기 → [(x, y), (x, y)] (S 좌표, y 작은 것 먼저) 또는 None.  s_pose = S 의 차체 기준 (x, y, yaw) 추정.
    depth 에서 가는 검은 기둥을 찾는 건 불안정했음 (실기 2026-11-13: 물건이 많이 바뀌면 10 번 중 2 번만 찾고, 찾아도 앞뒤 ±1 cm·좌우 ±2 cm 흔들림).
    사진에서는 기둥 모서리가 1~2 px 로 선명 → LSD 로 세로 선분을 뽑고, 선분을 따라 좌우 ±4 px 에서 가장 가까운 depth 를 줄마다 모아
    (기둥은 배경보다 앞) 3D 로 옮김. 같은 자리의 토막들을 합쳐 높이 0.5 m 넘게 이어지고 1 m 위까지 올라가는 것만 기둥 후보,
    그중 간격이 선반 폭 ±7 cm 이고 앞뒤 차 12 cm 안인 쌍 중 가장 가까운 것.
    loose=True: s_pose 가 많이 틀릴 수 있을 때 (첫 번째 시도) — 찾는 범위를 넓히고 앞뒤 차 조건을 풂 (방향이 틀리면 앞뒤 차가 커 보임)."""
    import cv2
    if cur.get("bgr") is None:
        return None
    mg, dx_max = (0.50, 0.30) if loose else (0.35, 0.12)
    gray = cv2.cvtColor(cur["bgr"], cv2.COLOR_BGR2GRAY)
    lines = cv2.createLineSegmentDetector(cv2.LSD_REFINE_STD).detect(gray)[0]
    if lines is None:
        return None
    lines = lines.reshape(-1, 4)
    d, (fx, fy, cx, cy) = cur["depth"], cur["K"]
    h, w = d.shape
    segs = []                                                    # (x, y, z 아래, z 위) — S 좌표
    for x1, y1, x2, y2 in lines:
        if abs(y2 - y1) < 50 or abs(x2 - x1) > 0.18 * abs(y2 - y1):          # 세로에 가깝고 50 px 넘는 것
            continue
        vs = np.arange(int(min(y1, y2)), int(max(y1, y2)) + 1, 2)
        vs = vs[(vs >= 0) & (vs < h)]
        us = np.round(x1 + (vs - y1) * (x2 - x1) / (y2 - y1)).astype(int)
        ok = (us >= 5) & (us < w - 5)
        vs, us = vs[ok], us[ok]
        if len(vs) < 15:
            continue
        win = np.stack([d[vs, us + k] for k in range(-4, 5)])
        win[win <= 0] = np.inf
        z = win.min(0)
        ok = np.isfinite(z) & (z < 4.0)
        if ok.sum() < 15:
            continue
        vs, us, z = vs[ok], us[ok], z[ok]
        pc = np.stack([(us - cx) / fx * z, (vs - cy) / fy * z, z, np.ones_like(z)], axis=1)
        p = apply_inv(s_pose, (pc @ cur["T"].T)[:, :3])
        x0 = np.median(p[:, 0])
        p = p[np.abs(p[:, 0] - x0) < 0.06]                          # 선분 중 배경으로 빠진 줄은 뺌
        if len(p) < 12 or not (gate[0] - mg < x0 < gate[1] + mg - 0.15):
            continue
        y0 = float(np.median(p[:, 1]))
        if not (gate[2] - mg < y0 < gate[3] + mg):
            continue
        segs.append((float(np.median(p[:, 0])), y0, float(p[:, 2].min()), float(p[:, 2].max()), len(p)))
    posts = []                                                   # 같은 자리 (4 cm 안) 토막 합치기
    for sg in sorted(segs, key=lambda t: t[1]):
        for q in posts:
            if abs(q["y"] - sg[1]) < 0.04 and abs(q["x"] - sg[0]) < 0.06:
                q["segs"].append(sg)
                n = sum(t[4] for t in q["segs"])
                q["x"], q["y"] = sum(t[0] * t[4] for t in q["segs"]) / n, sum(t[1] * t[4] for t in q["segs"]) / n
                break
        else:
            posts.append({"x": sg[0], "y": sg[1], "segs": [sg]})
    good = []
    for q in posts:
        cover = np.zeros(34, bool)                                # 5 cm 높이 칸으로 이어진 정도
        for t in q["segs"]:
            cover[max(0, int(t[2] / 0.05)):min(34, int(t[3] / 0.05) + 1)] = True
        if cover.sum() * 0.05 >= 0.5 and max(t[3] for t in q["segs"]) > 1.0:
            good.append((q["x"], q["y"]))
    best = None
    for i, a in enumerate(good):
        for b in good[i + 1:]:
            if abs(math.hypot(b[0] - a[0], b[1] - a[1]) - width) < 0.07 and abs(a[0] - b[0]) < dx_max:
                key = min(a[0], b[0])
                if best is None or key < best[0]:
                    best = (key, sorted([a, b], key=lambda t: t[1]))
    return best[1] if best else None


BAND = 0.02                   # 판 하나로 보는 높이 폭 ± [m] (실기 depth 는 판이 2~3 cm 두께로 퍼져 보임)


def board_levels(pts, gate, width=None):
    """선반 판 높이 목록 — 점이 몰려 있고 선반 폭을 거의 다 덮는 수평 구조 (윗면·아랫면·앞 모서리). 물건이 바뀌어도 남는 부분.
    1 cm 높이 칸 3 개 합이 전체 점의 6 % 를 넘는 봉우리 (실기 2026-11-13: 판 하나가 칸 하나당 3 % 씩 3 칸에 퍼짐 — 5 %/칸 기준으로는 0 장)."""
    width = width or shelf_box(gate)[2]             # 저장할 때 입력한 폭 (없으면 gate 에서)
    z = pts[:, 2]
    hist, edges = np.histogram(z, bins=np.arange(Z_MIN, Z_MAX + 0.01, 0.01))
    smooth = np.convolve(hist, np.ones(3), mode="same")
    out = []
    for i in np.argsort(smooth)[::-1][:20]:
        if smooth[i] < 0.06 * len(pts):
            break
        z0 = edges[i] + 0.005
        y = pts[np.abs(z - z0) < BAND, 1]
        fill = len(np.unique(np.floor(y / 0.03))) * 0.03          # 좌우로 실제 채워진 폭 (물건 몇 개가 양 끝에 있는 것과 구분)
        # 판끼리는 15 cm 넘게 떨어져 있음 → 더 뚜렷한 판 근처의 약한 봉우리 (판 위 물건 윗면) 는 뺌
        if len(y) > 30 and fill > 0.85 * width and all(abs(z0 - q) > 0.15 for q in out):
            out.append(float(z0))
    return out


def board_mask(pts, levels):
    m = np.zeros(len(pts), bool)
    for z0 in levels:
        m |= np.abs(pts[:, 2] - z0) < BAND
    return m


def front_edge(pts, z0, length=None):
    """높이 z0 판의 앞 모서리: (가운데 점 (x, y), 기울기 θ, 길이). 좌우 1.5 cm 칸마다 가장 앞 점 → 직선. 못 구하면 None.
    length 를 주면 그 길이만 한 구간 중 직선이 가장 잘 맞는 곳을 고름 — 옆에 다른 물건 면이 거의 일직선으로 이어져 있어도
    (시뮬: 선반 앞 가림판 옆면) 모서리만 골라냄."""
    p = pts[np.abs(pts[:, 2] - z0) < BAND]
    if len(p) < 20:
        return None
    b = np.floor(p[:, 1] / 0.015).astype(int)
    order = np.lexsort((p[:, 0], b))
    first = np.r_[True, b[order][1:] != b[order][:-1]]
    e = p[order][first][:, :2]                       # 칸마다 x 가 가장 작은 점
    if len(e) < 10:
        return None

    def fit(q):
        """q 안에서 직선 RANSAC → (a, k, inlier 마스크) 또는 None.  x = a + k·y"""
        rng, ok = np.random.default_rng(0), None
        for _ in range(60):
            i, j = rng.choice(len(q), 2, replace=False)
            if abs(q[i, 1] - q[j, 1]) < 0.10:
                continue
            k = (q[i, 0] - q[j, 0]) / (q[i, 1] - q[j, 1])
            inl = np.abs(q[:, 0] - (q[i, 0] + k * (q[:, 1] - q[i, 1]))) < 0.012
            if ok is None or inl.sum() > ok.sum():
                ok = inl
        if ok is None or ok.sum() < 8:
            return None
        k, a = np.polyfit(q[ok, 1], q[ok, 0], 1)
        ok = np.abs(q[:, 0] - (a + k * q[:, 1])) < 0.012
        return (a, k, ok) if ok.sum() >= 8 else None

    if length is None:
        r = fit(e)
        if r is None or r[2].sum() < 10:
            return None
        a, k, ok = r
        y0, y1 = np.percentile(e[ok, 1], 2), np.percentile(e[ok, 1], 98)
    else:
        best = None
        for ys in np.arange(e[:, 1].min(), e[:, 1].max() - length + 0.016, 0.015):
            q = e[(e[:, 1] >= ys) & (e[:, 1] <= ys + length)]
            if len(q) < 0.7 * length / 0.015:
                continue
            r = fit(q)
            if r is not None and (best is None or r[2].sum() > best[0]):
                best = (int(r[2].sum()), r[0], r[1], q[r[2], 1].min(), q[r[2], 1].max())
        if best is None or best[0] < 0.7 * length / 0.015:
            return None
        _, a, k, y0, y1 = best
    ym = (y0 + y1) / 2
    return (a + k * ym, ym), -math.atan(k), float(y1 - y0)


def edge_align(ref, cur, T, iters=4):
    """판 앞 모서리로 맞추기: 저장본과 지금 화면에서 판마다 앞 모서리(직선 + 양 끝)를 구해 x, y, yaw 를 직접 계산.
    수평면끼리 ICP 는 좌우·각도가 잘 안 잡혀서 (시뮬: 빈 선반 4.6 cm·4° 오차, 옮겨진 것 못 잡음) 물건이 바뀐 경우엔 이걸 씀.
    반환 (새 T, 쓴 판 개수)."""
    scene = cloud(cur["depth"], cur["K"], cur["T"])
    used = 0
    for _ in range(iters):
        s = apply_inv(T, scene)
        g = ref["gate"]
        s = s[(s[:, 1] > g[2] - 0.25) & (s[:, 1] < g[3] + 0.25)]
        dth, dxy = [], []
        for z0 in ref["levels"]:
            m = front_edge(ref["pts"], z0)
            if m is None:
                continue
            c = front_edge(s[np.abs(s[:, 0] - m[0][0]) < 0.22], z0)        # 그 판의 저장 앞면 ±22 cm (판마다 깊이가 다를 수 있음)
            if c is None or abs(c[2] - m[2]) > 0.12 * m[2]:               # 길이가 다르면 끝이 가려진 것 → 안 씀
                continue
            th = places.wrap(c[1] - m[1])
            R, _ = se2((0, 0, th))
            dth.append(th)
            dxy.append(np.array(c[0]) - R @ np.array(m[0]))
        used = len(dth)
        if not used:
            break
        th = float(np.median(dth))
        t = np.median(np.array(dxy), axis=0)
        T = places.compose(T, (float(t[0]), float(t[1]), th))
    return T, used


def visible_fit(pts_s, cur, T, tree, tol=0.025, want_mask=False):
    """저장 점(S 좌표)을 지금 카메라로 비춰 봤을 때: 맞음 / 가려짐(더 가까운 게 있음) / 없어짐(그 자리가 비어 뒤가 보임).
    반환 (맞는 비율 = 맞음 ÷ (맞음 + 없어짐), 보이는 비율 = 가려지지 않은 점 ÷ 전체). 가려진 점은 판단에서 뺌."""
    if len(pts_s) == 0:
        return 0.0, 0.0
    p = apply(T, pts_s)
    d, _ = tree.query(p, distance_upper_bound=tol)
    match = np.isfinite(d)
    pc = (np.c_[p, np.ones(len(p))] @ np.linalg.inv(cur["T"]).T)[:, :3]      # 카메라 좌표
    fx, fy, cx, cy = cur["K"]
    zc = np.maximum(pc[:, 2], 1e-3)
    u, v = np.round(pc[:, 0] / zc * fx + cx).astype(int), np.round(pc[:, 1] / zc * fy + cy).astype(int)
    h, w = cur["depth"].shape
    inside = (u >= 1) & (u < w - 1) & (v >= 1) & (v < h - 1) & (pc[:, 2] > 0.2)
    occ = np.zeros(len(p), bool)
    ui, vi = u[inside], v[inside]
    win = np.stack([cur["depth"][vi + dv, ui + du] for dv in (-1, 0, 1) for du in (-1, 0, 1)])   # 3×3 중 가장 가까운 값
    win[win <= 0] = np.inf
    occ[inside] = win.min(0) < zc[inside] - 0.06
    occ &= ~match
    gone = ~match & ~occ
    n_ok, n_gone = int(match.sum()), int(gone.sum())
    if want_mask:
        return (n_ok / max(1, n_ok + n_gone)), float((~occ).mean()), match
    return (n_ok / max(1, n_ok + n_gone)), float((~occ).mean())


def find_shelf_like(ref, cur, max_range=3.5):
    """찾기용 후보: 선반처럼 생긴 것 (저장한 판 높이 중 두 군데 이상에 가로 구조가 겹쳐 있고 크기가 선반만 한 것) 의 목록
    [(x, y, 앞면이 향한 방향) 차체 기준] — 방향은 가로로 긴 축에 수직, 카메라 쪽. 사진이 안 맞는 각도·거리에서도 "가서 볼 만한 곳" 을 고르는 용도 — 확정은 가까이 가서 measure 로.
    벽·상자처럼 여러 높이에 다 있는 세로 구조는 뺌."""
    from scipy import ndimage
    p = cloud(cur["depth"], cur["K"], cur["T"])
    p = p[(np.hypot(p[:, 0], p[:, 1]) < max_range) & (p[:, 2] > Z_MIN) & (p[:, 2] < 1.3)]
    if len(p) < 500 or len(ref["levels"]) < 2:
        return []
    cell = 0.04
    ij = np.floor((p[:, :2] + max_range) / cell).astype(int)
    n = int(2 * max_range / cell) + 1
    ok = (ij >= 0).all(1) & (ij < n).all(1)
    p, ij = p[ok], ij[ok]
    slices = np.zeros((12, n, n), bool)                            # 10 cm 높이 조각마다 점유 → 세로 구조 판별
    k = np.clip(((p[:, 2] - 0.08) / 0.10).astype(int), 0, 11)
    slices[k, ij[:, 0], ij[:, 1]] = True
    wall = ndimage.binary_dilation(slices.sum(0) >= 7)
    hits = np.zeros((n, n), int)
    for z0 in ref["levels"]:
        g = np.zeros((n, n), bool)
        m = np.abs(p[:, 2] - z0) < 0.015
        g[ij[m, 0], ij[m, 1]] = True
        hits += ndimage.binary_dilation(g)
    lab, cnt = ndimage.label((hits >= 2) & ~wall)
    width = shelf_box(ref["gate"])[2]
    out = []
    for c in range(1, cnt + 1):
        q = np.argwhere(lab == c) * cell - max_range + cell / 2
        if len(q) < 10:
            continue
        w, v = np.linalg.eigh(np.cov(q.T))
        ev = np.sqrt(np.maximum(w, 0)) * 3.46                                    # 고른 분포 가정한 길이
        if 0.5 * width < ev[1] < 1.5 * width and ev[0] < 0.6:
            c = q.mean(0)
            nrm = v[:, 0]                                                        # 짧은 축 = 선반 앞뒤 방향
            if nrm @ c > 0:                                                      # 카메라(원점) 쪽을 향하게 = 보이는 면이 앞
                nrm = -nrm
            out.append((float(c[0]), float(c[1]), float(math.atan2(nrm[1], nrm[0]))))
    return out


# ------------------------------------------------------------------ 재기
def match_feats(ref, cur, init, rng):
    """특징점 → (S 의 지금 차체 기준 (x, y, yaw) 또는 None, 일치 개수)."""
    import cv2
    cp, cd = feats(cur["bgr"], cur["depth"], cur["K"], cur["T"])
    if init is not None:
        near = in_gate(apply_inv(init, cp), ref["gate"], margin=0.6)   # 기억한 자리 근처 것만
        cp, cd = cp[near], cd[near]
    else:                                                          # 찾기: 화면 전체에서 (depth 를 믿을 만한 4 m 안)
        near = (np.hypot(cp[:, 0], cp[:, 1]) < 4.0) & (cp[:, 2] > Z_MIN) & (cp[:, 2] < Z_MAX)
        cp, cd = cp[near], cd[near]
    if len(ref["fd"]) < 4 or len(cd) < 4:
        return None, 0
    pairs = [m for m, n2 in (x for x in cv2.BFMatcher(cv2.NORM_L2).knnMatch(ref["fd"], cd, k=2) if len(x) == 2)
             if m.distance < 0.75 * n2.distance]
    a = np.array([ref["fp"][m.queryIdx] for m in pairs]).reshape(-1, 3)
    b = np.array([cp[m.trainIdx] for m in pairs]).reshape(-1, 3)
    ok = np.abs(a[:, 2] - b[:, 2]) < 0.05                           # 높이는 안 바뀜
    a, b = a[ok], b[ok]
    if len(a) < 4:
        return None, len(a)
    best = None
    for _ in range(400):
        i, j = rng.choice(len(a), 2, replace=False)
        if np.linalg.norm(a[i, :2] - a[j, :2]) < 0.05:
            continue
        T = fit2d(a[[i, j], :2], b[[i, j], :2])
        inl = np.linalg.norm(apply(T, a)[:, :2] - b[:, :2], axis=1) < 0.025
        if best is None or inl.sum() > best.sum():
            best = inl
    if best is None or best.sum() < 4:
        return None, 0
    T = fit2d(a[best, :2], b[best, :2])
    best = np.linalg.norm(apply(T, a)[:, :2] - b[:, :2], axis=1) < 0.025
    return fit2d(a[best, :2], b[best, :2]), int(best.sum())


def icp(model, gate, cur, init):
    """모양 → (S 의 지금 차체 기준 (x, y, yaw), 맞는 비율, 보이는 비율). model = S 좌표 점 (전체 또는 판만).
    맞는 비율은 가려진 점을 빼고 셈 (visible_fit) — 사람·새 물건이 앞을 가려도 보이는 부분만으로 판단."""
    from scipy.spatial import cKDTree
    scene = cloud(cur["depth"], cur["K"], cur["T"])
    scene = voxel(scene[in_gate(apply_inv(init, scene), gate, margin=0.5)])
    if len(scene) < 100 or len(model) < 100:
        return init, 0.0, 0.0
    tree = cKDTree(scene)
    T = init
    for dmax in (0.20, 0.15, 0.10, 0.07, 0.05, 0.035) + (0.03,) * 12:
        d, i = tree.query(apply(T, model), distance_upper_bound=dmax)
        ok = np.isfinite(d)
        if ok.sum() < 50:
            return T, 0.0, 0.0
        T = fit2d(model[ok, :2], scene[i[ok], :2])
    fit, vis = visible_fit(model, cur, T, tree)
    return T, fit, vis


def post_align(ref, cur, init):
    """앞 기둥 두 개로 맞추기 → S 의 지금 차체 기준 (x, y, yaw) 또는 None (기준에 기둥이 없거나 화면에서 못 찾음).
    기둥은 사진의 세로선으로 찾음 (find_posts_img). 기준 사진에서 못 찾았으면 depth 윤곽으로 (find_posts).
    init 근처 (선반 영역 앞 35 cm ~ 뒤 20 cm, 좌우 35 cm 여유) 에서 찾고, 간격이 저장본과 4 cm 넘게 다르면 버림."""
    if not ref.get("posts"):
        return None
    g = ref["gate"]
    if ref.get("posts_img"):                               # 기준을 사진 선으로 잡았으면 화면도 사진 선으로
        a = np.array(ref["posts"])
        found = find_posts_img(cur, init, g, ref["width"], loose=True)       # 넓게 한 번 → 그 결과로 좁게 한 번 더
        if found and abs(np.linalg.norm(a[1] - a[0]) - np.linalg.norm(np.array(found[1]) - np.array(found[0]))) < 0.05:
            init = places.compose(init, fit2d(a, np.array(found)))
            found = find_posts_img(cur, init, g, ref["width"])
        else:
            found = None
    else:
        s = apply_inv(init, cloud(cur["depth"], cur["K"], cur["T"]))
        s = s[(s[:, 0] > g[0] - 0.35) & (s[:, 0] < g[1] + 0.20) & (s[:, 1] > g[2] - 0.35) & (s[:, 1] < g[3] + 0.35)]
        found = find_posts(s, ref["width"])
    if not found:
        return None
    a, b = np.array(ref["posts"]), np.array(found)
    if abs(np.linalg.norm(a[1] - a[0]) - np.linalg.norm(b[1] - b[0])) > 0.04:
        return None
    return places.compose(init, fit2d(a, b))


VIS_MIN = 0.35                # 이만큼은 보여야 판단 (그보다 많이 가려지면 '없음' 이 아니라 판단 불가)
BOARD_FIT_MIN = 0.70          # 판만으로 판단할 때 맞는 비율


def measure(ref, cur, init, seed=0):
    """한 번 재기. init = 기억대로라면 S 가 있을 자리 (지금 차체 기준 x, y, yaw).
    반환 dict: present, conf(high/mid/low/none), pose(S 의 지금 차체 기준), n_feat, fit(전체 점), vis(보이는 비율),
               board_fit(판만), how(판정 근거)."""
    f_pose, n = match_feats(ref, cur, init, np.random.default_rng(seed))
    feat_ok = f_pose is not None and n >= FEAT_MIN
    start = f_pose if feat_ok else init
    i_pose, fit, vis = icp(ref["pts"], ref["gate"], cur, start)
    has_boards = len(ref["boards"]) >= 150

    def board_fit_at(T):
        from scipy.spatial import cKDTree
        scene = cloud(cur["depth"], cur["K"], cur["T"])
        scene = voxel(scene[in_gate(apply_inv(T, scene), ref["gate"], margin=0.5)])
        if len(scene) < 100:
            return 0.0, 0.0
        f, v, m = visible_fit(ref["boards"], cur, T, cKDTree(scene), tol=0.02, want_mask=True)
        # 맞은 점이 판마다 깊이 방향(x)으로 8 cm 넘게 펼쳐져 있어야 진짜 수평 판. 세워진 면(가림판·상자 앞면)에는 앞 모서리만 걸침
        deep = sum(1 for z0 in ref["levels"]
                   if (k := m & (np.abs(ref["boards"][:, 2] - z0) < BAND)).sum() >= 15
                   and np.percentile(ref["boards"][k, 0], 95) - np.percentile(ref["boards"][k, 0], 5) > 0.08)
        return (f, v) if deep >= min(2, len(ref["levels"])) else (0.0, v)

    # 모양이 맞더라도 그 자리에 선반 판(수평면)이 실제로 보여야 인정. 안 그러면 선반 앞에 세운 평평한 판·상자에도 맞아 버림
    #  (시뮬 2026-11-13: 가림판을 선반으로 보고 19 cm 틀린 자리로 접근 — 가려진 점을 빼고 세다 보니 앞면만으로 75 % 일치)
    ib_fit, ib_vis = board_fit_at(i_pose) if has_boards else (1.0, 1.0)
    icp_ok = fit >= FIT_MIN and vis >= VIS_MIN and ib_fit >= 0.6 and ib_vis >= VIS_MIN
    b_pose, b_fit, b_vis = (i_pose, ib_fit if has_boards else 0.0, ib_vis if has_boards else 0.0)
    used = 0
    if not icp_ok and has_boards:                       # 전체로는 안 맞음 → 판(틀)만으로 다시: 물건이 바뀐 경우
        b_pose, _, _ = icp(ref["boards"], ref["gate"], cur, start)
        b_pose, used = edge_align(ref, cur, b_pose)     # 좌우·각도는 판 앞 모서리로
        b_fit, b_vis = board_fit_at(b_pose)
    board_ok = b_fit >= BOARD_FIT_MIN and b_vis >= VIS_MIN and used >= 2
    near = lambda a, b: math.hypot(a[0] - b[0], a[1] - b[1]) < AGREE[0] and abs(places.wrap(a[2] - b[2])) < AGREE[1]
    p_pose = post_align(ref, cur, i_pose if icp_ok else start)
    if feat_ok and icp_ok and near(f_pose, i_pose):
        conf, pose, how = "high", i_pose, "사진·모양 둘 다 일치"
    elif p_pose is not None:
        conf, pose, how = "mid", p_pose, "앞 기둥 두 개로 맞춤 — 사진은 안 맞음 (물건이 바뀜)"
    elif icp_ok:
        conf, pose, how = "mid", i_pose, "모양만 일치"
    elif board_ok:
        conf, pose, how = "mid", b_pose, "판(틀)만 일치 — 물건이 바뀜"
    elif feat_ok:
        conf, pose, how = "low", f_pose, "사진만 일치"
    else:
        conf, pose, how = "none", None, ("많이 가려짐" if max(vis, b_vis) < VIS_MIN and (vis > 0 or b_vis > 0) else "일치하는 것 없음")
    return {"present": conf != "none", "conf": conf, "pose": pose, "n_feat": n, "fit": fit, "vis": vis,
            "board_fit": b_fit, "how": how}


def measure_n(node, mc, ref, init, n=3):
    """n 번 재서 가운데값. 반환 (요약 dict, 낱개 목록)."""
    runs = []
    for k in range(n):
        cur = grab(node, mc)
        if cur is None:
            continue
        if os.environ.get("SHELF_DEBUG"):                  # 본 화면을 저장 (나중에 로봇 없이 다시 돌려 보려고)
            np.savez_compressed(os.path.join(os.environ["SHELF_DEBUG"], f"chk_{time.strftime('%H%M%S')}_{k}.npz"),
                                bgr=cur["bgr"], depth=cur["depth"], K=np.array(cur["K"]), T=cur["T"], init=np.array(init))
        runs.append(measure(ref, cur, init, seed=k))
    good = [r for r in runs if r["present"]]
    if not runs or len(good) * 2 < len(runs):
        return {"present": False, "conf": "none", "pose": None, "n_feat": max([r["n_feat"] for r in runs], default=0),
                "fit": max([r["fit"] for r in runs], default=0.0), "vis": max([r["vis"] for r in runs], default=0.0),
                "board_fit": max([r["board_fit"] for r in runs], default=0.0),
                "how": runs[-1]["how"] if runs else "영상 없음"}, runs
    order = {"high": 0, "mid": 1, "low": 2}
    conf = max((r["conf"] for r in good), key=order.get)          # 가장 나쁜 등급
    xs, ys = np.median([r["pose"][0] for r in good]), np.median([r["pose"][1] for r in good])
    yaw = math.atan2(np.median([math.sin(r["pose"][2]) for r in good]), np.median([math.cos(r["pose"][2]) for r in good]))
    worst = max(good, key=lambda r: order[r["conf"]])
    return {"present": True, "conf": conf, "pose": (float(xs), float(ys), yaw), "how": worst["how"],
            "n_feat": int(np.median([r["n_feat"] for r in good])), "fit": float(np.median([r["fit"] for r in good])),
            "vis": float(np.median([r["vis"] for r in good])), "board_fit": float(np.median([r["board_fit"] for r in good]))}, runs


CONF_KO = {"high": "확실", "mid": "보통", "low": "낮음", "none": "없음"}


def describe(r):
    """판정 한 줄: 등급 (근거; 숫자)."""
    b = f", 판 {r['board_fit'] * 100:.0f} %" if r.get("board_fit") else ""
    return f"{CONF_KO[r['conf']]} ({r['how']}; 특징점 {r['n_feat']}, 모양 {r['fit'] * 100:.0f} %{b}, 보이는 부분 {r['vis'] * 100:.0f} %)"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("name")
    ap.add_argument("-n", type=int, default=3, help="재는 횟수")
    ap.add_argument("--places", help="places.yaml 경로 (기본: RTAB-Map 지도 옆)")
    a = ap.parse_args()
    rclpy.init()
    n = Node("shelf_check")
    mc = places.MapClient(n)
    path = places.find_places(mc, a.places)
    if path is None:
        print("RTAB-Map 에서 지도 경로를 못 얻음 — slam.launch 가 떠 있나? (또는 --places 로 지정)")
        return 1
    pl = places.load(path)
    if a.name not in pl:
        print(f"{path} 에 '{a.name}' 없음")
        return 1
    ref = load_ref(pl[a.name], path)
    if ref is None:
        print(f"'{a.name}' 에 보는 자리 기록이 없음 → place_mark.py {a.name} --view")
        return 1
    cur = mc.robot_pose()
    if cur is None:
        print("TF map→base_footprint 없음")
        return 1
    stand, _ = places.resolve(pl[a.name], mc.graph())
    init = places.rel(cur, stand)
    print(f"기준: 선반 점 {len(ref['pts'])} 개 (판 {len(ref['boards'])} 개), 특징점 {len(ref['fp'])} 개. 기억대로면 설 자리는 앞 {init[0]:.2f}, 왼 {init[1]:+.2f} m, {math.degrees(init[2]):+.0f}°")
    res, runs = measure_n(n, mc, ref, init, a.n)
    for r in runs:
        if r["present"]:
            d = places.rel(init, r["pose"])
            print(f"  {describe(r)}  옮겨짐 앞 {d[0] * 100:+5.1f} 왼 {d[1] * 100:+5.1f} cm, {math.degrees(d[2]):+5.1f}°")
        else:
            print(f"  {describe(r)}")
    print(f"판정: {describe(res)}")
    if res["present"]:
        d = places.rel(init, res["pose"])
        print(f"  기억보다 앞 {d[0] * 100:+.1f} cm, 왼 {d[1] * 100:+.1f} cm, {math.degrees(d[2]):+.1f}° (설 자리 기준. 지도 위치 오차도 여기 섞임)")
        good = [places.rel(init, r["pose"]) for r in runs if r["present"]]
        if len(good) >= 5:
            g = np.array(good)
            print(f"  흩어짐(표준편차): 앞 {g[:, 0].std() * 100:.1f} cm, 왼 {g[:, 1].std() * 100:.1f} cm, {math.degrees(g[:, 2].std()):.1f}°")
    return 0


if __name__ == "__main__":
    sys.exit(main())
