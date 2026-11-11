"""rtabmap DB 의 최적화 포즈(rtabmap-export --poses, 시각 포함) vs 참값(gt_abs.npy) → 최종 지도 정확도 + 그림.
  python3 slam_eval_db.py <출력디렉터리>   (run_slam_loop.sh 가 부름)
"""
import math, sys, glob
import numpy as np
OUT = sys.argv[1]
wrap = lambda a: math.atan2(math.sin(a), math.cos(a))
def rel(p0, p):
    dx, dy = p[0] - p0[0], p[1] - p0[1]; c, s = math.cos(p0[2]), math.sin(p0[2])
    return (c * dx + s * dy, -s * dx + c * dy, wrap(p[2] - p0[2]))
f = sorted(glob.glob(f"{OUT}/*poses*.txt"))[0]
P = []
for line in open(f):
    if line.startswith("#") or not line.strip():
        continue
    v = [float(x) for x in line.split()]
    t, x, y, qx, qy, qz, qw = v[0], v[1], v[2], v[4], v[5], v[6], v[7]
    P.append((t, x, y, math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))))
G = np.load(f"{OUT}/gt_abs.npy")
tl = G[:, 0]
def gt_at(t):
    i = int(np.clip(np.searchsorted(tl, t), 1, len(tl) - 1)); a, b = G[i - 1, 1:], G[i, 1:]
    r = (t - tl[i - 1]) / max(tl[i] - tl[i - 1], 1e-9)
    return (a[0] + r * (b[0] - a[0]), a[1] + r * (b[1] - a[1]), a[2] + r * wrap(b[2] - a[2]))
P = [p for p in P if tl[0] <= p[0] <= tl[-1]]
Gm = [gt_at(p[0]) for p in P]
og = np.array([rel(Gm[0], g) for g in Gm]); oo = np.array([rel(P[0][1:], p[1:]) for p in P])
e = np.hypot(oo[:, 0] - og[:, 0], oo[:, 1] - og[:, 1]); ey = np.degrees([abs(wrap(a - b)) for a, b in zip(oo[:, 2], og[:, 2])])
print(f"최종 지도: 노드 {len(P)} 개 (파일 {f.split('/')[-1]}), 위치 오차 평균 {e.mean()*100:.1f} cm / 최대 {e.max()*100:.1f} cm, "
      f"yaw 평균 {np.mean(ey):.2f}° / 최대 {np.max(ey):.2f}°")
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
fig, ax = plt.subplots(1, 2, figsize=(15, 5.5))
T = np.load(f"{OUT}/traj.npy"); gt, od = T[:, 0:3], T[:, 3:6]
ax[0].plot(og[:, 0], og[:, 1], lw=3, alpha=0.5, label="ground truth")
ax[0].plot(od[:, 0], od[:, 1], "--", label="odometry (odometry/filtered)")
ax[0].plot(oo[:, 0], oo[:, 1], "o-", ms=2, lw=1, label="RTAB-Map optimized")
ax[0].set_aspect("equal"); ax[0].grid(alpha=.3); ax[0].legend(); ax[0].set_title("trajectory (start-relative)")
g = np.load(f"{OUT}/grid.npy"); res, ox, oy = np.load(f"{OUT}/grid_info.npy")
img = np.full(g.shape, .5); img[g == 0] = 1; img[g > 50] = 0
ax[1].imshow(img, cmap="gray", origin="lower", extent=[ox, ox + g.shape[1] * res, oy, oy + g.shape[0] * res])
P0 = P[0]; ax[1].plot([p[1] for p in P], [p[2] for p in P], "r-", lw=1)
ax[1].set_title("RTAB-Map occupancy grid (map frame) + optimized path")
fig.tight_layout(); fig.savefig(f"{OUT}/final_map.png", dpi=90)
