#!/usr/bin/env python3
"""저장된 RTAB-Map 지도 (.db) → 2D 지도 PNG 한 장. ROS·화면 없이 (SSH 로 Jetson 에서 돌리고 그림만 가져와 보기).

  python3 map_png.py ~/maps/X.db                  # → /tmp/X_map.png
  python3 map_png.py ~/maps/X.db --out map.png
노드마다 저장된 지역 점유칸 (바닥·장애물·빈 곳, 5 cm) 을 노드 자세 (주행 기록) 로 이어 붙임 — 루프 클로저 보정은 안 들어가
실제 지도보다 조금 번져 보일 수 있음. 흰색 = 갈 수 있음, 검정 = 장애물, 회색 = 안 봄, 파란 선 = 다닌 길 (초록 출발·빨강 끝),
빨간 + = 지도 원점. 장소 (<지도>.places.yaml) 가 있으면 초록 화살표로. DB 는 읽기 전용으로 엶 (켜져 있는 매핑과 같이 써도 됨).
"""
import argparse
import sqlite3, zlib, struct, os, numpy as np, cv2
ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("db")
ap.add_argument("--out")
a = ap.parse_args()
db = os.path.expanduser(a.db); RES = 0.05
out = a.out or f"/tmp/{os.path.splitext(os.path.basename(db))[0]}_map.png"
c = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
def mat(blob):
    if not blob: return None
    rows, cols, typ = struct.unpack("iii", blob[-12:])        # 압축 뒤에 그대로 붙어 있음 (rtabmap compressData2)
    if rows * cols == 0: return None
    raw = zlib.decompress(blob[:-12])
    ch = 1 + (typ >> 3)
    return np.frombuffer(raw, dtype=np.float32).reshape(-1, ch)
poses, cells = {}, []
for nid, pose in c.execute("SELECT id, pose FROM Node"):
    if pose and len(pose) == 48:
        poses[nid] = np.frombuffer(pose, dtype=np.float32).reshape(3, 4)
for nid, gc, oc, ec in c.execute("SELECT id, ground_cells, obstacle_cells, empty_cells FROM Data"):
    if nid not in poses: continue
    T = poses[nid]
    for kind, b in (("free", gc), ("occ", oc), ("free", ec)):
        m = mat(b)
        if m is None or m.shape[1] < 2: continue
        p = m[:, :3] if m.shape[1] >= 3 else np.c_[m[:, :2], np.zeros(len(m))]
        w = p @ T[:, :3].T + T[:, 3]
        cells.append((kind, w[:, :2]))
allp = np.vstack([w for _, w in cells] + [np.array([T[:2, 3] for T in poses.values()])])
lo, hi = allp.min(0) - 0.5, allp.max(0) + 0.5
W, H = (np.ceil((hi - lo) / RES).astype(int) + 1)
free = np.zeros((H, W), np.int32); occ = np.zeros((H, W), np.int32)
for kind, w in cells:
    ij = ((w - lo) / RES).astype(int)
    tgt = occ if kind == "occ" else free
    np.add.at(tgt, (ij[:, 1], ij[:, 0]), 1)
img = np.full((H, W, 3), 205, np.uint8)
img[free > 0] = 255
img[(occ > 0) & (occ >= free * 0.5)] = 0
path = np.array([poses[k][:2, 3] for k in sorted(poses)])
pij = ((path - lo) / RES).astype(int)
for a, b in zip(pij, pij[1:]):
    cv2.line(img, tuple(int(v) for v in a), tuple(int(v) for v in b), (255, 120, 0), 1)
cv2.circle(img, tuple(int(v) for v in pij[0]), 4, (0, 170, 0), -1)        # 출발
cv2.circle(img, tuple(int(v) for v in pij[-1]), 4, (0, 0, 255), -1)       # 끝
o = ((np.zeros(2) - lo) / RES).astype(int)
cv2.drawMarker(img, tuple(int(v) for v in o), (0, 0, 255), cv2.MARKER_CROSS, 10, 1)
import yaml
pf = os.path.splitext(db)[0] + ".places.yaml"
if os.path.exists(pf):
    for k, v in (yaml.safe_load(open(pf)) or {}).items():
        if isinstance(v, dict) and "stand_pose" in v:
            x, y, yd = v["stand_pose"]
            p0 = ((np.array([x, y]) - lo) / RES).astype(int)
            p1 = ((np.array([x + 0.35 * np.cos(np.radians(yd)), y + 0.35 * np.sin(np.radians(yd))]) - lo) / RES).astype(int)
            cv2.arrowedLine(img, tuple(int(t) for t in p0), tuple(int(t) for t in p1), (0, 150, 0), 2, tipLength=0.4)
img = np.flipud(img).copy()
S = max(1, int(round(900 / max(W, H))))
img = cv2.resize(img, (W * S, H * S), interpolation=cv2.INTER_NEAREST)
cv2.imwrite(out, img)
print("그림:", out)
L = float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1)))
print(f"노드 {len(poses)} 개, 지도 {W * RES:.1f}×{H * RES:.1f} m, 다닌 길 {L:.1f} m, 빈 곳 {(free > 0).sum() * RES * RES:.1f} m², "
      f"장애물 칸 {((occ > 0) & (occ >= free * 0.5)).sum() * RES * RES:.1f} m², 끝 위치 ({path[-1][0]:.2f}, {path[-1][1]:.2f}) — 출발점에서 {np.linalg.norm(path[-1] - path[0]):.2f} m")
