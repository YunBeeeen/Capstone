#!/usr/bin/env python3
"""ZED 로 물체 찾기: 글자 → OWLv2 상자 (+SAM 마스크) → 로봇 바닥 기준 3D → 집을 목표·계획 명령. 로봇은 안 움직임.

  # 1) ML 서버 (GPU, 다른 터미널):  ~/capstone_ml/bin/python tools/vision_server.py
  # 2) ZED + TF (zed_base_tf):     ros2 launch capstone_navigation map.launch.py   (또는 loc.launch.py)
  source /opt/ros/humble/setup.bash && source ~/capstone/capstone_ws/install/setup.bash
  python3 tools/find_object.py banana                       # 'a banana' 로 찾음
  python3 tools/find_object.py "a red chili pepper" basket --zed-height 1.2 -n 5 --out /tmp/find.jpg

출력: 물체마다 점수, 3D 중심·크기·긴 축, 받침면(책상) 높이·앞 모서리, 그리고
  집을 목표 = 중심 + 잡는 깊이(--grasp-depth, 기본 2.5 cm) 만큼 앞(+x): IK 가 맞추는 점은 손가락 '끝' 이라
  물체 중심에 맞추면 손가락 끝으로만 집게 됨 → 끝이 중심을 지나 2~3 cm 들어가야 손가락 사이에 들어온다.
  그 목표로 바로 쓸 ik_sim_run 명령 (책상 = --target desk --front <앞 모서리>).
카메라 자세는 navigation/tools/shelf_check.grab (base_footprint ← 카메라, 보이는 바닥으로 높이·기울기 보정).
바닥이 안 보여 보정이 안 되면 --zed-height 로 카메라 높이를 맞춘다.
"""
import argparse
import base64
import json
import sys
import time
import urllib.parse
import urllib.request
from pathlib import Path
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT.parent / "navigation" / "tools"))
from arm_kinematics.scene import DESK_TOP  # noqa: E402
from arm_kinematics.vision import estimate_object, pixels_to_points  # noqa: E402

ZED_HEIGHT_AT_LIFT0 = 0.675
OPTICAL_ABOVE_SCREW = 0.03       # 삼각대 나사 → 왼쪽 렌즈 광학 중심 높이 (ZED 2i 근사)


def phrase(word):
    w = word.strip()
    return w if " " in w else ("an " if w[0].lower() in "aeiou" else "a ") + w


def ask(url, bgr, queries, sam, timeout=30):
    import cv2
    ok, jpg = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 92])
    u = f"{url}/find?q={urllib.parse.quote('|'.join(queries))}&sam={1 if sam else 0}"
    req = urllib.request.Request(u, data=jpg.tobytes(), headers={"Content-Type": "image/jpeg"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read())


def decode_mask(b64):
    import cv2
    arr = np.frombuffer(base64.b64decode(b64), np.uint8)
    return cv2.imdecode(arr, cv2.IMREAD_GRAYSCALE) > 0


def desk_front(frame, support_z, near_y, step=4):
    """받침면 높이 ±1 cm 점들 중 물체 좌우 30 cm 안에서 가장 가까운 x (2 % 분위) = 책상 앞 모서리. 못 찾으면 None."""
    d = frame["depth"]
    vs, us = np.mgrid[0:d.shape[0]:step, 0:d.shape[1]:step]
    p = pixels_to_points(d, frame["K"], frame["T"], us.ravel(), vs.ravel())
    s = p[(np.abs(p[:, 2] - support_z) < 0.01) & (np.abs(p[:, 1] - near_y) < 0.3) & (p[:, 0] > 0.2)]
    return float(np.percentile(s[:, 0], 2)) if len(s) > 200 else None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("words", nargs="+", help="찾을 말 (한 단어면 'a/an' 을 붙임). 예: banana basket \"a red chili pepper\"")
    ap.add_argument("--url", default="http://127.0.0.1:5578")
    ap.add_argument("--no-sam", action="store_true", help="SAM 마스크 없이 상자만 (책상 위 따로 놓인 물체는 이걸로도 됨)")
    ap.add_argument("-n", type=int, default=3, help="몇 장 찍어 중앙값 낼지")
    ap.add_argument("--min-score", type=float, default=0.15)
    ap.add_argument("--zed-height", type=float, help="바닥 → ZED 삼각대 나사 [m]. 바닥이 안 보여 높이 보정이 안 될 때 씀")
    ap.add_argument("--grasp-depth", type=float, default=0.025, help="집을 목표 = 중심 + 이만큼 앞 (+x) [m]")
    ap.add_argument("--out", type=Path, help="상자를 그린 그림 저장")
    ap.add_argument("--json", type=Path, help="결과 저장 (로봇 바닥 기준)")
    args = ap.parse_args()

    import cv2
    import rclpy
    from tf2_ros import Buffer, TransformListener
    import shelf_check                                     # navigation/tools (ZED 받기·바닥 보정)

    queries = [phrase(w) for w in args.words]
    rclpy.init()
    node = rclpy.create_node("find_object")
    buf = Buffer()
    TransformListener(buf, node)
    mc = SimpleNamespace(tf=buf)
    per = {q: [] for q in queries}
    last = None
    try:
        for i in range(args.n):
            fr = shelf_check.grab(node, mc)
            if fr is None:
                print("ZED 영상·깊이·TF 를 못 받음 — ZED 와 zed_base_tf (map/loc launch) 가 떠 있는지 확인")
                return 1
            cam_z = fr["T"][2, 3]
            if fr["leveled"] is None and args.zed_height is not None:
                want = args.zed_height + OPTICAL_ABOVE_SCREW
                fr["T"][2, 3] = want                       # 바닥 보정을 못 했으면 잰 높이로
                if i == 0:
                    print(f"바닥 보정 안 됨 → 카메라 높이를 {cam_z:.3f} → {want:.3f} m (--zed-height) 로")
            elif fr["leveled"] is None and i == 0:
                print(f"⚠ 바닥 보정 안 됨, 카메라 높이 {cam_z:.3f} m (TF 그대로). 리프트를 올렸으면 --zed-height 를 주세요")
            elif i == 0:
                print(f"바닥으로 보정함: 숙임 {fr['leveled'][0]:+.1f}°, 옆 {fr['leveled'][1]:+.1f}°, 높이 {fr['leveled'][2]:+.3f} m "
                      f"→ 카메라 높이 {fr['T'][2, 3]:.3f} m")
            t = time.time()
            ans = ask(args.url, fr["bgr"], queries, not args.no_sam)
            dt = time.time() - t
            firsts = {}
            for b, m in zip(ans["boxes"], ans["masks"]):
                if b[5] not in firsts and b[4] >= args.min_score:
                    firsts[b[5]] = (b, m)
            for q in queries:
                if q in firsts:
                    b, m = firsts[q]
                    est = estimate_object(fr["depth"], fr["K"], fr["T"], b, mask=decode_mask(m) if m else None)
                    if est is not None:
                        per[q].append((b, est))
            last = fr, firsts
            print(f"  {i + 1}/{args.n}: 서버 {dt:.1f} s, 찾음 " + ", ".join(f"{q} {firsts[q][0][4]:.2f}" for q in firsts))
    finally:
        node.destroy_node()
        rclpy.shutdown()

    result = {}
    print()
    for q in queries:
        got = per[q]
        if not got:
            print(f"{q}: 못 찾음 (점수 {args.min_score} 이상 상자 없음 또는 3D 실패)")
            continue
        c = np.median([e.center for _, e in got], axis=0)
        spread = 100 * float(np.max(np.linalg.norm([e.center - c for _, e in got], axis=1))) if len(got) > 1 else 0.0
        e = got[-1][1]
        score = float(np.median([b[4] for b, _ in got]))
        sup = [x.support_z for _, x in got if x.support_z is not None]
        sup = float(np.median(sup)) if sup else None
        front = desk_front(last[0], sup, c[1]) if sup is not None else None
        target = c + np.array([args.grasp_depth, 0.0, 0.0])
        print(f"{q}: 점수 {score:.2f}, {len(got)}/{args.n} 장, 흩어짐 {spread:.1f} cm")
        print(f"   중심 ({100 * c[0]:.1f}, {100 * c[1]:.1f}, {100 * c[2]:.1f}) cm, 크기 {100 * e.size[0]:.1f}×{100 * e.size[1]:.1f}"
              f"×{100 * e.size[2]:.1f} cm, 긴 축 {np.degrees(e.yaw):+.0f}°")
        if sup is not None:
            print(f"   받침면 {100 * sup:.1f} cm" + (f" (시뮬 책상 {100 * DESK_TOP:.0f} cm 와 {100 * (sup - DESK_TOP):+.1f} cm)"
                                                    if abs(sup - DESK_TOP) > 0.02 else "")
                  + (f", 책상 앞 모서리 x {100 * front:.1f} cm" if front is not None else ""))
        print(f"   집을 목표 (손끝, 중심 + {100 * args.grasp_depth:.1f} cm 앞): {target[0]:.3f} {target[1]:.3f} {target[2]:.3f}")
        if front is not None and args.zed_height is not None:
            print(f"   계획: python3 tools/ik_sim_run.py --target desk --front {front:.2f} --lateral={c[1]:.2f} "
                  f"--xyz {target[0]:.3f} {target[1]:.3f} {target[2]:.3f} --zed-height {args.zed_height} --no-sim --save-plan /tmp/p.json")
        result[q] = {"center": c.tolist(), "target": target.tolist(), "size": list(e.size), "yaw": e.yaw, "score": score,
                     "support_z": sup, "desk_front_x": front, "frames": len(got), "spread_cm": spread}
    if args.json:
        args.json.write_text(json.dumps(result, indent=1, ensure_ascii=False))
        print("저장:", args.json)
    if args.out and last is not None:
        img = last[0]["bgr"].copy()
        for q, (b, _) in last[1].items():
            x0, y0, x1, y1 = (int(v) for v in b[:4])
            cv2.rectangle(img, (x0, y0), (x1, y1), (0, 0, 255), 2)
            cv2.putText(img, f"{q} {b[4]:.2f}", (x0 + 3, y0 + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.imwrite(str(args.out), img)
        print("그림:", args.out)
    return 0 if result else 2


if __name__ == "__main__":
    sys.exit(main())
