#!/usr/bin/env python3
"""지금 로봇이 선 자리를 이름 붙여 기억 — slam.launch (매핑 또는 재위치) 가 떠 있을 때 실행.

  python3 src/navigation/tools/place_mark.py shelf_A                 # 지금 자리·방향을 shelf_A 로 저장
  python3 src/navigation/tools/place_mark.py desk_1 --kind desk      # 종류 지정 (기본 shelf)
  python3 src/navigation/tools/place_mark.py box --movable           # 옮겨질 수 있는 물체
  python3 src/navigation/tools/place_mark.py shelf_A --view          # 0.8~1.2 m 물러난 "보는 자리" 를 추가 저장 (도착 후 선반 확인용)
  python3 src/navigation/tools/place_mark.py shelf_A --side left     # (선택) 왼쪽 옆면 설 자리를 직접 저장. 안 하면 자동 계산
  python3 src/navigation/tools/place_mark.py --list                  # 저장된 장소와 지금 지도 좌표
  python3 src/navigation/tools/place_mark.py --delete shelf_A

저장하는 것 = "로봇이 설 자리" (선반 위치가 아님): 팔 작업이 되는 자리·방향에 로봇을 세워 놓고 실행.
파일은 RTAB-Map 이 쓰는 지도 옆에 자동으로 (<지도>.places.yaml, 형식은 places.py 머리 주석). --file 로 직접 지정 가능.
찾아가기: nav_goal_ahead.py --name shelf_A
"""
import argparse
import math
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


def snapshot(node, out_dir, name, timeout=3.0):
    """ZED 영상·depth 한 장씩 저장. 반환 {image, depth, K} (못 받은 건 빠짐)."""
    got = {}
    subs = [node.create_subscription(Image, ZED + "rgb/color/rect/image", lambda m: got.setdefault("rgb", m), qos_profile_sensor_data),
            node.create_subscription(Image, ZED + "depth/depth_registered", lambda m: got.setdefault("depth", m), qos_profile_sensor_data),
            node.create_subscription(CameraInfo, ZED + "rgb/color/rect/camera_info", lambda m: got.setdefault("info", m), qos_profile_sensor_data)]
    t0 = time.time()
    while time.time() - t0 < timeout and len(got) < 3:
        rclpy.spin_once(node, timeout_sec=0.1)
    for s in subs:
        node.destroy_subscription(s)
    out = {}
    try:
        import cv2
    except ImportError:
        print("  (cv2 없음 — 사진 저장 건너뜀)")
        return out
    out_dir.mkdir(parents=True, exist_ok=True)
    if "rgb" in got:
        m = got["rgb"]
        ch = {"bgra8": 4, "rgba8": 4, "bgr8": 3, "rgb8": 3}.get(m.encoding)
        if ch:
            img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step // ch, ch)[:, :m.width, :3]
            if m.encoding.startswith("rgb"):
                img = img[:, :, ::-1]
            p = out_dir / f"{name}_rgb.jpg"
            cv2.imwrite(str(p), np.ascontiguousarray(img), [cv2.IMWRITE_JPEG_QUALITY, 92])
            out["image"] = f"{out_dir.name}/{p.name}"
    if "depth" in got and got["depth"].encoding == "32FC1":
        m = got["depth"]
        d = np.frombuffer(m.data, np.float32).reshape(m.height, m.step // 4)[:, :m.width]
        mm = np.where(np.isfinite(d) & (d > 0), d * 1000.0, 0).clip(0, 65535).astype(np.uint16)
        p = out_dir / f"{name}_depth.png"
        cv2.imwrite(str(p), mm)
        out["depth"] = f"{out_dir.name}/{p.name}"
    if "info" in got:
        k = got["info"].k
        out["K"] = [round(float(v), 2) for v in (k[0], k[4], k[2], k[5])]
    return out


def mark_view(n, mc, a, path, pl, graph, pose):
    """보는 자리 저장: 사진·depth·카메라 TF + 설 자리의 상대 위치 + 선반 gate."""
    import cv2
    import shelf_check as sc
    if a.name not in pl:
        print(f"'{a.name}' 가 아직 없음 — 먼저 설 자리에서 place_mark.py {a.name}")
        return 1
    stand, _ = places.resolve(pl[a.name], graph)
    siv = places.rel(pose, stand)
    if not (0.4 < siv[0] < 2.0 and abs(siv[1]) < 0.5 and abs(siv[2]) < math.radians(25)):
        print(f"보는 자리가 맞지 않음: 설 자리가 앞 {siv[0]:.2f} m, 왼 {siv[1]:+.2f} m, {math.degrees(siv[2]):+.0f}° 에 있음 "
              "(설 자리 뒤 0.4~2 m 에서 선반을 정면으로 봐야 함)")
        return 1
    cur = sc.grab(n, mc)
    if cur is None:
        print("영상·depth·카메라 TF 를 못 받음")
        return 1
    gate = sc.find_gate(sc.apply_inv(siv, sc.cloud(cur["depth"], cur["K"], cur["T"])), a.shelf_width, a.shelf_depth)
    if gate is None:
        print(f"설 자리 앞에서 폭 {a.shelf_width} m 쯤 되는 키 큰(0.8 m 넘는) 물체를 못 찾음 — 선반이 화면에 들어오는지, --shelf-width 가 맞는지 확인")
        return 1
    d = path.with_suffix("")
    d.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(d / f"{a.name}_view_rgb.jpg"), cur["bgr"], [cv2.IMWRITE_JPEG_QUALITY, 95])
    cv2.imwrite(str(d / f"{a.name}_view_depth.png"), (cur["depth"] * 1000.0).clip(0, 65535).astype(np.uint16))
    pl[a.name]["view"] = {
        "pose": [round(pose[0], 3), round(pose[1], 3), round(math.degrees(pose[2]), 1)],
        "anchors": places.make_anchors(graph, pose),
        "stand_in_view": [round(siv[0], 4), round(siv[1], 4), round(math.degrees(siv[2]), 2)],
        "gate": gate,
        "shelf": [a.shelf_width, a.shelf_depth],           # 입력한 선반 폭·깊이
        "image": f"{d.name}/{a.name}_view_rgb.jpg",
        "depth": f"{d.name}/{a.name}_view_depth.png",
        "K": [round(float(v), 3) for v in cur["K"]],
        "T_base_cam": [round(float(v), 5) for v in cur["T"].ravel()],
    }
    places.save(path, pl)
    ref = sc.load_ref(pl[a.name], path)
    print(f"'{a.name}' 보는 자리 저장 → {path}")
    print(f"  보는 자리 map ({pose[0]:.2f}, {pose[1]:.2f}, {math.degrees(pose[2]):.0f}°), 설 자리는 앞 {siv[0]:.2f} m, 왼 {siv[1]:+.2f} m")
    xf, _, wd = sc.shelf_box(gate)
    yc, gap = sc.shelf_yc(gate), xf - sc.ROBOT_FRONT
    print(f"  선반: 설 자리에서 앞면까지 {xf:.2f} m (차체 앞에서 {gap * 100:.0f} cm), 폭 {wd:.2f} m, "
          f"설 자리 축에서 {'왼' if yc > 0 else '오른'}쪽으로 {abs(yc) * 100:.0f} cm 치우침")
    print(f"  선반 점 {len(ref['pts'])} 개 (판 {len(ref['levels'])} 장, 앞 기둥 {('사진 선으로 찾음' if ref['posts_img'] else 'depth 로 찾음') if ref['posts'] else '못 찾음'}), 특징점 {len(ref['fp'])} 개")
    if abs(wd - a.shelf_width) > 0.12:
        print(f"  경고: 찾은 폭 {wd:.2f} m 가 입력 {a.shelf_width} m 와 다름 — 옆 물건이 붙어 있거나 다른 것을 잡았을 수 있음")
    if gap > 0.25 or abs(yc) > 0.15:
        print("  경고: 설 자리가 선반에서 멀거나 가운데가 아님 — 팔이 닿는 자리면 그대로 두고, 아니면 설 자리부터 다시 저장 (place_mark.py 이름)")
    if len(ref["fp"]) < 30:
        print("  경고: 특징점이 적음 — 선반에 무늬 있는 물건이 안 보이면 모양으로만 확인하게 됨")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("name", nargs="?", help="장소 이름 (예: shelf_A)")
    ap.add_argument("--kind", default="shelf", help="종류 (기본 shelf)")
    ap.add_argument("--movable", action="store_true", help="옮겨질 수 있는 물체 (static: false)")
    ap.add_argument("--note", default="", help="메모")
    ap.add_argument("--file", help="places.yaml 경로 (기본: RTAB-Map 지도 옆)")
    ap.add_argument("--no-image", action="store_true", help="ZED 사진 저장 안 함")
    ap.add_argument("--view", action="store_true",
                    help="이미 저장한 장소에 '보는 자리' 추가: 설 자리에서 0.8~1.2 m 물러나 선반을 정면으로 보고 실행 (place_go.py 가 씀)")
    ap.add_argument("--side", choices=["left", "right"],
                    help="이미 저장한 장소에 '옆면 설 자리' 를 직접 저장 (정면이 막혔을 때 place_go.py 가 씀. 안 하면 선반 크기로 자동 계산)")
    ap.add_argument("--shelf-width", type=float, default=0.6, help="[--view] 선반 폭 [m]")
    ap.add_argument("--shelf-depth", type=float, default=0.33, help="[--view] 선반 깊이 [m] (뒤 벽을 빼려고)")
    ap.add_argument("--list", action="store_true", help="저장된 장소 보기")
    ap.add_argument("--delete", metavar="NAME", help="장소 지우기")
    a = ap.parse_args()
    if not (a.name or a.list or a.delete):
        ap.error("이름, --list, --delete 중 하나가 필요")

    rclpy.init()
    n = Node("place_mark")
    mc = places.MapClient(n)
    path = Path(a.file).expanduser() if a.file else None
    if path is None:
        db = mc.db_path()
        if not db:
            print("RTAB-Map 에서 지도 경로를 못 얻음 — slam.launch 가 떠 있나? (또는 --file 로 지정)")
            return 1
        path = places.places_path(db)
    pl = places.load(path)

    if a.delete:
        if a.delete not in pl:
            print(f"'{a.delete}' 없음. 있는 것: {', '.join(pl) or '(없음)'}")
            return 1
        pl.pop(a.delete)
        places.save(path, pl)
        print(f"'{a.delete}' 지움 → {path}")
        return 0

    graph = mc.graph()
    if a.list:
        print(f"{path}  ({len(pl)} 개, 지도 노드 {len(graph)} 개)")
        for k, v in pl.items():
            (x, y, yaw), how = places.resolve(v, graph)
            sp = v["stand_pose"]
            moved = math.hypot(x - sp[0], y - sp[1])
            print(f"  {k:14s} {v.get('kind', '?'):8s} map ({x:6.2f}, {y:6.2f}, {math.degrees(yaw):5.0f}°)  {how}"
                  + (f", 저장 때보다 {moved * 100:.0f} cm 옮겨짐 (지도 보정)" if moved > 0.02 else ""))
        return 0

    pose = mc.robot_pose()
    if pose is None:
        print("TF map→base_footprint 없음 — slam.launch 가 떠 있고 ZED 가 켜져 있나?")
        return 1
    if not graph:
        print("경고: 지도 노드를 못 얻음 (get_map_data) — 좌표만 저장. 루프 클로저로 지도가 보정되면 어긋날 수 있음")
    if a.view:
        return mark_view(n, mc, a, path, pl, graph, pose)
    if a.side:
        if a.name not in pl:
            print(f"'{a.name}' 가 아직 없음 — 먼저 정면 설 자리에서 place_mark.py {a.name}")
            return 1
        d = places.rel(places.resolve(pl[a.name], graph)[0], pose)
        pl[a.name].setdefault("sides", {})[a.side] = [round(d[0], 3), round(d[1], 3), round(math.degrees(d[2]), 1)]
        places.save(path, pl)
        print(f"'{a.name}' 옆면({a.side}) 설 자리 저장: 정면 설 자리 기준 앞 {d[0]:.2f} m, 왼 {d[1]:+.2f} m, {math.degrees(d[2]):+.0f}°")
        return 0
    entry = {
        "kind": a.kind,
        "static": not a.movable,
        "stand_pose": [round(pose[0], 3), round(pose[1], 3), round(math.degrees(pose[2]), 1)],
        "anchors": places.make_anchors(graph, pose),
        "last_seen": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "source": "manual",
    }
    if a.note:
        entry["note"] = a.note
    if not a.no_image:
        entry.update(snapshot(n, path.with_suffix(""), a.name))   # <지도>.places/
    old = pl.get(a.name)
    pl[a.name] = entry
    places.save(path, pl)
    print(f"'{a.name}' 저장 → {path}")
    print(f"  설 자리 map ({pose[0]:.2f}, {pose[1]:.2f}, {math.degrees(pose[2]):.0f}°)")
    if entry["anchors"]:
        a0 = entry["anchors"][0]
        print(f"  기준 노드 {[x['node'] for x in entry['anchors']]} (가장 가까운 노드에서 {math.hypot(*a0['offset'][:2]) * 100:.0f} cm)")
    print(f"  사진: {entry.get('image', '없음')}, depth: {entry.get('depth', '없음')}")
    if old:
        o = old["stand_pose"]
        print(f"  (같은 이름을 덮어씀 — 이전 ({o[0]:.2f}, {o[1]:.2f}, {o[2]:.0f}°), 직전 파일은 {path.with_suffix('.yaml.bak').name})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
