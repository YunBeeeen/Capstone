#!/usr/bin/env python3
"""기억한 자리에 없는 선반을 다시 찾기 (선반 기억 3 단계, 시뮬에서만 확인) — place_go.py 가 "없음" (종료 코드 2) 으로 끝났을 때.

  python3 src/navigation/tools/place_find.py shelf_A                       # 실기 (회전은 /nav/cmd_vel → RT 를 누르고 있어야 함)
  python3 src/navigation/tools/place_find.py shelf_A --cmd-topic /cmd_vel  # 시뮬
  python3 src/navigation/tools/place_find.py shelf_A --max-points 6        # 둘러볼 지점 수 제한
찾으면 places.yaml 의 설 자리·보는 자리를 새 위치로 고침 (종료 코드 0) → 이어서 place_go.py shelf_A 로 가서 섬. 못 찾으면 2.

방법
  1. 둘러볼 지점 = 매핑 때 지나간 지도 노드 (갈 수 있다는 게 보장됨). 2.5 m 간격으로 솎아서, 지금 자리에서 가까운 곳부터 이어서
  2. 지점마다 제자리에서 60° 씩 여섯 방향을 봄. 화면마다
       a. 저장 사진의 선반 특징점을 화면 전체에서 찾음 (shelf_check.match_feats, 4 m 안) → 맞으면 선반 위치·방향이 바로 나옴
       b. 안 맞으면 "선반처럼 생긴 것" (판 높이에 가로 구조가 겹친 것, shelf_check.find_shelf_like) 을 후보로 적어 둠.
          그 모양에서 앞면이 향한 방향도 나옴 (가로로 긴 축에 수직, 카메라 쪽)
  3. a·b 후보마다 그 선반의 정면 보는 자리로 가서 평소 확인 (shelf_check.measure_n) → 확실/보통이면 찾은 것
한계: 선반 앞면이 보여야 함 (옆·뒤에서 보면 사진이 안 맞음 → b 후보로만 잡힘), 지도에 없는 곳은 안 감, 물건이 많이 바뀌면 a 가 안 됨.
"""
import argparse
import math
import os
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

sys.path.insert(0, str(Path(__file__).resolve().parent))
import places  # noqa: E402
import shelf_check as sc  # noqa: E402
from place_go import Costmap, drain, nav_to, odom_pose, servo  # noqa: E402


def viewpoints(graph, start, spacing):
    """지도 노드 → 둘러볼 지점 목록 (x, y, yaw). spacing 간격으로 솎고 start(기억한 자리)에서 가까운 순."""
    nodes = sorted(graph.values(), key=lambda p: math.hypot(p[0] - start[0], p[1] - start[1]))
    keep = []
    for p in nodes:
        if all(math.hypot(p[0] - q[0], p[1] - q[1]) >= spacing for q in keep):
            keep.append(p)
    return keep


def turn_to(n, mc, pub, yaw):
    """제자리에서 odom 기준 yaw 만큼 더 돌기."""
    o = odom_pose(n, mc)
    if o is None:
        return False
    ok, _ = servo(n, mc, pub, (o[0], o[1], places.wrap(o[2] + yaw)), 0.05, 0.5, 0.05, math.radians(3.0), 20.0)
    return bool(ok)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("name")
    ap.add_argument("--places", help="places.yaml 경로 (기본: RTAB-Map 지도 옆)")
    ap.add_argument("--cmd-topic", default="/nav/cmd_vel", help="제자리 회전 명령 토픽 (실기 /nav/cmd_vel, 시뮬 /cmd_vel)")
    ap.add_argument("--spacing", type=float, default=2.5, help="둘러볼 지점 간격 [m]")
    ap.add_argument("--max-points", type=int, default=30, help="둘러볼 지점 수 상한")
    ap.add_argument("--around", type=float, nargs=2, metavar=("X", "Y"),
                    help="이 map 좌표 근처부터 둘러봄 (기본: 지금 로봇 자리에서 가까운 곳부터). 예: 사람이 '저쪽 방에 있을 걸' 할 때")
    ap.add_argument("--no-update", action="store_true", help="찾아도 places.yaml 을 안 고침")
    a = ap.parse_args()
    rclpy.init()
    n = Node("place_find")
    mc = places.MapClient(n)
    path = places.find_places(mc, a.places)
    if path is None:
        print("RTAB-Map 에서 지도 경로를 못 얻음 — slam.launch 가 떠 있나? (또는 --places 로 지정)")
        return 1
    pl = places.load(path)
    if a.name not in pl:
        print(f"{path} 에 '{a.name}' 없음")
        return 1
    place = pl[a.name]
    ref = sc.load_ref(place, path)
    if ref is None:
        print(f"'{a.name}' 에 보는 자리 기록이 없음 → place_mark.py {a.name} --view")
        return 1
    graph = mc.graph()
    if not graph:
        print("지도 노드를 못 얻음")
        return 1
    old_stand, _ = places.resolve(place, graph)
    siv = ref["siv"]
    c, s = math.cos(siv[2]), math.sin(siv[2])
    inv_siv = (-(c * siv[0] + s * siv[1]), -(-s * siv[0] + c * siv[1]), -siv[2])       # 설 자리 기준 보는 자리
    xf, depth, _ = sc.shelf_box(ref["gate"])
    yc = sc.shelf_yc(ref["gate"])
    old_centre = places.compose(old_stand, (xf + depth / 2, yc, 0.0))
    cm = Costmap(n)
    cli = ActionClient(n, NavigateToPose, "navigate_to_pose")
    if not cli.wait_for_server(timeout_sec=10.0):
        print("navigate_to_pose 액션 없음 — nav.launch 가 떠 있나?")
        return 1
    pub = n.create_publisher(Twist, a.cmd_topic, 10)
    pool = viewpoints(graph, old_stand, a.spacing)
    here = tuple(a.around) if a.around else (mc.robot_pose() or old_stand)
    vps = []
    while pool and len(vps) < a.max_points:                    # 가까운 다음 지점으로 이어 가기
        nxt = min(pool, key=lambda p: math.hypot(p[0] - here[0], p[1] - here[1]))
        pool.remove(nxt)
        vps.append(nxt)
        here = nxt
    print(f"둘러볼 지점 {len(vps)} 곳 (지도 노드 {len(graph)} 개에서 {a.spacing} m 간격, 지금 자리에서 가까운 곳부터 이어서)")
    rejected = [old_centre[:2]]            # 가 봤는데 아니었던 곳 (기억한 자리 포함) — 모양 후보는 여기 근처면 다시 안 감
    rejected_feat = [old_centre[:2]]       # 사진으로 잡혀서 가 봤는데 아니었던 곳. 사진 후보는 이것만 봄 —
    #  비스듬히 본 모양 후보는 위치 추정이 부정확해서 확인에 실패할 수 있는데, 그 실패로 진짜 선반 자리까지 막으면 안 됨
    #  (시뮬 2026-11-13: 옆 지점에서 모양 후보 확인 실패 → 바로 앞 지점에서 사진으로 잡은 것을 버림)
    t_start, stats = time.time(), {"frames": 0, "feat": 0, "shape": 0}

    def free(x, y):
        v = cm.cost(x, y)
        return v is not None and 0 <= v < 99

    def guesses(frame, cur):
        """화면에서 선반처럼 생긴 것 → [(중심 map, 설 자리 map)]."""
        out = []
        for x, y, ang in sc.find_shelf_like(ref, frame):
            centre = places.compose(cur, (x, y, ang))              # yaw = 앞면이 향한 방향
            out.append((centre, places.compose(centre, (xf + depth / 2, yc, math.pi))))
        return out

    def confirm(stand_m, why, by_feat=False):
        """후보 설 자리 (map) → 그 보는 자리로 가서 평소 확인. 찾으면 (새 설 자리, 결과), 아니면 None.
        비스듬히 보고 잡은 후보는 위치·방향이 40 cm·30° 쯤 틀릴 수 있어서, 도착해서 사진·모양으로 추정을 고치고 한 번 더 감 (최대 3 번)."""
        centre = places.compose(stand_m, (xf + depth / 2, yc, 0.0))
        if any(math.hypot(centre[0] - r[0], centre[1] - r[1]) < 0.6 for r in (rejected_feat if by_feat else rejected)):
            return None
        print(f"  후보 ({why}): 설 자리 map ({stand_m[0]:.2f}, {stand_m[1]:.2f}, {math.degrees(stand_m[2]):.0f}°) → 보는 자리로 가서 확인")
        for it in range(3):
            view_m = places.compose(stand_m, inv_siv)
            if not free(view_m[0], view_m[1]):
                print("    보는 자리에 갈 수 없음 (costmap)")
                break
            if not nav_to(n, cli, view_m):
                print("    Nav2 실패")
                break
            time.sleep(1.5)
            cur, frame = mc.robot_pose(), sc.grab(n, mc)
            if cur is None or frame is None:
                break
            pose, k = sc.match_feats(ref, frame, None, np.random.default_rng(it))
            init = pose if pose is not None and k >= 8 else places.rel(cur, stand_m)
            res, _ = sc.measure_n(n, mc, ref, init, 3)
            print(f"    확인: {sc.describe(res)}")
            if res["present"] and res["conf"] in ("high", "mid"):
                return places.compose(cur, res["pose"]), res
            near = [st for c, st in guesses(frame, cur) if math.hypot(c[0] - centre[0], c[1] - centre[1]) < 0.8]
            if pose is not None and k >= 8:
                better = places.compose(cur, pose)
            elif near:
                better = near[0]
            else:
                break
            if math.hypot(better[0] - stand_m[0], better[1] - stand_m[1]) < 0.05 and abs(places.wrap(better[2] - stand_m[2])) < math.radians(4):
                break                                              # 더 고칠 게 없음
            stand_m = better
            print(f"    추정 고침 → ({stand_m[0]:.2f}, {stand_m[1]:.2f}, {math.degrees(stand_m[2]):.0f}°), 다시 감")
        rejected.append(centre[:2])
        if by_feat:
            rejected_feat.append(centre[:2])
        return None

    def look(tag):
        """지금 화면 한 장: 특징점으로 바로 찾기 → 확인. 못 찾으면 선반처럼 생긴 후보 (map 좌표) 목록을 돌려줌."""
        frame = sc.grab(n, mc)
        cur = mc.robot_pose()
        if frame is None or cur is None:
            return None, []
        stats["frames"] += 1
        if os.environ.get("SHELF_DEBUG"):                 # 본 화면 저장 (원인 분석용)
            np.savez(os.path.join(os.environ["SHELF_DEBUG"], f"find_{stats['frames']:03d}.npz"), depth=frame["depth"],
                     K=np.array(frame["K"]), T=frame["T"], cur=np.array(cur), bgr=frame["bgr"])
        pose, k = sc.match_feats(ref, frame, None, np.random.default_rng(stats["frames"]))
        if pose is not None and k >= 8:
            stats["feat"] += 1
            got = confirm(places.compose(cur, pose), f"{tag}: 사진 특징점 {k} 개", by_feat=True)
            if got:
                return got, []
        shapes = [st for c, st in guesses(frame, cur) if all(math.hypot(c[0] - r[0], c[1] - r[1]) >= 0.6 for r in rejected)]
        return None, shapes

    found = None
    drain(n, 1.0)
    try:
        for i, vp in enumerate(vps):
            print(f"[{i + 1}/{len(vps)}] map ({vp[0]:.2f}, {vp[1]:.2f}) 로", flush=True)
            if not nav_to(n, cli, vp) and not (time.sleep(2.0) or nav_to(n, cli, vp)):     # 한 번 더 해 봄
                print("  Nav2 실패 — 다음 지점")
                continue
            pending = []
            for h in range(6):                    # 60° 씩 여섯 방향 (화각 110° — 90° 씩이면 선반이 화면 가장자리에 걸려 놓침)
                if h and not turn_to(n, mc, pub, math.pi / 3):
                    break
                time.sleep(0.7)
                found, shapes = look(f"지점 {i + 1}")
                if found:
                    break
                pending += [q for q in shapes if all(math.hypot(q[0] - r[0], q[1] - r[1]) >= 0.6 for r in pending)]
            for cnd in ([] if found else pending[:3]):
                stats["shape"] += 1
                found = confirm(cnd, "선반처럼 생긴 것")
                if found:
                    break
            if found:
                break
    except KeyboardInterrupt:
        print("\n멈춤")
        return 130
    dt = time.time() - t_start
    print(f"본 화면 {stats['frames']} 장, 사진으로 잡힌 후보 {stats['feat']} 번, 모양 후보 {stats['shape']} 곳, {dt:.0f} s")
    if not found:
        print("못 찾음")
        return 2
    new_stand, res = found
    moved = math.hypot(new_stand[0] - old_stand[0], new_stand[1] - old_stand[1])
    print(f"찾음: 설 자리 map ({new_stand[0]:.2f}, {new_stand[1]:.2f}, {math.degrees(new_stand[2]):.0f}°) — 기억한 자리에서 {moved:.1f} m. {sc.describe(res)}")
    if not a.no_update:
        new_view = places.compose(new_stand, inv_siv)
        place["stand_pose"] = [round(new_stand[0], 3), round(new_stand[1], 3), round(math.degrees(new_stand[2]), 1)]
        place["anchors"] = places.make_anchors(graph, new_stand)
        place["view"]["pose"] = [round(new_view[0], 3), round(new_view[1], 3), round(math.degrees(new_view[2]), 1)]
        place["view"]["anchors"] = places.make_anchors(graph, new_view)
        place["last_seen"] = time.strftime("%Y-%m-%dT%H:%M:%S")
        place["source"] = "search"
        places.save(path, pl)
        print(f"기억 고침 → {path}.  이어서: place_go.py {a.name}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
