#!/usr/bin/env python3
"""기억한 장소 여러 곳을 차례로 돌기 — slam.launch (재위치) + nav.launch 가 떠 있을 때.

  python3 src/navigation/tools/place_tour.py shelf_A desk_1              # 선반 → 책상 한 번
  python3 src/navigation/tools/place_tour.py shelf_A desk_1 --loops 3    # 선반 → 책상 → 선반 → ... 3 바퀴
  python3 src/navigation/tools/place_tour.py shelf_A desk_1 --loops 0    # Ctrl+C 까지 계속
  python3 src/navigation/tools/place_tour.py shelf_A desk_1 --wait 5     # 도착할 때마다 5 초 쉼 (팔 작업 자리)
  python3 src/navigation/tools/place_tour.py shelf_A desk_1 --dry        # 목표 좌표만 출력

가는 동안 가상 패드 N (게임패드 RT) 을 누르고 있어야 움직임. 한 곳이라도 실패하면 멈춤 (--keep-going 이면 다음으로).
장소마다 출발 직전에 지도 노드 위치로 좌표를 다시 계산 (도는 동안 지도가 보정돼도 따라감).
"""
import argparse
import math
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

sys.path.insert(0, str(Path(__file__).resolve().parent))
import places  # noqa: E402


def go(n, cli, mc, name, goal):
    """목표 하나 보내고 끝날 때까지 기다림. 반환 (성공 여부, 남은 오차 [m] 또는 None)."""
    gx, gy, gyaw = goal
    g = NavigateToPose.Goal()
    g.pose = PoseStamped()
    g.pose.header.frame_id = "map"
    g.pose.header.stamp = n.get_clock().now().to_msg()
    g.pose.pose.position.x, g.pose.pose.position.y = gx, gy
    g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(gyaw / 2), math.cos(gyaw / 2)
    fb = {}
    h = places.send_goal(n, cli, g, lambda m: fb.__setitem__("d", m.feedback.distance_remaining))
    if h is None or not h.accepted:
        print("  목표 거절됨")
        return False, None
    res = h.get_result_async()
    last = time.time()
    try:
        while not res.done():
            rclpy.spin_once(n, timeout_sec=0.1)
            if time.time() - last > 2.0 and "d" in fb:
                last = time.time()
                print(f"  {name}: 남은 거리 {fb['d']:.2f} m", flush=True)
    except KeyboardInterrupt:
        h.cancel_goal_async()
        rclpy.spin_once(n, timeout_sec=0.3)
        raise
    ok = res.result().status == 4
    cur = mc.robot_pose(timeout=1.0)
    return ok, (math.hypot(cur[0] - gx, cur[1] - gy) if cur else None)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("names", nargs="+", help="place_mark.py 로 기억한 장소 이름 (도는 순서)")
    ap.add_argument("--loops", type=int, default=1, help="바퀴 수 (0 = 계속, 기본 1)")
    ap.add_argument("--wait", type=float, default=2.0, help="도착 후 쉬는 시간 [s]")
    ap.add_argument("--places", help="places.yaml 경로 (기본: RTAB-Map 지도 옆)")
    ap.add_argument("--keep-going", action="store_true", help="한 곳 실패해도 다음으로")
    ap.add_argument("--dry", action="store_true", help="목표만 출력하고 보내지 않음")
    a = ap.parse_args()
    rclpy.init()
    n = Node("place_tour")
    mc = places.MapClient(n)
    path = Path(a.places).expanduser() if a.places else None
    if path is None:
        db = mc.db_path()
        if not db:
            print("RTAB-Map 에서 지도 경로를 못 얻음 — slam.launch 가 떠 있나? (또는 --places 로 지정)")
            return 1
        path = places.places_path(db)
    pl = places.load(path)
    miss = [x for x in a.names if x not in pl]
    if miss:
        print(f"{path} 에 없음: {', '.join(miss)}. 있는 것: {', '.join(pl) or '(없음)'}")
        return 1

    def target(name):
        (x, y, yaw), how = places.resolve(pl[name], mc.graph())
        if pl[name].get("anchors") and "기준 노드 없음" in how:
            print(f"  {name}: 기준 노드가 지금 지도에 없음 → 보내지 않음 (다른 지도이거나 옛 지도와 아직 안 이어짐)")
            return None
        return x, y, yaw

    if a.dry:
        for name in a.names:
            t = target(name)
            if t:
                print(f"  {name:14s} map ({t[0]:6.2f}, {t[1]:6.2f}, {math.degrees(t[2]):5.0f}°)")
        return 0
    cli = ActionClient(n, NavigateToPose, "navigate_to_pose")
    if not cli.wait_for_server(timeout_sec=10.0):
        print("navigate_to_pose 액션 없음 — nav.launch 가 떠 있나?")
        return 1
    print(f"{' → '.join(a.names)}, {'계속' if a.loops == 0 else f'{a.loops} 바퀴'}. 가상 패드 N (게임패드 RT) 을 누르고 있어야 움직임. Ctrl+C = 멈춤")
    lap, done, fail, errs = 0, 0, 0, []
    try:
        while a.loops == 0 or lap < a.loops:
            lap += 1
            for name in a.names:
                t = target(name)
                if t is None:
                    return 1
                print(f"[{lap} 바퀴] → {name} ({t[0]:.2f}, {t[1]:.2f}, {math.degrees(t[2]):.0f}°)")
                t0 = time.time()
                ok, err = go(n, cli, mc, name, t)
                es = f", 목표까지 {err * 100:.0f} cm (map 기준)" if err is not None else ""
                print(f"  {name}: {'도착' if ok else '실패'} ({time.time() - t0:.0f} s{es})")
                if ok:
                    done += 1
                    if err is not None:
                        errs.append(err)
                else:
                    fail += 1
                    if not a.keep_going:
                        return 1
                time.sleep(a.wait)
    except KeyboardInterrupt:
        print("\n멈춤 (목표 취소)")
    finally:
        if errs:
            print(f"도착 {done} 회, 실패 {fail} 회. map 기준 남은 오차 평균 {sum(errs) / len(errs) * 100:.0f} cm, 최대 {max(errs) * 100:.0f} cm")
    return 0


if __name__ == "__main__":
    sys.exit(main())
