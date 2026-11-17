#!/usr/bin/env python3
"""지금 로봇 위치 기준 상대 목표로 Nav2 보내기 (RViz 클릭 대신). map 프레임으로 바꿔서 NavigateToPose.

  python3 src/navigation/tools/nav_goal_ahead.py 1.5            # 앞 1.5 m (방향 그대로)
  python3 src/navigation/tools/nav_goal_ahead.py 1.0 0.5 90     # 앞 1.0, 왼 0.5 m, 왼쪽으로 90° 돈 방향
  python3 src/navigation/tools/nav_goal_ahead.py --abs 3.2 -1.0 180   # map 좌표 (x, y, yaw°) 그대로
  python3 src/navigation/tools/nav_goal_ahead.py --name shelf_A       # place_mark.py 로 기억한 자리 (지도 옆 places.yaml)
  python3 src/navigation/tools/nav_goal_ahead.py --name shelf_A --dry # 목표만 계산해 보여 주고 안 보냄
진행 중 남은 거리를 1 초마다 출력. Ctrl+C = 목표 취소.
도착할 때까지 안 끝남 (sh 에서 쓰기 좋게). 종료 코드: 0 = 도착 (Nav2 성공), 1 = 실패·취소·거절, 2 = 준비 안 됨 (TF·지도·Nav2·장소 없음), 130 = Ctrl+C.
--dry 는 0.
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
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, str(Path(__file__).resolve().parent))
import places  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("x", type=float, nargs="?", help="앞 [m] (--abs 면 map x)")
    ap.add_argument("y", type=float, nargs="?", default=0.0, help="왼 [m] (--abs 면 map y)")
    ap.add_argument("yaw", type=float, nargs="?", default=0.0, help="회전 [deg] (--abs 면 map yaw)")
    ap.add_argument("--abs", action="store_true", help="map 좌표로 직접")
    ap.add_argument("--name", help="place_mark.py 로 기억한 장소 이름")
    ap.add_argument("--back", type=float, default=0.0,
                    help="--name 장소에서 같은 방향을 본 채 이만큼 뒤 [m] (예: 선반 보는 자리 = --name shelf_A --back 0.6)")
    ap.add_argument("--places", help="places.yaml 경로 (기본: RTAB-Map 지도 옆)")
    ap.add_argument("--dry", action="store_true", help="목표만 출력하고 보내지 않음")
    ap.add_argument("--force", action="store_true", help="기준 노드가 지도에 없어도 저장 좌표로 보냄")
    a = ap.parse_args()
    if a.name is None and a.x is None:
        ap.error("x 또는 --name 이 필요")
    rclpy.init()
    n = Node("nav_goal_ahead")
    tf = Buffer()
    TransformListener(tf, n)
    t0 = time.time()
    cur = None
    while time.time() - t0 < 5 and cur is None:
        rclpy.spin_once(n, timeout_sec=0.1)
        try:
            tr = tf.lookup_transform("map", "base_footprint", rclpy.time.Time()).transform
            cur = (tr.translation.x, tr.translation.y, places.yaw_q(tr.rotation))
        except Exception:
            pass
    if cur is None:
        print("TF map→base_footprint 없음 — slam.launch(재위치)가 떠 있나?")
        return 2
    if a.name:
        mc = places.MapClient(n)
        path = Path(a.places).expanduser() if a.places else None
        if path is None:
            db = mc.db_path()
            if not db:
                print("RTAB-Map 에서 지도 경로를 못 얻음 — slam.launch 가 떠 있나? (또는 --places 로 지정)")
                return 2
            path = places.places_path(db)
        pl = places.load(path)
        if a.name not in pl:
            print(f"'{a.name}' 이 {path} 에 없음. 있는 것: {', '.join(pl) or '(없음)'}")
            return 2
        (gx, gy, gyaw), how = places.resolve(pl[a.name], mc.graph())
        if a.back:
            gx, gy = gx - a.back * math.cos(gyaw), gy - a.back * math.sin(gyaw)
            how += f", {a.back:.2f} m 뒤"
        print(f"'{a.name}' ({pl[a.name].get('kind', '?')}) — {how}")
        if pl[a.name].get("anchors") and "기준 노드 없음" in how and not a.force:
            # 이어서 매핑한 새 세션이 아직 옛 지도와 안 이어졌거나 다른 지도를 띄운 경우: 저장 좌표는 지금 지도와 기준이 다를 수 있음
            print("  기준 노드가 지금 지도에 없음 → 보내지 않음. 옛 지도와 아직 안 이어졌거나(이어서 매핑 중) 다른 지도일 수 있음."
                  " 그래도 저장 좌표로 보내려면 --force")
            return 2
        if not pl[a.name].get("static", True):
            print("  주의: 옮겨질 수 있는 물체로 표시됨 — 도착 후 실제로 있는지 확인 필요")
    elif a.abs:
        gx, gy, gyaw = a.x, a.y, math.radians(a.yaw)
    else:
        c, s = math.cos(cur[2]), math.sin(cur[2])
        gx, gy, gyaw = cur[0] + c * a.x - s * a.y, cur[1] + s * a.x + c * a.y, cur[2] + math.radians(a.yaw)
    print(f"지금 map ({cur[0]:.2f}, {cur[1]:.2f}, {math.degrees(cur[2]):.0f}°) → 목표 ({gx:.2f}, {gy:.2f}, {math.degrees(gyaw):.0f}°)"
          f"  직선 {math.hypot(gx - cur[0], gy - cur[1]):.2f} m")
    if a.dry:
        return 0
    cli = ActionClient(n, NavigateToPose, "navigate_to_pose")
    if not cli.wait_for_server(timeout_sec=10.0):
        print("navigate_to_pose 액션 없음 — nav.launch 가 떠 있나?")
        return 2
    g = NavigateToPose.Goal()
    g.pose = PoseStamped()
    g.pose.header.frame_id = "map"
    g.pose.header.stamp = n.get_clock().now().to_msg()
    g.pose.pose.position.x, g.pose.pose.position.y = gx, gy
    g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(gyaw / 2), math.cos(gyaw / 2)
    fb = {}
    h = places.send_goal(n, cli, g, lambda m: fb.__setitem__("d", m.feedback.distance_remaining))
    if h is None or not h.accepted:
        print("목표 거절됨")
        return 1
    print("목표 수락 — 라파 패드: 자동이면 그냥 감 (RT = 정지, Start = 다시), 데드맨 모드면 RT 를 누르고 있기")
    res = h.get_result_async()
    last = 0.0
    try:
        while not res.done():
            rclpy.spin_once(n, timeout_sec=0.1)
            if time.time() - last > 1.0 and "d" in fb:
                last = time.time()
                print(f"  남은 거리 {fb['d']:.2f} m", flush=True)
    except KeyboardInterrupt:
        h.cancel_goal_async()
        print("취소")
        return 130
    st = {4: "성공", 5: "취소됨", 6: "실패(중단)"}.get(res.result().status, str(res.result().status))
    try:
        tr = tf.lookup_transform("map", "base_footprint", rclpy.time.Time()).transform
        e = math.hypot(tr.translation.x - gx, tr.translation.y - gy)
        print(f"결과: {st}, 목표까지 남은 오차 {e * 100:.0f} cm (map 기준)")
    except Exception:
        print(f"결과: {st}")
    return 0 if res.result().status == 4 else 1


if __name__ == "__main__":
    sys.exit(main())
