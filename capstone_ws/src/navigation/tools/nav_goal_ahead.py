#!/usr/bin/env python3
"""지금 로봇 위치 기준 상대 목표로 Nav2 보내기 (RViz 클릭 대신). map 프레임으로 바꿔서 NavigateToPose.

  python3 src/navigation/tools/nav_goal_ahead.py 1.5            # 앞 1.5 m (방향 그대로)
  python3 src/navigation/tools/nav_goal_ahead.py 1.0 0.5 90     # 앞 1.0, 왼 0.5 m, 왼쪽으로 90° 돈 방향
  python3 src/navigation/tools/nav_goal_ahead.py --abs 3.2 -1.0 180   # map 좌표 (x, y, yaw°) 그대로
진행 중 남은 거리를 1 초마다 출력. Ctrl+C = 목표 취소 (로봇은 가상 패드 N 을 떼면 바로 멈춤).
"""
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("x", type=float, help="앞 [m] (--abs 면 map x)")
    ap.add_argument("y", type=float, nargs="?", default=0.0, help="왼 [m] (--abs 면 map y)")
    ap.add_argument("yaw", type=float, nargs="?", default=0.0, help="회전 [deg] (--abs 면 map yaw)")
    ap.add_argument("--abs", action="store_true", help="map 좌표로 직접")
    a = ap.parse_args()
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
            cur = (tr.translation.x, tr.translation.y, 2 * math.atan2(tr.rotation.z, tr.rotation.w))
        except Exception:
            pass
    if cur is None:
        print("TF map→base_footprint 없음 — slam.launch(재위치)가 떠 있나?")
        return
    if a.abs:
        gx, gy, gyaw = a.x, a.y, math.radians(a.yaw)
    else:
        c, s = math.cos(cur[2]), math.sin(cur[2])
        gx, gy, gyaw = cur[0] + c * a.x - s * a.y, cur[1] + s * a.x + c * a.y, cur[2] + math.radians(a.yaw)
    print(f"지금 map ({cur[0]:.2f}, {cur[1]:.2f}, {math.degrees(cur[2]):.0f}°) → 목표 ({gx:.2f}, {gy:.2f}, {math.degrees(gyaw):.0f}°)")
    cli = ActionClient(n, NavigateToPose, "navigate_to_pose")
    if not cli.wait_for_server(timeout_sec=10.0):
        print("navigate_to_pose 액션 없음 — nav.launch 가 떠 있나?")
        return
    g = NavigateToPose.Goal()
    g.pose = PoseStamped()
    g.pose.header.frame_id = "map"
    g.pose.header.stamp = n.get_clock().now().to_msg()
    g.pose.pose.position.x, g.pose.pose.position.y = gx, gy
    g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(gyaw / 2), math.cos(gyaw / 2)
    fb = {}
    fut = cli.send_goal_async(g, feedback_callback=lambda m: fb.__setitem__("d", m.feedback.distance_remaining))
    rclpy.spin_until_future_complete(n, fut)
    h = fut.result()
    if not h.accepted:
        print("목표 거절됨")
        return
    print("목표 수락 — 가상 패드 창에서 N 을 누르고 있어야 움직임")
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
        return
    st = {4: "성공", 5: "취소됨", 6: "실패(중단)"}.get(res.result().status, str(res.result().status))
    try:
        tr = tf.lookup_transform("map", "base_footprint", rclpy.time.Time()).transform
        e = math.hypot(tr.translation.x - gx, tr.translation.y - gy)
        print(f"결과: {st}, 목표까지 남은 오차 {e * 100:.0f} cm (map 기준)")
    except Exception:
        print(f"결과: {st}")


if __name__ == "__main__":
    main()
