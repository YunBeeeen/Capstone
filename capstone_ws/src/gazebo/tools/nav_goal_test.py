#!/usr/bin/env python3
"""Nav2 목표 시험 — localization 모드(저장 지도)에서 목표(world 좌표)로 보내고, 도착 결과를 참값과 비교.

  python3 nav_goal_test.py <출력디렉터리> <goal_x> <goal_y> <goal_yaw> [map_origin_x map_origin_y map_origin_yaw]
  goal 은 school_floor world 좌표. map 프레임 = 1 일차 매핑 시작 포즈 → map_origin (world) 로 변환해서 보냄
  (기본 1.25 1.25 0 = 스폰 기본값).
기록: 결과(성공/실패), 걸린 시간, 경로 길이, 도착 위치·방향 오차(참값), 장애물(참값 궤적과 월드 물체) 최소 거리는 생략.
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry


def yaw_q(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def main():
    out = sys.argv[1]
    gx, gy, gyaw = (float(v) for v in sys.argv[2:5])
    ox, oy, oyaw = (float(v) for v in sys.argv[5:8]) if len(sys.argv) >= 8 else (1.25, 1.25, 0.0)
    rclpy.init()
    nav = BasicNavigator()
    nav.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
    gt = {}

    def on_gt(m):
        p = m.pose.pose
        gt["p"] = (p.position.x, p.position.y, yaw_q(p.orientation))
    nav.create_subscription(Odometry, "sim/ground_truth", on_gt, 10)
    nav.waitUntilNav2Active(localizer="bt_navigator")  # AMCL 안 씀 (RTAB-Map 이 map→odom)

    # world → map (1 일차 시작 포즈 기준)
    c, s = math.cos(oyaw), math.sin(oyaw)
    mx, my = c * (gx - ox) + s * (gy - oy), -s * (gx - ox) + c * (gy - oy)
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = nav.get_clock().now().to_msg()
    goal.pose.position.x, goal.pose.position.y = mx, my
    myaw = wrap(gyaw - oyaw)
    goal.pose.orientation.z, goal.pose.orientation.w = math.sin(myaw / 2), math.cos(myaw / 2)
    while "p" not in gt:
        rclpy.spin_once(nav, timeout_sec=0.1)
    start = gt["p"]
    print(f"출발 (참값 world) ({start[0]:.2f}, {start[1]:.2f}, {math.degrees(start[2]):.0f}°) → "
          f"목표 world ({gx:.2f}, {gy:.2f}, {math.degrees(gyaw):.0f}°) = map ({mx:.2f}, {my:.2f})")

    t0 = time.monotonic()
    nav.goToPose(goal)
    path_len, last = 0.0, start
    while not nav.isTaskComplete():
        rclpy.spin_once(nav, timeout_sec=0.1)
        p = gt["p"]
        path_len += math.hypot(p[0] - last[0], p[1] - last[1])
        last = p
        if time.monotonic() - t0 > 300:
            nav.cancelTask()
            break
    res = nav.getResult()
    dt = time.monotonic() - t0
    for _ in range(10):
        rclpy.spin_once(nav, timeout_sec=0.05)
    p = gt["p"]
    ep = math.hypot(p[0] - gx, p[1] - gy)
    ey = math.degrees(wrap(p[2] - gyaw))
    name = {TaskResult.SUCCEEDED: "성공", TaskResult.FAILED: "실패", TaskResult.CANCELED: "취소(시간 초과)"}.get(res, str(res))
    line = (f"결과 {name}: {dt:.0f} s, 주행 {path_len:.1f} m, 도착 참값 ({p[0]:.2f}, {p[1]:.2f}, {math.degrees(p[2]):.0f}°), "
            f"위치 오차 {ep * 100:.1f} cm, 방향 오차 {ey:+.1f}°")
    print(line)
    open(f"{out}/nav_result.txt", "w").write(line + "\n")
    nav.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
