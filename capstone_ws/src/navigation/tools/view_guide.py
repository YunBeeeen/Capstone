#!/usr/bin/env python3
"""선반 '보는 자리' 맞추기 안내 (읽기만) — place_mark.py NAME --view 전에. 재위치·매핑이 떠 있을 때.
  python3 view_guide.py shelf_A          # 설 자리에서 같은 방향을 본 채 1.0 m 뒤 = 목표
  python3 view_guide.py shelf_A 0.9      # 0.9 m 뒤
0.5 초마다 "보는 자리까지 앞·왼·돌기" 를 보여 줌 → 게임패드로 돌기부터 0 근처, 그다음 앞·왼을 0 근처로.
place_mark --view 조건 (설 자리가 앞 0.8~1.2 m, 옆 10 cm, 6° 안) 이면 ✅ → Ctrl+C 하고 place_mark.py NAME --view."""
import math, sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rclpy
from rclpy.node import Node
import places
name = sys.argv[1] if len(sys.argv) > 1 else "shelf_A"
back = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
rclpy.init(); n = Node("view_guide"); mc = places.MapClient(n)
pl = places.load(places.places_path(mc.db_path())); graph = mc.graph()
stand, _ = places.resolve(pl[name], graph)
goal = (stand[0] - back * math.cos(stand[2]), stand[1] - back * math.sin(stand[2]), stand[2])
print(f"{name} 설 자리 ({stand[0]:.2f}, {stand[1]:.2f}, {math.degrees(stand[2]):.0f}°) → 보는 자리 ({goal[0]:.2f}, {goal[1]:.2f}, 같은 방향), Ctrl+C 로 끝")
try:
    while True:
        for _ in range(5):
            rclpy.spin_once(n, timeout_sec=0.1)
        pose = mc.robot_pose()
        if pose is None:
            print("\r위치 없음 (재위치 됐나?)                                   ", end="", flush=True); continue
        r = places.rel(pose, goal)             # 로봇 기준 목표: 앞 x, 왼 y, 돌 각
        s = places.rel(pose, stand)
        ok = 0.8 < s[0] < 1.2 and abs(s[1]) < 0.10 and abs(math.degrees(s[2])) < 6
        print(f"\r보는 자리까지: 앞 {r[0]:+.2f} m, 왼 {r[1]:+.2f} m, 돌기 {math.degrees(r[2]):+4.0f}° (+ = 왼쪽으로)  "
              f"설 자리는 앞 {s[0]:.2f} m  {'✅ 여기서 place_mark --view' if ok else ''}        ", end="", flush=True)
except (KeyboardInterrupt, Exception):
    print()
