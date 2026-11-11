#!/usr/bin/env python3
"""RTAB-Map 재위치(localization)·루프 클로저 감시 — slam.launch 를 띄운 상태에서 실행. 1 Hz 로 한 줄씩.

  python3 src/navigation/tools/rtabmap_loc_monitor.py

열:
  hyp      = 지도에서 가장 비슷한 장면(노드)과의 유사도 (Rtabmap/LoopThr 0.11 넘어야 후보)
  matches / inliers = 그 후보와 특징점 매칭 수 / 기하 검증 통과 수 (Vis/MinInliers 25 넘어야 채택)
  결과     = 위치 찾음(→ 노드 id) / 후보 거절 (비슷해 보였지만 기하 검증 실패) / 비슷한 장면 없음 /
             정지 — 비교 안 함 (로봇이 거의 안 움직이면 RTAB-Map 이 새로 비교하지 않음, 정상)
  map 위치 = TF map→base_footprint (위치를 찾으면 지도 좌표로 점프)
"""
import math

import rclpy
from rclpy.node import Node
from rtabmap_msgs.msg import Info
from tf2_ros import Buffer, TransformListener


class Mon(Node):
    def __init__(self):
        super().__init__("rtabmap_loc_monitor")
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.found = 0
        self.create_subscription(Info, "/rtabmap/info", self.cb, 10)

    def cb(self, m):
        s = dict(zip(m.stats_keys, m.stats_values))
        hyp = s.get("Loop/Highest_hypothesis_value/", 0.0)
        hid = int(s.get("Loop/Highest_hypothesis_id/", 0))
        mat = int(s.get("Loop/Visual_matches/", 0))
        inl = int(s.get("Loop/Visual_inliers/", 0))
        rej = s.get("Loop/RejectedHypothesis/", 0.0) > 0
        if m.loop_closure_id > 0 or m.proximity_detection_id > 0:
            self.found += 1
            res = f"위치 찾음 → 노드 {m.loop_closure_id or m.proximity_detection_id}" + (" (근접)" if not m.loop_closure_id else "")
        elif rej:
            res = f"후보 {hid} 거절"
        elif mat == 0 and hyp >= 0.11:
            res = "정지 — 비교 안 함"   # 로봇이 RGBD/LinearUpdate·AngularUpdate 만큼 안 움직이면 새 비교를 안 하고 이전 값이 남음
        else:
            res = "비슷한 장면 없음"
        try:
            t = self.tf.lookup_transform("map", "base_footprint", rclpy.time.Time()).transform
            yaw = 2 * math.atan2(t.rotation.z, t.rotation.w)
            pos = f"map ({t.translation.x:6.2f}, {t.translation.y:6.2f}, {math.degrees(yaw):5.0f}°)"
        except Exception:
            pos = "map TF 없음"
        print(f"노드 {m.ref_id:4d}  hyp {hyp:.3f} (→{hid:4d})  matches {mat:3d} inliers {inl:3d}  {res:22s} {pos}  [누적 찾음 {self.found}]",
              flush=True)


def main():
    rclpy.init()
    try:
        rclpy.spin(Mon())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
