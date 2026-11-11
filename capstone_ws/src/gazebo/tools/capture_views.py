"""시험용: 로봇을 여러 지점으로 순간이동 → ZED 흉내 RGB / depth 저장 + 토픽 주기 측정.
  world.launch.py 를 띄운 상태에서: python3 capture_views.py <출력디렉터리>
"""
import math
import sys
import time

import numpy as np
import rclpy
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from nav_msgs.msg import Odometry
from PIL import Image as PIL
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, PointCloud2

OUT = sys.argv[1]
VIEWS = [  # 이름, x, y, yaw
    ("01_spawn_south_corridor_east", 1.25, 1.25, 0.0),
    ("02_admin_door_north", 5.5, 1.3, math.pi / 2),
    ("03_admin_inside_desks", 5.5, 3.6, math.pi / 2),
    ("04_admin_docs_desk", 3.4, 5.2, 0.6),
    ("05_lab_from_door_south", 10.5, 10.6, -math.pi / 2),
    ("06_lounge_inside", 15.0, 3.6, math.pi / 2),
    ("07_west_corridor_elevator", 1.8, 4.6, 2.2),
    ("08_east_white_corridor", 18.75, 3.0, math.pi / 2),
    ("09_north_corridor_west", 18.5, 10.75, math.pi),
    ("10_south_corridor_lockers", 10.0, 1.4, -0.3),
]


class Cap(Node):
    def __init__(self):
        super().__init__("capture_views")
        self.rgb = self.depth = None
        self.count = {"rgb": 0, "depth": 0, "imu": 0, "pc": 0, "odom": 0, "gt": 0}
        ns = "/zed/zed_node/"
        self.create_subscription(Image, ns + "rgb/color/rect/image", self.on_rgb, 5)
        self.create_subscription(Image, ns + "depth/depth_registered", self.on_depth, 5)
        self.create_subscription(Imu, ns + "imu/data", lambda m: self.inc("imu"), 50)
        self.create_subscription(PointCloud2, ns + "point_cloud/cloud_registered", lambda m: self.inc("pc"), 5)
        self.create_subscription(Odometry, "odom", lambda m: self.inc("odom"), 50)
        self.create_subscription(Odometry, "sim/ground_truth", lambda m: self.inc("gt"), 50)
        self.cli = self.create_client(SetEntityState, "/gazebo/set_entity_state")

    def inc(self, k):
        self.count[k] += 1

    def on_rgb(self, m):
        self.inc("rgb")
        self.rgb = m

    def on_depth(self, m):
        self.inc("depth")
        self.depth = m


def main():
    rclpy.init()
    n = Cap()
    t0 = time.time()
    while time.time() - t0 < 30 and (n.rgb is None or not n.cli.service_is_ready()):
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.rgb is None:
        print("RGB 안 들어옴", n.count)
        return
    for k in n.count:
        n.count[k] = 0
    t0 = time.time()
    while time.time() - t0 < 4.0:
        rclpy.spin_once(n, timeout_sec=0.05)
    print("주기 [Hz]:", {k: round(v / 4.0, 1) for k, v in n.count.items()})
    m = n.rgb
    print(f"RGB {m.width}x{m.height} {m.encoding} frame={m.header.frame_id}; depth {n.depth.encoding}")
    for name, x, y, yaw in VIEWS:
        req = SetEntityState.Request()
        st = EntityState()
        st.name = "robot"
        st.pose.position.x, st.pose.position.y, st.pose.position.z = x, y, 0.0
        st.pose.orientation.z, st.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        st.reference_frame = "world"
        req.state = st
        fut = n.cli.call_async(req)
        rclpy.spin_until_future_complete(n, fut, timeout_sec=5)
        n.rgb = None
        t1 = time.time()
        while time.time() - t1 < 1.5:  # 새 위치 프레임
            rclpy.spin_once(n, timeout_sec=0.05)
        m = n.rgb
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
        d = np.frombuffer(n.depth.data, np.float32).reshape(n.depth.height, n.depth.width)
        dv = np.nan_to_num(d, nan=0, posinf=0)
        dn = (np.clip(dv / 8.0, 0, 1) * 255).astype(np.uint8)
        both = np.concatenate([img, np.stack([dn] * 3, -1)], 1)
        PIL.fromarray(both).save(f"{OUT}/{name}.png")
        valid = np.isfinite(d) & (d > 0)
        print(f"{name}: depth 유효 {valid.mean() * 100:.0f}%, 범위 {np.nanmin(np.where(valid, d, np.nan)):.2f}~"
              f"{np.nanmax(np.where(valid, d, np.nan)):.2f} m")
    rclpy.shutdown()


main()
