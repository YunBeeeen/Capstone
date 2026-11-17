#!/usr/bin/env python3
"""ROS 영상 (zed_wrapper 토픽) 한 장으로 물체 찾기 — 내비 (ZED ROS) 를 켠 채로. Jetson 에서 (source ~/nav_env.sh).

  python3 tools/ros_find.py pumpkin basket --json /tmp/find.json --out /tmp/find.jpg
  python3 tools/ros_find.py --json /tmp/find.json          # 말 없이: 카메라 높이·수평면 (선반 판 높이·앞 모서리) 만
svo_find --live 와 같은 분석 (analyze) — 다른 점은 영상·깊이를 ROS 토픽에서, 카메라 기울기를 TF (base_footprint → 카메라,
zed_base_tf) 에서 받는 것. 카메라 높이는 TF 값 (리프트 최저 가정) 대신 **보이는 바닥으로 잼** — 리프트를 올려도 됨.
필요: ZED ROS (zed.launch / jetson_nav), OWLv2 서버 (docker start arm_vision_server, 포트 5578).
"""
import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from arm_kinematics.vision import floor_height  # noqa: E402
from svo_find import OPTICAL_ABOVE_SCREW, analyze  # noqa: E402

Z = "/zed/zed_node/"


def to_bgr(m):
    a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)
    enc = m.encoding.lower()
    ch = {"bgra8": 4, "rgba8": 4, "bgr8": 3, "rgb8": 3}.get(enc)
    if ch is None:
        raise SystemExit(f"영상 형식 {m.encoding} 모름")
    a = a[:, :m.width * ch].reshape(m.height, m.width, ch)
    a = a[:, :, :3]
    return np.ascontiguousarray(a[:, :, ::-1] if enc.startswith("rgb") else a)


def to_depth(m):
    if m.encoding.upper() not in ("32FC1",):
        raise SystemExit(f"깊이 형식 {m.encoding} — 32FC1 이어야 함")
    d = np.frombuffer(m.data, np.float32).reshape(m.height, m.step // 4)[:, :m.width].copy()
    d[~np.isfinite(d)] = 0.0
    return d


def quat_to_R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def grab(timeout=8.0):
    """(bgr, depth, K, T 광학→base_footprint 4×4) 한 벌. 영상·깊이 스탬프가 0.1 s 안으로 맞는 것."""
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image
    from tf2_ros import Buffer, TransformListener
    rclpy.init()
    n = rclpy.create_node("ros_find")
    got = {}
    n.create_subscription(Image, Z + "rgb/color/rect/image", lambda m: got.__setitem__("rgb", m), qos_profile_sensor_data)
    n.create_subscription(Image, Z + "depth/depth_registered", lambda m: got.__setitem__("depth", m), qos_profile_sensor_data)
    n.create_subscription(CameraInfo, Z + "rgb/color/rect/camera_info", lambda m: got.__setitem__("info", m),
                          qos_profile_sensor_data)
    tf = Buffer()
    TransformListener(tf, n)
    t0, out = time.time(), None
    while time.time() - t0 < timeout and out is None:
        rclpy.spin_once(n, timeout_sec=0.05)
        if len(got) < 3:
            continue
        st = lambda m: m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec  # noqa: E731
        if abs(st(got["rgb"]) - st(got["depth"])) > 0.1:
            continue
        try:
            tr = tf.lookup_transform("base_footprint", got["depth"].header.frame_id, rclpy.time.Time()).transform
        except Exception:
            continue
        T = np.eye(4)
        T[:3, :3] = quat_to_R(tr.rotation)
        T[:3, 3] = [tr.translation.x, tr.translation.y, tr.translation.z]
        k = got["info"].k
        out = (to_bgr(got["rgb"]), to_depth(got["depth"]), (k[0], k[4], k[2], k[5]), T, got["depth"].header.frame_id)
    n.destroy_node()
    rclpy.shutdown()
    if out is None:
        missing = [k for k in ("rgb", "depth", "info") if k not in got]
        raise SystemExit(f"영상·깊이·TF 를 못 받음 (없는 것: {missing or 'TF base_footprint→카메라'}) — ZED ROS·zed_base_tf 가 떠 있나?")
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("words", nargs="*", help="찾을 말 (없으면 카메라 높이·수평면만)")
    ap.add_argument("--zed-height", default="auto", help="바닥 → ZED 삼각대 나사 [m], 기본 auto = 보이는 바닥으로 잼")
    ap.add_argument("--url", default="http://127.0.0.1:5578")
    ap.add_argument("--no-sam", action="store_true")
    ap.add_argument("--min-score", type=float, default=0.15)
    ap.add_argument("--grasp-depth", type=float, default=0.025)
    ap.add_argument("--out", type=Path)
    ap.add_argument("--json", type=Path)
    args = ap.parse_args()
    t_start = time.time()
    bgr, depth, K, T_tf, frame = grab()
    if args.zed_height == "auto":
        Tz = T_tf.copy()
        Tz[2, 3] = 0.0
        h = floor_height(depth, K, Tz)
        if h is None:
            print("⚠ 바닥이 충분히 안 보여 카메라 높이를 못 잼 — 바닥이 보이게 하거나 --zed-height <줄자 m>")
            return 2
    else:
        h = float(args.zed_height) + OPTICAL_ABOVE_SCREW
    T0 = T_tf.copy()
    T0[2, 3] = h
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, T_tf[2, 2] * -1))))      # 광축 (z) 이 아래로 기운 각
    head = (f"\n{time.strftime('%H:%M:%S')} ROS {frame}, {bgr.shape[1]}×{bgr.shape[0]}, TF 숙임 {pitch:.1f}°, "
            f"카메라 높이 {h:.3f} m" + (" (바닥으로 잼, TF 는 %.3f)" % T_tf[2, 3] if args.zed_height == "auto" else ""))
    analyze(bgr, depth, K, T0, h, args, head, t_start)
    return 0


if __name__ == "__main__":
    sys.exit(main())
