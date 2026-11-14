#!/usr/bin/env python3
"""시뮬 화면 녹화 — 구경 카메라(/spectator/cam/image_raw) + 로봇 카메라(작게) + 자막(/test/caption) → mp4.

  python3 record_view.py <출력.mp4>       # Ctrl+C (SIGINT) 로 끝내면 파일 닫음
구경 카메라: models/spectator_cam (run_shelf_test.sh 가 스폰). 10 Hz (시뮬 시간) 그대로 10 fps 로 기록.
"""
import signal
import sys

import cv2
import numpy as np
import rclpy
from PIL import Image as PImage, ImageDraw, ImageFont
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"


def to_bgr(m):
    a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step // 3, 3)[:, :m.width]
    return np.ascontiguousarray(a[:, :, ::-1] if m.encoding == "rgb8" else a)


class Rec(Node):
    def __init__(self, path):
        super().__init__("record_view")
        self.robot, self.cap, self.vw, self.path, self.n = None, "", None, path, 0
        try:
            self.font = ImageFont.truetype(FONT, 24, index=1)   # index 1 = KR
        except Exception:
            self.font = ImageFont.load_default()
        self.create_subscription(Image, "/zed/zed_node/rgb/color/rect/image", lambda m: setattr(self, "robot", m), qos_profile_sensor_data)
        self.create_subscription(String, "/test/caption", lambda m: setattr(self, "cap", m.data), 10)
        self.create_subscription(Image, "/spectator/cam/image_raw", self.on_img, qos_profile_sensor_data)

    def on_img(self, m):
        f = to_bgr(m)
        if self.robot is not None:
            r = cv2.resize(to_bgr(self.robot), (320, 180))
            f[8:188, f.shape[1] - 328:f.shape[1] - 8] = r
            cv2.rectangle(f, (f.shape[1] - 329, 7), (f.shape[1] - 7, 189), (255, 255, 255), 1)
        if self.cap:
            f[-44:] = (f[-44:] * 0.35).astype(np.uint8)
            im = PImage.fromarray(f[:, :, ::-1])
            ImageDraw.Draw(im).text((12, f.shape[0] - 38), self.cap, font=self.font, fill=(255, 255, 255))
            f = np.ascontiguousarray(np.array(im)[:, :, ::-1])
        if self.vw is None:
            self.vw = cv2.VideoWriter(self.path, cv2.VideoWriter_fourcc(*"mp4v"), 10.0, (f.shape[1], f.shape[0]))
        self.vw.write(f)
        self.n += 1


def main():
    rclpy.init()
    r = Rec(sys.argv[1])
    stop = []
    signal.signal(signal.SIGINT, lambda *_: stop.append(1))
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
    while not stop:
        rclpy.spin_once(r, timeout_sec=0.2)
    if r.vw:
        r.vw.release()
    print(f"{r.path}: {r.n} 장")


if __name__ == "__main__":
    main()
