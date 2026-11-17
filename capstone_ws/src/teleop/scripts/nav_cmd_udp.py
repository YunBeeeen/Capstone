#!/usr/bin/env python3
"""Nav2 속도 (/nav/cmd_vel) → UDP → 라파 패드 (virtual_pad.py --nav-udp 5006). **Nav2 가 도는 컴퓨터 (Jetson) 에서 실행.**

  python3 nav_cmd_udp.py                          # 보낼 곳 = $PAD_HOST (없으면 raspberrypi.local), 포트 5006
  python3 nav_cmd_udp.py --to 192.168.1.12:5006 --topic /nav/cmd_vel

왜: 게임패드를 라파에 꽂으면 패드 프로그램도 라파 (ROS Jazzy) 에서 돌아야 하는데, Nav2 는 Jetson (Humble) — 배포판이 달라
  ROS 토픽으로 주고받는 게 보장되지 않음. 그래서 Nav2 명령만 UDP 로 넘기고, RT 데드맨 판단은 그대로 라파 패드가 함.
받은 메시지를 그대로 하나씩 보냄 (따로 반복·유지 안 함) → Nav2 가 멈추거나 이 프로그램이 꺼지면 패드는 --nav-timeout (0.5 s)
  뒤 0. 1 초마다 보낸 개수를 한 줄로 보여 줌.
"""
import argparse
import os
import socket
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import udp_packet as up  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--to", default=os.environ.get("PAD_HOST", "raspberrypi.local"), help="HOST[:PORT] (기본 $PAD_HOST, 포트 5006)")
    ap.add_argument("--topic", default="/nav/cmd_vel")
    args, _ = ap.parse_known_args()
    host, _, port = args.to.partition(":")
    dest = (socket.gethostbyname(host), int(port or up.NAV_PORT))

    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.executors import ExternalShutdownException
    rclpy.init()
    node = rclpy.create_node("nav_cmd_udp")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    st = {"seq": 0, "n": 0, "last": (0.0, 0.0, 0.0)}

    def cb(m):
        st["seq"] = (st["seq"] + 1) & 0xFFFFFFFF
        st["last"] = (m.linear.x, m.linear.y, m.angular.z)
        try:
            sock.sendto(up.encode_nav(st["seq"], *st["last"]), dest)
            st["n"] += 1
        except OSError as e:
            print(f"보내기 실패: {e}")

    node.create_subscription(Twist, args.topic, cb, 10)
    print(f"{args.topic} → udp {dest[0]}:{dest[1]} (라파: virtual_pad.py --nav-udp {dest[1]}, RT 당기는 동안만 로봇에 감)")
    t_print = time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() - t_print >= 1.0:
                vx, vy, w = st["last"]
                print(f"\r{st['n']:3d} 개/s  마지막 vx {vx:+.2f} vy {vy:+.2f} ω {w:+.2f}   ", end="", flush=True)
                st["n"], t_print = 0, time.monotonic()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        print()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
