#!/usr/bin/env python3
"""노트북 비상정지 — 패드를 안 쥘 때. 라파 패드 (pad_pi.sh, --nav-auto) 에 '진행/정지' 를 10 Hz 로 보냄.

  python3 estop.py                 # 라파 = raspberrypi.local (안 풀리면 192.168.1.12 / 192.168.0.12)
  python3 estop.py --to 192.168.0.12

키 (이 터미널에 포커스):
  Space / s / Enter = ■ 정지   (Nav2·패드 수동·리프트 모두 0)
  g                 = ▶ 진행   (Nav2 명령 통과)
  q / Ctrl+C        = 정지 보내고 끝 → 라파는 1 초 뒤 '끊김' 으로 Nav2 정지 유지
처음엔 정지로 시작. 라파는 이 신호를 한 번 받은 뒤부터 '진행 신호가 1 초 안에 계속 와야' Nav2 를 넘김 —
노트북이 꺼지거나 Wi-Fi 가 끊기거나 이 창을 닫으면 로봇이 멈춤. 다시 가려면 이 프로그램을 켜고 g.
"""
import argparse
import os
import select
import socket
import sys
import termios
import time
import tty
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import udp_packet as up  # noqa: E402
from nav_cmd_udp import FALLBACK, pick_dest  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--to", default="raspberrypi.local", help="라파 주소")
    ap.add_argument("--port", type=int, default=up.NAV_PORT)
    ap.add_argument("--rate", type=float, default=10.0)
    a = ap.parse_args()
    ip, name = pick_dest([a.to] + [f for f in FALLBACK if f != a.to], a.port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    dest = (ip, a.port)
    print(f"라파 {name} ({ip}:{a.port}) 로 보냄. Space = 정지, g = 진행, q = 끝\n")

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    run, seq, last_ack = False, 0, None
    period = 1.0 / a.rate

    def send(r):
        nonlocal seq
        seq += 1
        try:
            sock.sendto(up.encode_estop(seq, r), dest)
        except OSError:
            pass

    try:
        tty.setcbreak(fd)
        nxt = 0.0
        while True:
            r, _, _ = select.select([fd], [], [], 0.02)
            if r:
                k = os.read(fd, 1)
                if k in (b" ", b"s", b"S", b"\n", b"\r"):
                    run = False
                    send(False)
                elif k in (b"g", b"G"):
                    run = True
                elif k in (b"q", b"Q", b"\x03"):
                    break
            while True:
                try:
                    data, _ = sock.recvfrom(64)
                except (BlockingIOError, OSError):
                    break
                if up.decode_estop_ack(data) is not None:
                    last_ack = time.monotonic()
            now = time.monotonic()
            if now >= nxt:
                send(run)
                nxt = now + period
                age = now - last_ack if last_ack else None
                link = "라파 응답 없음 ⚠ (pad_pi.sh 켜졌나?)" if age is None or age > 1.0 else f"라파 연결 OK ({age * 1000:.0f} ms)"
                state = "\033[42;30m ▶ 진행 \033[0m" if run else "\033[41;97m ■ 정지 \033[0m"
                print(f"\r{state}  {link}          ", end="", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(5):
            send(False)
            time.sleep(0.02)
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print("\n정지 보내고 끝 (라파는 1 초 뒤 '끊김' → Nav2 정지 유지)")


if __name__ == "__main__":
    main()
