#!/usr/bin/env python3
"""화면 가상 패드 → /cmd_vel (geometry_msgs/Twist). ksrc/pi/teleop/teleop_joystick.py 의 ROS 경로만 가져옴.

  ros2 run capstone_teleop virtual_pad.py
  ros2 run capstone_teleop virtual_pad.py --max-lin 0.3 --max-ang 1.0 --topic /cmd_vel
  ros2 launch capstone_sim sim.launch.py pad:=true          # 시뮬 + 이 패드

  # 노트북 → UDP → 라파 (노트북엔 ROS 필요 없음, pygame 만). 라파: ros2 run capstone_teleop udp_teleop_bridge.py
  python3 virtual_pad.py --udp raspberrypi.local --max-lin 0.15 --max-ang 0.5

게임패드: 노트북에 꽂혀 있으면 이 창이 같이 읽음 (RB 누르는 동안 게임패드가 마우스보다 우선).
  계산은 라파 gamepad_teleop 과 같은 gamepad_map.joy_to_cmd + config/shaks_s0.yaml (축·부호·데드존·scale).
  pygame 축은 왼쪽·위 = − 라서 ROS joy 규약(왼쪽·위 = +)으로 뒤집어 넣음 → yaml 하나로 두 경로가 같은 방향.
  최종 속도는 --max-lin / --max-ang 로 다시 자름.

패드 둘 (마우스 드래그):
  왼쪽  → 병진 (vx, vy)   화면 위 = 전진, 화면 왼쪽 = 좌(+y)
  오른쪽 → 회전 ω (가로축만)  오른쪽으로 끌면 시계방향(−)
  Q / E (누르고 있는 동안) → 반시계 / 시계 회전 — 마우스로 왼쪽 패드를 끌면서 호 주행
  Space = 즉시 0,  Esc / 창 닫기 = 종료 (0 을 몇 번 보내고 끝)
  마우스를 떼거나 창 포커스를 잃으면 0.

패드에서 손을 떼도 0 을 계속 보냄 (rate Hz) → swerve_controller 는 ACTIVE 유지(속도 0 = 제자리).
창을 닫으면 발행이 멈춰서 swerve_controller 가 cmd_timeout 뒤 HOLD.

입력 다듬기 (ksrc 와 같음, joystick_math.py):
  - 원형 데드존: 중앙 근처 손떨림이 바퀴 방향을 홱홱 돌리지 않게
  - 축 스냅: 앞·뒤·좌·우 ±snap_deg 안이면 축으로 (횡걸음 87↔93° 잔떨림 제거)
  - 저속 방향 고정: 작은 크기에서 바퀴 축을 크게 바꾸는 방향은 0 (오른쪽→왼쪽 옮기다 중앙 스칠 때 조향 안 돌게)
"""
import argparse
import json
import math
import socket
import struct
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from joystick_math import (apply_deadzone, clamp_to_unit_circle, left_pad_to_linear,  # noqa: E402
                           lock_low_speed_direction, right_pad_to_angular, snap_to_axes)
from gamepad_map import joy_to_cmd  # noqa: E402

# 게임패드 기본값 = gamepad_teleop.py PARAMS 와 같음 (yaml 이 있으면 yaml 값으로 덮음)
GAMEPAD_DEFAULTS = {
    "require_enable_button": True, "enable_button": 5,
    "axis_linear_x": 1, "axis_linear_y": 0, "axis_angular": 3,
    "scale_linear_x": 0.25, "scale_linear_y": 0.25, "scale_angular": 0.5,
    "deadzone": 0.15, "deadzone_angular": 0.15,
}


def load_gamepad_cfg():
    """config/shaks_s0.yaml 의 gamepad_teleop 파라미터 (소스 트리 → 설치 share 순). 없으면 기본값."""
    cfg = dict(GAMEPAD_DEFAULTS)
    here = Path(__file__).resolve().parent
    cands = [here.parent / "config" / "shaks_s0.yaml"]
    try:
        from ament_index_python.packages import get_package_share_directory
        cands.append(Path(get_package_share_directory("capstone_teleop")) / "config" / "shaks_s0.yaml")
    except Exception:
        pass
    for c in cands:
        if c.exists():
            try:
                import yaml
                p = yaml.safe_load(c.read_text())["gamepad_teleop"]["ros__parameters"]
                cfg.update({k: p[k] for k in cfg if k in p})
                return cfg, str(c)
            except Exception as e:  # noqa: BLE001
                print(f"[gamepad] {c} 읽기 실패 ({e}) — 기본값 사용")
    return cfg, "기본값"


class Gamepad:
    """pygame 조이스틱 → (enabled, vx, vy, ω). 연결/분리 자동."""

    def __init__(self, pygame, cfg):
        self.pg, self.cfg, self.js = pygame, cfg, None
        pygame.joystick.init()
        self.rescan()

    def rescan(self):
        self.js = self.pg.joystick.Joystick(0) if self.pg.joystick.get_count() > 0 else None
        if self.js is not None:
            self.js.init()
            print(f"[gamepad] {self.js.get_name()}  axes={self.js.get_numaxes()} buttons={self.js.get_numbuttons()}")

    def read(self):
        if self.js is None:
            return False, 0.0, 0.0, 0.0
        axes = [-self.js.get_axis(i) for i in range(self.js.get_numaxes())]  # ROS joy 규약: 왼쪽·위 = +
        buttons = [self.js.get_button(i) for i in range(self.js.get_numbuttons())]
        return joy_to_cmd(axes, buttons, self.cfg)

    def name(self):
        return self.js.get_name() if self.js is not None else None

WIDTH, HEIGHT = 640, 380
PAD_RADIUS = 100
LEFT_CENTER = (170, 190)
RIGHT_CENTER = (470, 190)
GRAB_SLACK = 1.4  # 반경 × 이 배수 안을 클릭해도 패드를 잡음


class Pad:
    def __init__(self, center, radius):
        self.center, self.radius = center, radius
        self.dragging = False
        self.nx = self.ny = 0.0

    def contains(self, pos):
        return math.hypot(pos[0] - self.center[0], pos[1] - self.center[1]) <= self.radius * GRAB_SLACK

    def start_drag(self, pos):
        self.dragging = True
        self.update(pos)

    def update(self, pos):
        self.nx, self.ny = clamp_to_unit_circle(pos[0] - self.center[0], pos[1] - self.center[1], self.radius)

    def release(self):
        self.dragging = False
        self.nx = self.ny = 0.0

    def handle_pixel_pos(self):
        return (self.center[0] + self.nx * self.radius, self.center[1] + self.ny * self.radius)


class RosSink:
    def __init__(self, topic):
        import rclpy
        from geometry_msgs.msg import Twist
        # Ctrl+C 를 rclpy 가 먼저 잡으면 컨텍스트가 닫혀 종료 때 보내는 0 이 실패함 → 신호는 파이썬이 처리
        try:
            from rclpy.signals import SignalHandlerOptions
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        except (ImportError, TypeError):
            rclpy.init()
        self.rclpy, self.Twist = rclpy, Twist
        self.node = rclpy.create_node("virtual_pad")
        self.pub = self.node.create_publisher(Twist, topic, 10)

    def send(self, vx, vy, omega):
        if not self.rclpy.ok():
            return
        m = self.Twist()
        m.linear.x, m.linear.y, m.angular.z = float(vx), float(vy), float(omega)
        self.pub.publish(m)

    def close(self):
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()


class UdpSink:
    """노트북 → 라파 udp_teleop_bridge.py. 패킷 b"CAPS" + <I seq> + <f vx vy w>, 라파가 5 Hz 로 JSON 상태를 돌려줌."""

    PKT = struct.Struct("<4sIfff")

    def __init__(self, dest):
        host, _, port = dest.partition(":")
        self.dest = (socket.gethostbyname(host), int(port or 5005))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.seq = 0
        self.tlm, self.tlm_time = None, -1e9
        print(f"[udp] -> {self.dest[0]}:{self.dest[1]} (라파에서 udp_teleop_bridge.py 실행 중이어야 함)")

    def send(self, vx, vy, omega):
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        try:
            self.sock.sendto(self.PKT.pack(b"CAPS", self.seq, vx, vy, omega), self.dest)
        except OSError:
            pass
        while True:
            try:
                data, _ = self.sock.recvfrom(4096)
            except (BlockingIOError, OSError):
                break
            try:
                self.tlm, self.tlm_time = json.loads(data), time.monotonic()
            except ValueError:
                continue
            f = self.tlm.get("fault", "")
            if f != getattr(self, "_last_fault", ""):
                print(f"[robot] FAULT: {f}" if f else "[robot] fault 해제")  # 창 글꼴은 한글 불가 → 터미널에
                self._last_fault = f

    def status(self):
        if self.tlm is None or time.monotonic() - self.tlm_time > 1.0:
            return "robot: no reply (check Pi udp_teleop_bridge / network)", (230, 120, 120)
        t = self.tlm
        txt = f"robot: steer {t.get('steer', '-')} | wheel {t.get('wheel', '-')} | vbus {t.get('vbus', '-')} V"
        if "steer_deg" in t:
            txt += f" | deg {t['steer_deg']}"
        if t.get("fault"):
            return txt + f" | FAULT {t['fault']}", (240, 90, 90)
        return txt, (120, 220, 140)

    def close(self):
        self.sock.close()


def run(args):
    import pygame

    sink = UdpSink(args.udp) if args.udp else RosSink(args.topic)
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption(f"capstone virtual pad → {args.topic}")
    font = pygame.font.SysFont(None, 22)
    clock = pygame.time.Clock()
    left, right = Pad(LEFT_CENTER, PAD_RADIUS), Pad(RIGHT_CENTER, PAD_RADIUS)
    gcfg, gsrc = load_gamepad_cfg()
    gpad = Gamepad(pygame, gcfg) if not args.no_gamepad else None
    if gpad is not None:
        print(f"[gamepad] 설정 {gsrc}: vx=axes[{gcfg['axis_linear_x']}]×{gcfg['scale_linear_x']} "
              f"vy=axes[{gcfg['axis_linear_y']}]×{gcfg['scale_linear_y']} ω=axes[{gcfg['axis_angular']}]×{gcfg['scale_angular']} "
              f"deadman 버튼 {gcfg['enable_button']}")
    source = "mouse"
    period = 1.0 / args.rate
    next_send = time.monotonic()
    last_dir = None
    running = True
    try:
        while running:
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT or (ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE):
                    running = False
                elif ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1:
                    if left.contains(ev.pos):
                        left.start_drag(ev.pos)
                    elif right.contains(ev.pos):
                        right.start_drag(ev.pos)
                elif ev.type == pygame.MOUSEBUTTONUP and ev.button == 1:
                    left.release()
                    right.release()
                elif ev.type == pygame.MOUSEMOTION:
                    if left.dragging:
                        left.update(ev.pos)
                    if right.dragging:
                        right.update(ev.pos)
                elif ev.type in (pygame.WINDOWFOCUSLOST, pygame.ACTIVEEVENT):
                    left.release()
                    right.release()
                elif gpad is not None and ev.type in (pygame.JOYDEVICEADDED, pygame.JOYDEVICEREMOVED):
                    gpad.rescan()

            keys = pygame.key.get_pressed()
            if keys[pygame.K_SPACE]:
                left.release()
                right.release()

            lnx, lny = apply_deadzone(left.nx, left.ny, args.deadzone)
            lnx, lny = snap_to_axes(lnx, lny, args.snap_deg)
            lnx, lny, last_dir = lock_low_speed_direction(lnx, lny, last_dir, args.hold_mag, args.hold_deg)
            vx, vy = left_pad_to_linear(lnx, lny, args.max_lin)
            if right.dragging:
                rnx, _ = apply_deadzone(right.nx, 0.0, args.deadzone)
                omega = right_pad_to_angular(rnx, args.max_ang)
            elif keys[pygame.K_q] and not keys[pygame.K_e]:
                omega = args.max_ang
            elif keys[pygame.K_e] and not keys[pygame.K_q]:
                omega = -args.max_ang
            else:
                omega = 0.0
            source = "mouse"
            if gpad is not None:
                g_on, gvx, gvy, gw = gpad.read()
                if g_on:  # deadman 누르는 동안 게임패드 우선
                    lim = lambda v, m: max(-m, min(m, v))
                    vx, vy, omega = lim(gvx, args.max_lin), lim(gvy, args.max_lin), lim(gw, args.max_ang)
                    source = "gamepad"
            if keys[pygame.K_SPACE]:
                vx = vy = omega = 0.0

            now = time.monotonic()
            if now >= next_send:
                sink.send(vx, vy, omega)
                next_send = max(next_send + period, now - period)  # 고정 주기 (화면 갱신 60 fps 에 밀리지 않게)

            screen.fill((24, 24, 28))
            for pad, label in ((left, "translate (vx, vy)"), (right, "rotate (omega)")):
                pygame.draw.circle(screen, (60, 60, 68), pad.center, pad.radius, 0)
                pygame.draw.circle(screen, (70, 70, 80), pad.center, pad.radius, 2)
                color = (90, 200, 255) if pad.dragging else (140, 140, 150)
                pygame.draw.circle(screen, color, [int(c) for c in pad.handle_pixel_pos()], 18)
                t = font.render(label, True, (150, 150, 160))
                screen.blit(t, (pad.center[0] - t.get_width() // 2, pad.center[1] + pad.radius + 10))
            screen.blit(font.render(f"vx={vx:+.2f} m/s  vy={vy:+.2f} m/s  omega={omega:+.2f} rad/s   [{source}]",
                                    True, (220, 220, 230)), (16, 16))
            gname = gpad.name() if gpad is not None else None
            screen.blit(font.render(f"gamepad: {gname}  (hold RB + sticks)" if gname else "gamepad: none",
                                    True, (120, 200, 230) if gname else (110, 110, 120)), (16, 70))
            screen.blit(font.render("drag pads with mouse | Q/E rotate | space=stop | Esc=quit",
                                    True, (140, 140, 150)), (16, 44))
            screen.blit(font.render(f"max {args.max_lin} m/s, {args.max_ang} rad/s @ {args.rate:.0f} Hz"
                                    + (f"  udp -> {args.udp}" if args.udp else f"  ros -> {args.topic}"),
                                    True, (110, 110, 120)), (16, HEIGHT - 30))
            if hasattr(sink, "status"):
                txt, col = sink.status()
                screen.blit(font.render(txt, True, col), (16, HEIGHT - 56))
            pygame.display.flip()
            clock.tick(60)
    finally:
        for _ in range(5):
            sink.send(0.0, 0.0, 0.0)
            time.sleep(0.02)
        sink.close()
        pygame.quit()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topic", default="/cmd_vel")
    ap.add_argument("--udp", metavar="HOST[:PORT]", help="ROS 대신 UDP 로 라파 udp_teleop_bridge.py 에 보냄 (기본 포트 5005)")
    ap.add_argument("--no-gamepad", action="store_true", help="게임패드 읽지 않음")
    ap.add_argument("--rate", type=float, default=20.0, help="발행 주기 [Hz] (swerve cmd_timeout 0.3 s 보다 충분히 빠르게)")
    ap.add_argument("--max-lin", type=float, default=0.3, help="패드 끝 |vx|,|vy| [m/s] (swerve max_wheel_mps 0.6)")
    ap.add_argument("--max-ang", type=float, default=1.0, help="패드 끝 |ω| [rad/s]")
    ap.add_argument("--deadzone", type=float, default=0.1, help="중앙 데드존 (반경 비율), 0 = 끔")
    ap.add_argument("--snap-deg", type=float, default=10.0, help="축 스냅 각도, 0 = 끔")
    ap.add_argument("--hold-mag", type=float, default=0.35, help="이 크기 미만에선 바퀴 축 크게 바꾸는 방향 무시, 0 = 끔")
    ap.add_argument("--hold-deg", type=float, default=20.0, help="--hold-mag 미만에서 허용하는 축 변화 [deg]")
    args, _ = ap.parse_known_args()  # ros2 launch 가 붙이는 --ros-args 무시
    try:
        run(args)
    except KeyboardInterrupt:
        pass  # 0 은 finally 에서 이미 보냄


if __name__ == "__main__":
    main()
