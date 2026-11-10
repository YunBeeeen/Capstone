#!/usr/bin/env python3
"""리프트 Pico 시리얼 ↔ ROS. **라파에서 실행** (Pico 가 라파 USB 에 꽂혀 있음).

  ros2 run lift_bridge lift_bridge_node.py          ← ros2 launch 말고 run 으로 (종료 때 주차에 수십 초 걸릴 수 있음)

  sub  /lift_cmd    capstone_msgs/LiftCommand   (udp_teleop_bridge 가 패드 입력을 넘김)
  pub  /lift_state  capstone_msgs/LiftState     10 Hz

종료 (Ctrl+C): 리프트를 최하단(0 mm)까지 내려 놓고 끝남 → 다음에 켤 때 자동 원점 등록.
  주차 중 Ctrl+C 한 번 더 = 즉시 정지 (주차 취소 → 다음엔 수동 원점 등록 필요)
주의: 동료 GUI (lift_position.py), mpremote, Thonny 가 Pico 포트를 잡고 있으면 연결 안 됨 (포트는 한 프로그램만).
로직은 lift_logic.py (ROS 없이 테스트).
"""
import glob
import json
import os
import signal
import sys
import time
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lift_logic import AUTO_CLEAR_FAULTS, LiftCmd, LiftConfig, LiftLogic  # noqa: E402

DEFAULT_PORT_GLOB = "/dev/serial/by-id/usb-MicroPython_Board*"


def load_store(path: Path):
    try:
        d = json.loads(path.read_text())
        return d if isinstance(d, dict) else {}
    except (OSError, ValueError):
        return {}


def save_store(path: Path, data: dict):
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(dict(data, saved_at=time.strftime("%Y-%m-%dT%H:%M:%S")), ensure_ascii=False))
    os.replace(tmp, path)


class PicoLink:
    """시리얼 열기/재연결, JSON 줄 읽기/쓰기."""

    def __init__(self, port, log):
        self.port_arg, self.log = port, log
        self.ser = None
        self.buf = b""
        self.next_try = 0.0
        self.n_bad = 0

    def port(self):
        if self.port_arg:
            return self.port_arg
        c = sorted(glob.glob(DEFAULT_PORT_GLOB))
        return c[0] if c else ""

    def ensure(self):
        if self.ser is not None or time.monotonic() < self.next_try:
            return self.ser is not None
        self.next_try = time.monotonic() + 1.0
        p = self.port()
        if not p:
            self.log(f"Pico 포트 없음 ({DEFAULT_PORT_GLOB}) — USB 확인")
            return False
        try:
            import serial
            self.ser = serial.Serial(p, 115200, timeout=0, write_timeout=0.05)
            self.buf = b""
            self.log(f"Pico 연결: {p}")
            return True
        except Exception as e:  # serial.SerialException, PermissionError …
            self.log(f"Pico 포트 열기 실패 {p}: {e} (다른 프로그램이 잡고 있나?)")
            self.ser = None
            return False

    def read(self):
        """받은 JSON dict 들."""
        if self.ser is None:
            return []
        try:
            self.buf += self.ser.read(4096)
        except Exception as e:
            self.close(f"읽기 오류: {e}")
            return []
        out = []
        *lines, self.buf = self.buf.split(b"\n")
        for ln in lines:
            ln = ln.strip()
            if not ln.startswith(b"{"):
                continue  # MicroPython 부팅 메시지 등
            try:
                out.append(json.loads(ln))
            except ValueError:
                self.n_bad += 1
        if len(self.buf) > 4096:
            self.buf = b""
        return out

    def send(self, m: dict):
        if self.ser is None:
            return
        try:
            self.ser.write((json.dumps(m, separators=(",", ":")) + "\n").encode())
        except Exception as e:
            self.close(f"쓰기 오류: {e}")

    def close(self, why=""):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            if why:
                self.log(f"Pico 연결 끊음: {why}")


class LiftBridge(Node):
    def __init__(self):
        super().__init__("lift_bridge")
        from capstone_msgs.msg import LiftCommand, LiftState
        self.LiftState = LiftState
        p = self.declare_parameter
        cfg = LiftConfig(
            mm_per_count=p("mm_per_count", 0.21574).value,
            encoder_sign=p("encoder_sign", 1).value,
            jog_duty=p("jog_duty", 0.50).value,
            max_jog_duty=p("max_jog_duty", 1.0).value,
            unhomed_max_duty=p("unhomed_max_duty", 0.5).value,
            jog_ramp_per_s=p("jog_ramp_per_s", 2.0).value,
            park_target_mm=p("park_target_mm", 10.0).value,
            park_move_duty=p("park_move_duty", 0.6).value,
            park_duty=p("park_duty", 0.25).value,
        )
        self.park_on_shutdown = p("park_on_shutdown", True).value
        self.store_path = Path(os.path.expanduser(
            p("state_file", "~/.local/state/capstone_lift/lift_state.json").value))
        self.logic = LiftLogic(cfg, store=load_store(self.store_path))
        self.link = PicoLink(p("port", "").value, self._log_link)
        self._last_link_log = ("", 0.0)
        self.last_fault = ""
        self.pub = self.create_publisher(LiftState, "lift_state", 10)
        self.create_subscription(LiftCommand, "lift_cmd", self.on_cmd, 10)
        self.create_timer(0.05, self.tick)
        self.create_timer(0.1, self.publish)
        st = self.logic.store
        self.get_logger().info(
            f"lift_bridge 시작 — 주차 파일 {self.store_path} (parked={st.get('parked')}, "
            f"잔차 {st.get('residual_mm', 0.0):+.2f} mm). 종료 시 주차: {'켬' if self.park_on_shutdown else '끔'}")

    def _log_link(self, text):
        last, t = self._last_link_log
        if text != last or time.monotonic() - t > 10.0:  # 같은 말 10 s 에 한 번
            self.get_logger().warning(text) if ("실패" in text or "없음" in text or "끊음" in text) \
                else self.get_logger().info(text)
            self._last_link_log = (text, time.monotonic())

    def on_cmd(self, m):
        self.logic.on_command(LiftCmd(jog=int(m.jog), duty=float(m.duty), stop=m.stop, park=m.park,
                                      zero=m.zero, clear=m.clear), time.monotonic())

    def pump(self):
        """시리얼 읽기 → 로직 → 시리얼 쓰기. 종료 루프에서도 씀."""
        now = time.monotonic()
        if self.link.ensure():
            for st in self.link.read():
                self.logic.on_status(st, now)
            for m in self.logic.step(now):
                self.link.send(m)
        if self.logic.store_dirty:
            try:
                save_store(self.store_path, self.logic.store)
                self.logic.store_dirty = False
            except OSError as e:
                self.get_logger().error(f"주차 파일 저장 실패: {e}")

    def tick(self):
        self.pump()
        lg = self.logic
        if lg.new_message:
            lg.new_message = False
            self.get_logger().info(lg.message)
        f = (lg.st or {}).get("fault", "") or ""
        if f != self.last_fault:
            if f:
                self.get_logger().warning(f"Pico fault: {f}" + ("  (자동 해제)" if f in AUTO_CLEAR_FAULTS else "  (패드 C / 게임패드 RB+LB 로 해제)"))
            self.last_fault = f

    def publish(self):
        s = self.logic.state(time.monotonic())
        m = self.LiftState()
        m.header.stamp = self.get_clock().now().to_msg()
        for k, v in s.items():
            setattr(m, k, float(v) if isinstance(v, float) else v)
        self.pub.publish(m)

    def shutdown_park(self):
        """rclpy 가 내려간 뒤 호출. print 만 씀."""
        lg = self.logic
        if not self.park_on_shutdown:
            print("[lift] 종료: 주차 끔 → 정지 + disarm 만")
            return
        now = time.monotonic()
        # 상태가 끊긴 지 오래면 다시 받아 봄
        deadline = now + 2.0
        while not lg.connected(time.monotonic()) and time.monotonic() < deadline:
            self.pump()
            time.sleep(0.05)
        if not lg.connected(time.monotonic()):
            print("[lift] Pico 응답 없음 — 주차 못 함 (다음 부팅 때 수동 원점 필요)")
            return
        lg.request_park(time.monotonic())
        print(f"[lift] {lg.message}")
        lg.new_message = False
        if not lg.busy():
            return
        print("[lift] 최하단으로 내리는 중 ... (Ctrl+C 한 번 더 = 즉시 정지)")
        last_print = 0.0
        deadline = time.monotonic() + lg.cfg.park_move_timeout + 30.0
        while lg.busy() and time.monotonic() < deadline:
            self.pump()
            if lg.new_message:
                lg.new_message = False
                print(f"[lift] {lg.message}")
            if time.monotonic() - last_print > 1.0:
                h = lg.height()
                print(f"[lift]   {lg.task:13s} 높이 {'%.1f mm' % h if h is not None else '(원점 해제 중)'}"
                      f"  rel {lg.rel_mm():+.1f} mm", flush=True)
                last_print = time.monotonic()
            time.sleep(0.05)
        if lg.new_message or lg.busy():
            print(f"[lift] {lg.message}" + ("  (시간 초과 — 주차 못 끝냄)" if lg.busy() else ""))

    def close(self):
        for m in ({"cmd": "stop"}, {"cmd": "disarm"}):
            self.link.send(m)
        if self.logic.store_dirty:
            try:
                save_store(self.store_path, self.logic.store)
            except OSError:
                pass
        self.link.close()


def main():
    rclpy.init()
    node = LiftBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    # 두 번째 Ctrl+C 는 KeyboardInterrupt 로 받아 즉시 정지
    signal.signal(signal.SIGINT, signal.default_int_handler)
    try:
        node.shutdown_park()
    except KeyboardInterrupt:
        print("\n[lift] 주차 취소 — 즉시 정지")
        node.logic.store["parked"] = False
        node.logic.store_dirty = True
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
