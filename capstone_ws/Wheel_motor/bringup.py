#!/usr/bin/env python3
"""XDrive(ODrive v3.6, fw 0.5.1) + BL41105 hub motor bring-up, USB, one step at a time.

Run inside the venv, with odrivetool CLOSED (only one program can hold the USB device):
    source ~/odrive_env/bin/activate
    python ~/capstone/capstone_ws/Wheel_motor/bringup.py status
    python ~/capstone/capstone_ws/Wheel_motor/bringup.py check --board 1          # 두 축 설정이 기대값과 같은지 (읽기만)
    python ~/capstone/capstone_ws/Wheel_motor/bringup.py clear                    # 두 축 에러 클리어 → 0.5 s 뒤 다시 읽기
    python ~/capstone/capstone_ws/Wheel_motor/bringup.py set-can --ids 3 4        # 두 축 CAN ID + baud 500k 저장, 재부팅
    python ~/capstone/capstone_ws/Wheel_motor/bringup.py set-hall --axis 0        # once: Hall mode + save + reboot
    python ~/capstone/capstone_ws/Wheel_motor/bringup.py hall --axis 0            # turn wheel by hand, Ctrl+C to stop
    python ~/capstone/capstone_ws/Wheel_motor/bringup.py config --axis 0 --pp 10 --kv 41.7
    python ~/capstone/capstone_ws/Wheel_motor/bringup.py calib --axis 0 [--save]
    python ~/capstone/capstone_ws/Wheel_motor/bringup.py spin --axis 0 --vel 0.5 --duration 5

Note: the config step also disables the XDrive factory auto-closed-loop flags (see COMMANDS.md §1).
Policy: no firmware update / reflash. This script only reads/writes config and requests axis states.
Integer values are used (not enum names) because odrive 0.6.x enums do not cover 0.5.1 everywhere.
"""
import argparse
import sys
import time

import odrive

# fw 0.5.x AxisState values
IDLE, MOTOR_CALIBRATION, ENCODER_OFFSET_CALIBRATION, CLOSED_LOOP = 1, 4, 7, 8
ENCODER_MODE_HALL = 1
CONTROL_MODE_VELOCITY, INPUT_MODE_VEL_RAMP = 2, 2


def connect(serial=None):
    try:
        odrv = odrive.find_any(serial_number=serial, timeout=10) if serial else odrive.find_any(timeout=10)
    except Exception as e:  # device held by odrivetool, or not powered
        sys.exit(f"[connect] failed: {e!r}\n -> odrivetool 을 닫았는지, 24V 가 들어갔는지 확인")
    print(f"connected: serial={odrv.serial_number:X} "
          f"fw={odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision} "
          f"vbus={odrv.vbus_voltage:.2f} V")
    return odrv


def errors(odrv, ax):
    return dict(axis=ax.error, motor=ax.motor.error, encoder=ax.encoder.error,
                controller=ax.controller.error)


def print_errors(odrv):
    try:
        odrive.utils.dump_errors(odrv)
    except Exception:
        for n in ("axis0", "axis1"):
            print(n, {k: hex(v) for k, v in errors(odrv, getattr(odrv, n)).items()})


def wait_idle(ax, timeout):
    t0 = time.time()
    time.sleep(0.5)
    while ax.current_state != IDLE:
        if time.time() - t0 > timeout:
            ax.requested_state = IDLE
            return False
        time.sleep(0.1)
    return True


def try_set(obj, name, value):
    try:
        setattr(obj, name, value)
        print(f"  {name} = {getattr(obj, name)}")
    except Exception as e:
        print(f"  {name}: FAILED ({e!r})")


def save_and_reboot(odrv):
    print("save_configuration() ...")
    odrv.save_configuration()
    print("reboot() ... (재연결은 다음 명령에서)")
    try:
        odrv.reboot()
    except Exception:
        pass  # connection drops on reboot; expected


# ---------------------------------------------------------------- commands
def cmd_status(odrv, ax, a):
    m, e, c = ax.motor.config, ax.encoder.config, ax.controller.config
    cf = ax.config
    print(f"axis{a.axis}: state={ax.current_state}  startup(motor_cal,enc_offset,closed_loop)="
          f"{cf.startup_motor_calibration},{cf.startup_encoder_offset_calibration},{cf.startup_closed_loop_control}")
    print(f"  motor  : pole_pairs={m.pole_pairs} current_lim={m.current_lim} calib_current={m.calibration_current} "
          f"R={m.phase_resistance:.4f} L={m.phase_inductance:.2e} torque_const={m.torque_constant:.4f} "
          f"calibrated={ax.motor.is_calibrated} pre_cal={m.pre_calibrated}")
    print(f"  encoder: mode={e.mode} cpr={e.cpr} ready={ax.encoder.is_ready} pre_cal={e.pre_calibrated} "
          f"hall_state={ax.encoder.hall_state} shadow_count={ax.encoder.shadow_count}")
    print(f"  ctrl   : control_mode={c.control_mode} input_mode={c.input_mode} vel_limit={c.vel_limit} "
          f"vel_gain={c.vel_gain:.4f} vel_int={c.vel_integrator_gain:.4f}")
    baud = getattr(getattr(odrv.can, "config", None), "baud_rate", "?")
    print(f"  can    : node_id={getattr(cf, 'can_node_id', '?')}  board baud={baud}")
    print_errors(odrv)


def cmd_check(odrv, _ax, a):
    """읽기 전용: 두 축 설정을 기대값(COMMANDS.md §4 / config 단계)과 비교해 OK / !! 로 표시."""
    tc, cpr = 8.27 / a.kv, 6 * a.pp
    node_ids = {1: (1, 2), 2: (3, 4)}[a.board]
    bad = 0

    def read(fn):
        try:
            return fn()
        except Exception as e:
            return f"<읽기 실패 {type(e).__name__}>"

    def row(name, fn, ok, want):
        nonlocal bad
        v = read(fn)
        good = not isinstance(v, str) and ok(v)
        bad += not good
        vs = f"{v:.4g}" if isinstance(v, float) else str(v)
        print(f"  {'OK' if good else '!!'}  {name:<34} {vs:<14} (기대 {want})")

    eq = lambda w: (lambda v: v == w)
    near = lambda w, tol: (lambda v: abs(v - w) <= tol)
    within = lambda lo, hi: (lambda v: lo <= v <= hi)

    for i in (0, 1):
        ax = getattr(odrv, f"axis{i}")
        cf, m, e, c = ax.config, ax.motor.config, ax.encoder.config, ax.controller.config
        print(f"axis{i}  (board #{a.board} → CAN ID {node_ids[i]})")
        print("  [공장 함정 해제]")
        row("startup_motor_calibration", lambda: cf.startup_motor_calibration, eq(False), False)
        row("startup_encoder_offset_calibration", lambda: cf.startup_encoder_offset_calibration, eq(False), False)
        row("startup_closed_loop_control", lambda: cf.startup_closed_loop_control, eq(False), False)
        print("  [모터 공통값 — 4개 모두 같음]")
        row("motor.pole_pairs", lambda: m.pole_pairs, eq(a.pp), a.pp)
        row("motor.torque_constant", lambda: m.torque_constant, near(tc, 0.002), f"{tc:.4f}")
        row("motor.current_lim", lambda: m.current_lim, near(10.0, 0.01), 10)
        row("motor.calibration_current", lambda: m.calibration_current, near(5.0, 0.01), 5)
        row("motor.current_control_bandwidth", lambda: m.current_control_bandwidth, near(100, 0.1), 100)
        row("encoder.mode", lambda: e.mode, eq(ENCODER_MODE_HALL), "1 (Hall)")
        row("encoder.cpr", lambda: e.cpr, eq(cpr), cpr)
        row("controller.control_mode", lambda: c.control_mode, eq(CONTROL_MODE_VELOCITY), "2 (VELOCITY)")
        row("controller.input_mode", lambda: c.input_mode, eq(INPUT_MODE_VEL_RAMP), "2 (VEL_RAMP)")
        row("controller.vel_limit", lambda: c.vel_limit, near(2.0, 0.01), 2)
        row("controller.vel_gain", lambda: c.vel_gain, near(0.02 * tc * cpr, 0.005), f"{0.02 * tc * cpr:.3f}")
        row("controller.vel_integrator_gain", lambda: c.vel_integrator_gain,
            near(0.1 * tc * cpr, 0.02), f"{0.1 * tc * cpr:.3f}")
        print("  [캘리브레이션 — 모터·배선마다 측정값]")
        row("motor.phase_resistance [ohm]", lambda: m.phase_resistance, within(0.15, 0.5),
            "0.15~0.5 (시험값 0.085 아님)")
        row("motor.phase_inductance [H]", lambda: m.phase_inductance, within(60e-6, 400e-6),
            "60~400µ (시험값 14µ 아님)")
        row("motor.pre_calibrated", lambda: m.pre_calibrated, eq(True), True)
        row("encoder.pre_calibrated", lambda: e.pre_calibrated, eq(True), True)
        row("motor.is_calibrated", lambda: ax.motor.is_calibrated, eq(True), True)
        row("encoder.is_ready", lambda: ax.encoder.is_ready, eq(True), True)
        print("  [CAN / 상태]")
        row("config.can_node_id", lambda: cf.can_node_id, eq(node_ids[i]), node_ids[i])
        row("current_state", lambda: ax.current_state, eq(IDLE), "1 (IDLE)")
        row("axis/motor/encoder/ctrl error", lambda: (ax.error, ax.motor.error, ax.encoder.error,
                                                      ax.controller.error), eq((0, 0, 0, 0)), "(0,0,0,0)")
    print("board")
    row("can baud_rate", lambda: odrv.can.config.baud_rate, eq(500000), 500000)
    print(f"\n{'모두 일치' if bad == 0 else f'!! 불일치 {bad}개'}")
    if bad:
        print("  공통값 불일치 → config, R/L·pre_cal 불일치 → calib --save, node_id → odrivetool (README 1단계)")


def cmd_clear(odrv, _ax, a):
    """두 축 에러 클리어 후 0.5 s 기다렸다 다시 읽음 — 바로 또 뜨면 지금도 계속 생기는 문제."""
    for i in (0, 1):
        ax = getattr(odrv, f"axis{i}")
        ax.error = ax.motor.error = ax.encoder.error = ax.controller.error = 0
    time.sleep(0.5)
    for i in (0, 1):
        ax = getattr(odrv, f"axis{i}")
        print(f"axis{i}: error(axis,motor,encoder,ctrl)={(ax.error, ax.motor.error, ax.encoder.error, ax.controller.error)}"
              f"  hall_state={ax.encoder.hall_state}")


def cmd_set_can(odrv, _ax, a):
    """두 축 CAN ID + 보드 baud 설정 후 저장·재부팅 (fw 0.5.1 이름)."""
    for i, cid in enumerate(a.ids):
        ax = getattr(odrv, f"axis{i}")
        ax.requested_state = IDLE
        try_set(ax.config, "can_node_id", cid)
    try:
        odrv.can.set_baud_rate(a.baud)          # 0.5.1: 함수
    except AttributeError:
        odrv.can.config.baud_rate = a.baud      # 0.5.2+ 이름 (대비용)
    print(f"  can baud = {getattr(getattr(odrv.can, 'config', None), 'baud_rate', a.baud)}")
    save_and_reboot(odrv)


def cmd_set_hall(odrv, ax, a):
    ax.requested_state = IDLE
    try_set(ax.encoder.config, "mode", ENCODER_MODE_HALL)
    save_and_reboot(odrv)


def cmd_hall(odrv, ax, a):
    if ax.encoder.config.mode != ENCODER_MODE_HALL:
        sys.exit("encoder mode 가 Hall(1) 이 아님 -> 먼저 set-hall 실행")
    ax.encoder.error = 0  # clear stale NO_RESPONSE from boot without sensor
    c0 = ax.encoder.shadow_count
    seen = set()
    print("바퀴에 표시하고 손으로 정확히 1바퀴 돌린 뒤 Ctrl+C.  (hall_state 는 1..6 만 나와야 정상)")
    try:
        while True:
            h = ax.encoder.hall_state
            seen.add(h)
            d = ax.encoder.shadow_count - c0
            print(f"\r hall_state={h}  seen={sorted(seen)}  Δcount={d:+6d}  Δ/6={abs(d) / 6:6.2f}   ", end="")
            time.sleep(0.02)
    except KeyboardInterrupt:
        d = ax.encoder.shadow_count - c0
        print(f"\n Δcount={d}  -> pole_pairs ≈ {abs(d) / 6:.2f}")
        bad = seen - set(range(1, 7))
        if bad:
            print(f" !! 비정상 hall_state {sorted(bad)} -> Hall 5V/GND/신호선 확인")
        if ax.encoder.error:
            print(f" !! encoder.error = {ax.encoder.error:#x}")


def cmd_config(odrv, ax, a):
    ax.requested_state = IDLE
    tc = 8.27 / a.kv
    cpr = 6 * a.pp
    print(f"axis{a.axis}: pp={a.pp} cpr={cpr} Kv={a.kv} -> torque_constant={tc:.4f} Nm/A")
    m, e, c = ax.motor.config, ax.encoder.config, ax.controller.config
    # XDrive factory trap: auto offset-cal + closed loop on boot, and stale test-motor R/L marked pre_calibrated.
    # startup 플래그는 두 축 모두 끔 — 재부팅 때 다른 축이 공장 설정으로 혼자 캘리브/폐루프 들어가지 않게
    for other in (odrv.axis0, odrv.axis1):
        other.requested_state = IDLE
        for flag in ("startup_motor_calibration", "startup_encoder_offset_calibration",
                     "startup_closed_loop_control"):
            setattr(other.config, flag, False)
    print("  startup_* = False (axis0, axis1)")
    try_set(m, "pre_calibrated", False)
    try_set(e, "pre_calibrated", False)
    try_set(m, "pole_pairs", a.pp)
    try_set(m, "torque_constant", tc)
    try_set(m, "resistance_calib_max_voltage", 4.0)
    try_set(m, "requested_current_range", 25.0)
    try_set(m, "current_control_bandwidth", 100.0)
    try_set(m, "calibration_current", a.calib_current)
    try_set(m, "current_lim", a.current_lim)
    try_set(e, "mode", ENCODER_MODE_HALL)
    try_set(e, "cpr", cpr)
    try_set(e, "bandwidth", 100.0)
    try_set(e, "calib_scan_distance", 150.0)
    try_set(c, "control_mode", CONTROL_MODE_VELOCITY)
    try_set(c, "input_mode", INPUT_MODE_VEL_RAMP)
    try_set(c, "vel_ramp_rate", 2.0)
    try_set(c, "vel_limit", a.vel_limit)
    try_set(c, "pos_gain", 1.0)
    # ODrive hoverboard-guide starting gains (fw 0.5.x units)
    try_set(c, "vel_gain", 0.02 * tc * cpr)
    try_set(c, "vel_integrator_gain", 0.1 * tc * cpr)
    save_and_reboot(odrv)  # requested_current_range only applies after reboot


def cmd_calib(odrv, ax, a):
    ax.requested_state = IDLE
    ax.error = ax.motor.error = ax.encoder.error = ax.controller.error = 0
    input("바퀴 띄웠나? 모터가 '삐-' 소리 후 천천히 돈다. Enter 로 시작 (Ctrl+C 취소) ")

    print("[1/2] MOTOR_CALIBRATION ...")
    ax.requested_state = MOTOR_CALIBRATION
    ok = wait_idle(ax, 15)
    m = ax.motor.config
    print(f"  R={m.phase_resistance:.4f} ohm  L={m.phase_inductance:.3e} H  is_calibrated={ax.motor.is_calibrated}")
    if not ok or not ax.motor.is_calibrated or ax.error or ax.motor.error:
        print_errors(odrv)
        sys.exit("motor calibration 실패")

    print("[2/2] ENCODER_OFFSET_CALIBRATION (Hall) ...")
    ax.requested_state = ENCODER_OFFSET_CALIBRATION
    ok = wait_idle(ax, 30)
    print(f"  encoder.is_ready={ax.encoder.is_ready}  offset={ax.encoder.config.offset}")
    if not ok or not ax.encoder.is_ready or ax.error or ax.encoder.error:
        print_errors(odrv)
        sys.exit("encoder offset calibration 실패")

    print("calibration OK")
    if a.save:
        ax.motor.config.pre_calibrated = True
        ax.encoder.config.pre_calibrated = True
        print("pre_calibrated=True, save_configuration()")
        odrv.save_configuration()
    else:
        print("(저장 안 함. spin 으로 확인 후 --save 로 다시 돌리면 다음 부팅부터 캘리브레이션 생략)")


def cmd_spin(odrv, ax, a):
    vl = ax.controller.config.vel_limit
    if abs(a.vel) > vl:
        sys.exit(f"|vel|={abs(a.vel)} > vel_limit={vl}. config --vel-limit 로 올리고 다시")
    ax.error = ax.motor.error = ax.encoder.error = ax.controller.error = 0
    ax.controller.config.control_mode = CONTROL_MODE_VELOCITY
    ax.controller.config.input_mode = INPUT_MODE_VEL_RAMP
    ax.controller.input_vel = 0.0
    ax.requested_state = CLOSED_LOOP
    time.sleep(0.3)
    if ax.current_state != CLOSED_LOOP:
        print_errors(odrv)
        sys.exit("CLOSED_LOOP 진입 실패")
    print(f"closed loop. input_vel={a.vel} turn/s for {a.duration}s  (Ctrl+C = 즉시 IDLE)")
    try:
        ax.controller.input_vel = a.vel
        t0 = time.time()
        while time.time() - t0 < a.duration:
            if ax.current_state != CLOSED_LOOP:
                print("\n!! 축이 CLOSED_LOOP 에서 빠짐")
                break
            print(f"\r t={time.time() - t0:4.1f}s  vel_est={ax.encoder.vel_estimate:+6.3f} turn/s  "
                  f"Iq={ax.motor.current_control.Iq_measured:+6.2f} A  vbus={odrv.vbus_voltage:5.2f} V   ", end="")
            time.sleep(0.05)
        ax.controller.input_vel = 0.0
        time.sleep(1.5)  # ramp down
    except KeyboardInterrupt:
        print("\nCtrl+C")
    finally:
        ax.controller.input_vel = 0.0
        ax.requested_state = IDLE
        print("\n-> IDLE")
    print_errors(odrv)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    for name in ("status", "check", "clear", "set-can", "set-hall", "hall", "config", "calib", "spin"):
        sp = sub.add_parser(name)
        sp.add_argument("--axis", type=int, choices=(0, 1), default=0)
        sp.add_argument("--serial", help="보드 시리얼(hex). USB 에 보드 여러 장일 때 지정 (#1 = 318C36673335)")
        if name == "set-can":
            sp.add_argument("--ids", type=int, nargs=2, required=True, metavar=("AXIS0", "AXIS1"),
                            help="CAN ID. 보드 #1 = 1 2, 보드 #2 = 3 4")
            sp.add_argument("--baud", type=int, default=500000)
        if name == "check":
            sp.add_argument("--board", type=int, choices=(1, 2), default=1, help="1 → CAN ID 1/2, 2 → CAN ID 3/4")
            sp.add_argument("--pp", type=int, default=10)
            sp.add_argument("--kv", type=float, default=41.7)
        if name == "config":
            sp.add_argument("--pp", type=int, required=True, help="pole pairs (hall 명령으로 측정)")
            sp.add_argument("--kv", type=float, required=True, help="rpm/V (데이터시트 무부하rpm / 정격전압)")
            sp.add_argument("--current-lim", type=float, default=10.0)
            sp.add_argument("--calib-current", type=float, default=5.0)
            sp.add_argument("--vel-limit", type=float, default=2.0, help="turn/s")
        if name == "calib":
            sp.add_argument("--save", action="store_true", help="성공 시 pre_calibrated 저장")
        if name == "spin":
            sp.add_argument("--vel", type=float, required=True, help="turn/s")
            sp.add_argument("--duration", type=float, default=5.0)
    a = p.parse_args()

    odrv = connect(a.serial)
    ax = getattr(odrv, f"axis{a.axis}")
    {"status": cmd_status, "check": cmd_check, "clear": cmd_clear, "set-can": cmd_set_can, "set-hall": cmd_set_hall, "hall": cmd_hall,
     "config": cmd_config, "calib": cmd_calib, "spin": cmd_spin}[a.cmd](odrv, ax, a)


if __name__ == "__main__":
    main()
