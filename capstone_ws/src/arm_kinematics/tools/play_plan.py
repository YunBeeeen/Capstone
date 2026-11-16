#!/usr/bin/env python3
"""실기 B601 팔로워에서 plan.json (tools/ik_sim_run.py --save-plan) 을 재생한다. 라즈베리파이에서 실행.

준비: 이 파일과 ../arm_kinematics/timeline.py 를 라파의 한 폴더에 복사 (numpy 만 필요, pinocchio 불필요).
  모바일 로봇 라파 (기본 --setup mobile, 팔 = SocketCAN can_arm_r):
    sudo ~/capstone/capstone_ws/can/can_up.sh                     # CAN 올리기 (can_arm_r 포함)
    ~/rebot_env/venv/bin/python play_plan.py --check-only          # 설정만 확인 (모터 연결 안 함)
  data_collection 팔 (USB, --setup data_collection):
    source <data_collection>/config.env && conda activate "$CONDA_ENV" && export PYTHONPATH="$LEROBOT_DIR/src"

  python3 play_plan.py plan.json --dry-run                 # 팔 연결 없이 재생 내용만 출력
  python3 play_plan.py --pose                              # 현재 모터 1~6 [도] 출력. 토크를 켜서 읽고, 받친 뒤 ENTER 로 해제
  python3 play_plan.py plan.json --stop-at approach        # 접근점까지만 (기본 속도 0.5)
  python3 play_plan.py plan.json --speed 1.0               # 끝까지, 계획 속도
  python3 play_plan.py --gripper open                      # 그리퍼만 열기 (팔은 지금 자세 유지), close 는 닫기
  python3 play_plan.py plan.json --grip-before open --grip-after close   # 열고 다가가서 → 끝에서 잡기
  python3 play_plan.py plan.json --grip-after open         # 들고 와서 → 끝에서 놓기

동작:
  - 팔로워 설정: mobile = 라파 ~/rebot_env/arm_follow_test.py 와 같음 (can_arm_r, id arm_r, 그리퍼 열림 340,
    중력 URDF ~/rebot_env 수정본, 관절 한계 collector USER_LIMITS_DEG, 그리퍼 900°/s). data_collection = config.env.
    둘 다 중력 보상 roll 45°, MIT, 100 Hz, 30°/s·120°/s², 명령 끊김 0.25 s, 끌림 15°,
    comm_loss_action 'hold' (경로 재생 중 끊겼을 때 home 으로 가면 선반에 부딪힐 수 있음).
  - 관절별 중력 배율 (--gravity-scale, 기본 wrist_flex=0.85) 을 collector 와 같은 방식으로 건다.
  - 시작 점검 (연결할 때마다): 모터 1~6 + 그리퍼 7 개가 모두 응답·켜짐(상태 1)·온도 정상이고, 1 초 동안 자세가 1° 넘게
    움직이지 않는지 (제대로 붙잡혔는지) 표로 보여 준다. 하나라도 아니면 아무것도 움직이지 않고 끝낸다.
    경로 재생 직전·복귀 직전에도 켜짐·오류를 다시 본다.
  - 현재 자세가 경로 시작과 --start-tol (기본 5°) 안이면 천천히 시작 자세로 맞춘 뒤 재생, 넘으면 중단.
  - 목표를 50 Hz 로 보낸다. 루프가 늦으면 경로 시각을 건너뛰지 않고 끊긴 데서 이어 간다 (재출발은 0.3 s 동안 서서히).
    끝나면 루프 타이밍 (최대 간격, 늦은 주기 수) 을 출력. 0.25 s 넘게 끊기면 팔로워가 정지하므로 이유를 알리고 종료.
  - 측정 위치가 목표보다 10° 넘게 늦으면 경로 시간을 멈추고 기다린다 (15° 끌림 정지 방지).
  - 늦은 상태가 --stall 초 (기본 1 s) 넘게 계속되면 부딪힌 것으로 보고 측정 자세로 고정(hold)하고 멈춘다
    (목표가 앞에 남아 장애물을 계속 미는 것을 막음). 그다음 ENTER 로 실제 팔이 있는 지점부터 같은 길로 복귀.
  - 리프트는 움직이지 않는다: plan 이 lift_fixed (ik_sim_run --zed-height) 가 아니면 실행 거부.
  - ik_sim_run 의 MuJoCo 검증을 통과한 plan (validated) 만 실행. 팔로워가 실제로 쓰는 관절 한계 밖 점이 있으면 거부.
  - 삽입 단계는 더 작은 지연(--insert-lag, 기본 5°)에서 멈춘다. 끝에서 --settle 초 안에 2° 안으로 못 들어오면 정지로 본다.
  - 그리퍼 (--grip-before: 재생 전, --grip-after: 끝에 도착했을 때만, 부딪혀 멈췄으면 안 함): 팔은 그 자리 목표를
    계속 보내며 그리퍼만 움직이고, 목표에 닿거나 멈출 때까지 기다린다. 팀 팔로워 그리퍼는 힘 제한 위치 모드라
    닫기(0°)를 보내면 물체에 막혀 멈춘다 → 멈춘 각도가 --grip-empty-deg (기본 −15°) 보다 열려 있으면 '잡힘',
    끝까지 닫히면 '빈손' 경고. 공개 각도 −270° = 완전 열림, 0° = 닫힘.
  - 끝나면 자세 유지 → Ctrl+C 또는 ENTER 로 같은 길 거꾸로 시작 자세 복귀, q 로 복귀 없이 종료. 그리퍼는 마지막 명령 그대로
    (잡았으면 쥔 채로 돌아감 — 삽입 구간을 거꾸로 빠지므로 면 위 물체는 끌린다. 들어 올리고 빠지는 길은 아직 없음).
  - 재생 중 Ctrl+C 는 재생한 구간만 거꾸로 복귀 (기본 시작 0° = home). 복귀 중 다시 Ctrl+C 는 hold·종료.
  - 복귀 시간: 지나온 구간을 원래 시간 그대로 거꾸로 (속도 배율·5차 곡선 여유를 다시 곱하지 않음).
  - 움직이기 직전마다 (시작 맞춤·그리퍼·재생·복귀) 모터 7 개가 0.2 s 동안 계속 켜짐인지 확인 (wait_enabled).
  - 종료는 팀 팔로워의 disconnect: 자세 유지한 채 '팔을 받친 뒤 ENTER' 를 기다렸다가 토크 해제.
"""
import argparse
import importlib.util
import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll"]  # 모터 1~6


def load_timeline_module():
    for cand in (HERE.parent / "arm_kinematics/timeline.py", HERE / "timeline.py"):
        if cand.is_file():
            spec = importlib.util.spec_from_file_location("timeline", cand)
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            return mod
    raise SystemExit("timeline.py 를 찾지 못함 (../arm_kinematics/timeline.py 또는 같은 폴더)")


def env(name, default):
    return os.environ.get(name, default)


# 모바일 로봇 라파 (2026-11-15 결정, ~/rebot_env/arm_follow_test.py 와 같은 값)
MOBILE = dict(port="can_arm_r", can_adapter="socketcan", id="arm_r", gripper_open_deg=340.0,
              gravity_urdf="/home/pi/rebot_env/B601_DM_wrist_d435i_no_plate.urdf",
              gripper_vel=900.0, gripper_acc=20000.0)
# collector/arm_limits.py USER_LIMITS_DEG (라파 ~/collector_run 에 있으면 그걸 씀)
USER_LIMITS_DEG = {"shoulder_lift": (-210.0, 1.0), "elbow_flex": (-200.0, 1.0),
                   "wrist_flex": (-85.0, 120.0), "wrist_roll": (-180.0, 170.0)}


def user_limits():
    try:
        sys.path.insert(0, os.path.expanduser("~/collector_run"))
        from collector.arm_limits import USER_LIMITS_DEG as lim
        return dict(lim), "~/collector_run/collector/arm_limits.py"
    except ImportError:
        return dict(USER_LIMITS_DEG), "play_plan.py 내장 (2026-11-15 값)"
    finally:
        sys.path.pop(0)


def make_robot(args):
    try:
        from lerobot.robots.rebot_b601_follower import RebotB601Follower, RebotB601FollowerRobotConfig
    except ImportError as e:
        hint = ("~/rebot_env/venv/bin/python 으로 실행했는지" if args.setup == "mobile"
                else "config.env 를 source 하고 conda 환경·PYTHONPATH 를 켰는지")
        raise SystemExit(f"LeRobot 팔로워를 못 불러옴 ({e}). {hint} 확인") from e
    if args.setup == "mobile":
        if not Path(MOBILE["gravity_urdf"]).is_file():
            raise SystemExit(f"중력 URDF 없음: {MOBILE['gravity_urdf']}")
        lim, src = user_limits()
        defaults = RebotB601FollowerRobotConfig(port=MOBILE["port"]).joint_limits
        limits = {**defaults, **lim, "gripper": (-270.0, 0.0)}
        cfg = RebotB601FollowerRobotConfig(
            port=MOBILE["port"], can_adapter=MOBILE["can_adapter"], id=MOBILE["id"],
            gripper_open_deg=MOBILE["gripper_open_deg"], joint_limits=limits,
            max_relative_target=15.0,
            gravity_comp=True, gravity_urdf_path=MOBILE["gravity_urdf"], gravity_mount_roll_deg=45.0,
            gravity_mount_pitch_deg=0.0, gravity_tau_scale=1.0, gravity_missing_finger_mass_kg=0.03,
            control_mode="mit", background_control=True, control_rate_hz=100.0, command_timeout_s=0.25,
            max_target_velocity_deg_s=30.0, max_target_acceleration_deg_s2=120.0,
            require_supported_release=True,
            gripper_max_velocity_deg_s=MOBILE["gripper_vel"], gripper_max_acceleration_deg_s2=MOBILE["gripper_acc"],
            comm_loss_action="hold",
        )
        print(f"설정 mobile: {MOBILE['port']} ({MOBILE['can_adapter']}), id {MOBILE['id']}, 그리퍼 열림 {MOBILE['gripper_open_deg']:.0f}, "
              f"중력 1.0 roll 45°, 관절 한계 {src}")
        return RebotB601Follower(cfg), cfg
    if "FOLLOWER_PORT" not in os.environ:
        raise SystemExit("FOLLOWER_PORT 가 없음 — data_collection 의 config.env 를 먼저 source 하세요")
    cfg = RebotB601FollowerRobotConfig(
        port=env("FOLLOWER_PORT", "/dev/ttyACM0"), motor_family=env("FOLLOWER_FAMILY", "dm"), id=env("FOLLOWER_ID", "follower"),
        max_relative_target=float(env("MAX_RELATIVE_TARGET", "15.0")),
        gravity_comp=env("GRAVITY_COMP", "1") == "1", gravity_urdf_path=env("GRAVITY_URDF", None),
        gravity_mount_roll_deg=float(env("GRAVITY_MOUNT_ROLL_DEG", "45.0")),
        gravity_mount_pitch_deg=float(env("GRAVITY_MOUNT_PITCH_DEG", "0.0")),
        gravity_tau_scale=float(env("GRAVITY_TAU_SCALE", "1.0")),
        gravity_missing_finger_mass_kg=float(env("GRAVITY_FINGER_MASS_KG", "0.03")),
        control_mode="mit", background_control=True,
        control_rate_hz=float(env("MOTOR_CONTROL_HZ", "100")),
        command_timeout_s=float(env("COMMAND_TIMEOUT_S", "0.25")),
        max_target_velocity_deg_s=float(env("MAX_TARGET_VELOCITY_DEG_S", "30")),
        max_target_acceleration_deg_s2=float(env("MAX_TARGET_ACCELERATION_DEG_S2", "120")),
        require_supported_release=True,
        gripper_max_velocity_deg_s=float(env("GRIPPER_MAX_VELOCITY_DEG_S", "300")),
        gripper_max_acceleration_deg_s2=float(env("GRIPPER_MAX_ACCELERATION_DEG_S2", "3000")),
        gripper_open_deg=float(env("GRIPPER_OPEN_DEG", "270")),
        comm_loss_action="hold",                        # 경로 재생 중 home 금지
        home_velocity_deg_s=float(env("HOME_VELOCITY_DEG_S", "20")),
        home_acceleration_deg_s2=float(env("HOME_ACCELERATION_DEG_S2", "60")),
    )
    return RebotB601Follower(cfg), cfg


def apply_gravity_scales(robot, spec: str):
    """collector.gravity_scaling.apply_joint_scales 와 같은 방식: compute() 결과에 관절별 배율."""
    scales = {}
    for item in filter(None, (s.strip() for s in spec.split(","))):
        name, value = item.split("=")
        name = NAMES[int(name) - 1] if name.isdigit() else name
        value = float(value)
        if name not in NAMES or not (math.isfinite(value) and 0.0 <= value <= 2.0):
            raise SystemExit(f"잘못된 --gravity-scale 항목 {item!r}")
        scales[name] = value
    gravity = getattr(robot, "_gravity", None)
    if not scales:
        return {}
    if gravity is None:
        raise SystemExit("중력 보상이 꺼져 있어 --gravity-scale 을 걸 수 없음 (GRAVITY_COMP=1?)")
    orig = gravity.compute

    def scaled(q):
        out = dict(orig(q))
        for n, f in scales.items():
            out[n] *= f
        return out
    gravity.compute = scaled
    return scales


def measured(robot) -> tuple[np.ndarray, float]:
    obs = robot.get_observation()
    return np.array([float(obs[f"{n}.pos"]) for n in NAMES]), float(obs.get("gripper.pos", 0.0))


def action(q_deg, gripper):
    a = {f"{n}.pos": float(v) for n, v in zip(NAMES, q_deg)}
    a["gripper.pos"] = float(gripper)
    return a


class Stalled(RuntimeError):
    """측정이 목표를 오래 못 따라옴 (부딪힘 추정). t_plan = 멈춘 경로 시각."""

    def __init__(self, t_plan, lag, q_meas):
        super().__init__(f"t={t_plan:.2f}s 에서 {lag:.1f}° 늦은 채 멈춤")
        self.t_plan, self.lag, self.q_meas = t_plan, lag, q_meas


GRIP_OPEN_DEG, GRIP_CLOSE_DEG = -270.0, 0.0
TEMP_MARGIN_C = 10.0          # 팔로워 max_temperature_c (75℃) 보다 이만큼 낮아야 시작


def motor_status(robot):
    """모터 1~6 + 그리퍼 상태표와 문제 목록. 상태 코드 (팀 DM 팔로워): 0 꺼짐, 1 켜짐, 그 밖 = 오류."""
    motors = getattr(robot, "motors", None)
    if not motors:
        return ["  (이 팔로워는 모터 상태를 주지 않음)"], []
    t_max = float(getattr(getattr(robot, "config", None), "max_temperature_c", 75.0))
    rows, bad = [], []
    for n in NAMES + ["gripper"]:
        label = f"모터 {NAMES.index(n) + 1}" if n in NAMES else "그리퍼"
        m = motors.get(n)
        st = m.get_state() if m is not None else None
        if st is None:
            rows.append(f"  {label:6s} {n:13s}  응답 없음")
            bad.append(f"{label} 응답 없음")
            continue
        code, temp = int(st.status_code), max(st.t_mos, st.t_rotor)
        word = {0: "꺼짐", 1: "켜짐"}.get(code, f"오류({code})")
        if code != 1:
            bad.append(f"{label} {word}")
        if temp >= t_max - TEMP_MARGIN_C:
            bad.append(f"{label} 온도 {temp:.0f}℃ (한도 {t_max:.0f}℃)")
        rows.append(f"  {label:6s} {n:13s}  {word:6s} {temp:3.0f}℃")
    err = getattr(robot, "_control_error", None)
    if err is not None:
        bad.append(f"팔로워 제어 오류: {err}")
    return rows, bad


def preflight(robot, hold_s=1.0, drift_deg=1.0):
    """연결 직후: 지금 자세로 붙잡고 hold_s 동안 움직임을 본 뒤 상태표를 출력. 반환 문제 목록 (비면 통과)."""
    robot.hold()
    q0, g0 = measured(robot)
    drift, t0 = 0.0, time.perf_counter()
    while time.perf_counter() - t0 < hold_s:
        q, g = measured(robot)
        drift = max(drift, float(np.max(np.abs(q - q0))), abs(g - g0))
        time.sleep(0.05)
    rows, bad = motor_status(robot)
    if drift > drift_deg:
        bad.append(f"붙잡은 뒤 {hold_s:.0f} s 동안 {drift:.1f}° 움직임 (제대로 안 붙잡힘)")
    print("시작 점검 (모터 1~6 + 그리퍼):")
    print("\n".join(rows))
    print(f"  붙잡은 뒤 {hold_s:.0f} s 움직임 최대 {drift:.2f}°")
    print("  → 통과" if not bad else "  → 문제: " + "; ".join(bad))
    return bad


def recheck(robot, what):
    """움직이기 직전: 7 개가 0.2 s 계속 켜짐인지 (wait_enabled) + 온도·제어 오류. 문제면 hold 후 RuntimeError."""
    wait_enabled(robot)
    _, bad = motor_status(robot)
    if bad:
        try:
            robot.hold()
        finally:
            raise RuntimeError(f"{what} 전 점검 실패: {'; '.join(bad)}")


def grip(robot, q_hold, target, rate_hz=50.0, timeout_s=4.0, tol_deg=2.0, still_deg=1.0, still_s=0.4,
         lag_stop_deg=10.0, label="[그리퍼] ", t_plan=0.0):
    """팔은 q_hold 목표를 계속 보내 그 자리에 두고, 그리퍼만 target [공개 도] 으로. 반환 (멈춘 각도, 이유).
    이유: 'reached' (목표 tol 안), 'blocked' (still_s 동안 still_deg 넘게 안 움직임 = 물체에 막힘), 'timeout'.
    팔이 lag_stop_deg 넘게 밀리면 (잡다가 물체·가구를 미는 경우) hold 후 Stalled."""
    period = 1.0 / rate_hz
    t0 = next_tick = time.perf_counter()
    hist = []                                       # (시각, 각도)
    while True:
        robot.send_action(action(q_hold, target))
        q_meas, g = measured(robot)
        now = time.perf_counter()
        lag = float(np.max(np.abs(np.asarray(q_hold) - q_meas)))
        if lag > lag_stop_deg:
            robot.hold()
            raise Stalled(t_plan, lag, q_meas)
        hist.append((now, g))
        if abs(g - target) <= tol_deg:
            why = "reached"
        else:
            old = [v for t, v in hist if now - t >= still_s]
            why = "blocked" if old and now - t0 > still_s and abs(g - old[-1]) < still_deg else None
            if why is None and now - t0 > timeout_s:
                why = "timeout"
        if why:
            print(f"{label}{target:.0f}° 명령 → {g:.1f}° 에서 {dict(reached='도착', blocked='막혀 멈춤', timeout='시간 초과')[why]}"
                  f" ({now - t0:.1f}s)", flush=True)
            return g, why
        next_tick += period
        if time.perf_counter() - next_tick > period:
            next_tick = time.perf_counter()
        time.sleep(max(0.0, next_tick - time.perf_counter()))


def do_grip(robot, q_hold, how, args, t_plan=0.0):
    """how = open/close. 반환 새 그리퍼 명령 [도]. 닫기면 잡힘/빈손을 알린다. t_plan = 지금 경로 시각 (멈췄을 때 복귀 기준)."""
    target = args.grip_open_deg if how == "open" else args.grip_close_deg
    g, why = grip(robot, q_hold, target, rate_hz=args.rate, timeout_s=args.grip_timeout, t_plan=t_plan)
    if how == "close":
        if g < args.grip_empty_deg:
            print(f"  잡힘: 손가락이 {g:.1f}° 에서 멈춤 (빈손이면 {args.grip_close_deg:.0f}° 근처까지 닫힘)")
        else:
            print(f"  ⚠ 빈손 추정: {g:.1f}° 까지 닫힘 (--grip-empty-deg {args.grip_empty_deg:.0f}° 보다 닫힘)")
    elif why != "reached":
        print(f"  ⚠ 다 안 열림: {g:.1f}° (목표 {target:.0f}°) — 손가락이 무언가에 걸렸는지 확인")
    return target


def play(robot, tl, gripper, rate_hz, lag_pause_deg, t_end=None, label="", stall_s=1.0,
         insert_lag_deg=None, settle_s=2.0, settle_deg=2.0, ramp_s=0.3, stats=None):
    """tl 을 rate_hz 로 재생. 끝 시각 반환.

    시간: 경로 시각은 벽시계가 아니라 '끊긴 데서 이어 가기'. 한 번에 최대 1.5 주기만 나아가고, 루프가
    2 주기 넘게 늦었거나 (GC·통신 지연) 지연 때문에 멈췄다가 다시 갈 때는 진행 속도를 0 → 1 로 ramp_s 동안
    올린다. 그래서 늦어도 목표가 건너뛰지 않고, 그 사이 멈춘 팔이 부드럽게 다시 출발한다 (대신 전체 시간이 늘어남).
    밀린 주기는 몰아 보내지 않고 버린다.
    안전: 측정이 lag_pause_deg (삽입 단계는 insert_lag_deg) 넘게 늦으면 경로 시각을 멈추고, 그 상태가 stall_s 넘게
    이어지면 hold 후 Stalled. 끝 시각에 닿은 뒤 settle_s 안에 settle_deg 안으로 못 들어와도 hold 후 Stalled.
    stats (dict) 에 루프 타이밍을 남긴다: max_gap_s, late_ticks, ticks.
    """
    period = 1.0 / rate_hz
    max_step = 1.5 * period
    t_end = tl.duration if t_end is None else t_end
    stats = {} if stats is None else stats
    stats.update(max_gap_s=0.0, late_ticks=0, ticks=0, t_plan=0.0)
    t_plan, rate, paused, stuck, settle = 0.0, 1.0, 0.0, 0.0, 0.0
    last_wall = next_tick = time.perf_counter()
    next_log = 0.0
    try:
        while True:
            now = time.perf_counter()
            dt = now - last_wall
            last_wall = now
            stats["ticks"] += 1
            stats["max_gap_s"] = max(stats["max_gap_s"], dt)
            if dt > 2 * period:
                stats["late_ticks"] += 1
                rate = 0.0                            # 끊겼다: 그 사이 팔로워는 마지막 목표에서 멈춤 → 천천히 재출발
            q_cmd, phase = tl.sample(t_plan)
            stats["t_plan"] = t_plan  # Ctrl+C 복귀는 실제 보낸 구간 안에서만 찾는다
            robot.send_action(action(q_cmd, gripper))
            q_meas, _ = measured(robot)
            lag = float(np.max(np.abs(q_cmd - q_meas)))
            limit = lag_pause_deg if insert_lag_deg is None or "insert" not in phase else min(lag_pause_deg, insert_lag_deg)
            if lag > limit:
                paused += dt
                stuck += dt
                rate = 0.0
                if stuck > stall_s:
                    robot.hold()                      # 측정 자세로 고정: 앞에 남은 목표를 버린다
                    raise Stalled(t_plan, lag, q_meas)
            else:
                stuck = 0.0
                rate = min(1.0, rate + dt / ramp_s)
                t_plan = min(t_plan + min(dt, max_step) * rate, t_end)
            if now >= next_log:
                print(f"{label}{t_plan:6.2f}/{t_end:5.2f}s {phase:10s} 늦음 {lag:5.1f}°"
                      + (f" (대기 누적 {paused:.1f}s)" if paused > 0 else ""), flush=True)
                next_log = now + 0.5
            if t_plan >= t_end:
                if lag < settle_deg:
                    return t_plan
                settle += dt
                if settle > settle_s:
                    robot.hold()
                    raise Stalled(t_plan, lag, q_meas)
            next_tick += period
            late = time.perf_counter() - next_tick
            if late > period:
                next_tick = time.perf_counter()       # 밀린 주기는 버림 (몰아 보내지 않음)
            time.sleep(max(0.0, next_tick - time.perf_counter()))
    finally:
        print(f"{label}루프: {stats['ticks']} 주기, 최대 간격 {1000 * stats['max_gap_s']:.0f} ms, "
              f"늦은 주기(> {2000 * period:.0f} ms) {stats['late_ticks']}", flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("plan", nargs="?", type=Path)
    ap.add_argument("--setup", choices=["mobile", "data_collection"], default="mobile",
                    help="팔로워 설정: mobile = 모바일 로봇 라파 (can_arm_r), data_collection = config.env (USB)")
    ap.add_argument("--check-only", action="store_true", help="팔로워 설정만 만들어 보고 끝 (모터 연결 안 함)")
    ap.add_argument("--pose", action="store_true", help="현재 모터 1~6 [도] 만 출력")
    ap.add_argument("--gripper", choices=["open", "close"], help="경로 없이 그리퍼만 열기/닫기 (팔은 지금 자세 유지)")
    ap.add_argument("--grip-before", choices=["open", "close", "keep"], default="keep", help="경로 재생 전 그리퍼")
    ap.add_argument("--grip-after", choices=["open", "close", "keep"], default="keep",
                    help="경로 끝에 도착하면 그리퍼 (close = 잡기, open = 놓기). 부딪혀 멈췄으면 하지 않음")
    ap.add_argument("--grip-open-deg", type=float, default=GRIP_OPEN_DEG, help="열기 목표 [공개 도], 기본 −270 = 완전 열림")
    ap.add_argument("--grip-close-deg", type=float, default=GRIP_CLOSE_DEG, help="닫기 목표 [공개 도], 기본 0 (힘 제한이라 물체에서 멈춤)")
    ap.add_argument("--grip-empty-deg", type=float, default=-15.0, help="닫은 뒤 이보다 더 닫혔으면 빈손으로 봄 [도]")
    ap.add_argument("--grip-timeout", type=float, default=4.0, help="그리퍼 동작 최대 시간 [s]")
    ap.add_argument("--dry-run", action="store_true", help="팔 연결 없이 재생 내용 출력")
    ap.add_argument("--speed", type=float, default=0.5, help="계획 속도 배율 (0~1], 기본 0.5")
    ap.add_argument("--stop-at", choices=["lift", "approach", "insert"], help="이 단계 끝에서 멈춤")
    ap.add_argument("--rate", type=float, default=50.0, help="목표 전송 주기 [Hz]")
    ap.add_argument("--start-tol", type=float, default=5.0, help="현재 자세 ↔ 경로 시작 허용 차 [도]")
    ap.add_argument("--lag-pause", type=float, default=10.0, help="이만큼 늦으면 경로 시간 멈춤 [도] (끌림 정지 15° 보다 작게)")
    ap.add_argument("--insert-lag", type=float, default=5.0, help="삽입 단계에서 멈추는 지연 [도]")
    ap.add_argument("--settle", type=float, default=2.0, help="끝 도착 후 --settle-deg 안으로 들어올 때까지 기다리는 시간 [s]")
    ap.add_argument("--settle-deg", type=float, default=2.0,
                    help="끝 도착으로 인정하는 오차 [도]. 실기 정상 오차(4번 처짐 등)로 끝에서 멈추면 4~5 로")
    ap.add_argument("--stall", type=float, default=1.0, help="이만큼 [s] 계속 늦으면 부딪힘으로 보고 고정·정지")
    ap.add_argument("--gravity-scale", default="wrist_flex=0.85", help="관절별 중력 배율, 예 'wrist_flex=0.85' 또는 '4=0.85', 빈 값이면 안 검")
    args = ap.parse_args()

    if not (-270.0 <= args.grip_open_deg < args.grip_close_deg <= 0.0):
        ap.error("그리퍼 각도: −270 ≤ --grip-open-deg < --grip-close-deg ≤ 0")
    if not (args.pose or args.gripper or args.check_only):
        if args.plan is None:
            ap.error("plan.json 이 필요 (또는 --pose / --gripper)")
        plan = json.loads(args.plan.read_text())
        T = load_timeline_module()
        v_limit = float(plan.get("v_limit_deg", 30.0))
        # 가속도: plan 값과 팔로워 설정(MAX_TARGET_ACCELERATION_DEG_S2) 중 작은 것
        a_limit = min(float(plan.get("a_limit_deg", 120.0)), float(env("MAX_TARGET_ACCELERATION_DEG_S2", "120")))
        tl = T.Timeline(plan["waypoints"], v_limit_deg=v_limit, speed=args.speed, a_limit_deg=a_limit)
        t_end = tl.phase_end(args.stop_at) if args.stop_at else tl.duration
        start = np.array([float(plan["waypoints"][0]["motors_deg"][str(i)]) for i in range(1, 7)])
        lift_mm = {w["lift_total_mm"] for w in plan["waypoints"]}
        print(f"경로 {args.plan}: 경로점 {len(plan['waypoints'])}, 재생 {t_end:.1f}s / 전체 {tl.duration:.1f}s "
              f"(속도 {args.speed}, 최고 {v_limit * args.speed:.0f}°/s·{a_limit:.0f}°/s²), 목표 {plan.get('target_xyz')}")
        print(f"  시작 자세 {np.round(start, 1).tolist()}, 리프트 {sorted(lift_mm)} mm, 한계 {plan.get('limits')}, "
              f"충돌 여유 {plan.get('collision', {}).get('margin_m')} m, 검증 {plan.get('validation')}")
        problems = []
        if not plan.get("validated"):
            problems.append("MuJoCo 검증을 통과한 plan 이 아님 (ik_sim_run.py 가 검증 통과 후에만 저장)")
        if not plan.get("lift_fixed") or len(lift_mm) != 1:
            problems.append("리프트가 고정된 plan 이 아님 (ik_sim_run.py --zed-height 로 다시 계획)")
        if problems:
            for msg in problems:
                print("  거부:", msg)
            return 2
        if args.dry_run:
            if args.grip_before != "keep":
                print(f"  재생 전 그리퍼 {args.grip_before}")
            for t in np.arange(0.0, t_end + 1e-9, 0.5):
                q, ph = tl.sample(t)
                print(f"  {t:6.2f}s {ph:10s} " + " ".join(f"{i}:{v:7.1f}" for i, v in enumerate(q, start=1)))
            if args.grip_after != "keep":
                print(f"  끝에서 그리퍼 {args.grip_after}")
            return 0

    robot, cfg = make_robot(args)
    if args.check_only:
        print("관절 한계 [도]:", {n: tuple(round(x, 1) for x in v) for n, v in robot._joint_limits.items()})
        print("--check-only: 모터에 연결하지 않았습니다.")
        return 0
    print(f"팔로워 연결: {cfg.port} (comm_loss_action=hold, 끌림 {cfg.max_relative_target}°)")
    robot.connect()
    try:
        scales = apply_gravity_scales(robot, args.gravity_scale)
        if scales:
            print("중력 배율:", scales)
        bad = preflight(robot)
        q0, gripper = measured(robot)
        print("현재 모터 1~6 [도]:", ",".join(f"{v:.2f}" for v in q0), f"  그리퍼 {gripper:.1f}")
        if args.pose:
            return 0 if not bad else 2
        if bad:
            print("시작 점검 실패 — 아무것도 움직이지 않고 끝냅니다 (팔은 지금 자세 유지).")
            return 2
        if args.gripper:
            recheck(robot, "그리퍼")
            do_grip(robot, q0, args.gripper, args)
            robot.hold()
            return 0
        bad = check_limits(robot, tl, t_end)
        if bad:
            print("거부: 팔로워의 실제 관절 한계 밖 점이 있음 (조용히 잘리면 검증한 경로와 달라짐):")
            for line in bad[:6]:
                print("  ", line)
            print("  → ik_sim_run.py --limits conservative (기본) 으로 다시 계획하세요.")
            return 2
        diff = float(np.max(np.abs(q0 - start)))
        if diff > args.start_tol:
            print(f"현재 자세가 경로 시작과 {diff:.1f}° 다름 (허용 {args.start_tol}°). "
                  f"ik_sim_run.py --start={','.join(f'{v:.2f}' for v in q0)} 로 다시 계획하세요.")
            return 2
        if diff > 0.5:
            align = T.Timeline([{"phase": "align", "motors_deg": {str(i): q0[i - 1] for i in range(1, 7)}},
                                {"phase": "align", "motors_deg": {str(i): start[i - 1] for i in range(1, 7)}}],
                               v_limit_deg=10.0, a_limit_deg=a_limit)
            recheck(robot, "시작 맞춤")
            play(robot, align, gripper, args.rate, args.lag_pause, label="[시작 맞춤] ", stall_s=args.stall,
                 settle_s=args.settle, settle_deg=args.settle_deg)
            robot.hold()          # ENTER 를 기다리는 동안 명령 끊김(0.25 s) 오류가 걸리지 않게 측정 자세 고정
        if args.grip_before != "keep":
            recheck(robot, "그리퍼")
            gripper = do_grip(robot, start, args.grip_before, args)
            robot.hold()
        input("\n경로 재생 시작: ENTER (재생 중 Ctrl+C = 같은 경로로 시작 자세 복귀)\n")
        recheck(robot, "경로 재생")
        progress = {"t_plan": 0.0}
        t_back_from = t_end
        try:
            play(robot, tl, gripper, args.rate, args.lag_pause, t_end=t_end, stall_s=args.stall,
                 insert_lag_deg=args.insert_lag, settle_s=args.settle, settle_deg=args.settle_deg, stats=progress)
            q_end, _ = measured(robot)
            print("도착. 측정 모터 1~6:", ",".join(f"{v:.1f}" for v in q_end))
            if args.grip_after != "keep":
                gripper = args.grip_open_deg if args.grip_after == "open" else args.grip_close_deg
                gripper = do_grip(robot, tl.sample(t_end)[0], args.grip_after, args, t_plan=t_end)
            robot.hold()
        except KeyboardInterrupt:
            robot.hold()
            q_now, _ = measured(robot)
            t_back_from = closest_time(tl, q_now, progress["t_plan"])
            print("\nCtrl+C — 이동을 멈추고 같은 경로로 시작 자세에 복귀합니다.")
            return_to_start(robot, T, tl, t_back_from, gripper, v_limit, a_limit, args)
            return 0
        except Stalled as e:
            t_back_from = closest_time(tl, e.q_meas, e.t_plan)
            print(f"\n⚠ 멈춤: {e} — 측정 자세로 고정함. Ctrl+C 또는 ENTER 로 같은 경로 복귀.")
        else:
            print("목표 자세 유지 중. Ctrl+C 또는 ENTER = 같은 경로로 시작 자세 복귀.")
        try:
            ans = input("\nCtrl+C / ENTER = 복귀, q = 복귀 없이 종료 (이후 받침 확인·토크 해제): ").strip().lower()
        except KeyboardInterrupt:
            ans = ""
        if ans != "q":
            return_to_start(robot, T, tl, t_back_from, gripper, v_limit, a_limit, args)
    except Stalled as e:
        print(f"\n⚠ 멈춤: {e} — 측정 자세로 고정한 채 종료합니다.")
        robot.hold()
    except RuntimeError as e:
        why = ("명령이 끊긴 시간이 팔로워 한도(0.25 s)를 넘어 팔로워가 정지함 — 루프 타이밍(위 '최대 간격')을 확인하세요"
               if "timeout" in str(e) else "점검 실패" if "점검 실패" in str(e) else "팔로워 오류")
        print(f"\n⚠ {why}: {e}. 팔로워는 자세를 유지합니다 (comm_loss_action=hold).")
        try:
            robot.hold()
        except Exception as e2:  # 이미 오류 상태면 hold 도 거부될 수 있음
            print("  hold 실패:", e2)
    except KeyboardInterrupt:
        print("\n중지 — 현재 자세 유지 (재생 전이거나 복귀가 중단됨)")
        robot.hold()
    finally:
        robot.disconnect()   # 팀 코드: 자세 유지 → '팔을 받친 뒤 ENTER' → 토크 해제
    return 0


def return_to_start(robot, T, tl, t_from, gripper, v_limit, a_limit, args):
    """측정 자세에서 출발해 재생한 구간만 거꾸로. 복귀 중 Ctrl+C 는 hold·종료."""
    robot.hold()
    recheck(robot, "복귀")
    q_now, _ = measured(robot)
    t_from = closest_time(tl, q_now, t_from)
    back = build_return_timeline(T, tl, t_from, q_now, v_limit, a_limit, args.speed)
    print(f"[복귀] 계획 시작 자세로 이동합니다 ({back.duration:.1f} s, 기본 계획은 0° home). 다시 Ctrl+C = 복귀 중지.")
    try:
        play(robot, back, gripper, args.rate, args.lag_pause, label="[복귀] ", stall_s=args.stall,
             insert_lag_deg=args.insert_lag, settle_s=args.settle, settle_deg=args.settle_deg)
    except Stalled as e:
        print(f"\n⚠ 복귀 중 멈춤: {e} — 현재 자세 유지. 팔을 받치고 확인하세요.")
    else:
        print("[복귀] 시작 자세 도착.")
    finally:
        robot.hold()


def build_return_timeline(T, tl, t_from, q_now, v_limit, a_limit, speed, min_s=0.5):
    """복귀 경로: 측정 자세 q_now → 경로의 t_from 지점 (짧은 맞춤) → 지나온 구간을 거꾸로 → 시작 자세.
    지나온 단계는 원래 시간 그대로 거꾸로 붙인다 (tl 에 이미 들어간 속도 배율·5차 곡선 여유를 다시 곱하지 않음).
    t_from 이 걸친 단계만 남은 길이로 새로 시간을 잡되, 원래 그 길이에 쓰던 시간보다 빠르게 하지 않는다 (삽입 3 cm/s 유지)."""
    t_from = min(max(float(t_from), 0.0), tl.duration)
    q_at = tl.sample(t_from)[0]

    def seg(phase, pts, floor=0.0):
        pts = np.asarray(pts, float)
        step = np.max(np.abs(np.diff(pts, axis=0)), axis=1) if len(pts) > 1 else np.zeros(0)
        arc = np.concatenate([[0.0], np.cumsum(step)])
        dur = max(min_s / speed, floor, (T.PEAK * arc[-1] / v_limit) / speed,
                  ((T.ACC_PEAK * arc[-1] / a_limit) ** 0.5) / speed)
        return (phase, pts, arc, dur)

    segs = [seg("back_align", [np.asarray(q_now, float), q_at])]
    k = min(int(np.searchsorted(tl.starts, t_from, side="right")) - 1, len(tl.segments) - 1)
    phase, pts, arc, dur = tl.segments[k]
    tau = min(max((t_from - tl.starts[k]) / dur, 0.0), 1.0)
    s = T._quintic(tau) * arc[-1]
    if s > 1e-9:
        j = min(int(np.searchsorted(arc, s, side="right")) - 1, len(arc) - 1)
        part = [q_at] + [pts[i] for i in range(j, -1, -1)]
        segs.append(seg(f"back_{phase}", part, floor=dur * s / max(arc[-1], 1e-12)))
    for p_, pts_, arc_, dur_ in reversed(tl.segments[:k]):
        segs.append((f"back_{p_}", pts_[::-1].copy(), arc_[-1] - arc_[::-1], dur_))
    back = T.Timeline.__new__(T.Timeline)
    back.segments = segs
    back.starts = np.concatenate([[0.0], np.cumsum([x[3] for x in segs])])
    back.duration = float(back.starts[-1])
    back.v_limit, back.a_limit = v_limit, a_limit
    return back


def wait_enabled(robot, stable_s=0.2, timeout_s=2.0, period_s=0.02):
    """모터 1~6 + 그리퍼가 stable_s 동안 계속 켜짐(상태 1)으로 응답하는지. timeout_s 안에 안 되면 hold 후 RuntimeError.
    팔로워 _read_feedback() (백그라운드 루프가 있으면 잠금 안에서 마지막 상태 사본) 을 쓴다. 제어 오류는 그대로 올린다."""
    read = getattr(robot, "_read_feedback", None)
    if read is None:
        return
    names = NAMES + ["gripper"]
    t0 = time.perf_counter()
    since = None
    while True:
        states = read()
        now = time.perf_counter()
        off = [n for n in names if states.get(n) is None or int(getattr(states[n], "status_code", -1)) != 1]
        if not off:
            since = now if since is None else since
            if now - since >= stable_s:
                return
        else:
            since = None
            if now - t0 >= timeout_s:
                robot.hold()
                raise RuntimeError("enable 확인 실패: " + ", ".join(
                    f"{n} 상태 {getattr(states.get(n), 'status_code', '응답 없음')}" for n in off))
        time.sleep(period_s)


def check_limits(robot, tl, t_end, step=0.05, tol=0.2):
    """팔로워가 실제로 쓰는 관절 한계 (연결 후 robot._joint_limits) 밖의 경로점 목록."""
    limits = getattr(robot, "_joint_limits", None)
    if not limits:
        print("주의: 팔로워 관절 한계를 읽지 못해 경로 한계 검사를 건너뜀")
        return []
    out = []
    for t in np.append(np.arange(0.0, t_end, step), t_end):
        q, ph = tl.sample(t)
        for n, v in zip(NAMES, q):
            lo, hi = limits[n]
            if v < lo - tol or v > hi + tol:
                out.append(f"t={t:.2f}s {ph} {n} {v:.1f}° (한계 {lo:.1f}~{hi:.1f})")
    return out


def return_waypoints(tl, t_from, q_now, step=0.05):
    """복귀 경로: 측정 자세 → (경로에서 가장 가까운 지점부터) 같은 길 거꾸로 → 시작.
    맨 앞이 측정 자세라 hold 뒤 첫 명령이 튀지 않는다."""
    first = {"t": 0.0, "phase": "back_align", "motors_deg": {str(i): float(q_now[i - 1]) for i in range(1, 7)}}
    rest = [{"t": t + 0.5, "phase": ph, "motors_deg": {str(i): float(q[i - 1]) for i in range(1, 7)}}
            for t, ph, q in _reverse_points(tl, t_from, step)]
    rest[0]["phase"] = "back_align"           # 측정 자세 → 경로 위 가장 가까운 점 (짧은 맞춤)
    return [first] + rest


def closest_time(tl, q_meas, t_max, step=0.02):
    """경로 [0, t_max] 에서 측정 자세와 가장 가까운 시각 (복귀를 실제 팔 위치에서 시작하려고)."""
    ts = np.append(np.arange(0.0, t_max, step), t_max)
    d = [float(np.max(np.abs(tl.sample(t)[0] - q_meas))) for t in ts]
    return float(ts[int(np.argmin(d))])


def _reverse_points(tl, t_end, step=0.05):
    """재생한 구간을 촘촘히 뽑아 거꾸로 (같은 길로 복귀). 시각도 거꾸로 붙여 단계별 시간이 줄지 않게 한다
    (Timeline 이 계획 시각 × 1.875 를 최소로 쓰므로 복귀는 갈 때보다 느리다)."""
    ts = np.append(np.arange(0.0, t_end, step), t_end)          # 끝점 포함
    pts = [(t, *tl.sample(t)) for t in ts]
    return [(round(t_end - t, 4), f"back_{ph}", q) for t, q, ph in reversed(pts)]


if __name__ == "__main__":
    sys.exit(main())
