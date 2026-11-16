#!/usr/bin/env python3
"""실기 B601 팔로워에서 plan.json (tools/ik_sim_run.py --save-plan) 을 재생한다. 라즈베리파이에서 실행.

준비 (팔로워가 연결된 라파, data_collection 환경):
  source <data_collection>/config.env && conda activate "$CONDA_ENV" && export PYTHONPATH="$LEROBOT_DIR/src"
  # 이 파일과 ../arm_kinematics/timeline.py 를 같은 상대 위치로 복사 (numpy 만 필요, pinocchio 불필요)

  python3 play_plan.py plan.json --dry-run                 # 팔 연결 없이 재생 내용만 출력
  python3 play_plan.py --pose                              # 현재 모터 1~6 [도] 출력. 토크를 켜서 읽고, 받친 뒤 ENTER 로 해제
  python3 play_plan.py plan.json --stop-at approach        # 접근점까지만 (기본 속도 0.5)
  python3 play_plan.py plan.json --speed 1.0               # 끝까지, 계획 속도

동작:
  - config.env 와 같은 팔로워 설정 (중력 보상 roll 45°, MIT, 100 Hz, 30°/s·120°/s², 명령 끊김 0.25 s, 끌림 15°).
    단 comm_loss_action 은 'hold' 로 고정 (경로 재생 중 끊겼을 때 home 으로 가면 선반에 부딪힐 수 있음).
  - 관절별 중력 배율 (--gravity-scale, 기본 wrist_flex=0.85) 을 collector 와 같은 방식으로 건다.
  - 현재 자세가 경로 시작과 --start-tol (기본 5°) 안이면 천천히 시작 자세로 맞춘 뒤 재생, 넘으면 중단.
  - 목표를 50 Hz 로 보낸다. 루프가 늦으면 경로 시각을 건너뛰지 않고 끊긴 데서 이어 간다 (재출발은 0.3 s 동안 서서히).
    끝나면 루프 타이밍 (최대 간격, 늦은 주기 수) 을 출력. 0.25 s 넘게 끊기면 팔로워가 정지하므로 이유를 알리고 종료.
  - 측정 위치가 목표보다 10° 넘게 늦으면 경로 시간을 멈추고 기다린다 (15° 끌림 정지 방지).
  - 늦은 상태가 --stall 초 (기본 1 s) 넘게 계속되면 부딪힌 것으로 보고 측정 자세로 고정(hold)하고 멈춘다
    (목표가 앞에 남아 장애물을 계속 미는 것을 막음). 그다음 ENTER 로 실제 팔이 있는 지점부터 같은 길로 복귀.
  - 리프트는 움직이지 않는다: plan 이 lift_fixed (ik_sim_run --zed-height) 가 아니면 실행 거부.
  - ik_sim_run 의 MuJoCo 검증을 통과한 plan (validated) 만 실행. 팔로워가 실제로 쓰는 관절 한계 밖 점이 있으면 거부.
  - 삽입 단계는 더 작은 지연(--insert-lag, 기본 5°)에서 멈춘다. 끝에서 --settle 초 안에 2° 안으로 못 들어오면 정지로 본다.
  - 끝나면 자세 유지 → ENTER 로 같은 길 거꾸로 돌아가기, q 로 그대로 종료.
  - 종료(또는 Ctrl+C)는 팀 팔로워의 disconnect: 자세 유지한 채 '팔을 받친 뒤 ENTER' 를 기다렸다가 토크 해제.
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


def make_robot(args):
    try:
        from lerobot.robots.rebot_b601_follower import RebotB601Follower, RebotB601FollowerRobotConfig
    except ImportError as e:
        raise SystemExit(f"LeRobot 팔로워를 못 불러옴 ({e}). config.env 를 source 하고 conda 환경·PYTHONPATH 를 켰는지 확인") from e
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
    stats.update(max_gap_s=0.0, late_ticks=0, ticks=0)
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
    ap.add_argument("--pose", action="store_true", help="현재 모터 1~6 [도] 만 출력")
    ap.add_argument("--dry-run", action="store_true", help="팔 연결 없이 재생 내용 출력")
    ap.add_argument("--speed", type=float, default=0.5, help="계획 속도 배율 (0~1], 기본 0.5")
    ap.add_argument("--stop-at", choices=["lift", "approach", "insert"], help="이 단계 끝에서 멈춤")
    ap.add_argument("--rate", type=float, default=50.0, help="목표 전송 주기 [Hz]")
    ap.add_argument("--start-tol", type=float, default=5.0, help="현재 자세 ↔ 경로 시작 허용 차 [도]")
    ap.add_argument("--lag-pause", type=float, default=10.0, help="이만큼 늦으면 경로 시간 멈춤 [도] (끌림 정지 15° 보다 작게)")
    ap.add_argument("--insert-lag", type=float, default=5.0, help="삽입 단계에서 멈추는 지연 [도]")
    ap.add_argument("--settle", type=float, default=2.0, help="끝 도착 후 2° 안으로 들어올 때까지 기다리는 시간 [s]")
    ap.add_argument("--stall", type=float, default=1.0, help="이만큼 [s] 계속 늦으면 부딪힘으로 보고 고정·정지")
    ap.add_argument("--gravity-scale", default="wrist_flex=0.85", help="관절별 중력 배율, 예 'wrist_flex=0.85' 또는 '4=0.85', 빈 값이면 안 검")
    args = ap.parse_args()

    if not args.pose:
        if args.plan is None:
            ap.error("plan.json 이 필요 (또는 --pose)")
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
            for t in np.arange(0.0, t_end + 1e-9, 0.5):
                q, ph = tl.sample(t)
                print(f"  {t:6.2f}s {ph:10s} " + " ".join(f"{i}:{v:7.1f}" for i, v in enumerate(q, start=1)))
            return 0

    robot, cfg = make_robot(args)
    print(f"팔로워 연결: {cfg.port} (comm_loss_action=hold, 끌림 {cfg.max_relative_target}°)")
    robot.connect()
    try:
        scales = apply_gravity_scales(robot, args.gravity_scale)
        if scales:
            print("중력 배율:", scales)
        q0, gripper = measured(robot)
        print("현재 모터 1~6 [도]:", ",".join(f"{v:.2f}" for v in q0), f"  그리퍼 {gripper:.1f}")
        if args.pose:
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
            play(robot, align, gripper, args.rate, args.lag_pause, label="[시작 맞춤] ", stall_s=args.stall,
                 settle_s=args.settle)
            robot.hold()          # ENTER 를 기다리는 동안 명령 끊김(0.25 s) 오류가 걸리지 않게 측정 자세 고정
        input("\n경로 재생 시작: ENTER (멈추려면 Ctrl+C — 자세 유지)\n")
        t_back_from = t_end
        try:
            play(robot, tl, gripper, args.rate, args.lag_pause, t_end=t_end, stall_s=args.stall,
                 insert_lag_deg=args.insert_lag, settle_s=args.settle)
            q_end, _ = measured(robot)
            print("도착. 측정 모터 1~6:", ",".join(f"{v:.1f}" for v in q_end))
            robot.hold()
        except Stalled as e:
            t_back_from = closest_time(tl, e.q_meas, e.t_plan)
            print(f"\n⚠ 멈춤: {e} — 부딪힘 추정, 측정 자세로 고정함. 측정 모터 1~6:",
                  ",".join(f"{v:.1f}" for v in e.q_meas), f"(경로 {t_back_from:.2f}s 지점과 가장 가까움)")
        ans = input("\nENTER = 같은 길로 시작 자세까지 되돌아가기, q = 여기서 유지한 채 종료: ").strip().lower()
        if ans != "q":
            q_now, _ = measured(robot)
            back = T.Timeline(return_waypoints(tl, t_back_from, q_now), v_limit_deg=v_limit, speed=1.0,
                              a_limit_deg=a_limit)
            try:
                play(robot, back, gripper, args.rate, args.lag_pause, label="[복귀] ", stall_s=args.stall,
                     insert_lag_deg=args.insert_lag, settle_s=args.settle)
            except Stalled as e:
                print(f"\n⚠ 복귀 중에도 멈춤: {e} — 고정한 채 종료합니다. 팔을 받치고 직접 빼 주세요.")
            robot.hold()
    except Stalled as e:
        print(f"\n⚠ 멈춤: {e} — 측정 자세로 고정한 채 종료합니다.")
        robot.hold()
    except RuntimeError as e:
        why = ("명령이 끊긴 시간이 팔로워 한도(0.25 s)를 넘어 팔로워가 정지함 — 루프 타이밍(위 '최대 간격')을 확인하세요"
               if "timeout" in str(e) else "팔로워 오류")
        print(f"\n⚠ {why}: {e}. 팔로워는 자세를 유지합니다 (comm_loss_action=hold).")
        try:
            robot.hold()
        except Exception as e2:  # 이미 오류 상태면 hold 도 거부될 수 있음
            print("  hold 실패:", e2)
    except KeyboardInterrupt:
        print("\n중지 — 현재 자세 유지")
        robot.hold()
    finally:
        robot.disconnect()   # 팀 코드: 자세 유지 → '팔을 받친 뒤 ENTER' → 토크 해제
    return 0


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
