#!/usr/bin/env python3
"""IK 비용이 가장 낮은 실행 가능 후보를 골라 MuJoCo 물리로 실제로 움직여 본다 (또는 --no-sim: 기구학 검증만).

  python3 tools/ik_sim_run.py --target desk --approach down
  python3 tools/ik_sim_run.py --target shelf3 --approach into --headless out/   # 창 없이 단계별 PNG + 보고
  python3 tools/ik_sim_run.py --target shelf4 --save-plan plan.json             # 실기용 경로 저장
  # 실기용: 라파에서 읽은 현재 팔 자세 + 줄자로 잰 ZED 높이 (리프트 고정)
  python3 tools/ik_sim_run.py --target shelf3 --start=1.6,-0.9,-3.0,0.5,-0.9,-0.6 --zed-height 0.885 --save-plan plan.json

시작 = --start 자세 (기본 0점), 리프트 = --zed-height 로 고정하거나 --lifts 후보.
순서: (리프트) → 접근점(목표에서 10/7/5 cm 뒤) → 직선으로 목표. 재생은 timeline.py 의 5차 곡선 (실기 실행기와 같음).
확인: 관절 추종 오차, 실제 시뮬 자세의 충돌(팔 ↔ 몸체·선반·책상, Pinocchio 메시), 최종 그리퍼 끝 오차, 베이스 밀림.
시뮬 장면은 팔 관절 범위를 실기 한계(collector USER_LIMITS_DEG + URDF)로 바꿔서 만든다.
--no-sim: MuJoCo 없이 (라파·젯슨에서 계획할 때). 실제로 재생될 5차 곡선 경로를 0.01 s 마다 뽑아
관절 한계·충돌(손가락 닫힘·벌림, 여유)·속도를 검사하고 끝 그리퍼 위치를 FK 로 확인한다. 통과하면 plan 저장.
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from arm_kinematics import B601Kinematics  # noqa: E402
from arm_kinematics.b601 import MOTORS  # noqa: E402
from arm_kinematics.collision import CollisionChecker  # noqa: E402
from arm_kinematics.gravity import conservative_limits  # noqa: E402
from arm_kinematics.ik import B601IK, describe  # noqa: E402
from arm_kinematics.motion import MotionPlanner  # noqa: E402
from arm_kinematics.timeline import Timeline, _quintic  # noqa: E402
from arm_kinematics.scene import (  # noqa: E402
    ALONG, APPROACH_HELP, FRONT, LATERAL, PRESETS, env_for, orientation_args, Placement, build_mujoco, preset_target, robot_to_room, room_to_robot, to_robot)

ROBOT_XML = ROOT.parent / "description/mjcf/mobile_robot_right_arm.xml"
FINGER_OPEN = 0.035
MAX_END_ERR_MM = 10.0        # 검증: 시뮬 끝 그리퍼 오차 상한
ZED_HEIGHT_AT_LIFT0 = 0.675   # 바닥 → ZED 삼각대 나사, 리프트 최저 (실측 2026-11-11)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--target", choices=PRESETS, default="desk")
    ap.add_argument("--xyz", type=float, nargs=3, help="로봇 바닥 기준 목표 [m]")
    ap.add_argument("--robot", type=float, nargs=3, metavar=("X", "Y", "YAW_DEG"))
    ap.add_argument("--approach", default="heading", help=APPROACH_HELP)
    ap.add_argument("--tilt", default="-10,45", help="heading 모드 기울기 범위 [도] (아래 +)")
    ap.add_argument("--front", type=float, default=FRONT, help="로봇 중심 → 가구 앞면 [m] (설 자리 거리)")
    ap.add_argument("--lateral", type=float, default=LATERAL, help="가구 가운데 y [m] (− = 로봇 오른쪽)")
    ap.add_argument("--along", type=float, default=ALONG, help="목표를 가구 가운데에서 y 로 옮긴 양 [m]")
    ap.add_argument("--height", type=float, default=0.05)
    ap.add_argument("--lifts", default="0,210,420,630")
    ap.add_argument("--speed", type=float, default=30.0, help="관절 최대 속도 [°/s] (실기 팔로워 제한 30)")
    ap.add_argument("--accel", type=float, default=120.0, help="관절 최대 가속도 [°/s²] (실기 팔로워 제한 120)")
    ap.add_argument("--start", help="시작 자세 모터 1~6 [도], 쉼표 (기본 0점). 음수로 시작하면 --start=-1,...")
    ap.add_argument("--zed-height", type=float, help="바닥 → ZED 삼각대 나사 높이 실측 [m]. 주면 리프트를 그 높이로 고정 "
                                                     "(리프트 최저에서 0.675 m)")
    ap.add_argument("--lift-free", action="store_true",
                    help="리프트도 IK 로 같이 고름 (--zed-height = 지금 높이, 이동량 비용). 실기 실행기는 아직 리프트를 못 움직여 거부함")
    ap.add_argument("--limits", choices=["conservative", "collector"], default="conservative",
                    help="관절 한계: conservative = collector ∩ 팀 중력 URDF (data_collection 팔로워 그대로일 때, 기본), "
                         "collector = 넓힌 한계 (팔로워도 collector 처럼 넓힌 URDF 를 쓸 때만)")
    ap.add_argument("--margin", type=float, default=0.02, help="가구 충돌 상자 여유 [m] (사방)")
    ap.add_argument("--no-env", action="store_true", help="가구 없이 (빈 공간 첫 실기 시험용, --xyz 와 같이)")
    ap.add_argument("--no-sim", action="store_true", help="MuJoCo 없이 기구학 검증만 (pinocchio·hpp-fcl·numpy 만 필요)")
    ap.add_argument("--headless", type=Path)
    ap.add_argument("--save-plan", type=Path, help="실기용 경로 JSON (모터 1~6 도, 리프트 mm, 시간)")
    args = ap.parse_args()

    kin = B601Kinematics()
    if args.limits == "conservative":
        kin.set_motor_limits_deg(conservative_limits(kin))
    print("관절 한계 [도]:", {n: (round(lo, 1), round(hi, 1)) for n, (lo, hi) in kin.motor_limits_deg().items()})
    if args.xyz:
        robot = Placement(args.robot[0], args.robot[1], math.radians(args.robot[2])) if args.robot else Placement(0, 0, 0)
        target, into = np.array(args.xyz), np.array([1.0, 0.0, 0.0])
    else:
        tgt_room, into_room, robot = preset_target(args.target, args.height, front=args.front, lateral=args.lateral, along=args.along)
        if args.robot:
            robot = Placement(args.robot[0], args.robot[1], math.radians(args.robot[2]))
        target = room_to_robot(tgt_room, robot)
        into = room_to_robot(robot_to_room([0, 0, 0], robot) + into_room, robot)
    ik_kw, plan_kw, approach = orientation_args(
        args.approach, into, [float(v) for v in args.tilt.split(",")])

    env_room = [] if args.no_env else env_for(args.target, args.front, args.lateral)
    if args.no_sim:
        sim_lim = kin.motor_limits_deg()
    else:
        import mujoco
        model, data = build_mujoco(ROBOT_XML, env_room, arm_limits_deg=kin.motor_limits_deg())   # 실기 한계로
        jid = {i: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"right_arm_joint{i}") for i in MOTORS}
        real = kin.motor_limits_deg()
        sim_lim = {i: (max(real[i][0], math.degrees(model.jnt_range[jid[i]][0])),
                       min(real[i][1], math.degrees(model.jnt_range[jid[i]][1]))) for i in MOTORS}

    checker = CollisionChecker(kin, to_robot(env_room, robot), margin=args.margin)   # 손가락 닫힘·벌림 둘 다 검사
    ik = B601IK(kin, collision=checker)
    if args.zed_height is not None:
        total = args.zed_height - ZED_HEIGHT_AT_LIFT0
        if not -0.005 <= total <= 2 * kin.lift_stage_max + 0.005:
            raise SystemExit(f"--zed-height {args.zed_height} → 리프트 {1000 * total:.0f} mm, 범위 0~{2000 * kin.lift_stage_max:.0f} 밖")
        lifts = [float(np.clip(total / 2, 0.0, kin.lift_stage_max))]
        print(f"리프트 고정: ZED {args.zed_height:.3f} m → 총 {2000 * lifts[0]:.0f} mm")
    else:
        lifts = [float(v) / 2000 for v in args.lifts.split(",")]
    start_deg = [float(v) for v in args.start.split(",")] if args.start else [0.0] * 6
    if len(start_deg) != 6:
        raise SystemExit("--start 는 모터 1~6 값 6개")
    if not kin.within_arm_limits(start_deg):
        print("주의: 시작 자세가 실기 한계 밖", kin.motor_limits_deg())
    start_lift = lifts[0] if args.zed_height is not None else 0.0
    t0 = time.time()
    lift_kw = dict(lift_free=True, start_lift_stage_m=start_lift) if args.lift_free else {}
    cands = ik.solve(target, lifts, n_random=32, seed=dict(zip(MOTORS, start_deg)), **lift_kw, **ik_kw)   # 시작 자세 = move 비용 기준
    planner = MotionPlanner(ik, limits_deg=sim_lim, max_joint_speed_deg=args.speed)
    start = dict(zip(MOTORS, start_deg))
    plan = planner.plan(start, target, cands, start_lift_stage_m=start_lift, **plan_kw)
    print(f"목표(로봇 기준) {np.round(target, 3)}  접근 {args.approach}  후보 {len(cands)} (OK {sum(c.ok for c in cands)})  "
          f"계획 {time.time() - t0:.1f}s")
    for why in (plan.tried if plan else []):
        print("  건너뜀:", why)
    if not plan or plan.candidate is None:
        print("실행 가능한 후보 없음")
        return 1
    print("선택:", describe(plan.candidate))
    tterms = " + ".join(f"{k} {v:.2f}" for k, v in plan.trajectory_terms.items())
    print(f"      경로 비용 {plan.trajectory_cost:.3f} = {tterms}  → 합 {plan.total_cost:.3f}")
    print(f"      경로 측정: {plan.trajectory_meas}")
    print("      비교한 경로 (합 / 자세 / 경로 / 리프트):",
          ", ".join(f"{a:.2f}/{b:.2f}/{c:.2f}/{2000 * d:.0f}mm" for a, b, c, d in plan.alternatives))
    wp_dicts = [{"t": round(w.t, 3), "phase": w.phase, "lift_total_mm": round(2000 * w.lift_stage_m, 1),
                 "motors_deg": {str(k): round(v, 3) for k, v in w.motors_deg.items()}} for w in plan.waypoints]
    lift_from, lift_to = plan.waypoints[0].lift_stage_m, plan.waypoints[-1].lift_stage_m
    lift_dur = abs(2 * (lift_to - lift_from)) / planner.lift_speed + 0.5
    tl = Timeline(wp_dicts, v_limit_deg=args.speed, phase_min_s={"lift": lift_dur}, a_limit_deg=args.accel)
    print(f"경로 {len(plan.waypoints)} 점, 재생 {tl.duration:.1f}s (5차 곡선, 최고 {args.speed:.0f}°/s), "
          f"접근 거리 {100 * plan.standoff:.0f} cm")
    t_lift_end = tl.phase_end("lift")

    def lift_at(t):
        return lift_from + (lift_to - lift_from) * _quintic(t / t_lift_end)

    if args.no_sim:
        res = kinematic_check(kin, checker, tl, lift_at, target, args.speed)
        print("\n== 기구학 검증 (MuJoCo 없음)")
        print(f"샘플 {res['samples']} 개, 끝 그리퍼 오차 {res['end_error_mm']:.2f} mm, 최고 관절 속도 {res['max_speed_deg_s']:.1f}°/s")
        for line in res["issues"][:6]:
            print("  문제:", line)
        validated = not res["issues"] and res["end_error_mm"] < MAX_END_ERR_MM
        print("검증", "통과" if validated else "실패")
        validation = {"method": "kinematic", **{k: v for k, v in res.items() if k != "issues"}, "collisions": 0}
        return write_plan(args, validated, validation, target, approach, plan, kin, checker, lift_to, wp_dicts)

    # ---- MuJoCo 초기화 ----
    mujoco.mj_resetDataKeyframe(model, data, 0)
    base = model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "base_free")]
    data.qpos[base:base + 2] = [robot.x, robot.y]
    data.qpos[base + 3:base + 7] = [math.cos(robot.yaw / 2), 0, 0, math.sin(robot.yaw / 2)]
    act = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, a): a for a in range(model.nu)}
    lift_q = [model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in ("lift_stage_1_joint", "lift_stage_2_joint")]
    arm_q = [model.jnt_qposadr[jid[i]] for i in MOTORS]
    for i, a in zip(MOTORS, arm_q):
        data.qpos[a] = math.radians(start_deg[i - 1])
    for a in lift_q:
        data.qpos[a] = lift_from
    finger = model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "right_arm_finger_left")]
    data.qpos[finger] = FINGER_OPEN
    data.qvel[:] = 0
    mujoco.mj_forward(model, data)

    def set_ctrl(q, s):
        for i in MOTORS:
            data.ctrl[act[f"right_arm_joint{i}"]] = q[i - 1]
        data.ctrl[act["lift_stage_1_joint"]] = s
        data.ctrl[act["right_arm_finger_left"]] = FINGER_OPEN

    def actual():
        q = np.array([data.qpos[a] for a in arm_q])
        s = float(data.qpos[lift_q[0]])
        x, y = data.qpos[base:base + 2]
        w, _, _, z = data.qpos[base + 3:base + 7]
        return q, s, Placement(float(x), float(y), 2 * math.atan2(z, w))

    renderer = shots = None
    if args.headless:
        args.headless.mkdir(parents=True, exist_ok=True)
        renderer = mujoco.Renderer(model, 720, 960)
        cam = mujoco.MjvCamera()
        cam.lookat[:] = robot_to_room(target, robot) - np.array([0, 0, 0.25])
        cam.distance, cam.azimuth, cam.elevation = 2.3, -35.0 + math.degrees(robot.yaw), -22.0
        shots = {}

    track_max, collisions, phase_end, done = 0.0, [], {}, {"ok": False}
    dt, T = model.opt.timestep, tl.duration + 1.5

    def command(t):
        q_deg, phase = tl.sample(t)
        return np.radians(q_deg), lift_at(t), phase

    def run(sync=None):
        nonlocal track_max
        steps = int(T / dt)
        last_phase = None
        for k in range(steps):
            t = k * dt
            q_cmd, s_cmd, phase = command(t)
            set_ctrl(q_cmd, s_cmd)
            mujoco.mj_step(model, data)
            if k % 25 == 0:
                q, s, _ = actual()
                if t > 0.3:
                    track_max = max(track_max, float(np.degrees(np.max(np.abs(q - q_cmd)))))
                pairs = checker.colliding_pairs(list(np.degrees(q)), max(0.0, min(0.315, s)))
                if pairs:
                    collisions.append((round(t, 2), phase, pairs[:2]))
            if phase != last_phase and last_phase is not None:
                phase_end[last_phase] = t
                if renderer is not None:
                    renderer.update_scene(data, cam); shots[last_phase] = renderer.render().copy()
            last_phase = phase
            if sync is not None and k % 8 == 0:
                if not sync(t):
                    return
        done["ok"] = True
        if renderer is not None:
            renderer.update_scene(data, cam); shots["final"] = renderer.render().copy()

    if args.headless:
        run()
    else:
        import mujoco.viewer
        with mujoco.viewer.launch_passive(model, data) as viewer:
            viewer.cam.lookat[:] = robot_to_room(target, robot) - np.array([0, 0, 0.25])
            viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 2.3, -35.0 + math.degrees(robot.yaw), -22.0
            wall = time.time()

            def sync(t):
                viewer.sync()
                lag = t - (time.time() - wall)
                if lag > 0:
                    time.sleep(lag)
                return viewer.is_running()
            run(sync)
            print("실행 끝 — 창을 닫으면 종료")
            while viewer.is_running():
                viewer.sync(); time.sleep(0.05)

    q, s, base_now = actual()
    s = float(np.clip(s, 0.0, kin.lift_stage_max))          # 시뮬 리프트가 상한을 살짝 넘을 수 있음
    ee = kin.fk(list(np.degrees(q)), s).translation          # 실제 베이스 기준
    ee_room = robot_to_room(ee, base_now)
    tgt_room = robot_to_room(target, robot)
    print("\n== 결과")
    print(f"그리퍼 끝 오차 {1000 * np.linalg.norm(ee_room - tgt_room):.1f} mm  (방 좌표, 베이스 밀림 포함)")
    print(f"베이스 밀림 {1000 * math.hypot(base_now.x - robot.x, base_now.y - robot.y):.1f} mm, "
          f"{math.degrees(base_now.yaw - robot.yaw):.2f}°")
    print(f"관절 추종 최대 오차 {track_max:.2f}°")
    print("충돌", "없음" if not collisions else f"{len(collisions)} 회, 처음: {collisions[0]}")
    err_mm = 1000 * float(np.linalg.norm(ee_room - tgt_room))
    validated = done["ok"] and not collisions and err_mm < MAX_END_ERR_MM
    print("검증", "통과" if validated else
          ("실패 — " + ("재생이 끝나기 전에 창을 닫음" if not done["ok"] else
                        "충돌" if collisions else f"끝 오차 {err_mm:.1f} mm > {MAX_END_ERR_MM} mm")))
    if shots:
        from PIL import Image
        for name, img in shots.items():
            Image.fromarray(img).save(args.headless / f"{name}.png")
        print("저장", args.headless, sorted(shots))
    validation = {"method": "mujoco", "end_error_mm": round(err_mm, 2), "tracking_max_deg": round(track_max, 2),
                  "collisions": len(collisions)}
    return write_plan(args, validated, validation, target, approach, plan, kin, checker, lift_to, wp_dicts)


def kinematic_check(kin, checker, tl, lift_at, target, v_limit, dt=0.01):
    """실제로 재생될 경로를 dt 마다 뽑아 한계·충돌·속도 검사, 끝 그리퍼 오차 [mm]."""
    lo, hi = np.degrees(kin.arm_lower), np.degrees(kin.arm_upper)
    issues, prev, vmax = [], None, 0.0
    ts = np.append(np.arange(0.0, tl.duration, dt), tl.duration)
    for t in ts:
        q, phase = tl.sample(t)
        s = float(np.clip(lift_at(t), 0.0, kin.lift_stage_max))
        if np.any(q < lo - 0.05) or np.any(q > hi + 0.05):
            issues.append(f"t={t:.2f}s {phase} 관절 한계 밖 {np.round(q, 1).tolist()}")
        elif checker.in_collision(list(q), s):
            issues.append(f"t={t:.2f}s {phase} 충돌 {checker.colliding_pairs(list(q), s)[:2]}")
        if prev is not None:
            vmax = max(vmax, float(np.max(np.abs(q - prev[1]))) / max(t - prev[0], 1e-9))
        prev = (t, q)
    if vmax > v_limit * 1.05:
        issues.append(f"관절 속도 {vmax:.1f}°/s > {v_limit}°/s")
    q_end, _ = tl.sample(tl.duration)
    end = kin.fk(list(q_end), float(np.clip(lift_at(tl.duration), 0.0, kin.lift_stage_max))).translation
    return {"samples": len(ts), "end_error_mm": round(1000 * float(np.linalg.norm(end - target)), 2),
            "max_speed_deg_s": round(vmax, 2), "issues": issues}


def write_plan(args, validated, validation, target, approach, plan, kin, checker, lift_to, wp_dicts):
    """검증 통과 시에만 plan.json 저장. 반환 = 종료 코드."""
    if not args.save_plan:
        return 0 if validated else 2
    if not validated:
        print(f"plan 저장 안 함: 검증 실패 ({args.save_plan})")
        return 2
    args.save_plan.parent.mkdir(parents=True, exist_ok=True)
    args.save_plan.write_text(json.dumps({
        "frame": "robot world (x 앞, y 왼쪽, z 바닥)", "target_xyz": np.round(target, 4).tolist(),
        "approach": None if approach is None else np.round(approach, 4).tolist(),
        "start_deg": {str(k): round(float(v), 3) for k, v in plan.waypoints[0].motors_deg.items()},
        "lift_fixed": args.zed_height is not None and len({w["lift_total_mm"] for w in wp_dicts}) == 1,
        "lift_total_mm": round(2000 * lift_to, 1),
        "lift_start_mm": wp_dicts[0]["lift_total_mm"],
        "zed_height_m": args.zed_height, "v_limit_deg": args.speed, "a_limit_deg": args.accel,
        "limits": args.limits, "limits_deg": {str(n): v for n, v in kin.motor_limits_deg().items()},
        "collision": {"margin_m": args.margin, "finger_openings_m": list(checker.finger_openings),
                      "env": "none" if args.no_env else args.target},
        "candidate": {"cost": plan.candidate.cost, "terms": plan.candidate.cost_terms,
                      "sigma_min": plan.candidate.sigma_min, "load": plan.candidate.load,
                      "trajectory_cost": plan.trajectory_cost, "trajectory_terms": plan.trajectory_terms,
                      "trajectory": plan.trajectory_meas, "total_cost": plan.total_cost},
        "validated": True, "validation": validation,
        "waypoints": wp_dicts,
    }, indent=1, ensure_ascii=False))
    print(f"plan 저장 (검증 통과, {validation['method']}):", args.save_plan)
    return 0


if __name__ == "__main__":
    sys.exit(main())
