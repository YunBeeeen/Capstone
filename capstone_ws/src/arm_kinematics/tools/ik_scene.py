#!/usr/bin/env python3
"""MuJoCo 에서 IK 후보와 작업 공간을 본다 (4층 사다리 선반 + 책상).

  python3 tools/ik_scene.py --target desk --approach down
  python3 tools/ik_scene.py --target shelf3 --approach into --workspace
  python3 tools/ik_scene.py --target shelf3 --front 0.40 --lateral -0.2   # 설 자리 바꿔 보기
  python3 tools/ik_scene.py --target shelf4 --headless out/          # 창 없이 후보별 PNG

창 키: N / → 다음 후보, P / ← 이전 후보, W 작업 공간 점 켜기/끄기 (목표 높이 ±3 cm 단면).
목표 = 빨간 공, 접근축 = 빨간 선, 작업 공간 점 색 = σmin (노랑 클수록 특이점에서 멂, 보라 가까움).
물리는 돌리지 않고 자세만 놓는다 (mj_forward).
"""
import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from arm_kinematics import B601Kinematics  # noqa: E402
from arm_kinematics.collision import CollisionChecker  # noqa: E402
from arm_kinematics.ik import B601IK, describe  # noqa: E402
from arm_kinematics.scene import (  # noqa: E402
    ALONG, APPROACH_HELP, FRONT, LATERAL, PRESETS, env_for, orientation_args, Placement, build_mujoco, preset_target, robot_to_room, room_to_robot,
    set_mujoco_pose, to_robot)

ROBOT_XML = ROOT.parent / "description/mjcf/mobile_robot_right_arm.xml"
VIEW_AZIMUTH = -35.0                # 로봇 왼쪽 뒤에서 앞(가구 쪽)을 봄
LIFTS = (0.0, 0.105, 0.21, 0.315)   # 단 이동량 → 총 0 / 210 / 420 / 630 mm


def workspace_points(kin, checker, lift, z, n=4000, band=0.03, seed=0):
    """로봇 바닥 기준, 높이 z±band 에서 충돌 없이 닿는 tool0 위치와 σmin."""
    rng = np.random.default_rng(seed)
    lo, hi = np.degrees(kin.arm_lower), np.degrees(kin.arm_upper)
    pts, sig = [], []
    for _ in range(n):
        arm = list(rng.uniform(lo, hi))
        p = kin.fk(arm, lift).translation
        if abs(p[2] - z) > band or checker.in_collision(arm, lift):
            continue
        pts.append(p)
        sig.append(kin.manipulability(arm, lift)["sigma_min"])
    return np.array(pts).reshape(-1, 3), np.array(sig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--target", choices=PRESETS, default="desk")
    ap.add_argument("--xyz", type=float, nargs=3, help="목표 [m], 로봇 바닥 기준 (x 앞, y 왼쪽, z 바닥에서). --target 대신")
    ap.add_argument("--robot", type=float, nargs=3, metavar=("X", "Y", "YAW_DEG"), help="방 좌표 로봇 자리 (기본: 목표별)")
    ap.add_argument("--approach", default="heading", help=APPROACH_HELP)
    ap.add_argument("--tilt", default="-10,45", help="heading 모드 기울기 범위 [도] (아래 +)")
    ap.add_argument("--front", type=float, default=FRONT, help="로봇 중심 → 가구 앞면 [m] (설 자리 거리)")
    ap.add_argument("--lateral", type=float, default=LATERAL, help="가구 가운데 y [m] (− = 로봇 오른쪽)")
    ap.add_argument("--along", type=float, default=ALONG, help="목표를 가구 가운데에서 y 로 옮긴 양 [m]")
    ap.add_argument("--height", type=float, default=0.05, help="면 위 목표 높이 [m] (preset)")
    ap.add_argument("--lifts", default=",".join(str(2000 * s) for s in LIFTS), help="총 리프트 mm 목록")
    ap.add_argument("--seeds", type=int, default=24)
    ap.add_argument("--top", type=int, default=8, help="출력/표시할 후보 수")
    ap.add_argument("--workspace", action="store_true", help="처음부터 작업 공간 점 표시")
    ap.add_argument("--headless", type=Path, help="창 없이 후보별 PNG 를 이 폴더에 저장")
    args = ap.parse_args()

    kin = B601Kinematics()
    if args.xyz:
        robot = Placement(*args.robot[:2], math.radians(args.robot[2])) if args.robot else Placement(0, 0, 0)
        target = np.array(args.xyz)
        into = np.array([1.0, 0.0, 0.0])
    else:
        tgt_room, into_room, robot = preset_target(args.target, args.height, front=args.front, lateral=args.lateral, along=args.along)
        if args.robot:
            robot = Placement(args.robot[0], args.robot[1], math.radians(args.robot[2]))
        target = room_to_robot(tgt_room, robot)
        into = room_to_robot(robot_to_room([0, 0, 0], robot) + into_room, robot)
    ik_kw, plan_kw, approach = orientation_args(
        args.approach, into, [float(v) for v in args.tilt.split(",")])
    lifts = [float(v) / 2000 for v in args.lifts.split(",")]

    env_room = env_for(args.target, args.front, args.lateral)
    checker = CollisionChecker(kin, to_robot(env_room, robot))
    ik = B601IK(kin, collision=checker)
    t = time.time()
    cands = ik.solve(target, lifts, n_random=args.seeds, **ik_kw)
    print(f"목표(로봇 기준) {np.round(target, 3)}  접근 {args.approach} {None if approach is None else np.round(approach, 2)}  "
          f"로봇 {robot}  후보 {len(cands)} (OK {sum(c.ok for c in cands)})  {time.time() - t:.1f}s")
    shown = cands[:args.top]
    for i, c in enumerate(shown):
        print(f"{i:2d} {describe(c)}")
        if c.collision_pairs:
            print("      충돌:", c.collision_pairs[:3])
    if not shown:
        print("후보 없음 — --front / --lateral 로 설 자리를 바꾸거나 --approach free / --lifts 를 바꿔 보세요.")
        return 1

    import mujoco
    model, data = build_mujoco(ROBOT_XML, env_room, arm_limits_deg=kin.motor_limits_deg())
    target_room = robot_to_room(target, robot)
    ws_cache = {}

    def ws_for(c):
        key = c.lift_stage_m
        if key not in ws_cache:
            pts, sig = workspace_points(kin, checker, key, target[2])
            ws_cache[key] = ([robot_to_room(p, robot) for p in pts], sig)
        return ws_cache[key]

    def decorate(scn, c, show_ws, reset=True):
        if reset:
            scn.ngeom = 0

        def add(gtype, size, pos, rgba, mat=np.eye(3)):
            if scn.ngeom >= scn.maxgeom:
                return
            mujoco.mjv_initGeom(scn.geoms[scn.ngeom], gtype, np.asarray(size, float), np.asarray(pos, float),
                                np.asarray(mat, float).flatten(), np.asarray(rgba, np.float32))
            scn.ngeom += 1
        add(mujoco.mjtGeom.mjGEOM_SPHERE, [0.015, 0, 0], target_room, [0.9, 0.1, 0.1, 0.9])
        if approach is not None and scn.ngeom < scn.maxgeom:
            a = robot_to_room(approach, robot) - robot_to_room([0, 0, 0], robot)
            g = scn.geoms[scn.ngeom]
            mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3), np.eye(3).flatten(),
                                np.array([0.9, 0.1, 0.1, 1], np.float32))
            mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, 0.004, target_room - 0.12 * a, target_room)
            scn.ngeom += 1
        if show_ws:
            pts, sig = ws_for(c)
            for p, s in zip(pts, sig):
                u = float(np.clip(s / 0.05, 0, 1))
                add(mujoco.mjtGeom.mjGEOM_SPHERE, [0.006, 0, 0], p, [0.27 + 0.7 * u, 0.0 + 0.85 * u, 0.33 - 0.2 * u, 0.8])

    def show(idx):
        c = shown[idx]
        set_mujoco_pose(model, data, robot, c.motors_deg, c.lift_stage_m)
        return c

    if args.headless:
        args.headless.mkdir(parents=True, exist_ok=True)
        renderer = mujoco.Renderer(model, 720, 960)
        cam = mujoco.MjvCamera()
        cam.lookat[:] = target_room - np.array([0, 0, 0.25])
        cam.distance, cam.azimuth, cam.elevation = 2.3, VIEW_AZIMUTH + math.degrees(robot.yaw), -22.0
        from PIL import Image
        for i in range(len(shown)):
            c = show(i)
            renderer.update_scene(data, cam)
            decorate(renderer.scene, c, args.workspace, reset=False)
            out = args.headless / f"cand_{i:02d}.png"
            Image.fromarray(renderer.render()).save(out)
            print("저장", out)
        return 0

    import mujoco.viewer
    state = {"i": 0, "ws": args.workspace, "dirty": True}

    def on_key(key):
        if key in (78, 262):        # N, →
            state["i"] = (state["i"] + 1) % len(shown)
        elif key in (80, 263):      # P, ←
            state["i"] = (state["i"] - 1) % len(shown)
        elif key == 87:             # W
            state["ws"] = not state["ws"]
        else:
            return
        state["dirty"] = True

    with mujoco.viewer.launch_passive(model, data, key_callback=on_key) as viewer:
        viewer.cam.lookat[:] = target_room - np.array([0, 0, 0.25])
        viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 2.3, VIEW_AZIMUTH + math.degrees(robot.yaw), -22.0
        while viewer.is_running():
            if state["dirty"]:
                state["dirty"] = False
                c = show(state["i"])
                with viewer.lock():
                    decorate(viewer.user_scn, c, state["ws"])
                print(f"\n[{state['i'] + 1}/{len(shown)}] {describe(c)}")
                if hasattr(viewer, "set_texts"):
                    try:
                        viewer.set_texts((None, None, f"후보 {state['i'] + 1}/{len(shown)}  cost {c.cost:.2f}",
                                          f"σ {c.sigma_min:.3f} 부하 {c.load:.2f} 리프트 {2000 * c.lift_stage_m:.0f}mm"))
                    except Exception:
                        pass
                viewer.sync()
            time.sleep(0.02)
    return 0


if __name__ == "__main__":
    sys.exit(main())
