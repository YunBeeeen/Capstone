#!/usr/bin/env python3
"""MuJoCo 임무 시뮬: 책상 과일 → 바구니 → 바구니 들고 3 m 뒤 선반 3층 → 팔 빼기 (arm_kinematics/mission.py).

  python3 tools/mission_sim.py                          # MuJoCo 창, 1 회, 실시간
  python3 tools/mission_sim.py --headless out/          # 창 없이 단계별 PNG
  python3 tools/mission_sim.py --trials 10 --headless out/   # 정지 오차를 바꿔 10 회 → 성공률
  python3 tools/mission_sim.py --nav-sigma 0.03,0.03,3 --nav-max 0.05,0.05,5   # 정지 오차 크게
"""
import argparse
import math
import resource
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from arm_kinematics.mission import MissionSim  # noqa: E402

ROBOT_XML = ROOT.parent / "description/mjcf/mobile_robot_right_arm.xml"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--trials", type=int, default=1)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--nav-sigma", default="0.02,0.02,2", help="정지 오차 표준편차: 앞뒤 m, 좌우 m, 방향 도")
    ap.add_argument("--nav-max", default="0.03,0.03,3", help="정지 오차 최대")
    ap.add_argument("--perception-noise", type=float, default=0.0, help="물체 위치 인식 오차 표준편차 [m]")
    ap.add_argument("--speed", type=float, default=30.0, help="관절 최대 속도 [°/s]")
    ap.add_argument("--headless", type=Path, help="창 없이, 단계 끝마다 PNG 를 이 폴더에")
    args = ap.parse_args()
    sigma = tuple(float(v) for v in args.nav_sigma.split(","))
    nmax = tuple(float(v) for v in args.nav_max.split(","))

    wins = 0
    sim = MissionSim(ROBOT_XML, seed=args.seed, nav_sigma=sigma, nav_max=nmax,
                     perception_noise=args.perception_noise, speed_deg=args.speed)
    renderer = None
    if args.headless:
        import mujoco
        from PIL import Image
        args.headless.mkdir(parents=True, exist_ok=True)
        renderer = mujoco.Renderer(sim.model, 600, 800)
        cam = mujoco.MjvCamera()
    for trial in range(args.trials):
        print(f"\n===== 시행 {trial + 1}/{args.trials} (seed {args.seed + trial}) =====")
        sim.reset(args.seed + trial)
        shots = []
        if args.headless:

            def snap(label):
                r = sim.base_pose()
                cam.lookat[:] = [r.x + 0.25 * math.cos(r.yaw), r.y + 0.25 * math.sin(r.yaw), 0.75]
                cam.distance, cam.azimuth, cam.elevation = 2.4, math.degrees(r.yaw) - 125.0, -25.0
                renderer.update_scene(sim.data, cam)
                path = args.headless / f"t{trial + 1:02d}_{len(shots):02d}.png"
                Image.fromarray(renderer.render()).save(path)
                shots.append((label, path))
            t0 = time.time()
            from arm_kinematics.mission import MissionResult
            orig_add = MissionResult.add
            MissionResult.add = lambda self, name, ok, detail="": (orig_add(self, name, ok, detail), snap(name))[0]
            try:
                res = sim.run()
            finally:
                MissionResult.add = orig_add
        else:
            import mujoco.viewer
            t0 = time.time()
            with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
                viewer.cam.distance, viewer.cam.elevation = 3.5, -30.0
                wall, count = [time.time(), 0], [0]

                class Throttle:        # 물리 2 ms → 화면은 실시간으로
                    def sync(self):
                        count[0] += 1
                        if count[0] % 10 == 0:
                            viewer.cam.lookat[:] = [*sim.data.qpos[sim.base_adr:sim.base_adr + 2], 0.7]
                            viewer.sync()
                            lag = count[0] * sim.model.opt.timestep - (time.time() - wall[0])
                            if lag > 0:
                                time.sleep(lag)
                sim.viewer = Throttle()
                res = sim.run()
                sim.viewer = None
                print("끝 — 창을 닫으면 다음으로")
                while viewer.is_running():
                    viewer.sync()
                    time.sleep(0.05)
        for s in res.steps:
            print(f"  [{'OK' if s.ok else '실패'}] {s.name}" + (f" — {s.detail}" if s.detail else ""))
        print(f"  → {'성공' if res.success else '실패'} ({time.time() - t0:.0f} s)")
        wins += res.success
        print(f"  (메모리 최대 {resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1e6:.1f} GB)")
    if renderer is not None:
        renderer.close()
    if args.trials > 1:
        print(f"\n성공 {wins}/{args.trials}")
    return 0 if wins == args.trials else 2


if __name__ == "__main__":
    sys.exit(main())
