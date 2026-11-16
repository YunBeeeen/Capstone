#!/usr/bin/env python3
"""시뮬 장면 + 가상 ZED 로 '글자 → 상자 → 3D' 전체를 시험 (정답 위치를 알므로 오차를 잰다).

  ~/capstone_ml/bin/python tools/vision_server.py &                  # 먼저 서버 (다른 터미널)
  MUJOCO_GL=egl python3 tools/vision_sim_test.py --out out/          # 물체 8 개, OWLv2 상자 + SAM
  MUJOCO_GL=egl python3 tools/vision_sim_test.py --oracle --out out/ # 정답을 투영한 상자로 (3D 계산만 시험)

가상 ZED: 로봇 중심 앞 0.167 m, 왼쪽 렌즈 +0.06 m, 높이 = 리프트 높이 + 0.675 + 0.03, 아래로 21.6°, 1280×720,
세로 화각 69° (ZED 2i HD720 정렬 영상 근사). 깊이 = 광축 방향 거리 (ZED depth 와 같은 뜻).
시뮬 물체는 단순 형상이라 OWLv2 결과는 실물과 다르다 — 실물 인식은 ZED 사진으로 따로 봐야 한다.
"""
import argparse
import base64
import io
import json
import math
import sys
import urllib.request
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from arm_kinematics.mission import LIFT_STAGE, MissionSim  # noqa: E402
from arm_kinematics.objects import OBJECTS  # noqa: E402
from arm_kinematics.vision import estimate_object  # noqa: E402

ROBOT_XML = ROOT.parent / "description/mjcf/mobile_robot_right_arm.xml"
QUERIES = {"fruit": "an orange", "basket": "a basket", "eggplant": "an eggplant", "banana": "a banana",
           "carrot": "a carrot", "corn": "a corn cob", "chilli": "a red chili pepper", "pepper": "a green bell pepper"}
W, H, FOVY = 1280, 720, 69.0
CAM_X, CAM_Y, PITCH = 0.167, 0.06, math.radians(21.6)


def camera(zed_height):
    """(MjvCamera 인자, K, T 로봇←광학)."""
    pos = np.array([CAM_X, CAM_Y, zed_height + 0.03])
    f = np.array([math.cos(PITCH), 0.0, -math.sin(PITCH)])           # 앞, 아래로 숙임
    x_opt = np.array([0.0, -1.0, 0.0])                                # 오른쪽 = −y
    y_opt = np.cross(f, x_opt)                                        # 아래
    T = np.eye(4)
    T[:3, :3] = np.column_stack([x_opt, y_opt, f])
    T[:3, 3] = pos
    fy = (H / 2) / math.tan(math.radians(FOVY) / 2)
    K = (fy, fy, W / 2, H / 2)
    return pos, f, K, T


def project(K, T, p):
    fx, fy, cx, cy = K
    q = np.linalg.inv(T) @ np.append(p, 1.0)
    return np.array([fx * q[0] / q[2] + cx, fy * q[1] / q[2] + cy]), q[2]


def truth_center(sim, name):
    """정답 중심: 형상 상자 가운데 (바구니는 손잡이까지 높이의 가운데)."""
    o = OBJECTS[name]
    a = sim.obj_adr[name]
    pos = np.array(sim.data.qpos[a:a + 3])
    w, x, y, z = sim.data.qpos[a + 3:a + 7]
    yaw = 2 * math.atan2(z, w)
    c = np.array(o.bbox_center)
    R = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    return pos + R @ c, yaw


def oracle_box(sim, name, K, T):
    o = OBJECTS[name]
    c, yaw = truth_center(sim, name)
    R = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    half = np.array(o.bbox) / 2
    pts = [project(K, T, c + R @ (half * np.array(s)))[0] for s in np.array(np.meshgrid([-1, 1], [-1, 1], [-1, 1])).T.reshape(-1, 3)]
    pts = np.array(pts)
    return [*pts.min(axis=0), *pts.max(axis=0), 1.0, "oracle"]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--zed-height", type=float, default=0.675 + 2 * LIFT_STAGE)
    ap.add_argument("--url", default="http://127.0.0.1:5578")
    ap.add_argument("--oracle", action="store_true", help="OWLv2 대신 정답을 투영한 상자")
    ap.add_argument("--no-sam", action="store_true")
    ap.add_argument("--out", type=Path)
    args = ap.parse_args()

    import mujoco
    from PIL import Image, ImageDraw
    sim = MissionSim(ROBOT_XML, log=lambda *a: None)
    sim.settle(1.5)
    sim.model.vis.global_.fovy = FOVY
    pos, f, K, T = camera(args.zed_height)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.distance = 1.0
    cam.lookat[:] = pos + f * cam.distance
    cam.azimuth, cam.elevation = 0.0, -math.degrees(PITCH)
    r = mujoco.Renderer(sim.model, H, W)
    r.update_scene(sim.data, cam)
    rgb = r.render().copy()
    r.enable_depth_rendering()
    r.update_scene(sim.data, cam)
    depth = r.render().copy()
    r.close()
    # 투영 확인: 오렌지 중심 픽셀의 깊이 = 계산한 깊이
    uv, zc = project(K, T, truth_center(sim, "fruit")[0])
    print(f"카메라 확인: 오렌지 중심 픽셀 {uv.round(1)}, 깊이 렌더 {depth[int(uv[1]), int(uv[0])]:.3f} m vs 계산 {zc:.3f} m "
          f"(앞면이라 렌더가 반지름만큼 작아야 정상)")

    buf = io.BytesIO()
    Image.fromarray(rgb).save(buf, format="JPEG", quality=92)
    found = {}
    if not args.oracle:
        q = "|".join(QUERIES.values())
        url = f"{args.url}/find?q={urllib.parse.quote(q)}&sam={0 if args.no_sam else 1}"
        req = urllib.request.Request(url, data=buf.getvalue(), headers={"Content-Type": "image/jpeg"})
        with urllib.request.urlopen(req, timeout=60) as resp:
            ans = json.loads(resp.read())
        for b, m in zip(ans["boxes"], ans["masks"]):
            name = next(k for k, v in QUERIES.items() if v == b[5])
            if name not in found:
                mask = None
                if m:
                    mask = np.array(Image.open(io.BytesIO(base64.b64decode(m)))) > 0
                found[name] = (b, mask)
    else:
        for name in QUERIES:
            found[name] = (oracle_box(sim, name, K, T), None)

    img = Image.fromarray(rgb)
    dr = ImageDraw.Draw(img)
    print(f"\n{'물체':10s} {'점수':>5s}   수평 오차   높이 오차   추정")
    errs = []
    for name in QUERIES:
        gt, _ = truth_center(sim, name)
        if name not in found:
            print(f"{name:10s}   못 찾음")
            continue
        b, mask = found[name]
        est = estimate_object(depth, K, T, b, mask=mask)
        dr.rectangle(b[:4], outline=(255, 0, 0), width=3)
        dr.text((b[0] + 3, b[1] + 3), f"{name} {b[4]:.2f}", fill=(255, 0, 0))
        if est is None:
            print(f"{name:10s} {b[4]:5.2f}   3D 실패 (점 부족)")
            continue
        e_xy = 100 * float(np.linalg.norm(est.center[:2] - gt[:2]))
        e_z = 100 * float(est.center[2] - gt[2])
        errs.append(e_xy)
        print(f"{name:10s} {b[4]:5.2f}   {e_xy:6.1f} cm   {e_z:+6.1f} cm   {est}")
    if errs:
        print(f"\n수평 오차: 중앙값 {np.median(errs):.1f} cm, 최대 {max(errs):.1f} cm ({len(errs)}/{len(QUERIES)} 개)")
    if args.out:
        args.out.mkdir(parents=True, exist_ok=True)
        img.save(args.out / ("vision_oracle.png" if args.oracle else "vision_owl.png"))
        print("그림:", args.out / ("vision_oracle.png" if args.oracle else "vision_owl.png"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
