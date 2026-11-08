#!/usr/bin/env python3
"""URDF → MJCF 생성 (URDF 를 고쳤을 때만 다시 실행). 시뮬은 결과 파일만 읽는다.

  python3 src/sim/scripts/build_mjcf.py            # capstone_ws 에서
  → src/description/mjcf/mobile_robot_right_arm.xml  (메쉬는 ../meshes 상대경로)
  python3 -m mujoco.viewer --mjcf=src/description/mjcf/mobile_robot_right_arm.xml   # 단독 보기

URDF 는 그대로 두고 (메쉬 단순화 없음, visual 전부 사용) MuJoCo 용으로만 덧붙인다:
  - base_link 에 freejoint (URDF 의 world→base_link 고정을 풂). URDF 의 'world' 링크 = 바닥 기준점
  - 질량: URDF 값 사용 (이동로봇 링크는 set_urdf_inertia.py 가 부품 무게표로 기록). URDF 에 없는 손가락만 MASS 임시값
  - 손목 D435 는 DAE → 같은 형상 STL 2개 (d435_0/1.stl, b601_dm 패키지에서 변환해 둔 것)
  - 접촉: 바퀴 원기둥(r 0.0525) ↔ 바닥만. 나머지 메쉬는 보기용 (충돌 메쉬는 그룹 3, 키 '3')
  - 조향: 기구 스토퍼 ±190° (URDF 는 continuous). 조향·구동 = 토크 모터 (시뮬 노드가 PD/PI 계산)
  - 리프트 2단 mimic, 그리퍼 mimic → equality. 리프트·팔·그리퍼 = 위치 액추에이터 (뷰어 Control 슬라이더)
  - 리프트·상판·팔은 중력보상 (자세 유지용 — 그 무게는 바퀴에 안 실림)
  - keyframe 'home' = launch 기본 자세 (joint2 −2, joint3 −1.2, joint4 −1, 그리퍼 0.035)
  - 카메라: 'wrist_color' (D435 컬러 광학 프레임), 'front' (전방 카메라)
"""
import argparse
import copy
import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

WS_SRC = Path(__file__).resolve().parents[2]
PKG = WS_SRC / "description"  # 패키지 이름은 mobile_robot_right_arm_description
URDF = PKG / "urdf" / "mobile_robot_right_arm.urdf"
OUT = PKG / "mjcf" / "mobile_robot_right_arm.xml"

MODULES = ["fr", "rr", "rl", "fl"]  # [1 FR, 2 RR, 3 RL, 4 FL]
WHEEL_RADIUS, WHEEL_HALF_WIDTH = 0.0525, 0.02
STEER_STOP = math.radians(190.0)
# [kg] URDF 에 inertial 이 없는 링크만 여기서 채움 (무게중심/관성 모양은 메쉬에서).
# 이동로봇 링크는 2026-11-07 부품 무게표로 URDF 에 직접 기록 → set_urdf_inertia.py
MASS = {
    "right_arm_finger_left_link": 0.03, "right_arm_finger_right_link": 0.03,  # b601_dm MJCF 와 같은 임시값
}
HOME = {"right_arm_joint2": -2.0, "right_arm_joint3": -1.2, "right_arm_joint4": -1.0, "right_arm_finger_left": 0.035}


def prepare_urdf(meshdir):
    root = ET.parse(URDF).getroot()
    for link in root.iter("link"):  # DAE → STL 2개
        for vis in list(link.findall("visual")):
            mesh = vis.find("geometry/mesh")
            if mesh is not None and mesh.get("filename", "").endswith("d435.dae"):
                mesh.set("filename", mesh.get("filename").replace("d435.dae", "d435_0.stl"))
                v2 = copy.deepcopy(vis)
                v2.find("geometry/mesh").set("filename", mesh.get("filename").replace("d435_0", "d435_1"))
                if v2.get("name"):
                    v2.set("name", v2.get("name") + "_1")
                link.insert(list(link).index(vis) + 1, v2)
    mimic = [(j.get("name"), j.find("mimic").get("joint"), float(j.find("mimic").get("multiplier", 1)),
              float(j.find("mimic").get("offset", 0))) for j in root.iter("joint") if j.find("mimic") is not None]
    mj = ET.SubElement(root, "mujoco")
    ET.SubElement(mj, "compiler", meshdir=str(meshdir), strippath="false", fusestatic="false",
                  discardvisual="false", balanceinertia="true")
    xml = ET.tostring(root, encoding="unicode").replace("package://mobile_robot_right_arm_description/meshes/", "")
    return xml, mimic


def quat_z_to(a):
    a = np.asarray(a, float) / np.linalg.norm(a)
    c = float(a[2])
    if c > 1 - 1e-9:
        return [1, 0, 0, 0]
    if c < -1 + 1e-9:
        return [0, 1, 0, 0]
    ax = np.cross([0, 0, 1], a)
    ax /= np.linalg.norm(ax)
    return [math.cos(math.acos(c) / 2), *(math.sin(math.acos(c) / 2) * ax)]


def build():
    xml, mimic = prepare_urdf(PKG / "meshes")
    spec = mujoco.MjSpec.from_string(xml)

    # 1) 질량: 메쉬 기반 관성(밀도 1000)을 한 번 계산 → 질량만 추정값으로 비례 조정
    probe = spec.copy().compile()
    for name, mass in MASS.items():
        bid = probe.body(name).id
        k = mass / probe.body_mass[bid]
        b = spec.body(name)
        b.explicitinertial = True
        b.mass = mass
        b.ipos = probe.body_ipos[bid].tolist()
        b.iquat = probe.body_iquat[bid].tolist()
        b.inertia = (probe.body_inertia[bid] * k).tolist()

    # 2) 장면
    spec.modelname = "mobile_robot_right_arm"
    spec.option.timestep = 0.002
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    spec.visual.global_.offwidth, spec.visual.global_.offheight = 1280, 960
    spec.visual.headlight.ambient = [0.35, 0.35, 0.35]
    spec.worldbody.add_light(pos=[1, -1, 3], dir=[-0.3, 0.3, -1], type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL)
    tex = spec.add_texture(name="grid", type=mujoco.mjtTexture.mjTEXTURE_2D, builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
                           rgb1=[0.82, 0.82, 0.85], rgb2=[0.7, 0.7, 0.74], width=512, height=512)
    mat = spec.add_material(name="grid", texrepeat=[8, 8], reflectance=0.05)
    mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = tex.name
    for g in spec.geoms:
        g.group = 2 if (g.contype == 0 and g.conaffinity == 0) else 3
        g.contype = g.conaffinity = 0
        if g.group == 3:
            g.rgba = [1.0, 0.4, 0.2, 0.3]
        elif g.material == "" and list(g.rgba) == [0.5, 0.5, 0.5, 1.0]:
            g.rgba = [0.62, 0.64, 0.68, 1.0]  # 재질 없는 이동로봇 메쉬 = 알루미늄 느낌
    spec.worldbody.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[10, 10, 0.05], material="grid",
                            contype=1, conaffinity=2, friction=[1.0, 0.005, 0.0001])

    # 3) 차체 자유
    spec.body("base_link").add_freejoint(name="base_free")

    # 4) 조향·구동
    for n in MODULES:
        sj, dj = spec.joint(f"steering_{n}_joint"), spec.joint(f"drive_{n}_joint")
        sj.limited, sj.range = mujoco.mjtLimited.mjLIMITED_TRUE, [-STEER_STOP, STEER_STOP]
        sj.armature, sj.damping = 0.006, 0.05
        dj.armature, dj.damping = 0.002, 0.01
        spec.body(f"wheel_{n}_link").add_geom(
            name=f"tire_{n}", type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[WHEEL_RADIUS, WHEEL_HALF_WIDTH, 0],
            quat=quat_z_to(dj.axis), contype=2, conaffinity=1, condim=3, friction=[1.0, 0.005, 0.0001],
            group=3, rgba=[0.2, 0.2, 0.2, 0.4])

    # 5) 중력보상 (리프트·상판·카메라·팔)
    for b in spec.bodies:
        if b.name.startswith(("lift_", "top_plate", "front_camera", "right_arm_")):
            b.gravcomp = 1.0

    # 6) mimic → equality
    for f, l, coef, off in mimic:
        e = spec.add_equality()
        e.type, e.objtype = mujoco.mjtEq.mjEQ_JOINT, mujoco.mjtObj.mjOBJ_JOINT
        e.name1, e.name2 = f, l
        e.data[:5] = [off, coef, 0, 0, 0]
    followers = {f for f, *_ in mimic}

    # 7) 액추에이터 (이름 = 관절 이름)
    for j in spec.joints:
        if j.type == mujoco.mjtJoint.mjJNT_FREE or j.name in followers:
            continue
        a = spec.add_actuator()
        a.name, a.target, a.trntype = j.name, j.name, mujoco.mjtTrn.mjTRN_JOINT
        if j.name.startswith(("steering_", "drive_")):
            a.set_to_motor()
            lim = 17.0 if j.name.startswith("steering_") else 6.0  # RobStride 02 피크 / BL41105 최대
            a.ctrlrange, a.ctrllimited = [-lim, lim], mujoco.mjtLimited.mjLIMITED_TRUE
        else:
            if j.name.startswith("lift_"):
                kp = 3000.0
            elif "finger" in j.name:
                kp, j.armature = 50.0, 0.001
            elif re.search(r"joint[456]$", j.name):
                kp, j.armature = 30.0, 0.01
            else:
                kp, j.armature = 200.0, 0.02
            a.set_to_position(kp=kp, dampratio=1.0)
            a.inheritrange = 1.0

    # 8) 카메라
    spec.body("right_arm_wrist_camera_color_optical_frame").add_camera(name="wrist_color", quat=[0, 1, 0, 0], fovy=60)
    spec.body("front_camera_link").add_camera(name="front", xyaxes=[0, -1, 0, 0, 0, 1], fovy=70)

    # 9) home keyframe
    m = spec.compile()
    qpos, ctrl = m.qpos0.copy(), np.zeros(m.nu)
    qpos[2] += 0.003  # 바퀴가 바닥에 살짝 떠서 시작
    for jn, v in HOME.items():
        qpos[m.joint(jn).qposadr[0]] = v
    qpos[m.joint("right_arm_finger_right").qposadr[0]] = -HOME["right_arm_finger_left"]
    for i in range(m.nu):
        if m.actuator_gaintype[i] == mujoco.mjtGain.mjGAIN_FIXED and m.actuator_biastype[i] == mujoco.mjtBias.mjBIAS_AFFINE:
            ctrl[i] = qpos[m.jnt_qposadr[m.actuator_trnid[i, 0]]]  # 위치 액추에이터 목표 = 시작 자세
    spec.add_key(name="home", qpos=qpos.tolist(), ctrl=ctrl.tolist())
    return spec


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-o", "--out", default=str(OUT))
    a = ap.parse_args()
    spec = build()
    m = spec.compile()
    out = Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    rel = Path(*[".."] * len(out.parent.relative_to(PKG).parts)) / "meshes" if out.parent.is_relative_to(PKG) else PKG / "meshes"
    xml = spec.to_xml()
    xml = re.sub(r'meshdir="[^"]*"', f'meshdir="{rel.as_posix()}"', xml, count=1)
    header = ("<!-- 자동 생성: src/sim/scripts/build_mjcf.py (원본 urdf/mobile_robot_right_arm.urdf). "
              "직접 고치지 말고 URDF/생성기를 고친 뒤 다시 실행. 질량은 추정값. -->\n")
    out.write_text(header + xml)
    # 저장한 파일을 다시 읽어 검증 (상대 메쉬 경로)
    m2 = mujoco.MjModel.from_xml_path(str(out))
    total = sum(m2.body_mass)
    print(f"saved {out}  bodies={m2.nbody} joints={m2.njnt} actuators={m2.nu} meshes={m2.nmesh} "
          f"총질량 {total:.1f} kg (중력보상 제외 {sum(m2.body_mass[b] for b in range(m2.nbody) if m2.body_gravcomp[b] == 0):.1f} kg)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
