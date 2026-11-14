#!/usr/bin/env python3
"""실제 로봇 URDF (팔 달린 것) → Gazebo 시험용 로봇  urdf/mobile_robot_sim.urdf.xacro

  python3 src/gazebo/tools/make_sim_robot.py <mobile_robot_right_arm.urdf>
  ros2 launch capstone_gazebo world.launch.py robot:=$(ros2 pkg prefix capstone_gazebo)/share/capstone_gazebo/urdf/mobile_robot_sim.urdf.xacro

바꾸는 것 (주행·카메라 시험용 — 바퀴·팔 물리는 안 씀, planar_move 가 차체를 직접 움직임):
  - 루트 world → base_footprint (Gazebo 에서 world 는 예약어)
  - 움직이는 관절 전부 fixed (팔 = 전부 0 인 접힌 주행 자세, 리프트 최저)
  - 충돌 형상은 전부 빼고 바닥에서 띄운 상자 하나 (실기 Nav2 외곽과 같은 크기)
  - 메시 경로 package:// → file://$(find mobile_robot_description)/  (손목 카메라 메시 두 개는 저장소 쪽 위치로)
  - ZED 장착: 실측·보정값으로 — 높이 0.675 m (바닥→삼각대 나사), 숙임 22.2° (ZED pose 20.9° + 보정 1.3°), 옆 기울기 0.7°
  - URDF 의 zed_* 기준 프레임은 빼고 sim_sensors.xacro 의 zed2i_sim (RGB-D + IMU, 실기 토픽 이름) 을 같은 자리에 붙임
"""
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

CAM_Z, CAM_PITCH, CAM_ROLL = 0.675, math.radians(20.9 + 1.3), math.radians(0.7)
MESH_MOVED = {"meshes/arm/camera/realsense/d435.dae": "meshes/arm/camera/d435.dae",
              "meshes/arm/camera/mounts/D435_Gemini2_Mount.stl": "meshes/arm/camera/D435_Gemini2_Mount.stl"}
COLORS = (("wheel", "Gazebo/Black"), ("steering", "Gazebo/DarkGrey"), ("right_arm_link2", "Gazebo/Yellow"),
          ("right_arm_link3", "Gazebo/Yellow"), ("right_arm", "Gazebo/DarkGrey"), ("front_camera", "Gazebo/Black"),
          ("lift", "Gazebo/Grey"), ("top_plate", "Gazebo/Grey"), ("base_link", "Gazebo/Grey"))


def main():
    src = Path(sys.argv[1])
    root = ET.parse(src).getroot()
    # 높이 계산용: 관절 0 에서 world → front_camera_link z
    joints = {j.find("child").get("link"): j for j in root.findall("joint")}
    z, link = 0.0, "front_camera_link"
    while link in joints:
        j = joints[link]
        z += float(j.find("origin").get("xyz").split()[2])
        link = j.find("parent").get("link")
    for el in list(root):
        name = el.get("name", "")
        if el.tag in ("link", "joint") and name.startswith("zed_"):
            root.remove(el)
        elif el.tag == "link" and name == "world":
            el.set("name", "base_footprint")
        elif el.tag == "link":
            for c in el.findall("collision"):
                el.remove(c)
        elif el.tag == "joint":
            for tag in ("parent", "child"):
                if el.find(tag).get("link") == "world":
                    el.find(tag).set("link", "base_footprint")
            if el.get("type") != "fixed":
                el.set("type", "fixed")
                for tag in ("axis", "limit", "mimic", "dynamics", "safety_controller"):
                    for c in el.findall(tag):
                        el.remove(c)
            if name == "front_camera_joint":
                o = el.find("origin")
                x, y, zz = (float(v) for v in o.get("xyz").split())
                o.set("xyz", f"{x:.6f} {y:.6f} {zz + (CAM_Z - z):.6f}")
                o.set("rpy", f"{CAM_ROLL:.6f} {CAM_PITCH:.6f} 0")
    for m in root.iter("mesh"):
        f = m.get("filename").replace("package://mobile_robot_description/", "")
        m.set("filename", "file://$(find mobile_robot_description)/" + MESH_MOVED.get(f, f))
    body = "\n".join(ET.tostring(el, encoding="unicode").strip() for el in root)
    mats = "\n".join(f'  <gazebo reference="{l.get("name")}"><material>{next((c for k, c in COLORS if k in l.get("name")), "Gazebo/Grey")}</material></gazebo>'
                     for l in root.findall("link") if l.find("visual") is not None)
    out = f'''<?xml version="1.0"?>
<!-- 생성 파일 (tools/make_sim_robot.py {src.name}) — 직접 고치지 말 것.
     ZED: 높이 {CAM_Z} m, 숙임 {math.degrees(CAM_PITCH):.1f}°, 옆 기울기 {math.degrees(CAM_ROLL):.1f}° (실측·보정값). 관절은 전부 0 에 고정. -->
<robot name="mobile_robot_sim" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:arg name="cam_z" default="0"/>      <!-- world.launch 가 넘기는 인자 (여기선 안 씀 — 장착값은 위에 고정) -->
  <xacro:arg name="cam_pitch" default="0"/>
  <xacro:include filename="$(find capstone_gazebo)/urdf/sim_sensors.xacro"/>
{body}
  <!-- 주행용 충돌 상자: 실기 Nav2 외곽 (앞 0.26, 뒤 0.28, 왼 0.22, 오른 0.33 m) 을 감싸는 크기, 바닥에서 5 cm 띄움 -->
  <link name="sim_collision_link">
    <collision><origin xyz="-0.01 -0.055 0.35"/><geometry><box size="0.54 0.55 0.60"/></geometry></collision>
    <inertial><mass value="30.0"/><inertia ixx="1.0" ixy="0" ixz="0" iyy="1.0" iyz="0" izz="1.0"/></inertial>
  </link>
  <joint name="sim_collision_joint" type="fixed"><parent link="base_footprint"/><child link="sim_collision_link"/></joint>
  <xacro:zed2i_sim parent="front_camera_link" xyz="-0.005 0 0.015" rpy="0 0 0"/>
  <xacro:planar_move_sim base="base_footprint"/>
  <xacro:ground_truth_sim body="base_footprint"/>
  <xacro:no_gravity link="base_footprint"/>
{mats}
</robot>
'''
    dst = Path(__file__).resolve().parents[1] / "urdf" / "mobile_robot_sim.urdf.xacro"
    dst.write_text(out)
    print(f"{dst}  (카메라 원래 높이 {z:.4f} → {CAM_Z} m)")


if __name__ == "__main__":
    main()
