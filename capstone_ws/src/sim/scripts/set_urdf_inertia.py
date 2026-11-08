#!/usr/bin/env python3
"""부품 무게표(2026-11-07, 사용자 제공, 단위 g) → URDF 이동로봇 링크에 <inertial> 기록.

  python3 src/sim/scripts/set_urdf_inertia.py          # capstone_ws 에서. URDF 를 제자리 수정
  python3 src/sim/scripts/build_mjcf.py                # 그다음 MJCF 다시 생성

질량 = 아래 표의 부품 합. 무게중심·관성 **모양**은 링크 메쉬(균일 밀도 가정)에서 계산해 그 질량으로 맞춤
→ 배터리처럼 한쪽에 몰린 무게는 반영 안 됨 (대략값).
링크 배정은 메쉬 범위로 판단:
  base_link  342×432×562 mm, z 0.12~0.68 → 차체 프로파일·전장·배터리 + **RobStride 본체**(조향 링크는 z 0.169 에서 끝,
             조향축 0.189 위는 차체 쪽) + **리프트 고정 기둥**(리프트 1단은 z 0.62 부터)
  steering_* 102×102×142 mm → 조향과 같이 도는 바퀴 브라켓 + 02 어댑터
  wheel_*    105 mm → BL41105 허브모터 전체
  lift_stage_1/2 → Flexispot E5 의 움직이는 내관 2개 (E5 5550 g 을 고정부/1단/2단으로 나눈 **가정**)
  top_plate  273×242 mm → 250 mm 프로파일 2개
표에 없는 것(무게 미기재): Jetson AGX + 캐리어, 라즈베리파이 본체, 헤드 카메라 본체, 케이블·볼트 → 미포함.
팔(right_arm_*)은 Seeed URDF 값 유지 (표의 팔 부품은 이미 그 값에 포함된 것으로 봄).
"""
import re
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import build_mjcf  # noqa: E402  (URDF 전처리·경로 공유)

URDF = build_mjcf.URDF

E5 = 5550.0  # Flexispot E5 리프트 — 고정부(모터·외관) / 1단 / 2단 으로 나눔 (가정: 내관 강관 무게 추정)
E5_STAGE1, E5_STAGE2 = 1200.0, 1000.0

BASE_ITEMS = {  # [g]
    "배터리 VOLTON 24V 30Ah": 3400, "배터리 케이스": 330, "배터리 받침대": 175.5,
    "BTS7960 + 연결부": 57.5 + 27.1, "피코 + 받침대": 3.1 + 6.6,
    "컨버터 24V 30A + 받침대": 442.2 + 97.7, "컨버터 24V 6A + 받침대": 271.6 + 45.2, "PD 레귤레이터": 24.6,
    "라즈베리파이 케이스": 97.1, "JST 3P": 2.2, "USB CAN ×4": 67.2,
    "단자대 빨강+검정": 73.2 + 70.5, "비상정지": 70.3, "릴레이": 85.4, "퓨즈박스": 153.4,
    "XDrive(ODrive v3.6) ×2": 2 * 78.3, "드라이버 마운트 왼/오": 32.8 + 40.9,
    "프로파일 20x40 157 ×2": 203.4, "프로파일 20x40 342 ×2": 443.8,
    "프로파일 20x20 392": 280, "프로파일 20x20 342": 244,
    "브라켓·너트·볼트": 29.2 + 7.7 + 7.2 + 1.9 + 1.5,
    "RobStride 02 ×4": 1651.6, "02 베어링 ×4": 296.4, "02 플레이트 ×4": 542.8,
    "02 브릿지 ×4": 258.4, "02 브릿지(6줄) ×4": 502.4,
    "E5 고정부": E5 - E5_STAGE1 - E5_STAGE2,
}
STEER_ITEMS = {"BLDC 인휠 브라켓": 221.2, "02 어댑터": 51.3}
WHEEL_ITEMS = {"BL41105 허브모터": 1500}
TOP_ITEMS = {"프로파일 20x20 250": 94.6, "프로파일 20x40 250": 161.6}
FRONT_CAM_ITEMS = {"헤드 카메라 마운트": 28.8, "D435 본체 (데이터시트 72 g, 표에 없음 — 손목 카메라와 같은 값)": 72}
WRIST_MOUNT_ITEMS = {"팔 카메라 마운트 (2개 중 1개 — 지금 팔 1개)": 46.1}

MASS_G = {"base_link": sum(BASE_ITEMS.values()),
          "lift_stage_1_link": E5_STAGE1, "lift_stage_2_link": E5_STAGE2,
          "top_plate_link": sum(TOP_ITEMS.values()),
          "front_camera_link": sum(FRONT_CAM_ITEMS.values()),
          "right_arm_wrist_camera_mount_link": sum(WRIST_MOUNT_ITEMS.values())}
for m in ["fl", "fr", "rl", "rr"]:
    MASS_G[f"steering_{m}_link"] = sum(STEER_ITEMS.values())
    MASS_G[f"wheel_{m}_link"] = sum(WHEEL_ITEMS.values())


def mesh_inertia():
    """링크별 (질량, 무게중심, 3×3 관성) — URDF 를 MuJoCo 로 읽어 메쉬 기반으로."""
    import copy
    import xml.etree.ElementTree as ET
    xml, _ = build_mjcf.prepare_urdf(build_mjcf.PKG / "meshes")
    xml = xml.replace('balanceinertia="true"', 'balanceinertia="true" inertiafromgeom="true"')
    # MuJoCo 는 URDF visual 에 질량을 안 줌 → collision 없는 링크(예: 손목 카메라 마운트)는 visual 을 collision 으로 복사
    root = ET.fromstring(xml)
    for link in root.iter("link"):
        if link.get("name") in MASS_G and link.find("collision") is None:
            for v in link.findall("visual"):
                c = copy.deepcopy(v)
                c.tag = "collision"
                c.attrib.pop("name", None)
                c.attrib.pop("material", None)
                for mat in c.findall("material"):
                    c.remove(mat)
                link.append(c)
    xml = ET.tostring(root, encoding="unicode")
    m = mujoco.MjSpec.from_string(xml).compile()
    out = {}
    for name in MASS_G:
        b = m.body(name).id
        q = m.body_iquat[b]
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, q)
        R = R.reshape(3, 3)
        if m.body_mass[b] <= 0:
            sys.exit(f"{name}: 메쉬에서 질량을 못 구함")
        out[name] = (m.body_mass[b], m.body_ipos[b].copy(), R @ np.diag(m.body_inertia[b]) @ R.T)
    return out


def inertial_xml(mass, com, I, indent):
    f = lambda v: f"{v:.6g}"
    return (f'{indent}<inertial>\n'
            f'{indent}  <origin xyz="{f(com[0])} {f(com[1])} {f(com[2])}" rpy="0 0 0" />\n'
            f'{indent}  <mass value="{f(mass)}" />\n'
            f'{indent}  <inertia ixx="{f(I[0,0])}" ixy="{f(I[0,1])}" ixz="{f(I[0,2])}" '
            f'iyy="{f(I[1,1])}" iyz="{f(I[1,2])}" izz="{f(I[2,2])}" />\n'
            f'{indent}</inertial>\n')


def main():
    text = URDF.read_text()
    probe = mesh_inertia()
    for name, grams in MASS_G.items():
        mass = grams / 1000.0
        m0, com, I0 = probe[name]
        I = I0 * (mass / m0)
        m = re.search(rf'([ \t]*)<link name="{re.escape(name)}"\s*>\n', text)
        if not m:
            sys.exit(f"링크 {name} 없음")
        indent = m.group(1) + "  "
        start = m.end()
        end = text.index("</link>", start)
        body = text[start:end]
        body = re.sub(r'[ \t]*<inertial>.*?</inertial>\n', "", body, flags=re.S)  # 다시 돌려도 같은 결과
        text = text[:start] + inertial_xml(mass, com, I, indent) + body + text[end:]
        print(f"{name:36s} {mass:7.3f} kg  com {np.round(com, 3)}")
    URDF.write_text(text)
    print(f"이동로봇 합계 {sum(v for k, v in MASS_G.items() if not k.startswith('right_arm')) / 1000:.2f} kg → {URDF}")


if __name__ == "__main__":
    main()
