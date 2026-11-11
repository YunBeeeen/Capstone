#!/usr/bin/env python3
"""손목 카메라 D435i 를 URDF 4 개(오른팔·양팔 × 원본·충돌)와 SRDF 4 개에 넣음. 여러 번 실행해도 안전 (이미 있으면 건너뜀).

  python3 src/description/tools/add_wrist_camera.py      # capstone_ws 에서
URDF 를 새로 만들어 교체했을 때 (카메라 빠진 모델) 다시 실행. 넣을 내용 = tools/wrist_camera_d435i.xml
SRDF: 카메라 ↔ 그리퍼(right_arm_end_link)·link6 을 Adjacent 로 충돌 제외 (붙어 있어서 항상 겹침).
  → collision_check.launch.py 로 실제 자기충돌이 안 나는지 확인할 것.
"""
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]
PART = (PKG / "tools" / "wrist_camera_d435i.xml").read_text()
SRDF_PAIRS = [("right_arm_wrist_camera_link", "right_arm_end_link"), ("right_arm_wrist_camera_link", "right_arm_link6")]


def main():
    for p in sorted((PKG / "urdf").glob("mobile_robot_*.urdf")):
        s = p.read_text()
        if "right_arm_wrist_camera_link" in s:
            print(f"건너뜀 (이미 있음): {p.name}")
            continue
        if 'link name="right_arm_end_link"' not in s:
            print(f"건너뜀 (right_arm_end_link 없음): {p.name}")
            continue
        i = s.rindex("</robot>")
        p.write_text(s[:i] + PART.rstrip() + "\n" + s[i:])
        print(f"넣음: {p.name}")
    for p in sorted((PKG / "config").glob("mobile_robot_*.srdf")):
        s = p.read_text()
        add = [f'  <disable_collisions link1="{a}" link2="{b}" reason="Adjacent"/>'
               for a, b in SRDF_PAIRS if f'link1="{a}" link2="{b}"' not in s and f'link1="{b}" link2="{a}"' not in s]
        if not add:
            print(f"건너뜀 (이미 있음): {p.name}")
            continue
        i = s.rindex("</robot>")
        p.write_text(s[:i] + "  <!-- 손목 카메라 (tools/add_wrist_camera.py) -->\n" + "\n".join(add) + "\n" + s[i:])
        print(f"충돌 제외 {len(add)} 쌍: {p.name}")


if __name__ == "__main__":
    main()
