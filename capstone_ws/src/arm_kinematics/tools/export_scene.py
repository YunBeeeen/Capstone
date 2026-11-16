#!/usr/bin/env python3
"""임무 장면 (로봇 + 책상 + 선반 + 책상 위 물체) 을 혼자 열리는 MJCF 폴더·zip 으로 내보내기.

  python3 tools/export_scene.py --repo                   # → description/mjcf/mission_scene.xml (레포 메시 그대로)
  python3 tools/export_scene.py ~/mission_scene          # → ~/mission_scene/ 와 ~/mission_scene.zip (혼자 열리는 폴더)

폴더 안: mission_scene.xml, meshes/ (장면이 쓰는 STL 만), README.txt.
받는 쪽은 mujoco 만 있으면 된다:  pip install mujoco && python3 -m mujoco.viewer --mjcf=mission_scene.xml
치수·자리는 arm_kinematics/scene.py (가구), objects.py (물체), mission.py (설 자리) 에서 온다 — 고치면 다시 내보내기.
"""
import argparse
import math
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from arm_kinematics.mission import (DESK_FRONT, LATERAL, LIFT_STAGE, SHELF_FRONT, SHELF_STAND,  # noqa: E402
                                    MissionSim)
from arm_kinematics.objects import OBJECTS  # noqa: E402
from arm_kinematics.scene import DESK_TOP, shelf_board_top  # noqa: E402

ROBOT_XML = ROOT.parent / "description/mjcf/mobile_robot_right_arm.xml"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out", type=Path, nargs="?", help="만들 폴더 (같은 이름의 .zip 도 만듦)")
    ap.add_argument("--repo", action="store_true", help="레포의 description/mjcf/mission_scene.xml 로 (메시 복사 없음)")
    ap.add_argument("--no-zip", action="store_true")
    args = ap.parse_args()
    if args.repo:
        import mujoco
        path = ROBOT_XML.parent / "mission_scene.xml"
        MissionSim(ROBOT_XML, log=lambda *a: None).save_scene(path)
        model = mujoco.MjModel.from_xml_path(str(path))
        print(f"{path} (body {model.nbody}, geom {model.ngeom}) — 열기: python3 -m mujoco.viewer --mjcf={path}")
        return 0
    if args.out is None:
        ap.error("폴더를 주거나 --repo")
    out = args.out.expanduser().resolve()
    if out.exists() and any(out.iterdir()):
        raise SystemExit(f"{out} 가 비어 있지 않음 — 다른 이름을 주거나 지우고 다시")
    (out / "meshes").mkdir(parents=True, exist_ok=True)

    sim = MissionSim(ROBOT_XML, log=lambda *a: None)
    spec = sim.spec
    src_dir = (ROBOT_XML.parent / (spec.meshdir or ".")).resolve()
    used = sorted({m.file for m in spec.meshes})
    for f in used:                                 # 하위 폴더 (arm/visual/…) 구조 그대로
        (out / "meshes" / f).parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src_dir / f, out / "meshes" / f)
    xml_path = out / "mission_scene.xml"
    sim.save_scene(xml_path)                       # 원래 메시 위치로 컴파일해 저장한 뒤 meshdir 만 바꿈
    text = xml_path.read_text()
    old = f'meshdir="{spec.meshdir}"'
    if text.count(old) != 1:
        raise SystemExit(f"저장된 XML 에서 {old} 를 못 찾음")
    xml_path.write_text(text.replace(old, 'meshdir="meshes"'))

    import mujoco                       # 내보낸 파일이 혼자 열리는지 확인
    model = mujoco.MjModel.from_xml_path(str(out / "mission_scene.xml"))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)

    objs = "\n".join(f"  - {n}: {o.label}" for n, o in OBJECTS.items())
    (out / "README.txt").write_text(f"""임무 장면 MJCF (capstone arm_kinematics/tools/export_scene.py 로 생성)

열기:  pip install mujoco && python3 -m mujoco.viewer --mjcf=mission_scene.xml
코드:  model = mujoco.MjModel.from_xml_path("mission_scene.xml"); mujoco.mj_resetDataKeyframe(model, data, 0)
       keyframe 'home' = 처음 상태 (로봇 책상 앞, 리프트 {2000 * LIFT_STAGE:.0f} mm, 팔 0점, 그리퍼 열림, 물체 제자리)

좌표 (m): 바닥 z=0, 로봇(책상 설 자리) 원점 +x 방향.
- 책상: 1.0 x 0.5, 윗면 {DESK_TOP:.2f}. 로봇 중심 → 앞면 {DESK_FRONT:.2f}, 책상 가운데 y {LATERAL:+.2f}
- 선반 (연구실 실측): 0.6 x 0.33 x 1.477, 판 윗면 {", ".join(f"{shelf_board_top(k):.2f}" for k in (1, 2, 3, 4))}
  로봇 뒤 {-SHELF_STAND.x:.0f} m, 그 설 자리(로봇이 {math.degrees(SHELF_STAND.yaw):.0f}° 돌아섬)에서 앞면 {SHELF_FRONT:.2f}
- 책상 위 물체 (자유 관절, 크기는 장난감 추정값):
{objs}
- 팔 관절 범위는 실기 보수 한계 (data_collection 팔로워 ∩ 팀 URDF) 로 바꿔 둠.
- 로봇 형상은 접촉 꺼짐 (contype 0, 원래 MJCF 그대로) — 가구·물체끼리만 부딪힘.
""")
    n_files = len(used) + 2
    print(f"{out}: mission_scene.xml + meshes {len(used)} 개 + README.txt — 다시 불러 확인함 "
          f"(body {model.nbody}, geom {model.ngeom}, 자유도 {model.nv})")
    if not args.no_zip:
        z = shutil.make_archive(str(out), "zip", root_dir=out.parent, base_dir=out.name)
        print(f"zip: {z} ({Path(z).stat().st_size / 1e6:.1f} MB, 파일 {n_files} 개)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
