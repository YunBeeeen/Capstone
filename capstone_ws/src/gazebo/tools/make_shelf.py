#!/usr/bin/env python3
"""사다리형 선반 모델 생성 → models/ladder_shelf/model.sdf  (연구실 실물: 검은 틀, 판 4 장, 폭 0.6 × 깊이 0.33 × 높이 1.5 m)

  python3 src/gazebo/tools/make_shelf.py
월드에 넣지 않고 따로 스폰 (옮기는 시험을 하려고):
  ros2 run gazebo_ros spawn_entity.py -entity ladder_shelf -file <share>/models/ladder_shelf/model.sdf -x 4.3 -y 2.24 -Y 0
앞면 = 로컬 −y. 원점 = 바닥 중심. static 이지만 /gazebo/set_entity_state 로 옮길 수 있음.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from make_world import material_xml  # noqa: E402

W, DEPTH, HEIGHT = 0.60, 0.33, 1.50
BOARDS = (0.12, 0.52, 0.90, 1.27)   # 판 가운데 높이
TH = 0.018


def make(variant):
    name = 'ladder_shelf' if variant == 'full' else f'ladder_shelf_{variant}'
    parts = []  # (geom, xyz, mat, collide)

    def box(size, xyz, mat, collide=True):
        parts.append((f"<box><size>{size[0]:.3f} {size[1]:.3f} {size[2]:.3f}</size></box>", xyz, mat, collide))

    def cyl(r, length, xyz, mat):
        parts.append((f"<cylinder><radius>{r:.3f}</radius><length>{length:.3f}</length></cylinder>", xyz, mat, False))

    for sx in (-W / 2 + 0.0125, W / 2 - 0.0125):          # 검은 기둥 4 개 + 위 가로대
        for sy in (-DEPTH / 2 + 0.0125, DEPTH / 2 - 0.0125):
            box((0.025, 0.025, HEIGHT), (sx, sy, HEIGHT / 2), "#161616")
        box((0.025, DEPTH, 0.025), (sx, 0, HEIGHT - 0.0125), "#161616")
    for z in BOARDS:                                      # 판 (연보라 윗면 + 나무색 앞 테두리)
        box((W - 0.05, DEPTH, TH), (0, 0, z), "#e6e1f0")
        box((W - 0.05, 0.004, TH), (0, -DEPTH / 2 - 0.002, z), "#d2a56d", collide=False)
    top = [z + TH / 2 for z in BOARDS]
    if variant == "full":
        # 맨 아래: 큰 골판지 상자 + 작은 상자들
        box((0.30, 0.22, 0.20), (0.10, -0.02, top[0] + 0.10), "Capstone/poster_4", collide=False)
        box((0.12, 0.12, 0.12), (-0.17, 0.02, top[0] + 0.06), "#1c1c1c", collide=False)
        box((0.10, 0.08, 0.07), (-0.16, -0.10, top[0] + 0.035), "Capstone/paper_1", collide=False)
        # 둘째: 카메라 상자, 테이프, 장갑, 저울
        box((0.16, 0.10, 0.06), (0.06, 0.03, top[1] + 0.03), "Capstone/screen_2", collide=False)
        box((0.12, 0.12, 0.02), (-0.17, 0.05, top[1] + 0.01), "#8a8f96", collide=False)
        box((0.10, 0.07, 0.03), (-0.02, -0.09, top[1] + 0.015), "#7d8a80", collide=False)
        cyl(0.03, 0.02, (0.13, -0.09, top[1] + 0.01), "#202020")
        cyl(0.03, 0.02, (0.20, -0.08, top[1] + 0.01), "#c0392b")
        cyl(0.045, 0.03, (0.19, 0.04, top[1] + 0.015), "#ecf0f1")
        # 셋째: 작은 로봇 (검은 몸통 + 흰 바퀴)
        box((0.22, 0.20, 0.15), (0.0, 0.0, top[2] + 0.17), "#1a1a1a", collide=False)
        box((0.10, 0.03, 0.03), (0.0, -0.11, top[2] + 0.18), "#dfe6e9", collide=False)
        for sx in (-0.15, 0.15):
            box((0.05, 0.16, 0.10), (sx, 0.0, top[2] + 0.12), "#f5f5f5", collide=False)
            box((0.04, 0.10, 0.10), (sx, -0.02, top[2] + 0.05), "#f0ead6", collide=False)
        # 맨 위: 바구니, 작은 상자
        box((0.22, 0.18, 0.15), (0.13, 0.0, top[3] + 0.075), "#cdb38b", collide=False)
        box((0.13, 0.09, 0.05), (-0.10, 0.0, top[3] + 0.025), "#b08d57", collide=False)

    elif variant == "alt":      # 물건이 바뀐 선반: 위치·모양·무늬가 전부 다름
        box((0.20, 0.20, 0.26), (-0.14, 0.0, top[0] + 0.13), "Capstone/books_1", collide=False)
        box((0.18, 0.16, 0.12), (0.15, -0.04, top[0] + 0.06), "#2e86c1", collide=False)
        box((0.26, 0.18, 0.10), (-0.08, -0.03, top[1] + 0.05), "Capstone/poster_9", collide=False)
        cyl(0.05, 0.16, (0.18, 0.0, top[1] + 0.08), "#27ae60")
        box((0.14, 0.14, 0.14), (0.12, 0.02, top[2] + 0.07), "#e67e22", collide=False)
        box((0.20, 0.12, 0.05), (-0.12, -0.05, top[2] + 0.025), "Capstone/paper_2", collide=False)
        box((0.30, 0.20, 0.10), (-0.05, 0.0, top[3] + 0.05), "#7f8c8d", collide=False)
    # variant == "empty": 물건 없음

    out = [f'<?xml version="1.0"?><sdf version="1.6"><model name="{name}"><static>true</static><link name="link">']
    for k, (g, xyz, mat, col) in enumerate(parts):
        pose = f"<pose>{xyz[0]:.4f} {xyz[1]:.4f} {xyz[2]:.4f} 0 0 0</pose>"
        out.append(f'<visual name="v{k}">{pose}<geometry>{g}</geometry>{material_xml(mat)}<cast_shadows>false</cast_shadows></visual>')
        if col:
            out.append(f'<collision name="c{k}">{pose}<geometry>{g}</geometry></collision>')
    out.append("</link></model></sdf>")
    d = Path(__file__).resolve().parents[1] / "models" / name
    d.mkdir(parents=True, exist_ok=True)
    (d / "model.sdf").write_text("\n".join(out) + "\n")
    (d / "model.config").write_text('<?xml version="1.0"?><model><name>' + name + '</name><version>1.0</version>'
                                    '<sdf version="1.6">model.sdf</sdf><description>사다리형 선반 (시험용)</description></model>\n')
    print(f"{d / 'model.sdf'}  ({len(parts)} 조각)")


def person():
    """서 있는 사람 흉내 (폭 0.45 m): 다리 2 + 몸통 + 머리. 가림·끼어들기 시험용."""
    def v(k, geom, xyz, mat):
        return (f'<visual name="v{k}"><pose>{xyz[0]} {xyz[1]} {xyz[2]} 0 0 0</pose><geometry>{geom}</geometry>{material_xml(mat)}'
                f'<cast_shadows>false</cast_shadows></visual><collision name="c{k}"><pose>{xyz[0]} {xyz[1]} {xyz[2]} 0 0 0</pose><geometry>{geom}</geometry></collision>')
    body = [v(0, "<cylinder><radius>0.075</radius><length>0.85</length></cylinder>", (-0.11, 0, 0.425), "#2c3e50"),
            v(1, "<cylinder><radius>0.075</radius><length>0.85</length></cylinder>", (0.11, 0, 0.425), "#2c3e50"),
            v(2, "<box><size>0.45 0.24 0.65</size></box>", (0, 0, 1.175), "#c0392b"),
            v(3, "<sphere><radius>0.11</radius></sphere>", (0, 0, 1.62), "#e0b090")]
    d = Path(__file__).resolve().parents[1] / "models" / "person_box"
    d.mkdir(parents=True, exist_ok=True)
    (d / "model.sdf").write_text('<?xml version="1.0"?><sdf version="1.6"><model name="person_box"><static>true</static><link name="link">\n'
                                 + "\n".join(body) + "\n</link></model></sdf>\n")
    (d / "model.config").write_text('<?xml version="1.0"?><model><name>person_box</name><version>1.0</version>'
                                    '<sdf version="1.6">model.sdf</sdf><description>서 있는 사람 흉내</description></model>\n')
    print(d / "model.sdf")


def spectator():
    """구경용 고정 카메라 (녹화용): /spectator/image_raw 960×540 10 Hz. 스폰할 때 위치·방향을 줌."""
    d = Path(__file__).resolve().parents[1] / "models" / "spectator_cam"
    d.mkdir(parents=True, exist_ok=True)
    (d / "model.sdf").write_text("""<?xml version="1.0"?><sdf version="1.6"><model name="spectator_cam"><static>true</static><link name="link">
<sensor type="camera" name="cam"><update_rate>10</update_rate><camera><horizontal_fov>1.5</horizontal_fov>
<image><width>960</width><height>540</height><format>R8G8B8</format></image><clip><near>0.05</near><far>40</far></clip></camera>
<plugin name="spectator" filename="libgazebo_ros_camera.so"><ros><namespace>/spectator</namespace></ros>
<camera_name>cam</camera_name><frame_name>spectator_link</frame_name></plugin></sensor>
</link></model></sdf>
""")
    (d / "model.config").write_text('<?xml version="1.0"?><model><name>spectator_cam</name><version>1.0</version>'
                                    '<sdf version="1.6">model.sdf</sdf><description>녹화용 고정 카메라</description></model>\n')
    print(d / "model.sdf")


if __name__ == "__main__":
    for v_ in ("full", "alt", "empty"):
        make(v_)
    person()
    spectator()
