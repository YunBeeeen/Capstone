# B601-DM + wrist D435i — 하단 베이스 판 제거본

사용할 파일: **`urdf/B601_DM_wrist_d435i_no_plate.urdf`**.
메시를 포함한 이 폴더 전체를 함께 옮기면 된다. 기존 `~/yorr` 파일은 수정하지 않았다.

## MuJoCo로 바로 열기

```bash
~/b601_dm_wrist_d435i_no_plate/open_mujoco.sh
```

MuJoCo용 파일은 **`mjcf/B601_DM_wrist_d435i_no_plate.xml`**이다.
완성 XML을 포함하므로 열기 전에 변환할 필요가 없다.
현재 컴퓨터의 `~/yorr/.venv/bin/python`을 자동으로 사용한다.
다른 환경에서는 MuJoCo/NumPy 설치 후 `B601_PYTHON=/경로/python ./open_mujoco.sh`로 지정한다.

마우스 왼쪽 드래그로 회전, 오른쪽 드래그로 평행 이동, 휠로 확대/축소한다.
오른쪽 `Control` 패널의 `joint1_target`~`joint6_target` 슬라이더로 관절 목표를 바꾸고,
`finger_left_target`으로 양쪽 그리퍼를 함께 연다. 필요하면 Tab으로 패널을 표시한다.
`--camera wrist_rgb`를 붙이면 손목 카메라 시점으로 열린다.

팔 6축과 그리퍼 2축, mimic 관계, D435i, 충돌 표시용 메시를 포함한다.
형상 확인과 관절 조작용으로 중력 보상을 켜고 접촉 계산은 껐다.
그리퍼에 원본 URDF 관성이 없어 손가락당 0.03 kg / 대각 관성 0.00001 kg·m²를 임시 적용했다.
이 XML은 접촉/파지 동역학 검증용 모델이 아니다.
장면에서는 `base_link`를 z = −0.014 m에 놓아 하우징 바닥이 월드 바닥에 맞는다.
D435i의 DAE 대신 동봉한 동일 형상의 STL 두 개를 사용한다.
Assimp 변환 시 적용되는 Z-up → Y-up 회전을 되돌려 원본 DAE와 같은 축을 유지했다.
따라서 카메라의 장착/광학 프레임은 원본을 유지하면서 외형도 같은 방향을 향한다.
`scripts/convert_camera_mesh.py`로 재생성할 수 있으며 Assimp CLI/NumPy/SciPy가 필요하다.
원본 DAE와 전체 STL 정점의 양방향 거리 및 삼각형 231,186개 보존을 검사한다.
수치 검증 기록은 `docs/camera_mesh_validation.json`에 있다.
두 손목 카메라는 프레임 확인용 이상적 pinhole이며 실제 영상/IMU 드라이버를 실행하지 않는다.

```bash
# XML 재생성: NumPy, SciPy, MuJoCo 필요
~/yorr/.venv/bin/python ~/b601_dm_wrist_d435i_no_plate/scripts/build_mujoco.py
# 창 없이 렌더
~/b601_dm_wrist_d435i_no_plate/open_mujoco.sh --headless --snapshot /tmp/b601.png
```

관절을 바꾼 두 자세에서 URDF와 전체 링크 FK를 대조하고, 5초 자세 유지 중
경고/발산 여부를 검사한다. 결과는 `docs/mujoco_validation.json`에 있다.

![베이스 분리 전후](docs/base_before_after.png)
![전체 팔과 손목 D435i](docs/robot_preview.png)
![손목 카메라 확대](docs/wrist_d435i.png)

## 무엇을 분리했는가

원본 `base_link`는 하나의 링크/STL에 여러 독립된 부품을 담고 있다.
STL 연결 성분을 분리한 결과, 판과 하우징은 이미 서로 다른 닫힌 솔리드다.
따라서 하우징을 평면으로 자르거나 구멍을 메우지 않고 **부품 전체를 선택해 분리**했다.

| 부품 | 처리 | 원본 base_link 좌표 |
|---|---|---|
| 하단 140 × 200 × 14 mm 판 | visual/collision 양쪽에서 제거 | z = 0 ~ 14 mm |
| 판에 머리가 매립된 긴 관통 나사 4개 | visual/collision 양쪽에서 제거 | z = 2.5 ~ 80.9 mm |
| 모터를 감싸는 하우징 | 삼각형 그대로 보존 | x/y = ±46 mm, z = 14 ~ 77.7 mm |
| 내부 모터·상단 링·나머지 체결물 | 보존 | 원본 위치 |
| 팔·그리퍼·D435i와 장착 브래킷 | 보존 | 원본 위치 |

분리된 판은 `meshes/removed/visual_colored_base_link_matte_black_removed.stl`에 있다.
분리된 나사와 충돌 메시의 제거 부분도 `meshes/removed/`에 따로 보관했다.
수정 URDF는 이 제거 메시들을 참조하지 않는다.
원본 베이스 메시 4개는 `source/original_base/`에 보존한다.

## 좌표와 관성

`base_link` 원점과 모든 관절 원점·축·한계는 원본과 같다.
**하우징의 하단 장착면은 `base_link`의 z = +0.014 m**다.
하우징 바닥을 월드 z = 0에 놓으려면 로봇의 `base_link`를 z = −0.014 m에 배치한다.
새 플랫폼에 결합할 때도 이 오프셋을 고려한다. 원점 자체를 옮기지 않아 기존 FK를 유지한다.

판 제거 후 베이스 관성을 원본 그대로 두지 않고 남은 솔리드에서 재계산했다.
부품별 재질/질량 정보가 없으므로 원본 베이스 질량 0.8366 kg과 전체 메시 부피에서
균일 유효 밀도 약 1030.94 kg/m³를 구해 적용했다.
결과는 베이스 질량 약 **0.436413 kg**, 질량중심 z 약 **0.0505773 m**다.
이 값은 **실측값이 아닌 형상 기반 추정치**다. 동역학 용도에는 실제 남은 어셈블리의 질량/관성으로 교체한다.
팔과 카메라의 나머지 관성은 출처 모델을 유지한다.
상세 수치와 분리한 모든 성분은 `docs/build_report.json`에 있다.

## 카메라 장착

추가 장착 점검에서 공식 카메라 어셈블리와 RS 플랜지 기준을 다시 받아 대조했다.
14개 팔 자세에서 카메라와 손목의 상대 위치/회전이 고정되고, RGB/depth 렌더 카메라가
각각 ROS optical 프레임과 일치하는 것을 확인했다. `docs/camera_mount_audit.json`과
네 방향 점검 이미지 `docs/camera_mount_inspection.png`에 기록했다.

형상은 공식 배치를 유지한다. 브래킷 받침면과 나사 기준점 근처에는 약 0.62 mm 여유가 있고,
상단 립 부근에는 약 0.32 mm의 국소 표면 겹침이 측정됐다.
따라서 공식 모델 기준의 장착 방향/좌표는 맞지만, 모든 면이 정확히 밀착된 CAD 조립은 아니다.
브래킷은 닫힌 솔리드 메시가 아니므로 겹침 수치는 정점-삼각형 근접면 분석값이다.
상세는 `docs/camera_mount_geometry_audit.json`에 있다. 점검 중 장착 변환은 변경하지 않았다.

```bash
~/yorr/.venv/bin/python ~/b601_dm/scripts/audit_camera_mount.py \
  --reference ~/b601_dm/source/camera_audit_reference.json
```

Seeed 공식 D435i 어셈블리와 D435/Gemini2 브래킷을 사용한다.
공식 카메라 어셈블리의 RS `gripper_end` 기준을 DM `link6` 플랜지에 맞추는
고정 어댑터를 포함한다. DM `end_link` 기준 어댑터 이동은 약 +10.82 mm다.
이는 모델의 플랜지를 일치시킨 값이며 실물 hand-eye 보정값은 아니다.

주요 프레임: `wrist_camera_link`, `wrist_camera_color_optical_frame`,
`wrist_camera_depth_optical_frame`, `wrist_camera_accel_optical_frame`,
`wrist_camera_gyro_optical_frame`. 추가 가동 관절은 없다.
URDF는 형상과 TF를 제공한다. 실시간 영상/IMU 발행은 별도 RealSense 드라이버가 담당한다.

## ROS 2 / RViz로 보기

이 폴더는 독립 `ament_cmake` description 패키지다. ROS 2 Humble 기준:

```bash
source /opt/ros/humble/setup.bash
colcon --log-base /tmp/b601_no_plate_log build \
  --base-paths ~/b601_dm_wrist_d435i_no_plate \
  --build-base /tmp/b601_no_plate_build \
  --install-base /tmp/b601_no_plate_install
source /tmp/b601_no_plate_install/setup.bash
ros2 launch b601_dm_wrist_d435i_no_plate display.launch.py
```

관절 슬라이더와 RViz가 열린다. `gui:=false rviz:=false`로 TF 발행만 할 수도 있다.
다른 ROS 작업공간에서는 이 폴더를 `src/`에 복사하거나 심볼릭 링크한 뒤 빌드한다.

## ROS 없이 가져오기

대부분의 URDF 가져오기 도구는 URDF 위치 기준 `../meshes/`를 해석한다.
절대 경로가 필요한 도구에는 아래 파일을 생성해서 사용한다:

```bash
python3 ~/b601_dm_wrist_d435i_no_plate/scripts/resolve_urdf.py \
  --output /tmp/B601_DM_wrist_d435i_no_plate_absolute.urdf
```

이 절대 경로 버전은 현재 폴더 위치에 종속된다. 배포/이동에는 원본 상대 경로 URDF와 메시 폴더를 사용한다.

## 재생성과 검증

완성 URDF 사용에는 메시 처리 라이브러리가 필요 없다. 베이스 분리 작업을 재현하려면:

```bash
python3 -m venv /tmp/b601_model_tools
/tmp/b601_model_tools/bin/pip install -r ~/b601_dm_wrist_d435i_no_plate/requirements-build.txt
/tmp/b601_model_tools/bin/python ~/b601_dm_wrist_d435i_no_plate/scripts/build_model.py
```

`source/arm_wrist_before_removal.urdf`는 판 제거 전 합성본이다.
생성기는 공식 DM URDF와의 일치, 카메라 공식 어셈블리, RS/DM 플랜지 정렬,
메시의 공식 Git blob SHA, 닫힌 판 분리, 삼각형 수 보존, 연결 트리,
관절·한계·축 보존, 모든 메시 경로, 관성 양의 정부호와 삼각 부등식을 검사한다.
결과는 `docs/validation.json`에 기록한다.
`scripts/render_preview.py`는 MuJoCo/Pillow가 있는 환경에서 미리보기만 렌더한다.
이 스크립트는 URDF visual 기반 정적 렌더다. 관절 조작은 위의 MuJoCo XML과 실행기를 사용한다.

로컬에서 `check_urdf` 파싱, ROS 2 패키지 빌드, GUI 없는 launch의
`robot_state_publisher`/`joint_state_publisher` 시작, 실제 visual 메시 렌더를 확인했다.
별도 ROS domain에서 가동 관절 상태 8개와 TF 자식 프레임 25개의 실제 발행을 수신했다.
초기 관절값과 수신 프레임 목록은 `docs/ros_smoke.json`에 있다.
RViz GUI 자체의 대화형 조작은 검사하지 않았다.
원본처럼 루트 `base_link`에 관성이 있어 KDL은 루트 관성을 무시한다는 경고를 출력한다.
TF 발행에는 지장이 없고, URDF의 관성 정보는 그대로 제공된다.

## 출처 및 라이선스

- [Seeed-Projects/reBot-DevArm](https://github.com/Seeed-Projects/reBot-DevArm),
  revision `52653034537b4e166d0b57a5b1c1609290aed207`.
- 공식 `Rebot_Arm_description/DM`의 URDF와 메시.
- 공식 `Rebot_Arm_description/Camera`의 D435i URDF, 브래킷, D435 DAE.
- 로컬 기존 `~/yorr/src/yor_description/arms/b601_dm`과 `cameras/d435i`에서
  자산을 복사했고, 공식 Git tree와 원본 로봇 메시·카메라 자산의 해시를 대조했다.
- 카메라 DAE와 같은 형상을 변환한 `d435_0.stl`, `d435_1.stl`은 미리보기용이다.
  변환 시 Assimp의 축 회전을 취소하여 DAE 좌표와 일치시켰다.
  실제 URDF는 재질이 포함된 공식 DAE를 사용한다.

출처 URDF·Git tree는 `source/`에, 라이선스는 `LICENSE`와 `licenses/`에 보존한다.
팔/브래킷의 CERN-OHL-W-2.0 및 RealSense 자산의 Apache-2.0 고지를 함께 유지한다.
