# 이동로봇 + B601-DM 오른팔

기존 이동로봇과 별도로 설치하는 ROS 2 Humble 패키지입니다.
오른팔 한 개, 그리퍼, 손목 D435i 형상/좌표계를 포함합니다.
기존 전방 카메라도 유지합니다. 카메라 영상 발행/하드웨어 제어 기능은 포함하지 않습니다.

## 설치 및 실행

기존 RViz launch는 먼저 Ctrl+C로 종료합니다.
압축파일을 `~/ros2_ws/src/`에 풀면 `mobile_robot_right_arm_description` 폴더가 생깁니다.

```bash
source /opt/ros/humble/setup.bash
sudo apt install ros-humble-robot-state-publisher ros-humble-joint-state-publisher-gui ros-humble-rviz2 python3-colcon-common-extensions
cd ~/ros2_ws
colcon build --packages-select mobile_robot_right_arm_description
source install/setup.bash
ros2 launch mobile_robot_right_arm_description display.launch.py
```

RobotModel, `/robot_description`, Fixed Frame `world`가 설정된 RViz가 자동으로 열립니다.
이전에 저장한 `~/mobile_robot.rviz` 파일을 사용하지 않아도 됩니다.
새 터미널에서 다시 실행할 때는 두 setup.bash를 먼저 불러옵니다.

## 반영한 리프트 설정

| 항목 | 값 |
|---|---|
| 시작 위치 | 최저 위치, 관절값 0 |
| 1단 origin | -0.02526 0 0.31918 |
| 2단 origin | 0 0 0.025 |
| 각 단 이동 | +Z, 0~0.325 m |
| 동기화 | 2단이 1단을 multiplier=1로 추종 |
| 상판 총 상승 | 0~0.650 m |

`lift_stage_1_joint` 슬라이더 하나로 두 단과 팔이 함께 올라갑니다.
슬라이더 표시 0.325는 상판 총 상승 0.650 m에 해당합니다.
각 단 325 mm는 대화에서 사용한 가정이며 실제 스트로크를 측정한 값은 아닙니다.

## 오른팔 장착 위치

오른쪽 암 플레이트의 Ø16 구멍 중심과 바깥쪽 장착면을 STL에서 추출해 정렬했습니다.
기존 CAD 좌표 기준 구멍 바깥면 중심은 (-3.26000, -92.25077, 1283.75359) mm입니다.
안쪽 원 중심은 (-3.25999, -88.71525, 1280.21806) mm이며 두 면 사이 간격은 약 5 mm입니다.
오른쪽은 기존 +X=전방, +Y=좌측 기준에서 -Y 방향입니다.
장착면의 바깥 법선은 (0, -0.70710678, 0.70710678)입니다.
팔의 +Z 축을 이 법선과 일치시키고 팔 하우징 바닥 중심을 Ø16 구멍 바깥면 중심에 맞췄습니다.
팔의 +X는 로봇 +X와 맞췄습니다. 구멍 축 주변 회전은 이 정렬을 사용했으며 볼트 패턴의 실제 결합은 검증하지 않았습니다.
팔 원점보다 하우징 바닥이 14 mm 높다는 제공 모델의 오프셋을 반영했습니다.

```xml
<joint name="right_arm_mount_joint" type="fixed">
  <parent link="top_plate_link"/>
  <child link="right_arm_base_link"/>
  <origin xyz="0.022000000 -0.082351275 -0.047325905" rpy="0.7853981633974483 0 0"/>
</joint>
```

판 두께/법선/중심은 STL의 원형 경계를 피팅한 값이므로 CAD 정밀도와 차이가 있습니다.
간섭, 체결 구멍 패턴, 부품의 지지 면적, 하중 적합성은 추가 확인이 필요합니다.

팔 링크/조인트/재질 이름은 모두 `right_arm_` 접두사를 붙여 기존 `base_link`와 충돌하지 않습니다.
팔의 관절축, 관절 한계, 관성 및 그리퍼 mimic은 제공 파일에서 유지했습니다.
팔 초기 자세는 제공된 launch와 같이 joint2=-2, joint3=-1.2, joint4=-1 rad입니다.
그리퍼는 `right_arm_finger_left` 슬라이더 하나로 함께 움직입니다.

## 편집 후 적용

이 launch는 **설치된 URDF**를 읽습니다. src 파일 수정 후 반드시 다시 빌드하고 실행을 재시작하세요.

```bash
cd ~/ros2_ws
colcon build --packages-select mobile_robot_right_arm_description
source install/setup.bash
ros2 launch mobile_robot_right_arm_description display.launch.py
```

## 검증 및 범위

XML/트리, 참조 메시 파일, STL 형식, DAE 내부 참조, 수치 값, mimic 관계 및 리프트 FK를 점검했습니다.
리프트 전후로 팔의 상판 대비 상대 변환이 유지되고, 두 단 최대값에서 상판과 팔이 650 mm 상승하는지 확인했습니다.
현재 생성 환경에는 ROS 2가 없어 colcon/RViz 실제 실행은 검증하지 않았습니다.
이동로봇에는 관성 정보가 없으므로 동역학/하중/안전성 검증용 모델이 아닙니다.
팔 초기 자세 및 전체 동작 범위의 충돌 여부는 검증하지 않았습니다.

## 출처

이동로봇은 사용자가 제공한 `mobile_robot_description.zip`을 바탕으로 대화의 리프트 변경을 반영했습니다.
사용자 PC의 수정본을 다시 업로드받은 것은 아니므로 다른 로컬 편집은 포함되지 않습니다.
팔은 제공받은 `b601_dm.zip`의 `urdf/B601_DM_wrist_d435i_no_plate.urdf`와 참조 메시입니다.
상세 출처 및 고지는 `source/B601-original-README.md`, `licenses/`를 참조하세요.
원본 참고 URDF는 `source/`에 보존했습니다. 실행용은 `urdf/mobile_robot_right_arm.urdf`입니다.
Seeed 원본: https://github.com/Seeed-Projects/reBot-DevArm
revision: 52653034537b4e166d0b57a5b1c1609290aed207
팔/브래킷: CERN-OHL-W-2.0, RealSense 자산: Apache-2.0.
패키지의 라이선스 표기는 제3자 자산 고지를 보존하기 위한 것이며 사용자 이동로봇의 권리를 재설정하지 않습니다.
