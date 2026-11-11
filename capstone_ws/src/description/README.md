# mobile_robot_description

질량 수정값을 반영한 오른팔·양팔 모델과 충돌 형상 확인용 모델입니다.
공용 메시를 함께 사용하며 URDF와 실행 파일은 네 구성으로 나뉩니다.

| 구성 | 실행 명령 |
|---|---|
| 오른팔 | `ros2 launch mobile_robot_description right_arm.launch.py` |
| 양팔 | `ros2 launch mobile_robot_description dual_arm.launch.py` |
| 오른팔 콜리전 | `ros2 launch mobile_robot_description right_arm_collision.launch.py` |
| 양팔 콜리전 | `ros2 launch mobile_robot_description dual_arm_collision.launch.py` |

## 빌드 및 실행

작업공간 루트에서:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select mobile_robot_description
source install/setup.bash
ros2 launch mobile_robot_description right_arm.launch.py
```

콜리전 버전은 충돌 메시를 박스·원통으로 단순화한 URDF입니다.
바퀴는 Y축 원통, 나머지는 메시 삼각형을 경계 평면에서 절단하여 공간별로 나눈 복합 박스입니다.
본체 최대 28개, 팔 링크 최대 12개, 기타 링크 최대 10개로 빈 공간을 줄였습니다.
분할은 박스 부피 합이 8% 이상 줄어드는 경우에만 진행합니다.
원본 삼각형을 분할 평면에서 절단하고, 각 조각이 해당 박스에 포함되도록 검증했습니다.
원본 visual·질량·관성·관절은 유지합니다. RViz에서는 충돌 형상을 기본 표시합니다.
이 모델은 형상 근사이므로 경로계획에 적용할 때는 SRDF 인접 링크 충돌 제외와
작업 범위 검증이 필요합니다. 손가락 관성 및 시뮬레이터·컨트롤러 설정은 별도입니다.
RViz: 왼쪽 드래그 회전, 가운데 드래그 이동, 휠 확대. 관절 GUI에서 팔과 리프트 조절.
네 구성은 같은 토픽을 사용하므로 한 번에 하나씩 실행하세요.

## 모델 기준

- 리프트: 수축 위치에서 시작, 각 단 0.315 m, 1:1 mimic, 총 0.630 m 이동.
- 본체 질량 13.527271003440 kg, 카메라 질량 0.412700000000 kg. 수정 ZIP의 COM·관성 반영.
- 팔 장착 좌표: 오른쪽 (0.020, -0.082921485063, -0.047325904937) m,
  왼쪽 (0.020, 0.082921485063, -0.047325904937) m, top_plate_link 기준.
- 팔 하단 140 × 200 × 14 mm 판만 제거, 나사는 유지.
  팔 베이스 질량 0.439072605524 kg은 균일 유효 밀도를 가정한 메시 기반 추정치.
  근거는 `docs/plate_mass_estimate.json`에 보관.
- 손가락 관성은 제공되지 않아 임의로 추가하지 않았습니다.
- `urdf/`: 네 모델, `meshes/`: 참조되는 메시, `launch/`: 실행 파일,
  `rviz/`: 표시 설정, `docs/`: 질량 추정 근거.

정리 전 백업과 삭제 목록은 작업공간 `artifacts/src_cleanup/`에 있습니다.

## 팔 인접 링크 충돌 제외 (MoveIt)

각 URDF와 같은 이름의 `config/*.srdf`를 제공합니다. 오른팔 10쌍, 양팔 20쌍의
직접 연결된 충돌 링크만 `Adjacent`로 제외합니다. 팔 장착부(상판↔팔 베이스),
팔 관절, 손목↔그리퍼, 그리퍼↔각 손가락을 포함합니다. 충돌 형상이 없는 tool0는 제외합니다.
비인접 팔 링크, 좌우 팔 사이, 그 외 본체 및 환경 물체와의 충돌은 제외하지 않습니다.

SRDF는 MoveIt의 충돌 허용 행렬 설정이며 URDF만 읽는 시뮬레이터에는 적용되지 않습니다.
현재 display.launch.py는 RViz 형상 표시용이므로 충돌 검사를 수행하지 않습니다.
MoveIt 노드에서 URDF와 짝이 맞는 SRDF를 `robot_description_semantic`으로 로드해야 합니다.
기존 MoveIt SRDF가 있다면 이 파일의 disable_collisions 요소를 병합하세요.
이 SRDF는 충돌 제외 설정만 포함하며 플래닝 그룹·IK·컨트롤러 설정은 별도입니다.

```python
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

share = Path(get_package_share_directory('mobile_robot_description'))
model = 'mobile_robot_right_arm_collision'  # 양팔: mobile_robot_dual_arm_collision
robot_parameters = {
    'robot_description': (share / 'urdf' / f'{model}.urdf').read_text(),
    'robot_description_semantic': (share / 'config' / f'{model}.srdf').read_text(),
}
# move_group 및 MoveIt RobotModel을 사용하는 노드의 parameters에 robot_parameters 추가
```

## SRDF를 실제 충돌 검사 노드에 로드하기

MoveIt Core가 필요합니다. 의존성은 `rosdep install --from-paths src --ignore-src -r -y`로 설치할 수 있습니다.
현재 패키지에는 MoveIt PlanningScene을 사용하는 충돌 검사 노드가 포함됩니다.
오른팔 또는 양팔 중 하나를 선택하세요. 기존 display 실행은 먼저 종료하세요.

```bash
# 오른팔: URDF + SRDF + RViz + 관절 GUI + 실제 충돌 검사
ros2 launch mobile_robot_description collision_check.launch.py model:=right_arm
# 양팔
ros2 launch mobile_robot_description collision_check.launch.py model:=dual_arm
```

`robot_description_semantic` 파라미터와 같은 이름의 transient-local 토픽으로 SRDF를 제공합니다.
인접 충돌 제외는 이 노드의 MoveIt PlanningScene에 적용됩니다. 관절 GUI의 joint_states를
받아서 상태를 갱신하고 실제 충돌이 있으면 로그로 알립니다. RViz 화면 자체는 충돌 여부에 따라
자동으로 색이 바뀌지 않습니다. 경로계획·실기 구동 기능은 포함하지 않습니다.

- `/get_planning_scene`: SRDF가 적용된 AllowedCollisionMatrix 및 씬 조회
- `/check_state_validity`: 요청 상태의 충돌·제약·관절 범위 확인
- `/apply_planning_scene`: 환경 충돌 물체 등 씬 업데이트
- `/monitored_planning_scene`: 현재 씬 (0.5초 주기)

```bash
# 인접 링크 충돌 제외 행렬 확인 (128 = ALLOWED_COLLISION_MATRIX)
ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene '{components: {components: 128}}'
# joint_states로 받은 현재 상태 검사
ros2 service call /check_state_validity moveit_msgs/srv/GetStateValidity '{robot_state: {is_diff: true}}'
```

## capstone 에서 쓰는 법 (2026-11-11 교체)

- 이전 패키지 `mobile_robot_right_arm_description` 을 이 패키지로 교체 (디렉터리 `src/description` 그대로)
- **시뮬·학습엔 `*_collision.urdf` (박스·원통 충돌)** 를 기본으로 씀:
  - MuJoCo: `python3 src/sim/scripts/build_mjcf.py` → `mjcf/mobile_robot_right_arm.xml` (기본 입력 = 충돌 버전)
  - Gazebo: `capstone_gazebo` 의 `robot:=mobile` (충돌 버전 + ZED 센서)
- 질량은 이 URDF 값이 기준. `src/sim/scripts/set_urdf_inertia.py` (옛 부품 무게표로 덮어쓰기) 는 쓰지 말 것 — 막아 둠
- 손목 카메라(D435i) 프레임이 이 모델엔 없음 → MuJoCo 'wrist_color' 카메라 생략
- `licenses/`: 팔 메쉬(Seeed reBot B601) CERN-OHL-W-2.0 등 — 메쉬를 재배포할 때 유지
- `source/`: 이전 모델의 원본 URDF·출처 기록
