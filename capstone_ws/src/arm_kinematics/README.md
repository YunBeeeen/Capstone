# arm_kinematics — B601-DM 오른팔 FK (모바일 + 리프트 + 45° 장착)

ROS 와 무관한 Python 모듈 (`COLCON_IGNORE`). 필요: `numpy`, `pinocchio` (ROS Humble 의 것 사용 가능).

```python
from arm_kinematics import B601Kinematics
from arm_kinematics.gravity import ArmGravity

kin = B601Kinematics()                       # description/urdf/mobile_robot_right_arm.urdf
arm = {1: 1.6, 2: -148.2, 3: -85.9, 4: 29.2, 5: 45.0, 6: 2.3}   # 모터 번호 → 공개 각도 [도]
# 이름 키({"shoulder_pan": 1.6, ...}, 기록 jsonl 의 joints 그대로, gripper 무시)나 1~6번 순서 리스트도 됨
T = kin.fk(arm, lift_stage_m=0.1)            # 로봇 바닥 기준(world) 그리퍼 끝(tool0) 자세
T.translation, T.rotation
kin.fk(arm, frame="right_arm_wrist_camera_color_optical_frame")   # 다른 프레임도 가능
kin.jacobian(arm)                            # 6×6, base_link 축, tool0 기준점
kin.manipulability(arm)                      # w_pos, w, sigma_min, cond
kin.motor_limits_deg()                       # {1: (lo, hi), ..., 6: (lo, hi)}
ArmGravity().torques_by_motor(arm)           # {1: τ, ..., 6: τ} 실기 팔로워와 같은 중력 토크 [N·m]
```

## 모터 번호

| 모터 | LeRobot 이름 | URDF | 사용 한계 [도] |
|---|---|---|---|
| 1 | shoulder_pan | right_arm_joint1 | ±160.4 (URDF) |
| 2 | shoulder_lift | right_arm_joint2 | −210 ~ 1 (collector) |
| 3 | elbow_flex | right_arm_joint3 | −200 ~ 1 (collector) |
| 4 | wrist_flex | right_arm_joint4 | −85 ~ 120 (collector), 중력 배율 0.85 |
| 5 | wrist_yaw | right_arm_joint5 | ±89.95 (URDF), **±90 이 손목 특이점** |
| 6 | wrist_roll | right_arm_joint6 | −180 ~ 170 (collector) |

`MOTORS`, `joint_name(4) == "wrist_flex"`, `by_motor(이름 dict)` 로 변환한다.
자코비안 열 k 는 모터 k+1 번이다.

## 규약

| 항목 | 값 | 근거 |
|---|---|---|
| 관절 입력 | LeRobot 공개 각도[도], 순서 shoulder_pan … wrist_roll | 팔로워 `get_observation()` 의 `*.pos` |
| 공개 → URDF | `q = radians(공개각 × joint_directions)`, DM 은 전 관절 +1 | `motor_family.py` DM_PROFILE, 팀 중력 보상기가 모터 각도를 URDF q 로 그대로 사용 |
| 45° 장착 | `right_arm_mount_joint` rpy = (+π/4, 0, 0), top_plate → arm base | 팀 `GRAVITY_MOUNT_ROLL_DEG=45` 와 같은 회전 (시험으로 대조) |
| 리프트 | `lift_stage_m` = 한 단의 이동량 [0, 0.315] m, 두 단 mimic → 상판 높이 +2× | URDF `lift_stage_1/2_joint` |
| 관절 한계 | URDF 한계를 collector `USER_LIMITS_DEG` 로 덮어씀 (shoulder_lift −210~1, elbow_flex −200~1, wrist_flex −85~120, wrist_roll −180~170) | 실기 팔로워가 넓힌 한계. 기록 데이터가 URDF ±180 를 넘는 이유 |
| 끝단 | `right_arm_tool0` = `right_arm_end_link` (x축이 접근 방향, 손가락은 y축) | URDF |

서버 원본 `arm_follower/arm_leader/arm_sent.jsonl` 의 `joints` 는 이 공개 각도[도] 그대로다
(102 에피소드 확인, 2026-11-16: shoulder_lift −199~0.4, gripper −270~0). 변환 없이 `fk()` 에 넣는다.

**리프트 실측값(`lift.jsonl` 의 `position_mm`) → `lift_stage_m` 변환은 아직 검증하지 않았다.**
리프트 원점·행정(실기 690 mm vs URDF 2×315 mm)을 대조한 뒤 연결한다.

## 중력 토크

`team_gravity.py` 는 data_collection@1758334 패치의 `gravity.py` 를 수정 없이 옮긴 것이다.
`assets/B601_DM_wrist_d435i_no_plate.urdf` 도 같은 커밋의 실기용 모델이다.
설정은 `config.env` 그대로 (roll 45°, pitch 0, tau_scale 1.0, 손가락 0.03 kg),
그 결과에 collector `apply_joint_scales` 와 같은 방식으로 관절별 배율을 곱한다.
기본 배율 `{4: 0.85}` (모터 번호 또는 이름 키) 는 기억에 의존한 값이므로 수집 PC 의 실제 설정과 대조해야 한다.

## 조작성

`manipulability()` 는 **팔 6관절 자코비안만** 쓴다. 특이값은 좌표축 회전에 불변이라
45° 장착·리프트 높이와 무관하고 관절값에만 의존한다. 장착 각도는 "목표에 닿으려면
어떤 관절값이 필요한가"(IK)를 통해 조작성에 영향을 준다.
`w` 와 `sigma_min` 은 각속도 행에 `length_scale_m`(기본 0.1 m)을 곱해 단위를 맞춘 값이다.

## IK (`arm_kinematics/ik.py`)

```python
from arm_kinematics.collision import CollisionChecker
from arm_kinematics.ik import B601IK, describe
ik = B601IK(kin, collision=CollisionChecker(kin, env_boxes))   # env_boxes: 로봇 바닥 기준 상자, 없으면 []
cands = ik.solve((0.0, -0.55, 0.96), lift_stage_m=[0, 0.105, 0.21, 0.315],
                 approach=(0, 0, -1))        # None=위치만, approach=접근축만, rotation=3×3 자세 전체
best = cands[0]; best.motors_deg, best.lift_stage_m, best.cost_terms
```

- 좌표: 로봇 바닥 기준 `world` (x 앞, y 왼쪽, z 바닥에서 위). `base_link` 는 이보다 20 mm 아래.
- 여러 시작점(시드 + 무작위 24개) × 리프트 높이마다 감쇠 최소제곱으로 풀고, 3° 안의 같은 해는 합친다.
- **cost 는 후보 하나(관절 자세 하나)의 값**이다. 네 항의 가중합이며 작을수록 좋다.
  `singular 0.02/σmin` + `load 중력 부하(0~1)` + `limit 한계 여유 10° 미만부터 0→1` + `move 시작 자세에서 움직인 양/π`
  (시드를 안 주면 move = 0). 수렴·충돌 없음 후보가 앞, 그 안에서 cost 순.

## 동작 계획 (`arm_kinematics/motion.py`)

후보 고르기 = **자세 비용 + 경로 비용** (참고: 6DOF_Manipulator 의 궤적 비용 선택). 충돌 없는 경로를 최대 4 개 만든 뒤
합이 가장 작은 것을 고른다. 경로 비용 (`TRAJ_WEIGHTS`):
- `time` 재생 시간 / 20 s
- `load_path` 경로 중 최대 중력 부하의 **포화 벌점** — 팀 보상기가 토크를 1~3번 10 N·m, 4~6번 3 N·m 에서 자르므로
  그 80 % 까지는 0, 80→100 % 에서 0→1. 팔을 뻗는 것 자체는 벌점 없음 (자세 비용의 `load` 도 같은 방식).
- `clearance` 이동 구간(시작 → 접근점)의 팔↔환경 여유가 5 cm (상자 여유 위) 미만이면 0→1. 삽입 구간은 원래 판에 가까우므로 제외.
  여유는 상자를 1/2.5/5 cm 더 키운 사본과의 충돌 판정으로 단계화 (정확한 메시 거리는 70~140 ms 라 느림).
- `singular_path` 정밀 구간(접근점 → 목표)의 최소 σmin (출발 접힌 자세는 모든 경로가 같아 제외).
접근점은 15 → 12 → 10 → 7 → 5 cm 순으로 시도 (가구 밖에서 다가와 안쪽으로만 직선 삽입).

cost 낮은 후보부터: 리프트 이동 → 관절 보간으로 접근점(목표에서 접근축 반대 15 → 12 → 10 → 7 → 5 cm) →
1 cm 걸음 직선 삽입(걸음마다 IK, 10° 넘게 튀면 버림). 경로 전체를 2° 간격으로 충돌 검사하고,
시작→접근점이 막히면 경유점(접근점·20·30 cm 뒤 × 0·6·12 cm 위)을 시도한다 — 든 물체가 판 앞 모서리에 걸릴 때 위에서
내려오게. 통과한 첫 후보를 고른다 (`plan.tried` 에 앞 후보 탈락 이유).
출발 자세에서 5° 안은 팔 링크끼리 충돌을 보지 않는다 (접힌 자세는 메시가 원래 0.5 mm 로 붙어 있어 거짓 충돌; 팔↔차체·가구는 봄).
속도: 관절 최대 30°/s, 리프트 50 mm/s, 삽입 3 cm/s.

## 리프트를 합친 IK, 설 자리 고르기

- `ik.solve(..., lift_free=True, start_lift_stage_m=지금)`: 리프트를 0~630 mm 13 단계로 고정해 풀고, 각 높이에서
  리프트를 7 번째 변수(자코비안 열 = z × 2)로 한 번 더 풀어 사이 높이도 찾는다. 비용에 `lift` (이동량/630 mm, 가중 0.5).
  `ik_sim_run.py --lift-free --zed-height <지금>` (실기 실행기는 아직 리프트를 못 움직여 리프트가 바뀌는 plan 은 거부).
- `tools/choose_stand.py --target shelf3 --along=-0.10`: 앞면 거리 × 가구 가운데 좌우(±10 cm, ZED 가 가구를 보도록)를
  훑어 계획하고, 비용 낮은 자리마다 정지 오차(앞뒤·좌우 ±3 cm, 방향 ±3°)를 넣어 **다시 계획**해 본다 (도착 후 ZED 로
  다시 재서 계획하는 실제 흐름). 모든 오차에서 되는 자리 중 비용 최소. 노트북 약 50 s.
  결과 (2026-11-16, 지금 리프트 210 mm): 선반 3층 앞면 0.40 m·선반 가운데 y −0.10 m·리프트 210 mm, 책상 앞면 0.35 m·
  y −0.10 m·210 mm — 둘 다 6/6. 리프트 고정으로 책상 집기와 선반 3층 두기가 같이 된다.

## 시험 장면 (`arm_kinematics/scene.py`)

- 선반: 연구실 실물 실측. 높이 1.477 m, 판 윗면 0.21 / 0.55 / 0.88 / 1.31 m (1층 21 cm, +34, +33, +43 cm),
  폭 0.6 × 깊이 0.33 m. (Gazebo `ladder_shelf` 는 아직 옛 높이 1.5 m / 0.129·0.529·0.909·1.279)
- 책상: 1.0 × 0.5 m, 윗면 0.88 m (선반 3층). 긴 변이 로봇을 본다.
- **내비게이션처럼 ZED(로봇 정면, +x)가 가구 가운데를 본다** (`--lateral 0`). 방 좌표 = 로봇 바닥 기준, 로봇은 원점.
  `--front` = 로봇 중심 → 가구 앞면 (차체 앞 끝 ZED 는 0.195 m), `--along` = 물체의 가구 폭 방향 위치 (− = 로봇 오른쪽).
- 기본 **front 0.45 m (차체 앞에서 25 cm), along −0.10 m** — `tools/stand_sweep.py` 결과 이 자리에서 책상·선반 2~4층이 된다.
  1층은 front 0.50 m, along −0.10 m 한 칸만 되고 중력 부하가 상한(1.00)이다.
- 팔이 오른쪽에 달려 있어 물체가 선반 가운데~오른쪽 10 cm 일 때 잘 되고, 오른쪽 끝(−0.2 m)은 3·4층에서 오른쪽 기둥에 손목이 걸린다.
- 목표 preset: `desk` (앞 모서리 15 cm 안쪽), `shelf1`~`shelf4` (앞면 10 cm 안쪽), 면 위 5 cm.

## 도구 (`tools/`)

```bash
cd capstone_ws/src/arm_kinematics
python3 tools/ik_scene.py   --target shelf3 --approach into --workspace   # MuJoCo 창: 후보 넘겨 보기 (N/P, W)
python3 tools/ik_sim_run.py --target desk --approach down                 # 최저 cost 실행 가능 후보를 물리로 실행
python3 tools/ik_sim_run.py --target shelf3 --approach into --headless out/ --save-plan plan.json
source /opt/ros/humble/setup.bash && source ../../install/setup.bash
python3 tools/ik_rviz.py    --target shelf4 --approach free               # RViz: 로봇·환경·목표·작업 공간 점구름
```

`--no-sim`: MuJoCo 없이 기구학 검증만 한다 (라파·젯슨에서 계획할 때, pinocchio·hpp-fcl·numpy 만 필요). 실제로 재생될
5차 곡선 경로를 0.01 s 마다 뽑아 관절 한계·충돌(손가락 닫힘·벌림, 여유)·속도를 검사하고 끝 그리퍼 위치를 FK 로 확인한다.
노트북에서 약 4 s. 실행 때마다 MuJoCo 물리를 돌릴 필요는 없다 — 물리 재생은 개발·회귀 시험용 (시뮬 추종 1° 대 실기 약 8° 라
추종 결과는 실기를 대표하지 못함).

`--approach` (기본 `heading`):
- `heading` — **그리퍼가 가구 안쪽을 본다.** 접근축의 수평 방향을 고정하고, 기울기만 `--tilt=-10,45` (아래 +, 도) 범위에서 자유.
  접근점은 목표에서 수평으로 10 → 7 → 5 cm 물러난 곳, 삽입은 수평 직선이며 그 동안 그리퍼 자세 유지.
- `into` (안쪽 수평 고정) / `down` (위에서) / `free` (방향 자유) / `ax,ay,az`.
`--xyz X Y Z` 로 로봇 기준 목표를 직접 줄 수 있다 (heading 은 로봇 정면 +x, 장애물은 --target 의 가구). 음수로 시작하는 값은 `--tilt=-10,70` 처럼 `=` 로.
시뮬 장면은 팔 관절 범위를 실기 한계로 바꿔서 만든다.

설 자리 훑기 (`tools/stand_sweep.py`, 선반 가운데 = ZED 정면, 칸 = cost, × = 불가):
2~4층은 front 0.40~0.50 m, along −0.1~+0.1 m 에서 넓게 되고, 2층은 0.45 m 보다 가까우면 거의 안 된다.

시뮬 실행 결과 (2026-11-16, 기본 설 자리, heading 모드, 시작 = 0점 자세):

| 목표 | 설 자리 | 리프트 | 끝 오차 | 베이스 밀림 | 추종 최대 오차 |
|---|---|---|---|---|---|
| desk | 0.45 m | 0 mm | 5.0 mm | 4.8 mm | 1.3° |
| shelf2 | 0.45 m | 0 mm | 4.4 mm | 4.1 mm | 1.2° |
| shelf3 | 0.45 m | 0 mm | 4.7 mm | 4.5 mm | 1.2° |
| shelf4 | 0.45 m | 420 mm | 4.9 mm | 4.7 mm | 1.2° |
| shelf1 | 0.50 m | 0 mm | 3.4 mm | | 부하 1.00 |

모두 충돌 없음. 끝 오차는 대부분 팔을 앞으로 뻗을 때 시뮬 차체가 밀린 양이다 (팔 자체 오차 < 1 mm).

## 임무 시뮬 (`arm_kinematics/mission.py`, `tools/mission_sim.py`)

책상에서 과일 집기 → 바구니에 넣기 → 바구니 손잡이 잡기 → 3 m 뒤 선반으로 주행 → 3층에 두기 → 팔 빼기. ROS·Nav2 없이
임무 순서·IK·충돌·정지 오차 강건성을 본다.

```bash
python3 tools/mission_sim.py                                   # MuJoCo 창, 실시간
python3 tools/mission_sim.py --trials 20 --headless out/       # 창 없이, 단계마다 PNG, 성공률
python3 tools/mission_sim.py --nav-sigma 0.04,0.04,4 --nav-max 0.06,0.06,6 --perception-noise 0.01
```

- 배치: `choose_stand.py` 결과 자리 (책상 앞면 0.35 m, 선반 앞면 0.40 m, 가구 가운데 로봇 오른쪽 0.10 m), 리프트 210 mm 고정.
- 주행: 제자리 회전 → 직진 → 회전 + 정지 오차 (정규분포, 최대값으로 자름). 도착 후 가구·물체를 **실제 베이스 자세 기준으로
  다시 재서** 계획한다 (실제 흐름).
- 인식: 볼 때마다 물체별 수평 오차 하나 (`--perception-noise`). 높이는 놓인 면에서 정하므로 오차 없음. 잡을 곳·장애물 상자 모두
  이 '본' 위치, 성공 판정은 실제 위치.
- 충돌 여유: 가구 2 cm, 받침면(집을 물체 아래 책상 상판, 놓을 층의 판) 0.5 cm, 위치를 아는 물체 0.5 cm (`Box.margin`).
- 잡기: 그리퍼를 닫으면 물체가 그리퍼를 따라가고 열면 물리로 떨어진다. 과일은 손가락 사이에 단단히, **바구니는 손잡이로
  매달림** (위치만 그리퍼를 따르고 똑바로 섬). 충돌 검사기도 같은 모델 (`attached=[(…, "hang")]`) 로 든 물체를 가구·차체와 본다.
- 들고 빠지기: 3 cm 곧게 들어 올림 → 삽입 구간을 그만큼 높여 다시 푼 길로 빠짐 → 접기. 이 길 전체를 든 물체 포함 충돌 검사.
  바구니를 들면 0점 자세 대신 **들고 다닐 자세** (그리퍼 (0.22, −0.28, 1.12) m) 로 — 0점 자세는 그리퍼가 책상 앞 모서리 위라
  매달린 바구니가 상판에 걸린다. 주행과 선반 계획은 이 자세에서 시작.
- 판정: 과일이 바구니 안 (벽 안, 바닥 위), 바구니 바닥이 3층 판 위 ±2 cm 이고 판 안, 선반에 둔 뒤에도 과일이 바구니 안,
  집지 않은 채소가 1 cm 넘게 안 움직임.
- 책상 위 물체 (`arm_kinematics/objects.py`, 형상·자리·계획용 상자를 한 곳에): 오렌지(`fruit`, 지름 6 cm), 바구니
  (21 × 13 × 10 cm 실측, 손잡이 높이 13 cm 는 가정), 가지·바나나·당근·옥수수·고추·피망 (장난감 크기 추정, 기본형 조합).
  집지 않는 채소는 계획 때 장애물로 넣는다. 실물을 재면 `objects.py` 만 고치면 된다.

장면 파일 (`tools/export_scene.py`): 로봇 + 책상 + 선반 + 물체를 MJCF 로 저장. keyframe `home` = 처음 상태
(책상 앞, 리프트 210 mm, 팔 0점, 그리퍼 열림). 치수를 고치면 다시 내보낸다.

```bash
python3 tools/export_scene.py --repo            # → description/mjcf/mission_scene.xml (레포 메시 그대로)
python3 tools/export_scene.py ~/mission_scene   # → 혼자 열리는 폴더(XML + 쓰는 메시 50 개) + zip (약 16 MB), 레포 없는 사람에게
python3 -m mujoco.viewer --mjcf=../description/mjcf/mission_scene.xml
```

결과 (2026-11-16, 노트북 시행당 약 14 s, 메모리 1.7 GB):

| 조건 | 성공 |
|---|---|
| 정지 오차 σ 2 cm·2° (최대 3 cm·3°) | 20/20 |
| 정지 오차 σ 4 cm·4° (최대 6 cm·6°) | 15/15 |
| + 인식 오차 σ 1 cm | 15/15 |
| + 인식 오차 σ 2 cm | 12/15 — 실패 3 은 모두 과일을 5~6.5 cm 빗나가 못 잡음 |

정지 오차는 도착 후 다시 재서 계획하므로 6 cm·6° 까지 문제없고, 한계는 인식 오차다.
MjSpec 컴파일마다 약 350 MB 가 풀리지 않아 장면은 한 번 만들고 시행마다 `reset(seed)` 한다.

## 실기 실행 (`tools/play_plan.py`, 라즈베리파이)

구조: 노트북(나중에 Jetson)이 계획(IK·경로·MuJoCo 검증 → `plan.json`), 라파가 실행 (팔로워에 50 Hz 목표 전송, CAN).
상위 기기는 **경로 전체를 한 번** 넘기고, 시간에 민감한 50 Hz 전송·정지 판단은 라파 안에서만 한다
(네트워크 지연이 팔 제어 주기에 끼지 않게). 지금은 파일(scp)로 넘기고, 나중에 UDP·ROS 메시지로 바꾼다.
라파 루프가 늦으면 경로 시각을 건너뛰지 않고 끊긴 데서 이어 가며 0.3 s 동안 서서히 재출발한다 (0.25 s 넘게 끊기면 팔로워가 정지).
팔로워(팀 LeRobot 패치)가 안에서 100 Hz MIT·중력 보상·30°/s·120°/s² 제한을 돌리므로 실행기는 목표만 보낸다.
궤적은 `arm_kinematics/timeline.py` (numpy 만) 의 5차 곡선을 노트북 시뮬과 라파 실행기가 똑같이 쓴다.
리프트는 움직이지 않는다 (`--zed-height` 로 현재 높이 고정).

```bash
# 노트북: 팔이 쉬는(0점) 자세, 리프트 높이는 바닥→ZED 삼각대 나사 실측 (리프트 최저 0.675 m)
python3 tools/ik_sim_run.py --xyz 0.45 -0.15 1.00 --no-env --zed-height 0.675 --save-plan plan_free.json   # 빈 공간 첫 시험
python3 tools/ik_sim_run.py --target shelf3 --zed-height 0.885 --save-plan plan_shelf3.json                # 선반 (앞면 0.45 m, 가운데)
scp plan_free.json tools/play_plan.py arm_kinematics/timeline.py raspberrypi:~/arm_play/   # ssh 별칭 raspberrypi = pi@raspberrypi.local

# 라파 (모바일 로봇): 팔 = SocketCAN can_arm_r, 파이썬 = ~/rebot_env/venv (팀 lerobot_patched)
sudo ~/capstone/capstone_ws/can/can_up.sh                      # CAN 올리기
cd ~/arm_play
PY=~/rebot_env/venv/bin/python
$PY play_plan.py --check-only                                  # 팔로워 설정만 (모터 연결 안 함)
$PY play_plan.py plan_free.json --dry-run                      # 팔 없이 내용 확인
$PY play_plan.py --pose                                        # 현재 모터 1~6 (쉬는 자세면 0점 5° 안)
$PY play_plan.py plan_free.json --stop-at approach             # 접근점까지, 속도 0.5
$PY play_plan.py plan_free.json                                # 끝까지 → ENTER 로 같은 길 복귀
python3 play_plan.py --gripper open                            # 그리퍼만 (팔은 지금 자세 유지), close = 닫기
# 노트북: --pose 로 읽은 모터 각 → 모델상 손끝 위치 (줄자로 잰 값과 비교)
python3 tools/fk_check.py --zed-height 0.885 --motors=0,0,0,0,0,0
python3 play_plan.py plan_desk.json --grip-before open --grip-after close   # 열고 다가가 → 끝에서 잡기
python3 play_plan.py plan_basket.json --grip-after open        # 들고 와서 → 끝에서 놓기
```

손끝 (`right_arm_tool0`, IK 가 맞추는 점) = **두 손가락 끝의 가운데** (손가락 몸통은 그 뒤 0~9 cm). 0점 자세·리프트 210 mm
에서 (앞 25.5, 오른쪽 21.8, 바닥에서 96.9 cm).

팔로워 설정 `--setup mobile` (기본) = 라파 `~/rebot_env/arm_follow_test.py` 와 같은 값 (2026-11-15 결정): `can_arm_r`
(socketcan), id `arm_r`, 그리퍼 열림 340, 중력 보상 1.0·roll 45°·wrist_flex 0.85, 중력 URDF `~/rebot_env/` 수정본
(한계를 사용자 값 ±5° 로 넓힘), 관절 한계 collector `USER_LIMITS_DEG`, 그리퍼 900°/s, 끊김 hold.
data_collection 팔(USB, `config.env`)은 `--setup data_collection`.

그리퍼: 팀 팔로워의 공개 각도 **−270° = 완전 열림 (손가락 관절 0.05 m), 0° = 닫힘**, 힘 제한 위치 모드라 닫기를 보내면
물체에 막혀 멈춘다. 그리퍼를 움직이는 동안 팔은 그 자리 목표를 계속 보낸다 (명령 끊김 방지). 목표에 닿거나 0.4 s 동안
1° 넘게 안 움직이면(막힘) 끝, 최대 4 s. 닫은 뒤 −15° (`--grip-empty-deg`) 보다 열린 채 멈췄으면 '잡힘', 끝까지 닫혔으면
'빈손' 경고. 잡다가 팔이 10° 넘게 밀리면 고정·정지. `--grip-after` 는 끝에 제대로 도착했을 때만 하고, 복귀 길에서도
그리퍼는 마지막 명령 그대로 (쥔 채 돌아옴). 아직 없음: 잡은 뒤 들어 올리고 빠지는 길 (지금 복귀는 삽입 구간을 거꾸로 →
면 위 물체가 끌림, 시뮬 `mission.py` 에는 있음), 실기 빈손 판정 각도 보정 (실제 과일·바구니 손잡이로 재야 함).

안전 (2026-11-16 검토 반영):
- **계획 쪽** (`ik_sim_run.py`): 손가락 닫힘·완전 벌림(0.05 m) 둘 다 충돌 검사, 가구 상자 사방 2 cm 여유 (`--margin`),
  관절 한계 기본 `--limits conservative` (collector 한계 ∩ 팀 중력 URDF 한계 — data_collection 팔로워가 실제로 허용하는 범위),
  시작 자세를 IK 시작점·이동 비용에 반영, **MuJoCo 검증을 통과해야만 plan.json 저장** (`validated: true`).
  MuJoCo 의 가구는 보이기만 하고, 충돌은 시뮬 자세를 Pinocchio 메시로 검사한다 (MJCF 팔 형상은 접촉이 꺼져 있음).
- **실행 쪽** (`play_plan.py`): `validated` 가 아니거나 리프트가 고정이 아니면 연결 전에 거부, 연결 후 팔로워의 실제 관절 한계
  밖 점이 있으면 거부 (조용히 잘리면 검증한 경로와 달라짐), `comm_loss_action=hold`, 시작 맞춤 뒤 `hold()`
  (ENTER 대기 중 0.25 s 명령 끊김 오류 방지), 측정이 10° (삽입 단계 5°, `--insert-lag`) 넘게 늦으면 경로 시간 멈춤,
  그 상태가 1 s (`--stall`) 이어지거나 끝에서 2 s (`--settle`) 안에 2° 안으로 못 들어오면 부딪힘으로 보고 측정 자세로 고정·정지,
  ENTER 로 **측정 자세에서 시작해** 같은 길로 복귀, 종료는 팀 코드대로 받친 뒤 ENTER 로 토크 해제.
- IK 는 모델에 넣은 것만 피한다 — 선반은 도착 후 ZED 로 잰 위치를 넣고 (예정), 빈 공간 시험은 정말 비운 곳에서.

층별 리프트 (기본 설 자리, 위 안전 조건, 검증 통과 — 리프트를 먼저 이 높이로 올리고 ZED 높이를 재서 `--zed-height`):

| 목표 | 리프트 | ZED 높이 |
|---|---|---|
| shelf2 | 0 mm | 0.675 m |
| desk, shelf3 | 210 mm | 0.885 m |
| shelf4 | 420 mm | 1.095 m |
| shelf1 | 여유 2 cm 에서는 불가 | |

## 시험

```bash
cd capstone_ws/src/arm_kinematics && python3 -m unittest discover -s tests -v
```

- 공개 각도 → URDF 변환
- 장착 회전이 정확히 roll +45°
- 우리 URDF 팔 체인 = 팀 중력 URDF 팔 체인 (end_link 자세 일치)
- 리프트가 tool 을 수직으로 2×stage 만큼 올림
- 자코비안 = FK 유한차분
- **우리 전체 모델(평범한 −z 중력)의 팔 중력 토크 = 팀 보상기(roll 45°)** — 부호가 틀리면(−45°) 크게 어긋남을 같이 확인
- 관절별 배율이 팀 compute 결과 뒤에 곱해짐
