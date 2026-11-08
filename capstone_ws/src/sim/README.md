# capstone_sim — MuJoCo 시뮬

실기의 **조향 브리지 + wheel_can_bridge (+ RobStride / XDrive)** 자리를 대신하는 노드.
같은 토픽을 쓰므로 `swerve_controller` 는 실기와 **같은 바이너리·같은 설정 파일**로 붙는다.

```
cmd_vel ─▶ swerve_controller ─▶ steer_cmd / wheel_cmd ─▶ mujoco_sim ─▶ steer_state / wheel_state
                ▲                                                    └─▶ joint_states, sim/ground_truth, TF odom→world
                └────────────────────────── steer_state ◀────────────┘
```

## 실행
```bash
cd ~/capstone/capstone_ws && colcon build && source install/setup.bash
ros2 launch capstone_sim sim.launch.py                   # MuJoCo 뷰어 + 컨트롤러 (기본: RViz·패드 없음)
ros2 run capstone_teleop virtual_pad.py                  # 가상 패드 — 다른 터미널에서 따로 (왼쪽=병진, 오른쪽=회전, Q/E 회전, Space 정지)
ros2 launch capstone_sim sim.launch.py pad:=true         # 패드를 같이 띄우고 싶을 때
ros2 launch capstone_sim sim.launch.py teleop:=true      # + SHAKS S0 게임패드 (RB 누른 채 스틱)
ros2 launch capstone_sim sim.launch.py rviz:=true        # + RViz
ros2 launch capstone_sim sim.launch.py headless:=true rviz:=true   # MuJoCo 창 없이 RViz 만

ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.4}}"
ros2 service call /swerve_controller/set_free std_srvs/srv/SetBool "{data: true}"
```
실기와 같은 DDS 망에 있으면 시뮬 명령이 실기로 갈 수 있음 → 실기 옆에서 돌릴 땐 `export ROS_DOMAIN_ID=<다른 번호>`.

뷰어: 카메라는 차체를 따라감. 키 `3` = 충돌 형상 (바퀴 원기둥) 표시. 오른쪽 Control 패널 = 리프트·팔·그리퍼 슬라이더 (조향·구동 슬라이더는 노드가 덮어씀).

RViz (`rviz:=true`): 고정 프레임 `odom`, 차체 따라가는 시점, 빨간 화살표 = 참값 궤적(`sim/ground_truth`).
TF = 시뮬이 `odom → world` + `joint_states`, `robot_state_publisher` 가 URDF 로 나머지 링크.

## 조향·구동이 움직이는 방식
시뮬 노드가 **물리 스텝(500 Hz)마다 모터 토크를 직접 계산**해서 MuJoCo 액추에이터(토크 모터)에 넣는다.
바퀴–바닥 마찰로 차체가 움직임 (위치를 직접 옮기지 않음).

| 축 | 명령 (토픽) | 토크 계산 | 한계 |
|---|---|---|---|
| 조향 ×4 | `steer_cmd` POSITION `angle[rad]` | τ = 40·(θ명령 − θ) − 1.5·θ̇ (PD) | ±17 Nm, 스토퍼 ±190° |
|  | `steer_cmd` FREE | τ = 0 (손으로 돌아감) | |
| 구동 ×4 | `wheel_cmd` DRIVE `velocity[turn/s]` | 목표가 **램프 2 turn/s²(= 0.66 m/s²)** 로만 변함 (보드 VEL_RAMP 와 같음) → τ = 0.6·e + 6·∫e (속도 PI) | ±6 Nm, ±2 turn/s |
|  | `wheel_cmd` HOLD (또는 DRIVE 0.2 s 끊김) | 목표 0 으로 램프 감속, 이후 적분항이 위치를 붙잡음 | |
|  | `wheel_cmd` FREE | τ = 0, 적분 리셋 | |

평소 흐름: 패드/게임패드 → `cmd_vel` → **swerve_controller** (IK → ±170° 접기 → 정렬 게이트) → `steer_cmd` + `wheel_cmd`
→ 시뮬 → `steer_state`(실제 조향각, 게이트용) + `wheel_state`.

### 컨트롤러 없이 직접 명령
swerve_controller 가 같은 토픽에 계속 쓰므로 **시뮬만 단독**으로 띄운 뒤:
```bash
ros2 run capstone_sim mujoco_sim.py
ros2 topic pub -r 50 /steer_cmd capstone_msgs/msg/SteerCommand "{mode: 1, angle: [0.785, 0.785, 0.785, 0.785]}"     # FR RR RL FL [rad] — 전부 45° = 왼쪽 앞 대각선
ros2 topic pub -r 50 /wheel_cmd capstone_msgs/msg/WheelCommand "{mode: 1, velocity: [0.5, 0.5, 0.5, 0.5]}"          # [turn/s], DRIVE
ros2 topic pub --once /wheel_cmd capstone_msgs/msg/WheelCommand "{mode: 2}"                                        # HOLD
```
`wheel_cmd` DRIVE 는 0.2 s 안에 계속 보내야 함 (`-r`) — 끊기면 HOLD (실기 브리지와 같음).
뷰어 Control 패널의 조향/구동 슬라이더는 노드가 매 스텝 덮어써서 안 먹음 (리프트·팔·그리퍼 슬라이더는 됨).

### 넘어짐 (2026-11-07)
차체가 키에 비해 앞뒤 바퀴 간격이 좁음 (조향축 x ±0.0911). 팔 home 자세면 무게중심이 앞쪽(x +0.04)으로 쏠려서
**앞으로 넘어지기 시작하는 감속 ≈ 1.55 m/s² (리프트 최저) / 1.05 m/s² (리프트 최고)**. 보드 램프 0.66 m/s² → 여유 1.6~2.3 배.
- 램프 없이(`vel_ramp_rate:=0`) 0.6 m/s 에서 정지 → 완전히 뒤집힘. 램프 2 turn/s² → 0.1°, 0.9 s·29 cm 에 걸쳐 정지
- 실기도 같음: **보드 `vel_ramp_rate` 를 올리거나 `input_mode` 를 VEL_RAMP 가 아닌 것으로 바꾸면 급정지 때 넘어질 수 있음**

## 모델
**`src/description/mjcf/mobile_robot_right_arm.xml`** (패키지 이름 `mobile_robot_right_arm_description`) — 미리 생성한 MJCF (켤 때 변환 없음).
원본은 같은 패키지의 `urdf/mobile_robot_right_arm.urdf` (이동로봇 + B601-DM 오른팔 + 손목 D435i, 메쉬 단순화 없음).

```bash
# 부품 무게 바뀌면: set_urdf_inertia.py 의 표 수정 → 실행 (URDF <inertial> 갱신) → build_mjcf.py
python3 src/sim/scripts/set_urdf_inertia.py
# URDF 고친 뒤 MJCF 다시 만들기 (capstone_ws 에서) → colcon build 후 시뮬 재시작
python3 src/sim/scripts/build_mjcf.py
# ROS 없이 MuJoCo 로만 보기 (Control 패널 = 리프트·팔·그리퍼 슬라이더)
python3 -m mujoco.viewer --mjcf=src/description/mjcf/mobile_robot_right_arm.xml
```

URDF 에 MuJoCo 용으로 덧붙인 것 (`build_mjcf.py`):
| 항목 | 값 | 비고 |
|---|---|---|
| 차체 | `base_link` freejoint | URDF `world` 링크 = 바닥 기준점 → TF `odom → world` |
| 질량 | URDF 값. 이동로봇 링크는 **부품 무게표(2026-11-07)** 로 기록: 차체 13.55, 조향 0.27 ×4, 바퀴 1.5 ×4, 리프트 1.2/1.0, 상판 0.26 kg (합 23.2 kg) | `set_urdf_inertia.py` (표·배정 근거는 그 파일). 무게중심·관성 모양은 메쉬(균일 밀도) |
| 손목 D435 | DAE → 같은 형상 STL 2개 | MuJoCo 는 DAE 못 읽음 |
| 접촉 | 바퀴 원기둥 r 0.0525 ↔ 바닥만 | 나머지는 보기용 (키 `3` = 충돌 형상) |
| 조향 | ±190° 스토퍼, 토크 모터 ±17 Nm | 노드가 PD (kp 40, kd 1.5) — RobStride 위치 모드 흉내 |
| 구동 | 토크 모터 ±6 Nm | 노드가 속도 PI (kp 0.6, ki 6) — ODrive 흉내, DRIVE 0.2 s 끊기면 HOLD |
| 리프트·팔·그리퍼 | 위치 액추에이터 + 중력보상, mimic → equality | 뷰어 Control 슬라이더로 조작. 무게는 바퀴에 안 실림 |
| 시작 자세 | keyframe `home` (joint2 −2, joint3 −1.2, joint4 −1, 그리퍼 0.035) | URDF launch 기본값과 같음 |
| 카메라 | `wrist_color`, `front` | 뷰어 Camera 메뉴 |

**모터 게인·마찰은 시뮬 전용 임의값** (식별 안 함). 질량은 부품 무게표 기반 대략값 (Jetson·라즈베리파이 본체·케이블 미포함). 다른 MJCF: `ros2 run capstone_sim mujoco_sim.py --ros-args -p mjcf:=<경로>`.

## 확인한 것 (2026-11-07, headless, mobile_robot_right_arm 모델)
- 전진 0.2 m/s → 0.2 m/s, 좌 횡이동 → +y (x 변화 ~0)
- 제자리 회전 0.5 rad/s → 0.497 rad/s, 조향 ±34°(+180°) = 조향축 (±0.0911, ±0.136) 계산값 33.8°
- 명령 끊김 → HOLD 로 즉시 정지, 이어서 unwind (144° → −36°, 구동 안 할 때 중앙 쪽으로 되감기)
- 정렬 게이트: 조향 0→90° 도는 동안 구동 보류

## 쓸 데
- 게임패드 부호 확인 (`capstone_teleop` 의 `scale_*` TODO) — 실기 전에 시뮬에서
- swerve_controller 파라미터(unwind, 게이트) 바꿔 보기
- 이후 오도메트리 노드: `sim/ground_truth` 와 비교
