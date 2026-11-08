# swerve_controller

차체 속도 `cmd_vel` → 조향 4 (RobStride 02) + 구동 4 (BL41105) 명령. 계산은 ksrc 로버 펌웨어
`firmware/common/{swerve_kinematics,drive_align_gate}.c` 를 C++ 로 옮긴 것 (로직·상수·테스트 케이스 동일).

전제: **조향축이 구동 바퀴 중심 바로 위** (캐스터 오프셋 없음) → 조향각 = 바퀴 굴러가는 방향.

```
cmd_vel ─▶ swerve_controller ─▶ wheel_cmd  ─▶ wheel_can_bridge ─▶ XDrive ×2 (CAN, 500k)
               ▲            └─▶ steer_cmd  ─▶ steer_can_bridge ─▶ RobStride 02 ×4 (CAN can_steer, 1 Mbps, MIT)
               └──────────────── steer_state ◀┘
```

## 규약
- 차체: REP-103 — x 전진, y 좌측, ω + = 위에서 보고 반시계.
- 조향각: 0 = 바퀴 전방, + = 반시계, **운용 한계 ±180°** (기구 스토퍼/케이블 ±190°, URDF `steer_limit` — 여유 10°).
- 바퀴 속도 + = 조향각 방향으로 굴러감. `wheel_cmd` 는 turn/s = (m/s) / (2π·wheel_radius).
- 배열 순서 = **[1 FR, 2 RR, 3 RL, 4 FL]** (`wheel_cmd`, `steer_cmd`, `steer_state`, 파라미터 `module_x/y` 모두).

## 한 주기 (50 Hz)
1. **IK** — 바퀴별 필요 속도벡터 v + ω×r. 직전 각에서 90° 넘게 돌아야 하면 180° 뒤집고 부호 반전.
   어느 바퀴든 `max_wheel_mps` 넘으면 전부 같은 비율로 축소 (운동 모양 보존).
   바퀴 속도가 `steer_min_mps`(0.03 m/s) 미만이면 **조향각 유지** + 그 속도의 현재 바퀴 방향 성분만 굴림
   (스틱을 놓을 때 튀는 1~2 cm/s 잡음이 최대 90° 조향 목표가 되던 것 방지).
2. **±180° 접기** — θ 와 θ±180° 중 직전 명령각에 가까운 것. 범위 밖이면 반대쪽 등가각 (이음매 — 피할 수 없음).
   unwind(멈춰 있을 때 |θ| > `unwind_deg` 면 반대쪽 등가각으로 미리 180° 돌림)는 **끔** (`unwind_deg: 0`, ksrc 원본과 같음).
   멈춰 있는 동안 명령 없이 바퀴가 돌아서 — 팔 작업 중·사람 근처에서 위험. 이음매 반전은 3·4 가 안전하게 처리.
3. **정지 후 조향** — 목표 조향각이 60° 넘게 바뀌는데 바퀴가 아직 구르면(`wheel_state` > 0.03 m/s) 조향은 직전 각 유지 + 구동 0,
   바퀴가 실제로 멈추면 새 각으로 돌림. ±180° 이음매 반전·큰 방향 전환 때 구르는 바퀴가 180° 돌아 **차체가 넘어지던 것** 대책
   (시뮬: 대책 전 0.5 m/s 에서 전복 → 대책 후 0.6 m/s 에서도 기울기 0.2°).
   `wheel_state` 가 없으면 보드 램프(0.66 m/s²)로 감속을 추정해서 기다림.
4. **조향 회전 속도 제한** — 목표각을 한 번에 보내지 않고 `max_steer_rate_deg_s`(360°/s, 180° 반전 0.5 s) 로 옮겨 감.
   제한이 없으면 정지 후 180° 반전 때 조향이 계단 명령으로 "확" 돎 (시뮬 실측 2400°/s → 제한 후 ~400°/s).
5. **정렬 게이트** (최종 목표각 기준) — 목표 조향각이 60° 넘게 바뀌면(첫 구동 포함) 구동 0 으로 보류,
   `steer_state` 실제각이 전 바퀴 목표 ±10° 안에 들면 재개. 한 바퀴만 먼저 굴러 생기는 의도치 않은 회전 방지.
   **실제각을 모르면(steer_state 없음/ready=false) 구동하지 않음.** ksrc 와 달리 시간초과 강제 해제 기본 끔.
6. **명령 0 이면 조향 그대로** — 전 바퀴 속도가 0 이면 지금 각 유지 + 3 에서 미뤄 둔 목표 취소.
   정지 대기 중엔 매 틱 붙잡은 각 기준으로 목표를 다시 계산 (실제 각에 가장 가까운 등가각 → 부호 반전으로 해결되면 안 돎).
   전엔 스틱 잡음이 남긴 목표를 바퀴가 선 뒤 실행해서 **명령 0 인데 0° → 180°(같은 방향) 로 돌았음** —
   무작위 조작 20분 시뮬(바인딩)에서 명령 0 중 30°+ 회전 783회 → 0회 (2026-11-07).

## 상태
| 상태 | 조향 | 구동 | 전환 |
|---|---|---|---|
| FREE (시작) | `MODE_FREE` 토크 끔 | `MODE_FREE` IDLE | 신선한 `cmd_vel` → ACTIVE (`~/set_free true` 로 고정했으면 안 감) |
| ACTIVE | `MODE_POSITION` | `DRIVE` (게이트 적용) | `cmd_vel` 이 `cmd_timeout` 넘게 끊기면 구동 `HOLD` + 조향 각 유지. `~/set_free true` → FREE |

ACTIVE 진입 때 IK 이력을 `steer_state` 실제각으로 시작 (FREE 중 손으로 돌려 둔 각에서 이어감).

## 실행
```bash
cd ~/capstone/capstone_ws && colcon build --packages-select capstone_msgs swerve_controller && source install/setup.bash
colcon test --packages-select swerve_controller && colcon test-result --verbose   # gtest 22 + Python 5

ros2 launch swerve_controller swerve_controller.launch.py
# 조향 브리지 없이 구동만 시험 (게이트 끔 — 바퀴 띄운 상태에서만!)
ros2 launch swerve_controller swerve_controller.launch.py require_steer_feedback:=false
ros2 launch wheel_can_bridge wheel_can_bridge.launch.py          # 다른 터미널

ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
ros2 service call /swerve_controller/set_free std_srvs/srv/SetBool "{data: true}"   # 전부 FREE 고정
```
`require_steer_feedback:=false` 면 조향 명령은 나가지만 실제 조향은 안 도는데 구동은 돌므로,
**조향각 0° 이 아닌 명령(횡이동·회전)은 바퀴가 엉뚱한 방향으로 굴러감** — 전진/후진만 시험할 것.

## 다른 곳에서 가져다 쓰기
계산 코어(`swerve_core`)는 ROS 와 무관하고 설치·export 돼 있음.

```python
# Python — source install/setup.bash 후 어디서든
import swerve_kinematics as sk
cfg = sk.ControllerConfig()
cfg.modules = [(0.1048, -0.1556), (-0.1048, -0.1556), (-0.1048, 0.1556), (0.1048, 0.1556)]  # FR RR RL FL
cfg.lo, cfg.hi = [-2.967] * 4, [2.967] * 4
ctl = sk.Controller(cfg)
r = ctl.step(0.2, 0.0, 0.5, actual_angle=[0.0] * 4, actual_ok=True)   # r.angle, r.speed, r.wheel_speed, r.gated
sk.fk(cfg.modules, r.angle, r.speed)                                   # (vx, vy, omega)
```
```cmake
# C++ 패키지 — package.xml 에 <depend>swerve_controller</depend>
find_package(swerve_controller REQUIRED)
target_link_libraries(my_target swerve_controller::swerve_core)   # #include "swerve_controller/kinematics.hpp"
```

## 파라미터
[config/swerve_controller.yaml](config/swerve_controller.yaml). **`module_x/y` 는 YOR placeholder 값** — 차체 CAD 나오면 교체.
`wheel_radius` 는 지면 하중 실은 뒤 실측(직진 거리 / 회전수)으로 보정.

## 파일
| 파일 | 내용 |
|---|---|
| `include/.../kinematics.hpp`, `src/kinematics.cpp` | ROS 무관: `ik_compute`, `fold_to_range`, `fk_compute`(오도메트리용 최소제곱), `AlignGate`, `Controller`(셋 묶음) |
| `src/swerve_node.cpp` | ROS 연결 (토픽/서비스/상태) |
| `src/python_bindings.cpp` | pybind11 → `import swerve_kinematics` |
| `test/test_kinematics.cpp` | ksrc 케이스 + ±170° 3바퀴 감기·unwind·FK 왕복·seed·정지 후 조향·멈춰 있을 때 조향 유지 |
| `test/test_bindings.py` | Python 바인딩이 C++ 와 같은 값 내는지 |

시뮬: [`../sim`](../sim/README.md) — `ros2 launch capstone_sim sim.launch.py`

## TODO
- ~~조향 브리지~~ → [`steer_can_bridge`](../steer_can_bridge/) (2026-11-07, 실기 미검증). `ready` = 전 축 응답 + 각도 확정 + 고장 없음 (위치 제어 중일 필요 없음 → ACTIVE 진입 때 seed 가 실제각으로 됨)
- 오도메트리 노드: `fk_compute` + `wheel_state`/`steer_state` → `odom` + TF.
- X-lock (작업 중 밀림 방지) 모드.
