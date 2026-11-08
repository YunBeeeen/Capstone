# steer_can_bridge — 조향 4축 CAN ↔ ROS2 (C++)

RobStride 02 ×4 (CAN `can_steer`, 1 Mbps, MIT 위치 제어)를 ROS2 토픽으로 묶는 노드. swerve_controller 의 `steer_cmd` 를 받아 돌림.
프로토콜 기준 구현과 실기 도구는 Python [`~/capstone/capstone_ws/Steer_motor/steer_can`](../../Steer_motor/steer_can/) —
두 쪽이 같은 테스트 벡터(`tests/rs02_vectors.csv`)로 검증됨. 배경(켤 때 각도 확정 등): [`Steer_motor/README.md`](../../Steer_motor/README.md).

| 인터페이스 | 타입 | 내용 |
|---|---|---|
| sub `steer_cmd` | `capstone_msgs/SteerCommand` | `mode` FREE(0) / POSITION(1), `angle[]` 조향 기준 rad (POSITION 만, 길이 4) |
| pub `steer_state` | `capstone_msgs/SteerStateArray` | `phase`, `ready`, `fault`, `angle[]` 실제 조향각, 축별 alive / confirmed / mode / fault / 각·속도·토크·온도 / 램프 위치 |
| srv `~/clear_errors` | `std_srvs/Trigger` | 모터 고장 클리어(Type04 clear) + FAULT 해제 → FREE |

## 동작

```
FREE ──POSITION 명령 + 전 축 응답·각도 확정·고장 없음──▶ ENTERING ──전 축 RUN 확인──▶ ACTIVE
  ▲   (Type04 20 Hz 폴링: 토크 끈 채로 각도 읽기)      (canTimeout 설정 + enable,      │ 매 틱 MIT: 램프 위치 + 속도 FF
  │                                                    RUN 된 축은 현재각에서 유지)    │ 명령 끊김(0.2 s) = 그 자리 유지
  └─────────────────────────── FREE 명령 (전 축 Type04 = 토크 끔) ◀──────────────────────┘
ENTERING/ACTIVE 에서: 피드백 끊김(0.15 s) / 고장 비트 / Type21 / RUN 이탈 / enable 실패(1 s) → FAULT (전 축 토크 끔, clear_errors 로만 해제)
종료(Ctrl+C): 전 축 토크 끔
```

- **MIT 램프**: 목표가 바뀌면 `max_rate`(2 rad/s)로 명령 위치를 옮기고, 그 속도를 `v` 피드포워드로 같이 보냄
  (램프 중 kd 가 움직임을 붙잡지 않게). 목표가 급변해도 토크가 kp × 큰 오차로 튀지 않음.
- **enable 직후 튐 없음**: RUN 확인된 순간의 실제각을 명령 시작점으로 잡음.
- **`ready`** = 전 축 응답 + 각도 확정 + 고장 없음. **위치 제어 중일 필요 없음** — swerve 가 ACTIVE 로 들어가는 순간
  (브리지는 아직 FREE) 실제각으로 IK 이력을 seed 해야 해서. 미확정이면 false → swerve 정렬 게이트가 구동을 막음.
- 명령이 끊기면 FREE 가 아니라 **그 자리 유지**(토크 유지) — 구동 HOLD 와 짝. 브리지 자체가 죽으면 모터 `canTimeout`(0.5 s)이 토크를 끔.
- ACTIVE 중 축이 밖에서 disable 되면(`enable.py --enable 0` 등) 다시 enable 하지 않고 FAULT — 비상정지를 덮어쓰지 않게.
- 한 축 고장 = 전 축 토크 끔 (스워브는 한 바퀴만 틀어져도 차체가 끌림).
- 버스 부하: 4축 × 200 Hz MIT + 응답 ≈ 1 Mbps 의 25%.

파라미터: [config/steer_can_bridge.yaml](config/steer_can_bridge.yaml) (`steer_sign`, `kp`/`kd`/`max_rate` — **실기 튜닝 TODO**, 타임아웃).

## 빌드 / 테스트

```bash
cd ~/capstone/capstone_ws && source /opt/ros/humble/setup.bash
colcon build --packages-select capstone_msgs steer_can_bridge
colcon test --packages-select steer_can_bridge && colcon test-result --verbose   # 프로토콜 벡터 + 상태기계 시나리오 18개
```

## 실기 시험 (바퀴 띄우고)

```bash
sudo ip link set can_steer up type can bitrate 1000000
source ~/capstone/capstone_ws/install/setup.bash

# 터미널 A — "전 축 응답 + 각도 확정" 로그가 떠야 함 (미확정이면 어느 축인지 경고)
ros2 launch steer_can_bridge steer_can_bridge.launch.py

# 터미널 B
ros2 topic echo /steer_state --once        # ready: true, motors[].confirmed 4개
ros2 topic pub -r 50 /steer_cmd capstone_msgs/msg/SteerCommand "{mode: 1, angle: [0.5, 0.5, 0.5, 0.5]}"
#   → 4축 약 29° 로 램프. Ctrl+C 로 pub 끊으면 0.2 s 뒤 그 자리 유지 (손으로 돌리면 버팀)
ros2 topic pub --once /steer_cmd capstone_msgs/msg/SteerCommand "{mode: 0}"   # 토크 끔
ros2 service call /steer_can_bridge/clear_errors std_srvs/srv/Trigger         # FAULT 해제
```

안전 확인: 위치 유지 중 터미널 A Ctrl+C → 전 축 토크 끔 / 모터 하나 CAN 선 뽑기 → FAULT, 전 축 토크 끔.

스워브와 같이: `ros2 launch swerve_controller swerve_controller.launch.py` + 이 노드 + `wheel_can_bridge` + `capstone_teleop`.

하드웨어 없이: vcan + 가짜 모터 (`Steer_motor/steer_can/tests/fake_rs02.py` 맨 위 설명) → `can_interface:=vcan1`.
