# wheel_can_bridge — 구동 바퀴 4축 CAN ↔ ROS2 (C++)

XDrive ×2 (ODrive v3.6 호환, fw 0.5.1, CANSimple 500 kbps) 위의 바퀴 4개를 ROS2 토픽으로 묶는 노드.
프로토콜 기준 구현과 실기 도구는 Python [`~/capstone/capstone_ws/Wheel_motor/wheel_can`](../../Wheel_motor/wheel_can/) — 두 쪽이 같은 테스트 벡터(`tests/cansimple_vectors.csv`)로 검증됨.

| 인터페이스 | 타입 | 내용 |
|---|---|---|
| sub `wheel_cmd` | `capstone_msgs/WheelCommand` | `mode` FREE(0) / DRIVE(1) / HOLD(2), `velocity[]` 바퀴 기준 turn/s (DRIVE 만, 길이 4) |
| pub `wheel_state` | `capstone_msgs/WheelStateArray` | `phase`, `ready`, `fault`, `vbus`, 바퀴별 alive / axis_state / axis_error / pos / vel / Iq |
| srv `~/clear_errors` | `std_srvs/Trigger` | 보드 에러 클리어 + FAULT 해제 → FREE |

## 동작

```
FREE ──DRIVE/HOLD 명령 + 4축 heartbeat──▶ ENTERING ──4축 CLOSED_LOOP 확인──▶ ACTIVE
  ▲        (속도 0, 진입 요청 0.5 s 재시도, 3 s 제한)                         │ DRIVE: 명령 속도 / HOLD·명령 끊김(0.2 s): 0
  └──────────────────────── FREE 명령 (속도 0 + IDLE) ◀──────────────────────┘
어디서든: heartbeat 끊김 / axis_error / 폐루프 이탈 / 진입 실패 → FAULT (전 축 0 + IDLE, clear_errors 로만 해제)
종료(Ctrl+C): 전 축 0 + IDLE
```

- 한 축 고장 = 전 축 정지 (스워브는 바퀴 하나만 서도 차체가 끌림).
- ACTIVE 중 축이 밖에서 IDLE 로 바뀌면(`stop.py` 등) 다시 진입하지 않고 FAULT — 비상정지를 덮어쓰지 않게.
- 명령이 끊기면 FREE 가 아니라 **HOLD**(폐루프 속도 0) — 브리지가 살아 있는 한 경사에서도 안 굴러감. 브리지 자체가 죽는 경우는 보드 워치독 몫 (미설정).
- 속도 명령은 매 틱(50 Hz) 재송신 → 워치독 켜도 그대로 동작하게.
- 피드백 RTR 은 틱당 4개씩 나눠 송신(한 틱 총 ≤ 8 프레임 — SocketCAN 송신 큐 기본 10). FREE 에선 5 Hz + 엔코더만 (Iq 생략) — heartbeat 는 보드가 알아서 보내므로 항상 받음.
- 어댑터 LED: RX 는 보드 heartbeat(100 ms, 보드가 항상 보냄) 때문에 FREE 에서도 깜빡임 — 정상.

파라미터: [config/wheel_can_bridge.yaml](config/wheel_can_bridge.yaml) (`can_ids`, `wheel_sign`, `vel_max`, 주기, 타임아웃).

## 빌드 / 테스트

```bash
cd ~/capstone/capstone_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select capstone_msgs wheel_can_bridge
colcon test --packages-select wheel_can_bridge && colcon test-result --verbose   # 프로토콜 벡터 + 상태기계 시나리오
```

## 실기 시험 (바퀴 띄우고)

```bash
sudo ip link set can_drive up type can bitrate 500000 restart-ms 100
sudo ip link set can_drive txqueuelen 100        # 기본 10 — 여유 (노드는 틱당 ≤8 프레임으로 맞춰 둠)
source ~/capstone/capstone_ws/install/setup.bash

# 터미널 A — "모든 바퀴 heartbeat 확인" 로그가 떠야 함
ros2 launch wheel_can_bridge wheel_can_bridge.launch.py

# 터미널 B
ros2 topic echo /wheel_state --once                       # ready: true, alive 4개, vbus ~24
ros2 topic pub -r 20 /wheel_cmd capstone_msgs/msg/WheelCommand "{mode: 1, velocity: [0.3, 0.3, 0.3, 0.3]}"
#   → 4바퀴 0.3 turn/s. Ctrl+C 로 pub 끊으면 0.2 s 뒤 HOLD (폐루프 속도 0, 손으로 돌리면 버팀)
ros2 topic pub --once /wheel_cmd capstone_msgs/msg/WheelCommand "{mode: 0}"   # FREE → IDLE (자유 구름)
ros2 service call /wheel_can_bridge/clear_errors std_srvs/srv/Trigger         # FAULT 해제
```

안전 확인: 회전 중 터미널 A 를 Ctrl+C → 전 축 IDLE / 회전 중 보드 하나 CAN 선 뽑기 → FAULT, 전 축 IDLE.

하드웨어 없이: vcan + 가짜 보드 (`~/capstone/capstone_ws/Wheel_motor/wheel_can/tests/fake_odrive.py` 맨 위 설명) → `can_interface:=vcan0`.

## 구조

| 파일 | 역할 |
|---|---|
| `include/.../cansimple.hpp` | 프레임 인코드/디코드 (순수 함수, Python `lib/cansimple.py` 와 1:1) |
| `include/.../supervisor.hpp`, `src/supervisor.cpp` | 상태기계 (ROS·소켓 무관, 시간 주입 → gtest) |
| `src/socketcan.cpp` | SocketCAN raw 논블로킹 소켓 |
| `src/bridge_node.cpp` | 소켓 ↔ 상태기계 ↔ ROS 연결만 |
| `test/` | `test_cansimple` (공용 벡터), `test_supervisor` (시나리오 16개) |
