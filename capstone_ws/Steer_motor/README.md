# Steer_motor — 조향 RobStride 02 ×4

스워브 조향축 4개. RS02 출력축 → 베어링 → 조향축 (추가 감속 없음, 조향각 = RS02 출력각).
프로토콜·도구는 `~/idle_ws/motor` (팔에서 실기 검증된 MIT 구현)를 RS02 전용으로 옮긴 것.

| 항목 | 값 |
|---|---|
| 모터 | RobStride 02 (내장 감속 7.75:1, 엔코더 14 bit 1회전 절대, 17 Nm 피크) |
| CAN | **`can_steer`**, 1 Mbps, 확장 29-bit (RS02 private 프로토콜). 버스 배정: can_drive 구동 / can_steer 조향 / can_arm_r 오른팔 / can_arm_l 왼팔 |
| CAN ID | 위에서 보고 시계방향 **1 FR, 2 RR, 3 RL, 4 FL** (구동 XDrive 와 같은 번호), 호스트 0xFD |
| 영점 | 바퀴 정면 = 0, `zero_sta = 1` 저장 (켤 때 −π~π) — 2026-11-07 4축 저장 |
| 제어 | **MIT 위치 제어** (모터 안에서 t = kp·(p − p_actual) + kd·(v − v_actual)). 호스트가 램프(속도 제한) + 속도 피드포워드 |
| 기구 한계 | 스토퍼/케이블 ±190°, 운용 ±180° (swerve 가 접음, 여유 10°) |

도구 2단계: **1 이 폴더 `steer_can/` (Python, 실기 시험·설정)** → 2 C++ ROS2 브리지
[`capstone_ws/src/steer_can_bridge`](../src/steer_can_bridge/) (운용).
프로토콜은 `steer_can/lib/frames.py` 가 기준, C++ `rs02.hpp` 는 1:1 대응이며 **같은 벡터 파일**
[`steer_can/tests/rs02_vectors.csv`](steer_can/tests/rs02_vectors.csv) 로 둘 다 테스트.

## 켤 때 각도 확정 (중요)

RS02 는 켤 때 **한 바퀴 안의 각도만** 앎 (`zero_sta=1` → −180°~180°). 조향 기구는 ±190° 까지 가므로
실제 +185° 는 켤 때 **−175° 로 읽힘** (접힘). 접힌 값은 항상 |θ| ≥ 360° − 190° = 170° 이므로:

- 읽은 값 |θ| < **165°** (여유 5°) → 실제 각도 확실 → **확정**
- |θ| ≥ 165° → **미확정** → 브리지가 enable 안 함 (`steer_state.ready = false`). 손으로 정면 쪽으로 돌려 |θ| < 165° 로.
  정말 접힌 상태면 정면 쪽으로 돌려도 값이 줄지 않고 커짐 → 반대로 돌릴 것.
- 한 번 확정되면 전원이 유지되는 동안 모터가 회전 수를 셈 (±720°). 피드백이 끊겼다 오면(전원 재투입일 수 있음) 다시 판정.

평소엔 꺼질 때 조향이 ±165° 안에 있으므로 신경 쓸 일 없음. 끄기 전에 FREE 로 손으로 크게 돌려 놓은 경우만 해당.

## 준비

```bash
sudo ip link set can_steer up type can bitrate 1000000   # 어댑터 꽂을 때/재부팅마다. 이름은 ../can/ udev 규칙으로 고정 (전부 한 번에: ../can/can_up.sh)
cd ~/capstone/capstone_ws/Steer_motor/steer_can     # python-can 필요 (시스템 python3 에 있음)
```

## 스크립트 (`steer_can/`)

| 파일 | 하는 일 | 모터 움직임 |
|---|---|---|
| `scan.py` | Type00 으로 버스의 CAN ID + UID 찾기 | 없음 |
| `angle.py` | 4축 조향각 + 확정 여부. 기본은 Type04 폴링(토크 끔), 브리지 돌 때는 `--listen` | 없음 (토크 끔) |
| `monitor.py` | Type02 / Type21 수신 출력 (송신 없음) — 모터 상태(위치·속도·토크·온도·mode·fault) | 없음 |
| `active_report.py` | Type24 active report 켜기/끄기 (`--enable 1/0`) — 켜면 명령 없이도 Type02 주기 송신 → `monitor.py` 로 상태 보기. RAM 설정, **다 보면 끌 것** | 없음 |
| `step.py` | **MIT 스텝 시험**: 현재각에서 목표각들로 램프 → 유지 → 토크 끔. 도착 시간·최종 오차·최대 토크, `--log` CSV | 있음 |
| `enable.py` | Type03 enable / `--enable 0` Type04 stop (`--clear_fault 1`) | enable 만으론 안 움직임 |
| `zero.py` | Type06 현재 위치를 0 으로 (**바퀴 정면에서만**) → `param_write_save.py` 로 저장은 따로 | 없음 |
| `set_can_id.py` | Type07 CAN ID 변경 (바꿀 모터만 연결) | 없음 |
| `param_read.py` / `param_write_save.py` | Type17 읽기 / Type18 쓰기 + Type22 저장 (`--list` 로 표) | 없음 |
| `tests/fake_rs02.py` | 가짜 RS02 ×4 (vcan) — 하드웨어 없이 브리지 시험 | — |

```bash
python3 scan.py --max 10                     # 1~4 가 보여야 함
python3 angle.py                             # 손으로 돌려 보며 부호 확인: 위에서 보고 반시계 = + 여야 함
python3 param_read.py --can_id 1 2 3 4 --param 0x7029   # zero_sta (1 이어야 함)
python3 step.py --can_id 1 --target 30 0 --rate 1.0     # 바퀴 띄우고 먼저
python3 step.py --can_id 1 --target 90 0 --log fr_kp20.csv   # 바닥 제자리 조향 → 최종 오차로 kp 결정
python3 -m unittest discover -s tests -t .   # 프레임 벡터 + 가짜 모터 테스트
```

## 실기 확인 TODO

- [ ] 부호: `angle.py` 로 손으로 돌려 위에서 보고 반시계 = + 인지 → 아니면 브리지 `steer_sign` 을 −1
- [ ] 켤 때 각도: 예) +90° 로 돌려 두고 전원 껐다 켜서 `angle.py` 가 +90° 인지 (1회전 절대 엔코더가 출력축 기준인지)
- [ ] 바닥 제자리 조향 `step.py --target 90 0` → 최종 오차 / 도착 시간 / 최대 토크로 `kp`, `kd`, `max_rate` 결정
- [ ] `canTimeout`(0x7028) 동작: 브리지 강제 종료(kill -9) 시 0.5 s 뒤 모터가 토크 끄는지
- [ ] Type21 고장 프레임 바이트 순서 (매뉴얼 미명시, 지금은 0/비0 판단에만 씀)
