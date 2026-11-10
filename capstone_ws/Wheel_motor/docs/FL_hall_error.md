# 4번(FL) 구동 모터 Hall 에러 — 증상·확인 기록

작성 2026-11-10. 아직 **미해결** (원인 후보: Hall 선이 레귤레이터 위를 지나감 + 5V 선 분리).

## 증상

- 바닥 주행 중 **구동만 갑자기 멈춤**. 조향(RobStride)은 계속 응답
- 구동 브리지 로그: `ACTIVE → FAULT: id4 axis_error 0x100`
- 보드 에러: `axis_error 0x100 ENCODER_FAILED` / `encoder_error 0x10 ILLEGAL_HALL_STATE` / **`motor_error 0x0`**
  - ILLEGAL_HALL_STATE = Hall 3비트가 나올 수 없는 조합(000 또는 111)으로 읽힘
  - 3상(모터 코일·굵은 선) 쪽 에러는 한 번도 없음 → **Hall 신호만 문제**
- 한 축 고장 = 전 축 정지 (브리지 규칙) → 4바퀴 모두 섬. 껐다 켜거나 `clear_errors` 해야 다시 움직임
- **바퀴를 띄우면 안 나고, 바닥에서 하중 받으며 주행할 때만** 남. 특히 회전이 섞인 호 주행·급하게 꺾을 때

| 대상 | 내용 |
|---|---|
| 모터 | 4번 FL = BL41105, XDrive 보드 #2 M1 (axis1), CAN ID 4 |
| 특이사항 | 4번은 처음부터 **내부 마찰 큼** (띄워서 Iq ≈ 다른 바퀴의 2배, 2026-11-05) |
| 배선 | 3상 3가닥 꼬음 / Hall 은 **5V 만 빼고** GND·A·B·C 4가닥 꼬음 (네 바퀴 모두 같은 방식) |
| 4번만 다른 점 | **Hall 선이 레귤레이터(DC-DC) 위를 지나감** (고무 피복으로 감쌈) |

## 경과 (2026-11-09 ~ 11-10)

1. 구동 배선 새로 한 뒤 4번 `ILLEGAL_HALL_STATE` 상시 발생 — `hall_state = 7` 고정 (센서 3개 모두 응답 없음)
2. Hall 커넥터에서 **빠져 있던 초록선, 5V 핀** 다시 꽂음 → `hall_state` 1~6 정상, 에러 사라짐
3. 이후 **주행 중 간헐 발생** (8 초 ~ 3 분 주행 뒤). 클리어하면 다시 됨

## 확인한 것

| 시험 | 하중 | 구동 전류 | 조향 모터 | 결과 |
|---|---|---|---|---|
| odrivetool `hall_state` / `config.mode` | — | — | — | 설정 정상 (HALL), 같은 보드 axis0(3번) 정상 |
| 띄우고 손으로 회전 (USB `hall_debug`) | 없음 | 없음 | 꺼짐 | 1~6 순서대로, 0/7 없음 |
| 띄우고 `vel.py` 1.0 turn/s × 30 s | 없음 | 0.6 A | 꺼짐 | 29.98 바퀴 정확, 에러 없음 |
| 띄우고 손으로 제동 | 없음 | **3 A** | 꺼짐 | 에러 없음 |
| 바닥에서 FL 누르며 손으로 밀기 100 s (12 바퀴+) | 세로 | 없음 | 꺼짐 | 에러 없음 |
| **바닥 주행** (가상 패드) | 세로 + 옆 | ≤ 1 A | 켜짐 | **에러** |

에러 직전 기록 (`ground_run3`, `ground_run4` — bag + monitor):

- 명령: 전진 0.15 + 횡 0.11 + 회전 ±0.5 → **호 주행** (두 번 모두)
- FL 조향각 고정 (−35°), FL 조향 토크 **0.5 Nm 이하** → 조향 모터 전류는 작음
- FL 구동 전류 0.2 ~ 1.0 A (제한 10 A 의 10 %) → **PI 포화 아님**
- FL 구동 속도가 명령(≈ 0.8 turn/s) 대비 **0.1 ↔ 0.8 로 출렁임** → 지면 속도 게인이 약함 (vel_gain 0.238, 띄운 상태 기준값). 일부는 저속 Hall 속도 추정 노이즈
- 배터리 28.2 V 안정, CAN 버스 에러 0 (ERROR-ACTIVE)

## 배제

- **코드·설정**: odrivetool 로 보드에 직접 붙어도 같은 값. 보드 설정 변경 없음
- **CAN 통신**: 버스 깨끗, 보드가 에러 코드를 정상 송신 (통신 문제면 heartbeat LOST 가 나야 함)
- **UDP / 패드**: 명령 경로일 뿐, 보드의 Hall 에러를 만들 수 없음
- **3상**: `motor_error 0x0` 유지
- **전류 크기 단독**: 띄워서 3 A 정상 (단, 기록이 10 ~ 20 Hz 라 ms 단위 튐은 못 봄)
- **세로 하중 단독**: 바닥에서 누르며 밀기 정상

## 원인 후보 (가능성 순)

1. **레귤레이터 노이즈 + 5V 분리 배선**
   - 레귤레이터는 수백 kHz 로 스위칭, 인덕터 주변 자기장이 강함. **피복은 노이즈를 못 막음**
   - 5V 가 따로 가면 5V–GND 고리가 커서 노이즈를 잘 받음 → 센서 3개가 **동시에** 출력을 놓침 → 111 = ILLEGAL
     (신호선 하나 노이즈면 보통 옆 상태로 튈 뿐 000/111 이 되기 어려움)
   - 주행 중 레귤레이터 부하가 커지면 노이즈도 커짐 → "주행할 때만" 과 맞음
   - **4번만 레귤레이터 위를 지남** → "4번만" 과 맞음
2. **4번 모터 자체** — 원래 내부 마찰 큼. 하중·옆 힘에서 회전자 치우침 / 내부 Hall·선 여유 부족
3. **옆 방향 하중에 의한 축 출구 케이블 눌림** — 호 주행에서만 났음
4. 공통 GND 경로 (조향 CAN GND 미연결) — 간접 영향, 우선순위 낮음

## 다음 조치

1. **4번 Hall 선을 레귤레이터에서 떼기** — 수 cm 이상, 인덕터 바로 위 피하기, 가로지르면 직각·짧게
2. **5V 와 GND 를 함께 꼬기** (가능하면 Hall 5가닥 전부) — 네 바퀴 모두
3. (선택) Hall 선 보드 쪽에 페라이트 코어, 조향 CAN GND 연결
4. 바닥 주행 재시험 — 동작별로: 직진만 / 횡이동만 / 제자리 회전 / **호 주행** (각 1 ~ 2 분, 기록 켜고)
5. 그래도 나면 **4번 모터를 다른 자리와 맞바꿔** 같은 주행 → 에러가 모터를 따라가면 모터 교체
   (3상 + Hall 을 모터와 함께 옮길 것. 보드 채널만 바꾸면 두 축 재캘리브레이션 + CAN ID 변경 필요)
6. 별도: 지면 속도 게인 튜닝 (USB, `vel_gain` 0.238 → 0.36 / 0.5 / 0.7, 적분 ≈ ×5) — 출렁임 감소

## 명령 모음

라파 (`~/capstone/capstone_ws/Wheel_motor/wheel_can`):

```bash
python3 monitor.py --can-id 4 --errors                      # 4번 상태·에러 (브리지 켜져 있어도 됨)
python3 monitor.py --errors --log --rate 10 | tee ground_runN.txt   # 주행 중 기록
python3 clear.py                                            # 보드 에러 클리어 (브리지 끄고)
python3 vel.py --can-id 4 --vel 0.3 --duration 20           # 4번만 돌리기 (브리지·컨트롤러 끄고)
ros2 service call /wheel_can_bridge/clear_errors std_srvs/srv/Trigger   # 브리지 FAULT + 보드 에러 해제
cd ~ && ros2 bag record -o ground_runN /cmd_vel /wheel_cmd /steer_cmd /wheel_state /steer_state
grep -n -B8 ENCODER ground_runN.txt | head -12               # 에러 직전
```

PC (USB, `source ~/odrive_env/bin/activate`, 같은 폴더):

```bash
python hall_debug.py --usb --axis 1         # 손으로 돌리며 hall_state 1~6 확인
python hall_log.py --axis 1 --out x.csv     # Hall·에러·속도·전류 CSV 기록 (주행 중 같이 켜도 됨)
odrivetool → odrv0.axis1.encoder.hall_state / dump_errors(odrv0) / odrv0.clear_errors()
```

## 기록 위치 (라파)

- `~/capstone/capstone_ws/Wheel_motor/wheel_can/ground_run3.txt`, `ground_run4.txt` (monitor), `push_test*.txt`
- `~/ground_run3/`, `~/ground_run4/` (ros2 bag)
- `~/.ros/log/wheel_can_bridge_node_*.log` (FAULT 시각)

## 참고: 다른 문제와 헷갈리지 말 것

- 2026-11-09 22:02, 22:17 **라파가 종료 기록 없이 재부팅** (전원 끊김) — 구동 정지와는 별개. 라파 5V 전원 확인 필요
- USB 의 Pico(리프트, MicroPython)가 13 초마다 재연결 반복 — 별개, 확인 필요
- 라파에 구동 브리지·컨트롤러가 켜진 채 `vel.py` 를 돌리면 명령이 덮어써져 안 돎 (heartbeat 끊김처럼 보임)
