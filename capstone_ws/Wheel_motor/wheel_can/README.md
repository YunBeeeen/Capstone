# wheel_can — 구동 바퀴 CAN 도구 (XDrive = ODrive v3.6 호환, fw 0.5.1, CANSimple)

구조는 `idle_ws/motor` 와 같음: 기능별 스크립트 + `lib/`.
설정(pp, 게인, 캘리브레이션, CAN ID, baud)은 **USB 로 저장된 값**을 씀 — CAN 으로는 저장 불가(0.5.1).
명령어/배경: `../COMMANDS.md`.

위치: 도구 3단계 중 **2단계(시험·진단)** — 1 odrivetool/bringup.py(USB 설정) → **2 이 폴더** → 3 C++ ROS2 브리지
[`capstone_ws/src/wheel_can_bridge`](../../src/wheel_can_bridge/) (운용). 설명: [`../README.md`](../README.md#도구-3단계-odrivetool--python--c).
- `lib/cansimple.py` 가 프로토콜 기준 구현. [`tests/cansimple_vectors.csv`](tests/cansimple_vectors.csv) 를 C++ 테스트도 같이 읽음.
- 브리지가 ACTIVE 일 때 `vel`/`state`/`stop` 을 쓰면 브리지가 FAULT 로 전 축 정지 (의도된 동작). `monitor` 는 같이 써도 됨.
- 하드웨어 없이 브리지 시험: `python -m tests.fake_odrive --ch vcan0` (vcan 만드는 법은 파일 맨 위).

## 준비
```bash
source ~/odrive_env/bin/activate            # python-can 설치돼 있음
sudo ip link set can_drive up type can bitrate 500000   # 어댑터 꽂을 때/재부팅마다
cd ~/capstone/capstone_ws/Wheel_motor/wheel_can
```

## 스크립트
| 파일 | 하는 일 | 모터 움직임 |
|---|---|---|
| `monitor.py` | 상태·에러(heartbeat), 위치/속도, Iq, 버스전압 주기 표시 | 없음 (읽기 전용) |
| `hall_debug.py` | Hall 카운트/섹터, 극쌍수 측정. `--usb` 면 raw hall_state | 없음 |
| `state.py` | 축 상태 전환 (idle / closed_loop / motor_cal / offset_cal) + heartbeat 로 확인 | closed_loop·cal 이면 있음 |
| `vel.py` | 폐루프 진입 → 속도 유지(주기 재송신) → 0 → IDLE. Ctrl+C·에러 시 즉시 정지 | 있음 |
| `setvel.py` | 속도 명령만 1회 송신 (보드가 유지). CLOSED_LOOP 아니면 안 보냄. `state.py` 와 조합해 단계별로 | 있음 |
| `mode.py` | 제어 모드/입력 모드 변경 (RAM 만, 재부팅 시 복귀) | 없음 |
| `stop.py` | 속도 0 + IDLE (`--estop` 은 ESTOP) | 정지 |
| `clear.py` | 에러 클리어 | 없음 |

```bash
python monitor.py                         # Ctrl+C 종료
python monitor.py --can-id 1 --errors --log
python hall_debug.py --can-id 1             # 손으로 1바퀴 → Ctrl+C → Δ/6 = 극쌍수
python hall_debug.py --usb --axis 1       # USB 연결 + odrivetool 닫고
python state.py --can-id 1 --state closed_loop
python vel.py --can-id 1 --vel 0.5 --duration 10      # 5바퀴
python vel.py --can-id 1 2 --vel 0.5 -0.5

# 단계별 (진입 / 명령 / 정지 분리)
python state.py  --can-id 1 --state closed_loop
python setvel.py --can-id 1 --vel 0.5
python setvel.py --can-id 1 --vel 0
python state.py  --can-id 1 --state idle

python stop.py
```

`--can-id` 생략 시 `lib/config.py` 의 `CAN_IDS` 전체 (1~4: 보드 #1 = 1/2, 보드 #2 = 3/4).

프레임 ID = `CAN ID << 5 | cmd` → heartbeat(cmd 0x01): ID1 `021`, ID2 `041`, ID3 `061`, ID4 `081`.

## 비상정지
다른 터미널: `python stop.py` 또는 `cansend can_drive 027#01000000` (ID1 IDLE; ID2 `047`, ID3 `067`, ID4 `087`), 최종은 24V 차단.

## 피드백이 오는 방식 (fw 0.5.1)
- **heartbeat (상태·축 에러)**: 보드가 100 ms 마다 **자동 송신** (push)
- **위치/속도, Iq, 전압, motor/encoder 에러**: PC 가 **RTR 로 요청할 때만** 응답 (pull) — `monitor.py` 가 `--rate` 주기로 요청
- 속도 명령을 보낸다고 피드백이 오는 건 아님. 0.5.4+ 는 엔코더 주기 송신 설정이 있지만 0.5.1 은 없음

## candump
```bash
candump can_drive                                   # 전부
candump can_drive | grep -v -E " (021|041|061|081) "   # heartbeat 빼고 (요청/응답만)
candump -td can_drive,021:7FF                       # CAN ID 1 heartbeat 만 + 간격
```

## 테스트 (하드웨어 없이, 가상 CAN 위 가짜 XDrive)
```bash
python -m unittest discover -s tests -t . -v
```
`tests/test_cansimple.py` 는 2026-11-03 실기 candump 프레임으로 인코드/디코드 검증.

## 0.5.1 제약 (문서 + 실측)
- Get_* 은 RTR 요청→응답만 (주기 송신 없음, heartbeat 만 100 ms 자동) → 스크립트가 폴링
- 0x00F 는 속도 제한만, 게인 변경 메시지 없음, 설정 저장 불가
- 워치독이 어떤 CAN 메시지로 feed 되는지 미확인 → `vel.py` 는 미리 주기 재송신하도록 짜둠
- 에러 비트 이름은 0.5.x enum 기준 — 이상하면 hex 를 USB `dump_errors` 와 비교
