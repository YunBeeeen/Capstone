# CAN 버스 구성 — 이름 고정 + can_up.sh 로 올리기

USB-CAN(candleLight) 어댑터 4개를 **어댑터 시리얼로 역할 이름에 고정**하고(udev), bitrate 설정·up 은 **`can_up.sh` 로 직접** 한다.
꽂는 순간 자동 up 은 하지 않음 (2026-11-08 결정 — udev RUN 으로 해 보니 어댑터 초기화 전에 실행돼 실패하기도 했음).

## 버스 배정

| 이름 | 버스 | bitrate | 장치 | 프레임 | 어댑터 시리얼 |
|---|---|---|---|---|---|
| `can_drive` | 구동 | **500 kbps** | XDrive ×2 → BL41105 ×4 | 표준 11-bit (ODrive CANSimple) | `001D00335053530720373232` |
| `can_steer` | 조향 | **1 Mbps** | RobStride 02 ×4 | 확장 29-bit (RS02 private, MIT) | `003200273830511420393658` |
| `can_arm_r` | 오른팔 | 1 Mbps | B601-DM (DM 모터) | | `003E002A5353510C20353934` |
| `can_arm_l` | 왼팔 | 1 Mbps | B601-DM (DM 모터) | | `004200495056511620363138` |

예전 이름: can0 구동 / can1 조향 / can2 오른팔 / can3 왼팔 (배정 순서 같음, 2026-11-07 역할 이름으로 변경).
코드 기본값: `wheel_can_bridge`·`wheel_can` = `can_drive`, `steer_can_bridge`·`steer_can` = `can_steer`.

## 왜 이름을 고정하나
커널은 **먼저 인식된 어댑터에 `can0`** 을 붙인다 → 꽂는 순서·부팅마다 구동/조향 버스가 뒤바뀔 수 있고, bitrate 가 달라서
둘 다 통신이 안 됨. 그래서 어댑터 고유 시리얼로 이름을 정한다.
숫자 이름(`can0..3`)으로 고정하지 않는 이유: 커널 자동 이름과 겹치면 rename 이 실패("File exists") → 순서에 따라 될 때/안 될 때가 생김.

## udev 란
리눅스에 원래 들어 있는 장치 관리자 (`systemd-udevd`, 항상 실행 중). USB 장치가 꽂히면 `/etc/udev/rules.d/` 의 규칙대로
이름·권한·명령을 처리한다 (이 PC 엔 이미 ODrive·ST-Link 규칙 등이 있음).
**"설치" = 규칙 파일 하나를 그 폴더에 복사**. 어댑터를 꽂는 **컴퓨터마다** 1회 (개발 PC, 라즈베리파이 각각).
[`80-capstone-can.rules`](80-capstone-can.rules): 시리얼이 ○○ 인 CAN 어댑터 → 이름 `can_○○` (이름만).

## 1회 설치
```bash
cd ~/capstone/capstone_ws/can
sudo cp 80-capstone-can.rules /etc/udev/rules.d/
sudo udevadm control --reload
# 어댑터 전부 뽑았다 다시 꽂기 (또는 재부팅)
ip -br link | grep can                              # can_drive / can_steer / can_arm_r / can_arm_l (아직 DOWN)
```
되돌리기: `sudo rm /etc/udev/rules.d/80-capstone-can.rules && sudo udevadm control --reload`

### 라즈베리파이 (Pi5) 에서
같은 어댑터면 시리얼이 같으니 **같은 규칙 파일**을 같은 방법으로 설치. 처음 한 번 확인:
- 드라이버: `lsmod | grep gs_usb` (없으면 `sudo modprobe gs_usb`)

## 실물 라벨
어댑터 겉모양이 같으니 **하나씩 꽂아서** 이름을 보고 라벨을 붙인다:
```bash
ip -br link | grep can          # 방금 꽂은 것 하나만 보임 → 그 이름을 어댑터에 적기
```

## 버스 올리기 (꽂을 때 / 재부팅마다)
```bash
./can_up.sh          # 꽂혀 있는 것만 bitrate·restart-ms 100·txqueuelen 100 으로 올림
./can_up.sh down
```
```bash
ip -br link | grep can                                       # UP 확인
ip -details link show can_steer | grep -o 'bitrate [0-9]*'   # bitrate 확인
```
- `can0` 같은 숫자 이름이 남아 있으면 스크립트가 경고 → 규칙 미설치이거나 모르는 어댑터

## bitrate 를 이렇게 정한 이유
**PC 쪽** bitrate 는 어댑터에 저장되지 않음 → 인터페이스를 올릴 때마다 설정 (`can_up.sh`). bitrate 는 인터페이스가 DOWN 일 때만 바꿀 수 있음.
**장치 쪽**은 장치에 저장 (XDrive: USB `odrv0.can.set_baud_rate(500000)` + 저장 / RobStride·DM: 모터에 저장된 기본 1M). 양쪽이 다르면 통신 안 됨.

버스 점유율 (표준 프레임 ≈ 130 bit, 확장 ≈ 150 bit):

| 버스 | 트래픽 | 500k | 1M |
|---|---|---|---|
| 구동 | 속도 50 Hz ×4 + heartbeat 10 Hz ×4 + 피드백 RTR 20 Hz ×9 ×2 ≈ **600 프레임/s ≈ 78 kbit/s** | **≈16% ✓** | ≈8% |
| 조향 | MIT 200 Hz ×4 + 응답 ≈ **1600 프레임/s ≈ 240 kbit/s** | ≈48% ✗ | **≈24% ✓** |

권장 점유율 30~40% 이하. **구동은 500k 로 충분** — 비트 시간이 1M 의 2배라 스텁(분기선)·장치 간 샘플 시점 차이·노이즈에 여유가 큼.
**조향은 1M 필요** (200 Hz 위치 제어). 버스가 따로라 서로 맞출 필요 없음.

참고 — bitrate 별 비트 시간 / 일반적 최대 버스 길이: 250k 4 µs / ~250 m, 500k 2 µs / ~100 m, 1M 1 µs / ~25–40 m.
우리 버스는 1 m 미만이라 길이는 문제 아님 (1M 도 가능 — 500k 는 "필요 없어서 안 올린 것").
1M 은 스텁 30 cm 이하 권장 — 구동 버스 스텁: 보드 #1 3–5 cm, 보드 #2 약 20 cm, 종단 60 Ω.

## 상태 확인
```bash
ip -br link | grep can
ip -details -statistics link show can_drive    # state ERROR-ACTIVE 가 정상, bus-off/에러 카운트 확인
candump can_drive                              # XDrive heartbeat 021/041/061/081 이 100 ms 마다
candump can_steer
```

## 어댑터 바꿀 때
```bash
udevadm info -q property -p /sys/class/net/<이름> | grep ID_SERIAL_SHORT
```
→ `80-capstone-can.rules` 의 시리얼 수정 → 다시 설치. 시리얼이 같은 어댑터끼리면 시리얼 대신 USB 포트 위치(`ID_PATH`)로 고정
(그 경우 꽂는 포트를 바꾸면 안 됨).

## 메모
- 인터페이스 이름 최대 15자
- 나중에 원하면 부팅 시 자동 up 은 systemd 서비스로 (udev RUN 은 쓰지 말 것)
- 조향 램프: `steer_can_bridge` `max_rate` 2 rad/s(≈115°/s) 가 swerve 의 `max_steer_rate_deg_s` 360 보다 느려서 실기 180° 반전 ≈ 1.6 s — 바닥 조향 시험 후 맞출 것
