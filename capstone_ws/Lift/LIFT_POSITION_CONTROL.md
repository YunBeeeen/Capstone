# 리프트 원점 기준 위치 제어 · 추가 상승량 0–650 mm

노트북 `/home/parkshinyoung/yorr`에서 준비한 별도 펌웨어/조작 프로그램이다.
라파 복사, Pico 업로드, 모터 동작 시험은 아직 하지 않았다. 기존 수동 조그,
공식 원본, 공식 기반 PWM 수정본은 별도 파일로 유지한다.

- Pico: `firmware/pico_lift_position/main.py`
- 라파/노트북 Tk 조작 창: `scripts/lift_position.py`와 `scripts/lift_joystick.py`
- 전용 JSON Lines protocol: `yor-lift-position-v1`

## 가능한 동작과 끝단 표시

원점에서 **추가로 올라간 상판 높이**가 0–650 mm다. 바닥에서의 높이나
각 단별 650 mm가 아니다. 원점은 최초 한 번 등록하고, 예를 들어 현재 200 mm에서
목표 300 mm를 주면 +100 mm 이동한다. 다시 200 mm를 주면 -100 mm 이동한다.
도착은 위치 오차 1 mm 이내이고, 추정 속도 2 mm/s 이하를 200 ms 유지했을 때다.
이 값들은 제어기의 초기 판정값이며 실제 정확도를 보장하는 측정값이 아니다.

사용자가 상·하단 센서 유무를 `없음 / 모름`으로 답해 **물리 리미트 센서가 없는
구성**으로 구현했다. 자동으로 하단에 부딪혀 원점을 잡는 동작은 제공하지 않는다.

| 표시/상태 | 의미 |
|---|---|
| 원점 미등록 | 실제 높이를 모름. 수동 조그만 가능하고 위치 제한도 적용할 수 없음 |
| 계산상 하한 / `at_min` | 엔코더 추정 높이 ≤ 1 mm. 아래 방향 조그 차단 |
| 계산상 상한 / `at_max` | 엔코더 추정 높이 ≥ 649 mm. 위 방향 조그 차단 |
| `target_reached` | 목표 허용오차 및 정착 조건을 만족 |
| `encoder_no_pulse` | 최초/운전 중 800 ms 무펄스. 걸림·센서 유실·실제 끝단 중 무엇인지 확정하지 않음 |
| `encoder_direction_mismatch` | 명령과 반대 방향으로 측정됨. 원점 무효화 |
| `position_out_of_range` | 0–650 mm 범위를 허용오차 이상 벗어남. 원점 무효화 |

`at_min`/`at_max`는 센서가 직접 감지한 끝단이 아니다. 항상
`limit_source: "encoder_soft"`, `physical_limits_available: false`를 함께 보낸다.
목표는 0 이상 650 이하만 허용한다. 정지 관성과 엔코더 보정 오차가 있으므로 실제
행정 끝단까지의 여유·정지 거리·상판 움직임을 실측하기 전에는 중앙 구간에서 시험한다.
실제 끝단 확정이나 독립적인 이동 차단이 필요하면 별도 상·하단 센서가 필요하다.

## 엔코더 보정과 원점 등록

기존 YOR GPIO 배치를 사용한다: GP2 RPWM, GP3 LPWM, GP4 R_EN, GP5 L_EN,
GP27 encoder A, **GP28 encoder B**. GPIO 번호이며 물리 헤더 번호가 아니다.
원래 수동 코드의 A 활동 펄스와 달리, A 상승 에지에서 B를 읽어 x1 방향 카운트를
얻으며 출력이 정지한 동안의 움직임도 센다. A/B가 실제 quadrature인지, 입력 전압이
Pico에 맞는지 실물 확인이 먼저다. B 배선 유무만으로 확인된 것으로 간주하지 않는다.

1. 모터 전력을 분리하고 보드·핀·배선 및 A/B 위상을 확인한다.
2. 짧은 저속 조그로 실제 상승/하강 방향과 창의 `A/B 누적 카운트`를 확인한다.
   보정과 원점 등록 전에는 끝단 위치를 알 수 없으므로 끝단을 피한다.
3. 실제 **상판** 이동거리와 카운트 차이를 측정한다.
   `mm/count = 실제 상판 이동거리(mm) / abs(카운트 차이)`다.
   상승 시 카운트가 늘면 부호 `1`, 줄면 `-1`이다. 하강 때 반대 부호인지도 확인한다.
   원본의 `0.20018`이나 모델의 두 단 비율을 실측 없이 복사하지 않는다.
4. 조그를 멈추고 비활성화한 뒤 실측 mm/count와 부호를 입력해
   `A/B 위상 확인 및 실측값 적용`을 누른다. mm/count는 (0, 1]을 지원한다.
   다른 분해능의 센서라면 허용 범위와 1 mm 판정값부터 검토한다.
5. 실제 최하단 기준으로 수동 위치시킨 뒤 정지·비활성화한다.
   `최하단을 직접 확인한 현재 위치 = 0 mm`를 누른다. 마지막 엔코더 펄스 이후
   300 ms 이상 정지하고 보정된 상태여야 한다. 임의의 중간 위치를 0으로 잡으면
   650 mm 상한의 물리적 의미가 틀려진다.
6. 다시 활성화하고, 중앙 구간의 작은 목표부터 입력한다.

기본 `MM_PER_COUNT=None`, `ENCODER_AB_VERIFIED=False`이므로 실측 확인 전
위치 이동은 거부한다. 런타임 보정값과 원점은 재부팅 후 복구하지 않는다.
실측 확정 뒤 필요하면 펌웨어 상단 `MM_PER_COUNT`, `ENCODER_SIGN`,
`ENCODER_AB_VERIFIED`를 설정할 수 있지만, **원점은 부팅마다 새로 등록**해야 한다.
걸림/무펄스·방향 불일치·범위 이탈로 원점이 무효화되면 원인 해결 후 다시 등록한다.
통신 재연결은 자동 재출발하지 않는다.

## 나중에 라파/Pico에 적용

라파가 다시 켜졌을 때 노트북에서 필요한 파일을 복사한다. 이 명령은 지금 실행하지 않았다.

```bash
ssh pi@raspberrypi.local 'mkdir -p ~/lift/position/firmware'
scp scripts/lift_position.py scripts/lift_joystick.py pi@raspberrypi.local:~/lift/position/
scp firmware/pico_lift_position/main.py pi@raspberrypi.local:~/lift/position/firmware/main.py
scp docs/LIFT_POSITION_CONTROL.md pi@raspberrypi.local:~/lift/position/
```

모터 전력은 분리하고, 기존 조그 창·mpremote·Thonny를 닫는다. 라파의 실제 Pico
포트를 `/dev/serial/by-id/`에서 확인한다. 아래 포트는 예시다. 기존 Pico `main.py`를
백업한 뒤 새 펌웨어를 올린다. 라파에는 Python 3, Tk, pyserial, mpremote가 필요하다.

```bash
ssh -X pi@raspberrypi.local
cd ~/lift/position
mpremote connect /dev/ttyACM0 fs cp :main.py ./main.before-position.py
mpremote connect /dev/ttyACM0 fs cp firmware/main.py :main.py
mpremote connect /dev/ttyACM0 reset
python3 lift_position.py --port /dev/ttyACM0
```

이 프로토콜은 `lift_yor_pwm.py` 및 기존 수동 조그 펌웨어와 호환되지 않는다.
잘못된 버전의 상태는 조작 창이 무시하며 활성화하지 않는다.

노트북에서 창만 확인:

```bash
python3 scripts/lift_position.py --demo
```

## 제어/통신 동작

- 목표 위치 제어·PWM ramp·소프트 리미트는 Pico에서 처리한다. 호스트는 목표를
  한 번 보내고 50 ms마다 heartbeat를 보낸다. 목표 명령 반복은 타이머를 갱신하지 않는다.
- 기본 목표 출력 상한 25%, 조작 창에서 10–35%. 목표 근처 P 감속과 출력 상승 ramp를
  사용한다. 이는 실기 튜닝 전 시작값이며 PD/PID 조정 완료 상태가 아니다.
- 위치 오차 안으로 들어오면 출력 0으로 정착을 확인한다. 큰 목표 통과나 정지 후
  오차 이탈은 성공으로 처리하지 않고 오류로 정지한다. 자동 왕복 보정은 하지 않는다.
- heartbeat 300 ms 만료, 최초/운전 중 엔코더 A 무펄스 800 ms, 한 동작 120초 초과는 정지한다.
  반복 조그/방향 반전은 활성 동작의 무펄스·동작시간 타이머를 초기화하지 않는다.
- Esc/정지 버튼/포커스 상실/창 닫기는 비활성화한다. 600 ms 동안 유효한 응답이 없으면
  연결을 닫고 표시된 높이를 지운다. 재연결/오류 해제 후 명시적 활성화가 필요하다.
- 정지는 RPWM/LPWM 0, EN 유지다. 실제 하중 유지·제동과 정지 거리는 미검증이다.
  물리 비상정지/하중 지지/끝단 보호를 소프트웨어만으로 대체할 수 없다.

프로토콜 예시(실측 보정 및 최하단 확인 이후에만 사용):

```json
{"cmd":"calibrate","mm_per_count":0.1,"encoder_sign":1,"ab_verified":true}
{"cmd":"zero","at_lower_reference":true}
{"cmd":"arm"}
{"cmd":"move_to","target_mm":300,"max_duty":0.25}
{"cmd":"heartbeat"}
{"cmd":"disarm"}
```

위 `0.1`은 프로토콜 설명용 값이며 실물에 사용하라는 보정값이 아니다.

## 검증

2026-11-08 노트북에서 새 위치 제어 18개, 기존 수동 조그 12개, 기존 공식 기반
PWM 7개로 **총 37개 테스트를 통과**했다. Tk 창도 열어 전체 조작부가 잘리지
않는지 확인했다. 라파/Pico에 배포하거나 실물 모터를 움직인 결과는 아니다.

```bash
python3 -m unittest discover -s scripts/tests -p 'test_lift_position.py' -v
python3 -m unittest discover -s scripts/tests -p 'test_lift_jog.py' -v
python3 -m unittest discover -s scripts/tests -p 'test_yor_adjustable.py' -v
```

합성 엔코더·이상적인 모터 모델로 200→300→200 mm, 상하한, 범위 밖 명령,
미등록 원점, 역방향 엔코더, 걸림, 초과 이동, 정착 실패, 통신 만료·카운트/시간 wrap을
검사한다. PTY로 실제 GUI의 명령/heartbeat·보정 확인·재부팅·유실 상태를 검사한다.
이 검증은 Pico의 ISR 타이밍·USB 처리·물리 정확도·상하단·브레이크를 검증하지 않는다.
