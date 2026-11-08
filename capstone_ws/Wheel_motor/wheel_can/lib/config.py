"""기본 CAN 채널 / CAN ID / 모터 상수 (XDrive V3.6, fw 0.5.1, BL41105)."""

DEFAULT_CH = "can_drive"
BITRATE = 500000          # 보드: odrv0.can.set_baud_rate(500000) 로 저장된 값과 같아야 함

# CAN ID (= 보드의 axis.config.can_node_id) = 바퀴 번호 1~4. 출력 단자(M0/M1)에 붙는 번호 — 모터가 아님
# 바퀴 위치: 위에서 보고 앞오른쪽부터 시계방향 1 FR → 2 RR → 3 RL → 4 FL (2026-11-07)
#   → 보드 #1 = 오른쪽 두 바퀴, 보드 #2 = 왼쪽 두 바퀴. 조향 RobStride 02 도 같은 번호.
CAN_IDS = {
    1: "FR  M0 (XDrive#1 axis0)",
    2: "RR  M1 (XDrive#1 axis1)",
    3: "RL  M0 (XDrive#2 axis0)",
    4: "FL  M1 (XDrive#2 axis1)",
}

POLE_PAIRS = 10
CPR = 6 * POLE_PAIRS      # Hall: 전기각 1주기당 6카운트

VEL_MAX = 2.0             # turn/s, 보드 controller.config.vel_limit 과 맞춤 (스크립트 측 이중 안전장치)
VEL_RAMP_RATE = 2.0       # turn/s^2, 보드 vel_ramp_rate (정지 대기시간 계산용)

HEARTBEAT_TIMEOUT = 0.5   # s, 보드 heartbeat 주기 100 ms 의 5배

# 배터리: 볼턴 24V 30Ah 리튬이온 (LG 셀) = 7S (충전기 29.4 V 로 확인, 2026-11-05)
#   완충 29.4 V (4.2/셀), 공칭 25.9 V. 보드 저전압 트립은 21.0 V (3.0/셀, USB 설정) — 그 전에 화면 경고
VBUS_FULL = 29.4
VBUS_WARN = 22.5          # 3.2 V/셀 — "충전 필요" 경고
VBUS_TRIP = 21.0          # 보드 dc_bus_undervoltage_trip_level 목표값
