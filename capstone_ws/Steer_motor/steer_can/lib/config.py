"""조향 버스 기본 설정. 버스 배정: can_drive 구동 / can_steer 조향 / can_arm_r 오른팔 / can_arm_l 왼팔 (2026-11-07)."""

HOST_ID = 0xFD          # idle_ws 와 같은 호스트 ID (조향 버스엔 호스트가 Pi 하나뿐)
DEFAULT_CH = "can_steer"     # 조향 RS02 ×4, 1 Mbps

# 바퀴 번호 = 위에서 보고 앞오른쪽부터 시계방향. 구동(XDrive CAN ID)과 같은 번호
STEER_IDS = {1: "FR", 2: "RR", 3: "RL", 4: "FL"}

SCAN_MIN_ID = 1
SCAN_MAX_ID = 10
