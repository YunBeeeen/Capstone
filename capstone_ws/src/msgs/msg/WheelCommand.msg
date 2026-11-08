# 구동 바퀴 명령 (wheel_can_bridge 입력). 바퀴 순서 = 브리지 파라미터 can_ids 순서 (기본 1,2,3,4)
#
# FREE  : IDLE — 전류 끊김, 바퀴 자유 구름
# DRIVE : CLOSED_LOOP + 속도 명령 (velocity 사용)
# HOLD  : CLOSED_LOOP + 속도 0 — 적분기가 위치를 붙잡음 (작업 중 정지)
uint8 MODE_FREE = 0
uint8 MODE_DRIVE = 1
uint8 MODE_HOLD = 2

std_msgs/Header header
uint8 mode
float32[] velocity   # [turn/s] 바퀴 기준, + = 트레드 V 꼭짓점 방향. DRIVE 에서만 사용, 길이 = 바퀴 수
                     # 순서 = 브리지 can_ids = [1 FR, 2 RR, 3 RL, 4 FL]
