# wheel_can_bridge 출력
uint8 PHASE_FREE = 0       # 전 축 IDLE (명령 대기)
uint8 PHASE_ENTERING = 1   # CLOSED_LOOP 진입 중 (속도 0)
uint8 PHASE_ACTIVE = 2     # 전 축 CLOSED_LOOP, 명령 반영 중
uint8 PHASE_FAULT = 3      # 고장 래치 — 전 축 IDLE, ~/clear_errors 서비스로 해제

std_msgs/Header header
uint8 phase
bool ready                 # 모든 바퀴 heartbeat 살아 있음
string fault               # PHASE_FAULT 원인 (그 외엔 빈 문자열)
float32 vbus               # [V] 첫 바퀴 보드 기준
WheelState[] wheels
