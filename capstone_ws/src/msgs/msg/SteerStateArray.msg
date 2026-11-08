# 조향 상태 (steer_can_bridge → swerve_controller 등). 순서 = [1 FR, 2 RR, 3 RL, 4 FL]
uint8 PHASE_FREE = 0       # 전 축 토크 끔 (Type04 로 위치만 읽음)
uint8 PHASE_ENTERING = 1   # enable 중 — RUN 된 축은 현재각 유지
uint8 PHASE_ACTIVE = 2     # 전 축 MIT 위치 제어, 명령 반영 중
uint8 PHASE_FAULT = 3      # 고장 래치 — 전 축 토크 끔, ~/clear_errors 서비스로 해제

std_msgs/Header header
uint8 phase
bool ready        # 전 축 통신 OK + 각도 확정 + 고장 없음 (켤 때 |θ| ≥ 165° 면 false — 손으로 돌려 확정 필요).
                  # 위치 제어 중일 필요는 없음: swerve 가 ACTIVE 로 들어가는 순간(브리지는 아직 FREE) 실제각으로 seed 하려면 true 여야 함
string fault      # PHASE_FAULT 원인 (그 외엔 빈 문자열)
float32[] angle   # [rad] 실제 조향각, SteerCommand.angle 과 같은 규약 (= motors[i].angle)
SteerState[] motors
