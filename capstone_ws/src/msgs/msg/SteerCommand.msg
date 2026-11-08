# 조향 명령 (swerve_controller → 조향 브리지, RobStride 02 ×4). 순서 = [1 FR, 2 RR, 3 RL, 4 FL]
#
# FREE     : 토크 끔 — 손으로 돌릴 수 있음 (구동 FREE 와 같이 씀)
# POSITION : angle 로 위치 제어
uint8 MODE_FREE = 0
uint8 MODE_POSITION = 1

std_msgs/Header header
uint8 mode
float32[] angle   # [rad] 차체 기준, 0 = 바퀴 전방, + = 위에서 보고 반시계. 운용 한계 ±170° 안
