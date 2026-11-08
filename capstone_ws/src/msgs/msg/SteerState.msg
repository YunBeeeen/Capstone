# 조향 모터 1개 상태. 각도는 SteerCommand 와 같은 조향 기준 (0 = 바퀴 전방, + = 위에서 보고 반시계)
uint8 can_id
bool alive             # Type02 피드백이 fb_timeout 안에 들어왔는지
bool confirmed         # 켤 때 각도 확정: |θ| < confirm_limit 를 본 적 있음. 피드백 끊기면 다시 false (전원 재투입일 수 있음)
uint8 mode_status      # RS02 Type02: 0 reset(토크 끔) / 1 cali / 2 run
uint8 fault_bits       # RS02 Type02 bit21~16 (0 = OK): 0 저전압 1 과전류 2 과열 3 엔코더 4 과부하(stall) 5 미캘리브레이션
float32 angle          # [rad] 실제 조향각
float32 velocity       # [rad/s]
float32 torque         # [Nm]
float32 temperature    # [°C]
float32 angle_cmd      # [rad] 지금 보내는 램프 위치 (ENTERING/ACTIVE 에서만 의미)
