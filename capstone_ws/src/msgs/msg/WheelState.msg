# 바퀴 1개 상태 (부호는 WheelCommand 와 같은 바퀴 기준)
uint8 can_id
bool alive             # heartbeat 가 hb_timeout 안에 들어왔는지
uint32 axis_state      # ODrive AxisState (1 IDLE, 8 CLOSED_LOOP_CONTROL ...)
uint32 axis_error      # ODrive AxisError 비트 (0 = OK, 0x800 = WATCHDOG_TIMER_EXPIRED ...)
float32 position       # [turn]
float32 velocity       # [turn/s]
float32 iq_setpoint    # [A]
float32 iq_measured    # [A]
