# 리프트 상태 (lift_bridge 출력, 10 Hz)
std_msgs/Header header
bool connected        # Pico 시리얼 응답이 0.6 s 안에 있음
string pico_state     # UNHOMED / IDLE / JOG / MOVING / SETTLING / FAULT
bool armed
bool homed            # 원점 등록됨 → height_mm 유효, 소프트 리미트(10..620 mm) 동작
float32 height_mm     # 최하단 기준 높이 [mm]. 원점 미등록이면 NaN
float32 rel_mm        # 브리지 켠 뒤 움직인 거리 [mm] (원점 없어도 엔코더로 셈)
float32 velocity_mm_s
string task           # "" / PARK_MOVE / PARK_DISARM / PARK_FORGET / PARK_DESCEND / PARK_SETTLE / ZERO
bool parked           # 주차 완료 후 안 움직임 → 다음 부팅 때 자동 원점 등록
string fault          # Pico fault (예: command_timeout, encoder_no_pulse)
string event          # Pico 마지막 event
string message        # 사람용 한 줄
