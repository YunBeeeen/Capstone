# 리프트 명령 (lift_bridge 입력). 조작 중엔 20 Hz 이상으로 계속 보낼 것 — cmd_timeout(0.3 s) 넘게 끊기면 조그 정지.
# 버튼류는 "누르고 있는 동안 true" 로 보내면 됨 (브리지가 눌린 순간·누른 시간을 판단).
std_msgs/Header header
int8 jog        # +1 위 / 0 정지 / -1 아래 (누르는 동안)
float32 duty    # 조그 PWM 비율 0..1 (패드: 느림 0.5 / 빠름 1.0). 0 = 브리지 기본값 (jog_duty).
                # 브리지가 0.3 부터 올림 (램프), 원점 미등록이면 unhomed_max_duty(0.5) 까지만
bool stop       # 정지 + 진행 중 작업(주차·원점 등록) 취소
bool park       # 눌린 순간: 주차 = 최하단(0 mm)까지 내려가서 멈춤 (원점 등록돼 있어야 함)
bool zero       # 1 s 넘게 누르고 있으면: 지금 위치 = 0 mm 로 등록 (최하단에 있을 때만!)
bool clear      # 눌린 순간: Pico 오류 해제
