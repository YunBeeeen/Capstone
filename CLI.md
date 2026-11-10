# CLI 모음

실행 위치를 먼저 확인할 것: **라파** (Pi 5, Ubuntu 24.04 / ROS 2 Jazzy, CAN 어댑터 연결) · **노트북/PC** (Ubuntu 22.04 / Humble).
상세 배경은 각 폴더 README, 4번 Hall 에러는 [`capstone_ws/Wheel_motor/docs/FL_hall_error.md`](capstone_ws/Wheel_motor/docs/FL_hall_error.md).

---

## 0. 처음 한 번 (라파)

```bash
# 터미널마다 자동 source
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
echo 'source ~/capstone/capstone_ws/install/setup.bash' >> ~/.bashrc

# CAN 어댑터 이름 고정 (설치 후 어댑터 다시 꽂기)
sudo cp ~/capstone/capstone_ws/can/80-capstone-can.rules /etc/udev/rules.d/
sudo udevadm control --reload

sudo apt install -y python3-can can-utils ros-jazzy-joy ros-jazzy-teleop-twist-joy
```

PC 는 ROS 2 Humble 기준 (`source /opt/ros/humble/setup.bash`). 구동 USB 도구는 `source ~/odrive_env/bin/activate`.

## 1. 코드 갱신 · 빌드

```bash
cd ~/capstone && git pull
cd ~/capstone/capstone_ws
source /opt/ros/jazzy/setup.bash          # PC 는 humble
colcon build                              # 한 패키지만: --packages-select <이름>
source install/setup.bash
colcon test && colcon test-result         # 테스트
```

---

## 2. 실기 주행 (바닥)

### 라파 (터미널마다 source 후)

| 터미널 | 명령 | 하는 일 |
|---|---|---|
| 0 | `~/capstone/capstone_ws/can/can_up.sh` | `can_drive` 500k / `can_steer` 1M up (꽂을 때·재부팅마다) |
| 1 | `ros2 launch steer_can_bridge steer_can_bridge.launch.py` | 조향 RobStride ×4 |
| 2 | `ros2 launch wheel_can_bridge wheel_can_bridge.launch.py` | 구동 XDrive ×2 |
| 3 | `ros2 launch swerve_controller swerve_controller.launch.py` | cmd_vel → 조향·구동 |
| 4 | `ros2 run capstone_teleop udp_teleop_bridge.py` | 노트북 패드(UDP) → cmd_vel |

### 노트북 (ROS 필요 없음)

```bash
cd ~/capstone/capstone_ws
python3 src/teleop/scripts/virtual_pad.py --udp raspberrypi.local --max-lin 0.15 --max-ang 0.5
```

- 마우스: 왼쪽 패드 = 병진, 오른쪽 패드 / Q·E = 회전, Space = 정지, Esc = 종료
- 노트북에 게임패드 꽂으면 같이 읽음: **RB 누른 채** 왼쪽 스틱 = 병진, 오른쪽 스틱 좌우 = 회전
- 창 아래 `robot: steer … | wheel …` 초록 = 연결됨, `no reply` = 라파 터미널 4 / 네트워크 확인
- `--no-gamepad` 마우스만, 주소 대신 IP(`172.20.10.2`)도 됨

### 게임패드를 라파에 직접 꽂을 때

라파 터미널 4 를 `ros2 launch capstone_teleop teleop.launch.py` 로 (UDP 노드 대신). 노트북 패드와 동시에 켜지 말 것.

### 주의

- 처음엔 **바퀴 띄우고** 방향 확인 → 바닥은 0.15 m/s 부터
- 리프트 내리고 팔 접고. 24 V 비상정지 손 닿는 곳에 (구동 워치독 아직 꺼짐)
- 실기 옆에서 시뮬 돌릴 땐 `export ROS_DOMAIN_ID=<다른 번호>`

---

## 3. 멈추기 · 해제

| 상황 | 명령 / 조작 |
|---|---|
| 정지 | 패드 Space / 게임패드 RB 놓기 |
| 전부 힘 빼기 (FREE) | `ros2 service call /swerve_controller/set_free std_srvs/srv/SetBool "{data: true}"` (해제 `false`) |
| 구동 자유 회전 | 라파 터미널 2 Ctrl+C |
| 조향 토크 끔 | 라파 터미널 1 Ctrl+C |
| 구동 FAULT 해제 | `ros2 service call /wheel_can_bridge/clear_errors std_srvs/srv/Trigger` |
| 조향 FAULT 해제 | `ros2 service call /steer_can_bridge/clear_errors std_srvs/srv/Trigger` |
| 최종 | 24 V 비상정지 |

---

## 4. 상태 보기 · 디버그 (라파)

```bash
ros2 topic echo --once /wheel_state | grep -E "^phase|^fault|^vbus"   # phase 0 FREE / 1 ENTERING / 2 ACTIVE / 3 FAULT
ros2 topic echo --once /steer_state | grep -E "^phase|^ready|^fault"
ros2 topic echo /cmd_vel                                              # 패드 명령 들어오나
ros2 topic echo --once /wheel_cmd | grep "^mode"                      # 0 FREE / 1 DRIVE / 2 HOLD

ip -br link | grep can
ip -details -statistics link show can_drive    # can state (ERROR-ACTIVE 정상), bus-errors
candump can_drive                              # 구동 heartbeat 021/041/061/081 이 100 ms 마다
candump can_steer

pgrep -af "bridge_node|swerve_node|udp_teleop|joy_node"   # 뭐가 떠 있나
```

### 주행 기록 (주행 전에 켜기, 이름은 매번 바꾸기)

```bash
cd ~/capstone/capstone_ws/Wheel_motor/wheel_can
python3 monitor.py --errors --log --rate 10 | tee ground_run5.txt
cd ~ && ros2 bag record -o ground_run5 /cmd_vel /wheel_cmd /steer_cmd /wheel_state /steer_state
# 끝난 뒤
grep -n -B8 ENCODER ~/capstone/capstone_ws/Wheel_motor/wheel_can/ground_run5.txt | head -12
```

---

## 5. 구동 (XDrive) 도구

### 라파 — CAN (`~/capstone/capstone_ws/Wheel_motor/wheel_can`)

`monitor` 는 브리지와 같이 써도 됨. **나머지는 구동 브리지·컨트롤러 끄고.**

```bash
python3 monitor.py                              # 4축 상태·위치·속도·Iq·vbus
python3 monitor.py --errors                     # + motor/encoder 에러
python3 monitor.py --can-id 4 --errors          # 4번만
python3 clear.py                                # 에러 클리어 (결과를 heartbeat 로 확인)
python3 vel.py --can-id 4 --vel 0.3 --duration 3             # 한 축
python3 vel.py --can-id 1 2 3 4 --vel 0.3 --duration 3       # 네 축 (−0.3 = 반대)
python3 stop.py                                 # 전 축 속도 0 + IDLE (비상)
python3 hall_debug.py --can-id 4                # 손으로 돌리며 Hall 카운트 (1 바퀴 = 60)
python3 state.py --can-id 1 --state closed_loop # 상태 전환 (idle / closed_loop / motor_cal / offset_cal)
```

### PC — USB (`source ~/odrive_env/bin/activate`, 보드 하나만 연결, odrivetool 닫고)

```bash
cd ~/capstone/capstone_ws/Wheel_motor
python bringup.py status                  # 설정 요약
python bringup.py check --board 2         # 기대값과 비교 (읽기만)
python bringup.py clear                   # 에러 클리어

cd wheel_can
python hall_debug.py --usb --axis 1       # raw hall_state (1~6 정상, 0/7 비정상), 손으로 돌리기
python hall_log.py --axis 1 --out x.csv   # Hall·에러·속도·전류 CSV (주행 중 같이 켜도 됨)

odrivetool
#   odrv0.axis1.encoder.hall_state / odrv0.axis1.encoder.error / dump_errors(odrv0)
#   start_liveplotter(lambda: [odrv0.axis1.encoder.hall_state])
#   odrv0.clear_errors()  /  odrv0.save_configuration()
```

보드 #1 = 1번 FR(axis0) · 2번 RR(axis1), 보드 #2 = 3번 RL(axis0) · 4번 FL(axis1).
펌웨어 업데이트·재플래시 금지 — 설정 변경·저장만.

---

## 6. 조향 (RobStride 02) 도구 — 라파 (`~/capstone/capstone_ws/Steer_motor/steer_can`)

```bash
python3 scan.py --max 10                       # 버스의 ID (1~4)
python3 angle.py                               # 4축 각도 + 확정 여부 (토크 끔으로 폴링)
python3 angle.py --listen                      # 브리지 돌 때 (송신 안 함)
python3 active_report.py --enable 1            # 명령 없이 상태 주기 송신 → monitor.py 로 보기 (다 보면 --enable 0)
python3 monitor.py                             # 위치·속도·토크·온도·mode·fault
python3 enable.py --can_id 1 2 3 4 --enable 0 --clear_fault 1   # 토크 끔 + 고장 클리어
python3 step.py --can_id 1 --target 30 0 --rate 1.0             # 바퀴 띄우고 스텝 시험
python3 zero.py --can_id 1 2 3 4               # 현재 위치 = 0 (바퀴 정면에서만)
python3 param_write_save.py --can_id 1 2 3 4 --param zero_sta --value 1 --verify   # 영점 저장
python3 param_read.py --can_id 1 2 3 4 --param 0x7029
```

조향 속도 바꿔 보기 (빌드 없이, 터미널 1 대신):

```bash
ros2 run steer_can_bridge steer_can_bridge_node --ros-args \
  --params-file ~/capstone/capstone_ws/install/steer_can_bridge/share/steer_can_bridge/config/steer_can_bridge.yaml \
  -p max_rate:=1.0
```

---

## 7. CAN 버스

```bash
~/capstone/capstone_ws/can/can_up.sh            # 꽂혀 있는 것만 up (bitrate·txqueuelen 100, 실제 값 표시)
~/capstone/capstone_ws/can/can_up.sh down
sudo ip link set can_steer down                 # 하나만
sudo ip link set can_steer up type can bitrate 1000000
udevadm info -q property -p /sys/class/net/can_drive | grep ID_SERIAL_SHORT   # 어댑터 시리얼
```

| 이름 | bitrate | 장치 |
|---|---|---|
| `can_drive` | 500k | XDrive ×2 |
| `can_steer` | 1M | RobStride 02 ×4 |
| `can_arm_r` / `can_arm_l` | 1M | 팔 |

---

## 8. 시뮬 (PC, 하드웨어 없이)

```bash
ros2 launch capstone_sim sim.launch.py                  # MuJoCo + swerve_controller
ros2 run capstone_teleop virtual_pad.py                 # 다른 터미널 (ROS 모드)
ros2 launch capstone_sim sim.launch.py rviz:=true       # + RViz
ros2 launch capstone_sim sim.launch.py teleop:=true     # + 게임패드
ros2 launch capstone_sim sim.launch.py headless:=true rviz:=true
python3 src/sim/scripts/build_mjcf.py                   # URDF 바뀌면 MJCF 재생성 (capstone_ws 에서)
python3 src/sim/scripts/set_urdf_inertia.py             # 부품 무게표 → URDF <inertial>
python3 -m mujoco.viewer --mjcf=src/description/mjcf/mobile_robot_right_arm.xml   # ROS 없이 보기
```

---

## 9. 라파 접속

```bash
ssh raspberrypi                    # = pi@raspberrypi.local (안 되면 IP: hostname -I 로 확인)
iw dev wlan0 link | grep SSID      # 연결된 와이파이 (sysysy / ybyb777 / J 등록됨)
```

---

## 10. 주행 실행 요약 — 라파 4개 + PC 1개 (복사해서 쓰기)

라파는 터미널 5개를 열어 각각 붙여넣기 (`.bashrc` 에 source 넣었으면 첫 줄 생략 가능). 순서대로 켜기.

**라파 터미널 0 — CAN (한 번)**
```bash
~/capstone/capstone_ws/can/can_up.sh
```

**라파 터미널 1 — 조향 브리지**
```bash
source /opt/ros/jazzy/setup.bash && source ~/capstone/capstone_ws/install/setup.bash
ros2 launch steer_can_bridge steer_can_bridge.launch.py
```

**라파 터미널 2 — 구동 브리지**
```bash
source /opt/ros/jazzy/setup.bash && source ~/capstone/capstone_ws/install/setup.bash
ros2 launch wheel_can_bridge wheel_can_bridge.launch.py
```

**라파 터미널 3 — 스워브 컨트롤러**
```bash
source /opt/ros/jazzy/setup.bash && source ~/capstone/capstone_ws/install/setup.bash
ros2 launch swerve_controller swerve_controller.launch.py
```

**라파 터미널 4 — UDP 텔레옵 수신**
```bash
source /opt/ros/jazzy/setup.bash && source ~/capstone/capstone_ws/install/setup.bash
ros2 run capstone_teleop udp_teleop_bridge.py
```

**PC — 가상 패드 + 게임패드 (ROS source 불필요)**
```bash
cd ~/capstone/capstone_ws
python3 src/teleop/scripts/virtual_pad.py --udp raspberrypi.local --max-lin 0.15 --max-ang 0.5
```

확인: 터미널 1 "전 축 응답 + 각도 확정", 터미널 2 "모든 바퀴 heartbeat 확인", PC 창 아래 `robot: steer … | wheel …` 초록.
끄기: PC Esc → 라파 4 → 3 → 2 → 1 순서로 Ctrl+C.
