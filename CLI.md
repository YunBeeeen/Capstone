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
source /opt/ros/jazzy/setup.bash && source ~/capstone/capstone_ws/install/setup.bash
ros2 run lift_bridge lift_bridge_node.py

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

## 10. 주행 실행 요약 — 라파 6개 + PC 1개 (복사해서 쓰기)

라파는 터미널 6개를 열어 각각 붙여넣기 (`.bashrc` 에 source 넣었으면 첫 줄 생략 가능). 순서대로 켜기.

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

**라파 터미널 5 — 리프트 (Pico)** — `launch` 말고 `run` (종료 때 최하단까지 내리느라 수십 초 걸릴 수 있음)
```bash
source /opt/ros/jazzy/setup.bash && source ~/capstone/capstone_ws/install/setup.bash
ros2 run lift_bridge lift_bridge_node.py
```
- 동료 GUI (`~/start_position.sh`, `lift_position.py`), mpremote, Thonny 는 닫기 — Pico 포트는 한 프로그램만
- 켤 때: 지난번에 이 노드로 주차하고 껐으면 **자동 원점 등록**. 아니면 "원점 미등록" → 조그로 최하단까지 내린 뒤 Z 1 초
- 끌 때 (Ctrl+C): **최하단(0 mm)까지 내려 놓고 꺼짐**. 내려가는 중 Ctrl+C 한 번 더 = 즉시 정지 (다음엔 수동 원점)
- 주차 기록: `~/.local/state/capstone_lift/lift_state.json`

**PC — 가상 패드 + 게임패드 (ROS source 불필요)**
```bash
cd ~/capstone/capstone_ws
python3 src/teleop/scripts/virtual_pad.py --udp raspberrypi.local --max-lin 0.15 --max-ang 0.5
```

리프트 조작 (패드 창 / 게임패드는 RB 누른 채)

| 동작 | 키보드 | 게임패드 |
|---|---|---|
| 빠른 위 / 아래 (PWM 100 %, 누르는 동안) | Shift+R / Shift+F | Y / A |
| 느린 위 / 아래 (PWM 50 %) | R / F | X / B |
| 주차 (최하단까지) | P | Start |
| 지금 위치 = 0 mm (최하단일 때만!) | Z 1 초 | Select 1 초 |
| Pico 오류 해제 | C | LB |
| 정지 · 주차 취소 | Space | — |

- 빠른 조그는 0.3 → 1.0 을 0.35 s 램프로 올림. **원점 미등록이면 50 % 까지만** (끝단 보호 없음)
- PWM 바꾸기: PC `--lift-fast 80 --lift-slow 40`

주행 속도 상한 (게임패드 십자키, RB 안 눌러도 됨, 누르고 있으면 반복 — xy 축처럼 위·오른쪽 = +)

| 십자 | 바뀌는 것 | 범위 |
|---|---|---|
| 위 / 아래 | 선속도 상한 ± 0.05 m/s | 0.05 .. 0.6 |
| 오른쪽 / 왼쪽 | 각속도 상한 ± 0.1 rad/s | 0.1 .. 2.0 |

- 시작값 = PC 명령의 `--max-lin` / `--max-ang`. 스틱 끝 = 지금 상한 (마우스 패드도 같은 상한). 바뀔 때 PC 터미널에 `[speed] …`

확인: 터미널 1 "전 축 응답 + 각도 확정", 터미널 2 "모든 바퀴 heartbeat 확인", PC 창 아래 `robot: steer … | wheel …` 초록,
`lift: IDLE | h … mm` (원점 미등록이면 노랑 `NOT HOMED`).
끄기: PC Esc → 라파 5 (리프트, 내려갈 때까지 기다림) → 4 → 3 → 2 → 1 순서로 Ctrl+C.

---

## 11. 매핑 · 자율주행 (ZED 2i + RTAB-Map + Nav2) — 노트북

라파는 10번대로 먼저 켜 둠. 노트북은 로봇에 얹고 ZED 2i 를 USB3 에 연결. 리프트 **최저**, 팔 전부 0 (접힌 자세).
배경·측정값은 [`capstone_ws/src/navigation/`](capstone_ws/src/navigation/) 의 launch·config 머리 주석. 대회 당일 순서는 [`CHALLENGE.md`](CHALLENGE.md).

**새 터미널마다**
```bash
source /opt/ros/humble/setup.bash && source ~/zed_ws/install/setup.bash && source ~/capstone/capstone_ws/install/setup.bash
```

### 공통 (매핑·자율주행 둘 다)

**T1 — ZED** (첫 실행은 모델 최적화로 수 분 걸릴 수 있음)
```bash
ros2 launch capstone_navigation zed.launch.py
```

**T4 — 가상 패드** (10번의 PC 명령 대신 이걸로. RB = 수동, **RT = Nav2 데드맨**)
```bash
~/capstone/capstone_ws/src/navigation/tools/pad.sh
```

### 매핑

**T2 — 매핑** (`X` = 지도 이름. 기존 지도에 이어서 하려면 `new_map:=true` 를 뺌)
```bash
ros2 launch capstone_navigation map.launch.py map:=X new_map:=true
```

**T3 — 확인 · 녹화 · 장소 저장**
```bash
# 바닥이 평평하게 보이는지 (장소 옮기면 한 번. "평평함" 이 나와야 함, 아니면 알려 주는 값을 보정에 더함)
python3 ~/capstone/capstone_ws/src/navigation/tools/zed_floor_level.py

# 녹화 (선택 — 나중에 로봇 없이 다시 돌려 볼 수 있음, 15 분에 약 1.4 GB)
ros2 service call /zed/zed_node/start_svo_rec zed_msgs/srv/StartSvoRec "{compression_mode: 2, bitrate: 0, target_framerate: 0, svo_filename: '$HOME/svo/X.svo2'}"

# 설 자리·방향에 로봇을 멈춰 세우고 저장 (→ ~/maps/X.places.yaml)
python3 ~/capstone/capstone_ws/src/navigation/tools/place_mark.py shelf_A
python3 ~/capstone/capstone_ws/src/navigation/tools/place_mark.py desk_1 --kind desk
python3 ~/capstone/capstone_ws/src/navigation/tools/place_mark.py --list
python3 ~/capstone/capstone_ws/src/navigation/tools/place_mark.py --delete shelf_A

# 끝낼 때: 녹화 먼저 멈추고 → T2 를 Ctrl+C → 프롬프트가 돌아올 때까지 기다림 (이때 지도 저장)
ros2 service call /zed/zed_node/stop_svo_rec std_srvs/srv/Trigger
```

- 출발점 · 목표(선반 앞) · 갈림길에서 **제자리 한 바퀴** (재위치는 지도 만들 때 본 방향만 찾음)
- 주요 경로는 **양방향**으로, 끝날 땐 출발점에 **같은 방향**으로 돌아오기
- 0.15 m/s 이하, 사람 적을 때. 카메라 앞에 사람이 서 있으면 지도에 남음
- 장소는 가구에서 10~15 cm 띄워 세우고 저장 (저장되는 것 = 로봇이 설 자리)

### 재위치 + 자율주행

**T2 — 재위치** (지도 고정)
```bash
ros2 launch capstone_navigation loc.launch.py map:=X
```

**T3 — Nav2** (`Managed nodes are active` 가 나올 때까지 기다림)
```bash
ros2 launch capstone_navigation drive.launch.py
```

**T5 — RViz** (선택. 핑크 = 장애물, 하늘색 = 못 감, 빨강~파랑 = 갈 수 있지만 꺼림)
```bash
rviz2 -d ~/capstone/capstone_ws/install/capstone_navigation/share/capstone_navigation/config/nav.rviz
```

**T6 — 위치 찾기 → 이동** (가는 동안 **RT 를 누르고 있어야** 움직임, 떼면 정지)
```bash
# 패드(RB)로 조금 움직이거나 돌려서 "위치 찾음" 이 나오면 Ctrl+C. 그 전엔 출발하지 말 것
python3 ~/capstone/capstone_ws/src/navigation/tools/rtabmap_loc_monitor.py

# 기억한 장소로
python3 ~/capstone/capstone_ws/src/navigation/tools/nav_goal_ahead.py --name shelf_A --dry   # 좌표만 확인
python3 ~/capstone/capstone_ws/src/navigation/tools/nav_goal_ahead.py --name shelf_A

# 여러 곳 왕복 (--loops 0 = 계속, --keep-going = 실패해도 다음으로)
python3 ~/capstone/capstone_ws/src/navigation/tools/place_tour.py shelf_A desk_1 --loops 3 --wait 5

# 선반 확인하고 서기 (2 단계, 시뮬에서만 확인 — 실기 미확인). 먼저 매핑·재위치 중에 두 번 저장:
#   설 자리에서  place_mark.py shelf_A   →  0.8~1.2 m 물러나 선반을 정면으로 보고  place_mark.py shelf_A --view
python3 ~/capstone/capstone_ws/src/navigation/tools/shelf_check.py shelf_A -n 20      # 지금 자리에서 재기만 (흩어짐 확인)
python3 ~/capstone/capstone_ws/src/navigation/tools/place_go.py shelf_A --check-only  # 보는 자리까지 가서 재기만
python3 ~/capstone/capstone_ws/src/navigation/tools/place_go.py shelf_A               # 확인 → 선반 기준으로 접근 (RT 계속 누름)
#   --go-anyway = 선반을 확실히 못 알아봐도 멈추지 않고 기억한 자리로 감 (수 cm 오차, 통로 감시는 그대로. 차체가 몇 cm 틀려도 되는 작업용)
#   물건이 바뀌는 선반: 팔 작업 뒤  place_go.py shelf_A --leave  → 보는 자리만큼 물러나며 기준 사진을 새로 찍음 (뒤는 카메라가 못 봄)
#   대회장에서 --view 저장 직후  shelf_check.py shelf_A -n 10  으로 10 번 다 "확실" 인지 확인.
#   --view 출력의 "차체 앞에서 ○ cm, 폭 ○ m, 치우침" 이 실제와 다르면 선반이 아닌 것을 잡은 것 → 다시
#   종료 코드 0 = 섬, 2 = 선반 없음, 3 = 너무 많이 옮겨짐(40 cm·20°), 4 = 확신 낮음, 5 = 접근 시간 초과, 6 = 앞이 막힘·가려짐
#   --allow-mid = 사진은 안 맞고 모양·판만 맞아도 접근 (선반 위 물건이 바뀐 경우), --wait-blocked 30 = 앞이 막혔으면 30 초까지 기다림
#   접근하는 동안 차체 폭 통로에 뭔가 들어오면 바로 멈춤
#   정면이 막혀 있으면 선반 옆면(왼쪽·오른쪽 중 costmap 에서 갈 수 있는 쪽)으로 가서 섬. --side none = 안 감, --side left|right = 그쪽만
#   옆면 설 자리는 선반 크기로 자동 계산 (벽에 붙은 선반 가정). 직접 정하려면 그 자리에 세워 놓고  place_mark.py shelf_A --side left

# 다시 찾기 (3 단계, 시뮬에서만 확인 — 장기 계획). place_go 가 "없음"(코드 2)으로 끝났을 때:
#   매핑 때 지나간 지점을 가까운 곳부터 돌며 60° 씩 둘러보고, 저장 사진·선반 모양이 보이면 가서 확인 → 찾으면 기억을 고침
python3 ~/capstone/capstone_ws/src/navigation/tools/place_find.py shelf_A                  # 종료 코드 0 = 찾음 (이어서 place_go), 2 = 못 찾음
python3 ~/capstone/capstone_ws/src/navigation/tools/place_find.py shelf_A --max-points 4   # 가까운 4 곳만
python3 ~/capstone/capstone_ws/src/navigation/tools/place_find.py shelf_A --around 12.5 2.0   # 이 지도 좌표 근처부터

# 그 밖의 목표
python3 ~/capstone/capstone_ws/src/navigation/tools/nav_goal_ahead.py 1.5               # 앞 1.5 m
python3 ~/capstone/capstone_ws/src/navigation/tools/nav_goal_ahead.py 1.0 0.5 90        # 앞 1.0, 왼 0.5 m, 왼쪽 90°
python3 ~/capstone/capstone_ws/src/navigation/tools/nav_goal_ahead.py --abs 3.2 -1.0 180   # 지도 좌표
```

- 재위치 모드에서도 `place_mark.py` 로 장소 추가 가능 ("위치 찾음" 뒤에)
- 회복 동작 중 **후진**할 수 있음 — 카메라는 앞만 봄, 뒤에 뭐가 있으면 RT 떼기

### 옵션

실기용 짧은 launch (`zed` · `map` · `loc` · `drive`) 와 `tools/pad.sh` 는 이 로봇 값이 기본으로 들어 있음 — 바꿀 것만 뒤에 붙임
(예: `map.launch.py map:=X cam_roll_offset:=-0.9`, `drive.launch.py inflation_radius:=0.30`, `pad.sh --max-lin 0.3`).
기본값 자체를 바꾸려면 `launch/map.launch.py` 의 `ROBOT` 을 고치고 `colcon build --packages-select capstone_navigation`.
시뮬은 원래 것 (`slam.launch.py` · `nav.launch.py`) 을 씀. 아래 표의 기본은 원래 launch 기준.

| 인자 | 어디 | 기본 | 뜻 |
|---|---|---|---|
| `cam_x` `cam_y` `cam_z` | slam.launch | 0 / 0 / 1.0 | 차체 중심 → ZED 밑면 나사 [m]. 이 로봇 0.167 / 0 / 0.675 (리프트 최저) |
| `cam_pitch_offset` `cam_roll_offset` | slam.launch | 0 | ZED 가 추정한 숙임·옆 기울기에 더할 보정 [deg]. 이 로봇 1.3 / 0.7 |
| `inflation_radius` | nav.launch | 0.45 | 장애물 주변 여유 구역 폭 [m] |
| `floor_clear` | nav.launch | true | 바닥 점으로도 장애물 지우기 (false = 예전 방식) |
| `--nav-topic` | virtual_pad | 없음 | 이게 있어야 Nav2 명령을 RT 누를 때 전달 |

### 지도 보기 · 점검

```bash
rtabmap-databaseViewer ~/maps/X.db        # 저장된 지도 (Edit → View 3D map). 끌 때 저장은 No
python3 ~/capstone/capstone_ws/src/navigation/tools/zed_check.py --vio-only        # ZED 토픽·주기
python3 ~/capstone/capstone_ws/src/navigation/tools/zed_mount_check.py --cam-z 0.675 --pitch 21.6   # 높이·IMU 각·바닥 잡음
python3 ~/capstone/capstone_ws/src/navigation/tools/zed_motion_check.py            # 줄자 거리·제자리 회전과 비교
```

바퀴 오도메트리가 얼마나 맞는지 (EKF 에 넣기 전에). 라파에서 `ros2 launch swerve_controller swerve_odom.launch.py publish_tf:=false` 를 먼저 켬:
```bash
python3 ~/capstone/capstone_ws/src/navigation/tools/odom_check.py line    # Enter → 똑바로 5 m → Enter → 줄자 거리 입력
python3 ~/capstone/capstone_ws/src/navigation/tools/odom_check.py spin    # Enter → 제자리 한 바퀴, 처음 방향에 맞춤 → Enter
```
바퀴와 ZED VIO 값을 나란히 보여 줌. 기준: 거리 2 %, 한 바퀴 5°.

끄기: T6 → T3 (Nav2) → T2 (재위치·매핑, 저장 기다림) → T1 (ZED) → 패드 Esc → 라파 (10번 순서).
