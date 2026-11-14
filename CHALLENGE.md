# 대회 실행 순서

처음 가는 대회장에서 **1 일차에 지도를 만들고, 2 일차에 그 지도로 선반 앞까지 자율주행**하는 순서.
명령 설명·옵션은 [`CLI.md`](CLI.md) 10번(라파 주행)·11번(매핑·자율주행). 여기는 당일에 위에서 아래로 따라 치는 용도.

지도 이름은 `venue` 로 통일 (`~/maps/venue.db`, 장소 파일 `~/maps/venue.places.yaml`).

---

## 0. 가기 전 (전날)

- [ ] 노트북·라파 코드 최신 + 빌드 (`colcon build`), 노트북 충전기
- [ ] 연구실에서 한 번 끝까지 돌려 보기: 매핑 → 장소 저장 → 껐다 켜서 `--name` 으로 이동
- [ ] 구동 4 축 · 조향 4 축 alive 확인 (`ros2 topic echo --once /wheel_state`, `/steer_state`)
- [ ] 디스크 여유 10 GB 이상 (`df -h ~`) — 지도 15 분에 약 0.6 GB, 녹화 1.4 GB
- [ ] 챙길 것: ZED USB3 케이블, 게임패드, 줄자, 바닥 테이프, 멀티미터, 예비 CAN 선
- [ ] 라파가 붙을 와이파이 (휴대폰 핫스팟) — 노트북도 같은 망

## 1. 도착해서 (두 날 모두)

**라파** — 터미널마다 (`ssh raspberrypi`)
```bash
~/capstone/capstone_ws/can/can_up.sh
ros2 launch steer_can_bridge steer_can_bridge.launch.py
ros2 launch wheel_can_bridge wheel_can_bridge.launch.py
ros2 launch swerve_controller swerve_controller.launch.py
ros2 run capstone_teleop udp_teleop_bridge.py
ros2 run lift_bridge lift_bridge_node.py
```

**노트북** — 새 터미널마다
```bash
source /opt/ros/humble/setup.bash && source ~/zed_ws/install/setup.bash && source ~/capstone/capstone_ws/install/setup.bash
```

**T1 — ZED**
```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i camera_name:=zed publish_tf:=false publish_map_tf:=false ros_params_override_path:=$HOME/capstone/capstone_ws/install/capstone_navigation/share/capstone_navigation/config/zed_vio_only.yaml
```

**T4 — 가상 패드** (RB = 수동, RT = 자율주행)
```bash
python3 ~/capstone/capstone_ws/src/teleop/scripts/virtual_pad.py --udp raspberrypi.local --nav-topic /nav/cmd_vel --max-lin 0.15 --max-ang 0.4
```

- [ ] 리프트 **최저**, 팔 전부 0 (접힌 자세)
- [ ] 패드 창 아래 `robot: steer … | wheel …` 초록
- [ ] 패드(RB)로 앞뒤·회전이 되는지 짧게

## 2. 1 일차 — 매핑

**T2 — 매핑**
```bash
ros2 launch capstone_navigation slam.launch.py odom_source:=zed new_map:=true database_path:=$HOME/maps/venue.db cam_x:=0.167 cam_z:=0.675 cam_pitch_offset:=1.3 cam_roll_offset:=0.7
```

**T3 — 바닥 확인** (앞 바닥이 2 m 이상 빈 곳에서)
```bash
python3 ~/capstone/capstone_ws/src/navigation/tools/zed_floor_level.py
```
- [ ] "평평함" 이 나옴. 보정값이 나오면 T2 를 끄고 그 값을 `cam_pitch_offset` / `cam_roll_offset` 에 **더해서** 다시 켬

**T3 — 녹화 시작**
```bash
ros2 service call /zed/zed_node/start_svo_rec zed_msgs/srv/StartSvoRec "{compression_mode: 2, bitrate: 0, target_framerate: 0, svo_filename: '$HOME/svo/venue.svo2'}"
```

**주행** (패드 RB, 0.15 m/s 이하)
- [ ] 출발점에서 제자리 한 바퀴
- [ ] 출발점 → 선반까지 가는 길, 갈림길마다 제자리 한 바퀴
- [ ] 선반 앞에서 제자리 한 바퀴
- [ ] 선반 → 출발점 (같은 길을 반대 방향으로)
- [ ] 출발점에 **처음과 같은 방향**으로 서서 끝
- 카메라 앞에 사람이 서 있지 않게. 지나간 사람 자국은 그 자리를 한 번 더 지나가면 대부분 지워짐

**T3 — 장소 저장** (설 자리·방향에 멈춰 세우고. 선반에서 10~15 cm 띄움, 바닥에 테이프로 바퀴 위치 표시)
```bash
python3 ~/capstone/capstone_ws/src/navigation/tools/place_mark.py shelf_A
python3 ~/capstone/capstone_ws/src/navigation/tools/place_mark.py start --kind start
python3 ~/capstone/capstone_ws/src/navigation/tools/place_mark.py --list
```
- [ ] `기준 노드 [...]` 줄이 나옴 ("지도 노드를 못 얻음" 경고가 뜨면 다시 저장)

**끝내기** — 순서 지킬 것
```bash
ros2 service call /zed/zed_node/stop_svo_rec std_srvs/srv/Trigger
```
- [ ] T2 를 Ctrl+C → **프롬프트가 돌아올 때까지 기다림** (이때 지도 저장)
- [ ] `ls -la ~/maps/venue.db ~/maps/venue.places.yaml` 로 파일 확인

**그날 바로 확인** (2 일차 조건을 미리 해 봄)
- [ ] 3번(2 일차) 순서대로 다시 켜서 `--name shelf_A` 로 한 번 가 보기
- [ ] 안 되는 구간이 있으면 `new_map:=true` 를 **빼고** T2 를 켜서 이어서 매핑 (예전에 지나간 곳에서, 그때 본 방향으로 시작)
- [ ] 지도·장소 파일 백업: `cp ~/maps/venue.db ~/maps/venue.places.yaml ~/maps/backup/` (없으면 `mkdir -p ~/maps/backup`)

## 3. 2 일차 — 자율주행

1번(도착해서)을 먼저 한 뒤.

**T2 — 재위치**
```bash
ros2 launch capstone_navigation slam.launch.py odom_source:=zed localization:=true database_path:=$HOME/maps/venue.db cam_x:=0.167 cam_z:=0.675 cam_pitch_offset:=1.3 cam_roll_offset:=0.7
```

**T3 — Nav2** (`Managed nodes are active` 까지 기다림)
```bash
ros2 launch capstone_navigation nav.launch.py odom_topic:=/zed_base_tf/odom cmd_vel_topic:=/nav/cmd_vel
```

**T5 — RViz**
```bash
rviz2 -d ~/capstone/capstone_ws/install/capstone_navigation/share/capstone_navigation/config/nav.rviz
```

**T6 — 위치 찾기**
```bash
python3 ~/capstone/capstone_ws/src/navigation/tools/rtabmap_loc_monitor.py
```
- [ ] 1 일차 출발점에, 그때와 같은 방향으로 세움
- [ ] 패드(RB)로 조금 움직이거나 제자리에서 천천히 돌림 → **"위치 찾음"** 이 여러 번 나오면 Ctrl+C
- [ ] RViz 에서 로봇이 지도의 맞는 자리에 있는지 눈으로 확인

**T6 — 출발** (RT 를 누르고 있어야 움직임, 떼면 정지)
```bash
python3 ~/capstone/capstone_ws/src/navigation/tools/nav_goal_ahead.py --name shelf_A --dry
python3 ~/capstone/capstone_ws/src/navigation/tools/nav_goal_ahead.py --name shelf_A
```
- [ ] `--dry` 가 보여 주는 직선 거리가 실제와 비슷함 → 그 다음에 보내기
- [ ] 도착하면 `결과: 성공` — 이후 팔 작업

## 4. 안 될 때

| 증상 | 할 일 |
|---|---|
| "위치 찾음" 이 안 나옴 | 1 일차에 지나간 곳으로 옮겨 그때 본 방향을 보게 함. 제자리에서 천천히 한 바퀴. 조명·가구가 크게 바뀐 곳은 피함 |
| `--name` 이 "기준 노드가 지금 지도에 없음" | 다른 지도를 띄웠거나 이어서 매핑한 구간이 아직 안 이어짐. `database_path` 확인 |
| "목표 거절됨" · 바로 "실패" | 목표 자리가 장애물에 너무 가까움. RViz 에서 목표 주변 색 확인, 패드로 근처까지 옮긴 뒤 다시 |
| 빈 곳인데 막힌 것처럼 머뭇거림 | 잠시 기다리면 회복 동작이 costmap 을 지움. 안 풀리면 RB 로 그 구간만 수동 통과 후 RT |
| 회복 동작으로 후진함 | 뒤는 카메라가 못 봄 — 뒤에 뭐가 있으면 RT 떼기 |
| RT 를 눌러도 안 움직임 | 패드를 `--nav-topic /nav/cmd_vel` 로 켰는지, Nav2 가 `cmd_vel_topic:=/nav/cmd_vel` 인지, 패드 창이 초록인지 |
| 수동(RB)도 안 움직임 | 라파: `ros2 topic echo --once /wheel_state` (alive·ready), `ip -details link show can_drive` (ERROR-ACTIVE 가 정상) → `can_up.sh` 다시, 구동 브리지 재시작, 그래도 안 되면 구동 CAN 커넥터를 뺐다 다시 꽂음 (11/13 에 이걸로 복구), 전원 확인 |
| 경로가 벽 쪽으로 붙거나 지도와 어긋남 | 위치가 틀어진 것 — RT 떼고, 제자리에서 돌려 "위치 찾음" 다시 |
| 노트북이 느려지고 끊김 | RViz 부터 끔 |
| `zed_floor_level.py` 가 `점 부족` | 리프트가 최저가 아님 (카메라 높이가 다름). `zed_mount_check.py --cam-z 0.675 --pitch 21.6` 로 높이·숙임 확인 → 리프트 내림 |
| 바닥이 좌우로 기울어 보임 | ZED 가 추정하는 옆 기울기가 켤 때마다 1.5° 쯤 다름. `zed_floor_level.py` 가 알려 주는 값을 `ros2 param set /zed_base_tf roll_offset_deg <값>` 으로 넣음 (지금 값 + 권장값) |
| RT 를 눌러도 `place_go` 가 `[1] 보는 자리로` 에서 멈춰 있음 | 패드를 `--nav-topic /nav/cmd_vel` 없이 켠 것 |
| ZED 가 `NO GPU DETECTED` · `CUDA error =999` | 노트북이 절전에 들어갔다 깨어난 뒤 생김. ZED 를 끄고 `sudo ~/capstone/capstone_ws/src/navigation/tools/nvidia_uvm_reload.sh` → ZED 다시 켬 (자동 설치 방법은 그 파일 머리 주석) |

**정지**: RT·RB 놓기 → 패드 Space → 24 V 비상정지.

## 5. 끄는 순서

T6 → T3 (Nav2) → T2 (재위치·매핑, 저장 기다림) → T1 (ZED) → 패드 Esc → 라파 리프트(내려갈 때까지 기다림) → 텔레옵 → 컨트롤러 → 구동 → 조향.
