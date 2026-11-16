# Jetson CLI — 대회장: 1. 매핑 → 2. 이동 자동화

구성 (2026-11-17~): **Jetson = ZED·매핑·Nav2·가상 패드(게임패드)**, **라파 = 주행·리프트 브리지**, 둘은 iDLE 유선 (Jetson 192.168.1.11, 라파 192.168.1.12).
노트북은 접속·RViz 용 (랜선으로 iDLE 에 꽂으면 `ssh jetson` 이 바로 붙음).

> Jetson 에서 확인한 것: ZED ROS (영상 13 Hz·깊이 15 Hz·자세 30 Hz), 게임패드 주행. **매핑·Nav2 주행은 Jetson 에서 아직 안 돌려 봄** — 대회장에서 처음이면 짧게 먼저 시험.
> 노트북 때의 배경·옵션은 [`CLI.md`](CLI.md) 11장, 대회 순서는 [`CHALLENGE.md`](CHALLENGE.md).

---

## 0. 켜기

### 라파 (CLI.md 10장 그대로 — 터미널 6 개, 순서대로)

```bash
~/capstone/capstone_ws/can/can_up.sh                                                   # 0: CAN (한 번)
source /opt/ros/jazzy/setup.bash && source ~/capstone/capstone_ws/install/setup.bash   # 1~5 터미널마다
ros2 launch steer_can_bridge steer_can_bridge.launch.py      # 1: 조향
ros2 launch wheel_can_bridge wheel_can_bridge.launch.py      # 2: 구동
ros2 launch swerve_controller swerve_controller.launch.py    # 3: cmd_vel → 조향·구동
ros2 run capstone_teleop udp_teleop_bridge.py                # 4: 패드 UDP 수신 ← 빠지면 안 움직임
ros2 run lift_bridge lift_bridge_node.py                     # 5: 리프트
```

### Jetson

접속: 노트북에서 `ssh jetson` (또는 Jetson 모니터 앞 터미널). **터미널마다 첫 줄**:
```bash
source ~/nav_env.sh        # ROS + ZED + capstone_ws, ROS_DOMAIN_ID=21, 라파 주소, $NAV = 내비 도구 폴더
```

| 터미널 | 명령 | |
|---|---|---|
| J1 ZED | `ros2 launch capstone_navigation zed.launch.py` | 영상·깊이·자세 (깊이 모델 최적화는 끝나 있음) |
| J4 패드 | `pad` | 게임패드 (Jetson USB, X-Box 모드). SSH·화면 없이 됨. Jetson 모니터 앞이면 `$NAV/pad.sh` = 창 |

게임패드: **RB 누른 채 스틱 = 수동**, **RT 당기고 있기 = Nav2 명령 통과** (떼면 정지), 십자키 = 속도 상한, RB+Y/A/X/B = 리프트.
안 움직이면: 라파 4번 켜졌나 → `pad` 첫 줄에 `Xbox 360 Controller` 나오나 (안 나오면 게임패드 다시 꽂기).

---

## 1. 매핑

리프트 **최저**, 팔 **0 (접힘)** — 카메라 높이 기본값 `cam_z 0.675` 가 이 상태 기준. 리프트를 올린 채면 `cam_z:=<바닥→ZED 삼각대 나사 m>`.

```bash
# J2 — 매핑 (X = 지도 이름, 새로: new_map:=true / 이어서: 빼기)
ros2 launch capstone_navigation map.launch.py map:=X new_map:=true

# J3 — 바닥 보정 (켤 때마다 한 번, 장소 저장 전에) → "평평함" 될 때까지
python3 $NAV/zed_floor_level.py --apply

# J3 — 장소 저장: 그 자리에 로봇을 세우고 (저장 = 로봇이 설 자리·방향)
python3 $NAV/place_mark.py desk_1 --kind desk
python3 $NAV/place_mark.py shelf_A
python3 $NAV/place_mark.py shelf_A --view      # 선반 0.8~1.2 m 뒤에서 정면으로 보고 (place_go 용)
python3 $NAV/place_mark.py --list
python3 $NAV/place_mark.py --delete desk_1

# 끝: J2 를 Ctrl+C → 프롬프트 돌아올 때까지 기다림 (이때 ~/maps/X.db 저장)
```

- 출발점·목표 앞·갈림길에서 **제자리 한 바퀴**, 주요 길은 **양방향**, 끝은 출발점에 **같은 방향**으로
- **책상·선반은 1.5~2.5 m 떨어진 정면에서 한 번씩 보고 지나가기** — 리프트 최저 (ZED 약 0.7 m) 에선 카메라보다 높은 곳이
  멀리서만 보임 (아래 "높이 주의"). 안 하면 책상 윗면 모서리·선반 위 판이 지도에 안 남아 리프트·팔이 걸릴 수 있음
- 0.15 m/s 이하. 카메라 앞에 사람이 서 있으면 지도에 남음
- **설 자리 거리 (팔 계산, 1 m 책상):** 책상 앞 모서리가 로봇 중심에서 **0.30 m** (차체와 책상 사이 약 10 cm), 집을 물체가 로봇 정면에서 **오른쪽 10 cm** 쯤 오게 세우고 저장. 팔이 닿는 곳 = 앞 0.42~0.66 m, 오른쪽 0.30 ~ 왼쪽 0.10 m
- 선반은 앞면에서 10~15 cm 띄워 세우고 저장

---

## 2. 이동 자동화 (재위치 + Nav2)

J1 (ZED) · J4 (패드) 는 그대로.

```bash
# J2 — 재위치 (지도 고정)
ros2 launch capstone_navigation loc.launch.py map:=X

# J3 — Nav2 ("Managed nodes are active" 나올 때까지 기다림)
ros2 launch capstone_navigation drive.launch.py

# J5 — 위치 찾기: 패드(RB)로 조금 움직이거나 돌려서 "위치 찾음" 나오면 Ctrl+C. 그 전엔 출발 금지
python3 $NAV/rtabmap_loc_monitor.py

# J5 — 가기 (가는 동안 RT 를 계속 당기고 있어야 움직임)
python3 $NAV/nav_goal_ahead.py --name desk_1 --dry     # 좌표만 확인
python3 $NAV/nav_goal_ahead.py --name desk_1
python3 $NAV/place_tour.py shelf_A desk_1 --loops 3 --wait 5      # 여러 곳 왕복 (--loops 0 = 계속, --keep-going)
python3 $NAV/place_go.py shelf_A                                   # 선반 확인 후 선반 기준으로 정밀 접근 (실기 미확인)
python3 $NAV/place_go.py shelf_A --check-only                      #   보는 자리까지 가서 재기만
```

### 지도 띄우기 (노트북 RViz, 실시간·저장 지도 둘 다)

```bash
# Jetson 1 (ssh jetson)
source ~/nav_env.sh && ros2 launch capstone_navigation zed.launch.py
# Jetson 2 — 저장 지도 보기 = 재위치 / 매핑하며 보기 = map.launch.py map:=X
source ~/nav_env.sh && ros2 launch capstone_navigation loc.launch.py map:=X
# 노트북 (처음 한 번 시계 맞추기 → RViz)
ssh -t jetson "sudo date -s @$(date +%s.%N)"
export ROS_DOMAIN_ID=21 && source /opt/ros/humble/setup.bash && source ~/capstone/capstone_ws/install/setup.bash
rviz2 -d ~/capstone/capstone_ws/install/capstone_navigation/share/capstone_navigation/config/nav.rviz
# 지도가 안 뜨면 (Jetson 아무 터미널)
source ~/nav_env.sh && ros2 service call /rtabmap/publish_map rtabmap_msgs/srv/PublishMap "{global: true, optimized: true, graph_only: false}"
```
로봇 위치는 재위치가 '위치 찾음' 이 된 뒤 맞게 올라감 (패드 RB 로 조금 움직이거나 돌리기).

### 지도 보기 (저장 파일)

```bash
# 저장된 지도 → 그림 한 장 (Jetson 에서, SSH 로도 됨) → 노트북으로 가져와 보기
python3 $NAV/map_png.py ~/maps/X.db                       # Jetson: /tmp/X_map.png
scp jetson:/tmp/X_map.png ~/ && xdg-open ~/X_map.png      # 노트북

# 3D 까지 자세히 (RTAB-Map 뷰어, 창이 있는 곳에서): Jetson 모니터 앞이면 바로, 노트북이면 파일을 가져와서
rtabmap-databaseViewer ~/maps/X.db                                                    # Jetson 모니터
scp jetson:maps/X.db ~/maps/X_jetson.db && source /opt/ros/humble/setup.bash && rtabmap-databaseViewer ~/maps/X_jetson.db   # 노트북 (Edit → View 3D map, 끌 때 저장은 No)
```
- 매핑·재위치 **중에** 실시간으로 보려면 아래 RViz.
- 그림은 노드별 지역 지도를 주행 기록 위치로 이어 붙인 것이라 실제보다 조금 번져 보일 수 있음 (뷰어·재위치는 루프 클로저로 보정된 지도).

### RViz (선택)
- Jetson 모니터 앞: `rviz2 -d ~/capstone_ws/install/capstone_navigation/share/capstone_navigation/config/nav.rviz`
- 노트북 (랜선으로 iDLE): `export ROS_DOMAIN_ID=21 && source /opt/ros/humble/setup.bash && rviz2 -d ~/capstone/capstone_ws/install/capstone_navigation/share/capstone_navigation/config/nav.rviz`
  - Jetson 은 인터넷이 없어 시계가 안 맞음 (2026-11-17: 노트북보다 3.06 s 늦음) → 노트북 RViz 에서 일부가 '너무 오래된 데이터' 로 빠지면
    한 번 맞추기 (노트북 터미널): `ssh -t jetson "sudo date -s @$(date +%s.%N)"`
  - 저장된 지도를 이 화면으로 보려면 Jetson 에서 J1 (ZED) + J2 `loc.launch.py map:=X` 를 켜 두면 됨

---

## 높이 주의 (리프트·카메라)

1. **매핑: 높은 곳은 멀리서만 보임.** 리프트 최저면 ZED 약 0.7 m, 21° 숙임, 화면 위 끝이 수평선 위 약 13°.
   책상 윗면 0.78 m → 0.3 m 밖부터, **1.0 m → 1.25 m 밖부터**, 선반 맨 위 판 1.31 m → 약 2.5 m 밖부터 보임.
   한 번 보이면 지도·장애물에 남음 (장애물 높이 0.08~1.9 m, 시야 밖으로 나가도 안 지워짐). 다리·기둥은 항상 보임.
2. **설 자리로 가기는 위치가 고정이라 윗면이 안 보여도 됨.** 다만 책상에 10 cm 붙는 자리는 Nav2 가 "장애물에 너무 가깝다"
   며 못 가거나 덜 갈 수 있음 → 마지막은 `place_go` (보는 자리까지 Nav2 → 천천히 직접 접근, 통로 감시).
3. **팔 작업은 리프트를 올려야 책상 위가 보임** (1 m 책상: ZED 약 1.15 m, 리프트 약 470 mm). 내비는 카메라 높이를 리프트
   최저 (`cam_z 0.675`) 로 가정하고 장애물을 계산하므로 **리프트 올린 채로 주행 금지**:
   `리프트 최저로 이동 → 설 자리 도착 → 리프트 올림 → 찾기·집기 → 리프트 내림 → 다음 장소로`
   (`place_mark --view` 사진도 리프트 최저에서 찍어야 도착 후 비교가 맞음)

---

## 끄기

Jetson: J5 → J3 (Nav2) → J2 (매핑이면 **저장 끝날 때까지 기다림**) → J1 → J4 순서로 Ctrl+C.
라파: 5 (리프트, 최하단까지 내려가며 꺼짐 — 기다림) → 4 → 3 → 2 → 1.

---

## 알아 둘 것

| 것 | 내용 |
|---|---|
| 경로 | 노트북 `~/capstone/capstone_ws/...` 대신 Jetson `~/capstone_ws/...` (`$NAV`). 지도 `~/maps/` (노트북 지도는 없음). `unable to open database file` 이면 `mkdir -p ~/maps` |
| 인터넷 | Jetson 은 인터넷 없음. 설치가 더 필요하면 노트북을 랜선으로 꽂고 터널 (`ssh -N -R 127.0.0.1:1080 jetson`), apt 에 `-o Acquire::http::Proxy=socks5h://127.0.0.1:1080 -o Acquire::https::Proxy=socks5h://127.0.0.1:1080` |
| 게임패드 | Jetson 커널에 X-Box 드라이버를 직접 빌드해 넣음 (`~/xpad_build`, 부팅 때 자동). JetPack 업데이트하면 다시 빌드 |
| ZED 한 대 | 매핑·Nav2 가 ZED 를 쓰는 동안 물체 찾기 `svo_find --live` 는 ZED 를 못 엶 (ROS 영상으로 찾는 건 다음 작업) |
| 선반 글자 찾기 | OWLv2 선반 서버 (5577) 는 Jetson 에 아직 안 띄움 → place_mark/place_go 는 예전 방식으로 동작 |
| 리프트 경고 | `pad` 의 `[lift] 경고: Pico mm/count 0.10980 ≠ 브리지 0.21574` = 리프트 높이 단위 불일치 (라파 리프트 설정). 주행·조그엔 영향 없음, 높이 mm 표시가 틀림 |
| 패드 오류 | `source ~/nav_env.sh` 에서 `syntax error near '('` 가 나오면 옛 `pad` 별칭이 남은 터미널 — 고쳐 둠, 새 터미널에서 다시 |
