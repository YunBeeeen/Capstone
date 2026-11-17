# 실기 CLI — 1. 매핑 · 2. 내비게이션 · 3. 명령 보내기 · 4. 팔 (IK)

구성 (2026-11-17): **Jetson = ZED·매핑·Nav2·물체 찾기 (OWLv2+SAM)**, **라파 = 게임패드·주행·리프트 브리지·팔 (IK·실행)**,
노트북 = 접속·RViz. 모두 iDLE 공유기 (WiFi 든 유선이든 같은 대역).

| 접속 | |
|---|---|
| Jetson | `ssh jetson` (이름 `jetson.local`) |
| 라파 | `ssh raspberrypi` — 이름이 안 풀리면 `ssh pi@192.168.1.12` (또는 0.12) |
| 주소 | iDLE 대역이 **192.168.1.x ↔ 192.168.0.x 로 바뀐 적 있음** (2026-11-17 두 번). Jetson .11, 라파 .12 는 같음 |

> 실기 확인: 매핑 ✅ (Jetson), 게임패드 주행 ✅, Nav2 명령 중계 Jetson→라파 ✅ (망 시험). **Nav2 실주행·팔 집기는 아직.**

---

## 0. 켜기 (매번 공통)

### 라파 — 터미널 0~6, 순서대로
```bash
~/capstone/capstone_ws/can/can_up.sh                                                   # 0  CAN (한 번)
source /opt/ros/jazzy/setup.bash && source ~/capstone/capstone_ws/install/setup.bash   # 1~5 터미널마다 먼저
ros2 launch steer_can_bridge steer_can_bridge.launch.py      # 1  조향
ros2 launch wheel_can_bridge wheel_can_bridge.launch.py      # 2  구동
ros2 launch swerve_controller swerve_controller.launch.py    # 3  cmd_vel → 조향·구동
ros2 run capstone_teleop udp_teleop_bridge.py                # 4  패드 UDP 수신 ← 빠지면 안 움직임
ros2 run lift_bridge lift_bridge_node.py                     # 5  리프트
~/pad/scripts/pad_pi.sh                                      # 6  게임패드 (source 필요 없음)
```
게임패드: **RB 누른 채 스틱 = 수동**, **Nav2 는 누를 것 없이 통과 (실기 배포), RT 당기면 정지**, RB 수동이 우선 (예전 데드맨: `NAV_DEADMAN=1 ~/pad/scripts/pad_pi.sh`), 십자키 = 속도 상한, RB+Y/A/X/B = 리프트.
6 번 첫 줄에 `[gamepad] Xbox 360 Controller` 가 나와야 함 (안 나오면 다시 꽂기). 리프트 **최저**, 팔 **접힘** 으로 시작.

### Jetson — 터미널마다 첫 줄
```bash
source ~/nav_env.sh        # ROS + ZED + capstone_ws, ROS_DOMAIN_ID=21, $NAV = 내비 도구 폴더
```
(`$NAV/...: No such file` → 이 줄을 안 한 터미널)

```bash
ros2 launch capstone_navigation zed.launch.py               # J1  ZED (매핑·내비 둘 다)
```

---

## 1. 매핑 CLI

```bash
# J2  지도 만들기 (X = 지도 이름. 새로: new_map:=true / 기존 X 에 이어서: 빼기)
ros2 launch capstone_navigation map.launch.py map:=X new_map:=true

# J3  바닥 보정 (켤 때마다, 앞에 빈 바닥 2 m 이상) → "평평함" 될 때까지
python3 $NAV/zed_floor_level.py --apply
```
게임패드 (RB) 로 돌아다니기:
- 0.15 m/s 이하, 출발점·목표 앞·갈림길에서 **제자리 한 바퀴**, 주요 길은 양방향, 끝은 출발점에 같은 방향으로
- **책상·선반은 1.5~2.5 m 떨어진 정면에서 한 번씩** (리프트 최저면 카메라보다 높은 곳은 멀리서만 보임)

장소 저장 (J3, 그 자리에 세우고 — 매핑 중이든 재위치 "위치 찾음" 뒤든 됨):
```bash
python3 $NAV/place_mark.py desk_1 --kind desk     # 책상: 앞 모서리가 로봇 중심에서 0.30 m, 물체가 정면 오른쪽 10 cm
python3 $NAV/place_mark.py shelf_A                # 선반: 차체 앞면과 선반 사이 18~26 cm (바구니를 앞에서 넣으려면, 4-A)
python3 $NAV/view_guide.py shelf_A                 # (선반 보는 자리 안내: "✅" 뜰 때까지 패드로 맞춤 → Ctrl+C)
python3 $NAV/place_mark.py shelf_A --view         # 선반 보는 자리 (설 자리 1 m 뒤, 정면). 선반 전용 — 책상엔 안 씀
python3 $NAV/place_mark.py shelf_A --view --shelf-width 0.5 --shelf-depth 0.33   # 선반 크기가 기본 (0.6×0.33) 과 다르면
python3 $NAV/place_mark.py --list                 # 확인
python3 $NAV/place_mark.py --delete desk_1        # 지우기
```
끝: **J2 Ctrl+C → 프롬프트 돌아올 때까지 기다림** (이때 `~/maps/X.db` 저장).

지도 확인:
```bash
python3 $NAV/map_png.py ~/maps/X.db                     # Jetson → /tmp/X_map.png
scp jetson:/tmp/X_map.png ~/ && xdg-open ~/X_map.png    # 노트북
```

---

## 2. 내비게이션 CLI (재위치 + Nav2)

```bash
# J2  재위치 (저장 지도 X, 지도는 안 바뀜)
ros2 launch capstone_navigation loc.launch.py map:=X

# J3  바닥 보정 (켤 때마다) → Nav2 ("Managed nodes are active" 까지 기다림)
python3 $NAV/zed_floor_level.py --apply
ros2 launch capstone_navigation drive.launch.py

# J4  Nav2 명령 → 라파 패드 (UDP 5006). "192.168.x.12 로 보냄" 나오면 정상
python3 ~/capstone_ws/src/teleop/scripts/nav_cmd_udp.py

# J5  위치 찾기: 패드 (RB) 로 조금 움직이거나 돌려서 "위치 찾음" → Ctrl+C. 그 전엔 출발 금지
python3 $NAV/rtabmap_loc_monitor.py
```

노트북 RViz (실시간 지도·위치·경로):
```bash
export ROS_DOMAIN_ID=21 && source /opt/ros/humble/setup.bash && source ~/capstone/capstone_ws/install/setup.bash
rviz2 -d ~/capstone/capstone_ws/install/capstone_navigation/share/capstone_navigation/config/nav.rviz
# 지도가 안 뜨면 (Jetson):
ros2 service call /rtabmap/publish_map rtabmap_msgs/srv/PublishMap "{global: true, optimized: true, graph_only: false}"
```
Jetson 시계는 1970 (인터넷 없음, 원래 그럼) — Jetson 안에선 문제 없음. 노트북 RViz 에서 일부가 안 보이면 `ssh -t jetson "sudo date -s @$(date +%s.%N)"` 를 **Jetson ROS 를 켜기 전에**.

---

## 3. 명령 보내기 CLI (J5, 누를 것 없음 — 멈추려면 **RT**)

```bash
python3 $NAV/place_mark.py --list                              # 갈 수 있는 장소
python3 $NAV/nav_goal_ahead.py --name desk_1 --dry             # 좌표만 확인 (안 움직임)
python3 $NAV/nav_goal_ahead.py --name desk_1                   # 장소로
python3 $NAV/nav_goal_ahead.py --name shelf_A
python3 $NAV/place_go.py shelf_A                               # 선반: 보는 자리까지 Nav2 → 선반 다시 재고 → 천천히 직접 접근 (--view 필요, 실기 미확인)
python3 $NAV/place_go.py shelf_A --check-only                  #   보는 자리까지 가서 재기만
python3 $NAV/place_tour.py shelf_A desk_1 --loops 3 --wait 5   # 여러 곳 왕복 (--loops 0 = 계속, --keep-going)
```
안 움직이면: J4 가 `N 개/s` 를 세나 (0 이면 Nav2 가 명령 안 냄 → J3 확인) → 라파 6 번에 `[nav] ... 수신 시작` 이 나왔나 → RB 를 같이 누르고 있지 않나.
책상에 10 cm 붙는 자리는 Nav2 가 덜 가서 멈출 수 있음 (장애물 여유) — 정상.

---

## 4. 팔 CLI (IK · 찾기 · 집기) — 나중에 하나로 합칠 단계별 정리

**순서 (4-A 가 이걸 한 번에): 도착 → 리프트 올림 (책상 위가 보이게, 1 m 책상: 리프트 약 470 mm) → 찾기 → 계획 → 실행 → 리프트 내림 → 다음 장소.**
지금은 **물체 찾기가 ZED 를 직접 열어서 J1 (ROS ZED) 을 꺼야 함** (ROS 영상으로 찾게 바꾸는 건 다음 작업).

### 4-A. 전체 임무 한 번에 (`mission_run.py`, 라파) — 계획·MuJoCo·대회장 시뮬 통과, 실기 미확인

desk_1 → 물체 찾기 → 집기 → 바구니에 넣기 → 바구니 로봇 쪽 **짧은 벽** 잡고 그 자리에서 10 cm 들기 → shelf_A_view 에서 **팔로 바구니를 판 위 높이로** → shelf_A (바구니가 판 위로 들어감) → 선반 3 층 (1.00 m) 오른쪽에 수평으로 밀어 넣기.
**리프트는 처음부터 끝까지 325 mm = 카메라 바닥에서 1.00 m** (2026-11-18 계획 검사). 팔 연결 하나로 끝까지. 물체 찾기는 ROS 영상 (`ros_find`).
```bash
# 준비 — 리프트를 325 mm (카메라 1.00 m) 로 올려 두고 시작 (게임패드 RB+X/Y)
#   Jetson 내비는 카메라 높이를 맞춰서:  ros2 launch capstone_navigation jetson_nav.launch.py map:=X cam_z:=1.0
#   docker start arm_vision_server, 라파 can_arm_r 올림, 라파 6 번 패드
cd ~/arm && PY=~/rebot_env/venv/bin/python
$PY tools/mission_run.py pumpkin                 # 기본: 리프트 325 mm 고정, 3 층
$PY tools/mission_run.py pumpkin --to drop       # 바구니에 넣기까지만
$PY tools/mission_run.py pumpkin --from find     # 중간부터 (바구니·선반 단계는 find 결과가 필요)
```
단계: `nav_desk → lift_up → find → pick → drop → basket_grab → carry → lift_down → nav_shelf → lift_place → place → finish`
(리프트 고정이면 lift_* 단계는 안 묻고 지나감)

| 조건 (계획 검사 2026-11-18) | 값 |
|---|---|
| 리프트 | **325 mm 고정 (카메라 1.00 m)** → 내비 `cam_z:=1.0`. 다른 높이면 `--lift-mm` 와 `cam_z` 를 같이 (cam_z = 0.675 + mm/1000) |
| 바구니 두는 방향 (책상) | **짧은 변이 로봇을 보게 (긴 축이 앞뒤)**. 로봇 쪽 짧은 벽을 잡음 |
| 선반 앞 | `shelf_A_view` 에서 팔이 바구니를 판 5 cm 위 높이로 올린 뒤 Nav2 로 `shelf_A` — 바구니가 차체 앞으로 ~20 cm 나와 있음, **shelf_A_view 는 선반 정면 (똑바로 다가가게)** |
| 놓는 자리 | 바구니 가운데 로봇 오른쪽 15 cm (`--place-y`, 오른팔이라 가장 쉬움) — 선반 가운데를 보고 서면 선반 가운데보다 15 cm 오른쪽 |
| 손 기울기 | 잡을 때 30~50° 앞으로 → 들고 다니고 넣을 때도 그대로 (선반 3 층은 윗판까지 35 cm 라 위에서 수직으로는 못 넣음) |
| **shelf_A 설 자리** | **선반 앞면이 로봇 중심에서 0.44~0.52 m = 차체 앞면과 선반 사이 18~26 cm** (예전 10~15 cm 는 못 넣음 → 다시 저장) |
| 선반 모양 | 시뮬 선반 (폭 0.6·깊이 0.33·기둥) 에 판 높이 0.21/0.55/1.00/1.35 (`--shelf-tops`) — 계획 때 선반·든 바구니 충돌 검사 |

- 단계마다 ENTER (s = 건너뜀, q = 지금 자세 유지하고 끝). 이동은 누를 것 없음, **멈추려면 RT**
- 막히거나 Ctrl+C → 팔 고정 → ENTER = 그 동작 시작 자세로 복귀, q = 그대로
- 바구니 벽을 잡은 뒤 "잘 끼었나?" 를 눈으로 확인하고 ENTER
- 끝날 때 팀 팔로워가 "팔을 받친 뒤 ENTER" 를 물음 (바구니를 들고 있으면 먼저 받치기)
- 노트북 모의: `yes "" | python3 tools/mission_run.py pumpkin --dry-run --yes --vision v.json --vision-shelf s.json --start=0,0,0,0,0,0`

### 4-0. 준비
```bash
# Jetson: 물체 찾기 서버 (OWLv2+SAM, 포트 5578). 재부팅 뒤엔 꺼져 있음 → 켜고 20~30 초 (모델 읽기)
docker start arm_vision_server && sleep 25 && curl -s 127.0.0.1:5578/health; docker logs --tail 3 arm_vision_server
# 라파: 팔 CAN (꺼져 있는 게 기본)
sudo ~/capstone/capstone_ws/can/can_up.sh
cd ~/arm && PY=~/rebot_env/venv/bin/python                     # 라파 팔 코드 (~/arm = ~/arm_play/ws/arm_kinematics)
```

### 4-1. 한 번에 (라파) — 지금 쓰는 것
```bash
$PY tools/pick.py pumpkin --hover          # 첫 시험: 물체 6 cm 위까지만 (그리퍼 안 씀) → 정렬 눈으로 확인
$PY tools/pick.py pumpkin                  # 그리퍼 열기 → 6 cm 위 → 곧게 내려옴 → 닫기 → (ENTER) 들어 올리며 복귀
$PY tools/pick.py "a toy corn" --dz -0.01  # 목표 1 cm 낮게 (더 깊이 잡기)
```
y 를 입력해야 움직임. 내부 순서 = 아래 4-2 ~ 4-5.

### 4-2. 단계별 (합칠 때 이 입출력으로 이음)

| 단계 | 어디서 | 명령 | 입력 → 출력 |
|---|---|---|---|
| ① 팔 자세 읽기 | 라파 | `$PY tools/arm_watch.py --once` (계속: 옵션 빼기) | 모터 1~6 → `--motors=a,b,c,d,e,f` (토크 안 켬, 읽기만) |
| ② 물체 찾기 | Jetson | `cd ~/arm_vision && PYTHONPATH=~/zedpy python3 tools/svo_find.py --live --zed-height auto pumpkin --json /tmp/v.json --out /tmp/v.jpg` | 말 → 물체 중심 (로봇 바닥 기준 x,y,z m), 책상 높이 `support_z`, 앞 모서리 `desk_front_x`, ZED 높이. 다음 명령 (③) 도 찍어 줌 |
| ③ 계획 (IK·경로) | 라파 | `$PY tools/ik_sim_run.py --target desk --desk-top Z --front X --lateral=Y --xyz x y z --zed-height H --start=<①> --no-sim --descend 0.06 --save-plan /tmp/p.json` | 목표점 + 책상 → `/tmp/p.json` (관절 경유점·시간, 리프트 높이). 실패면 위치 안내 |
| ④ 실행 | 라파 | `$PY tools/play_plan.py /tmp/p.json --grip-before open --grip-after close` (`--stop-at approach` = 6 cm 위까지만, `--check-only` = 점검만) | 계획 → 팔 움직임. 시작 점검·enable 확인·Ctrl+C = 온 길로 복귀·막힘 정지 |
| ⑤ 확인 | 라파 | `$PY tools/fk_check.py` | 모터 → 손끝 위치 (계획과 비교) |

- ② 의 `--zed-height auto` = 보이는 바닥으로 카메라 높이를 잼 (리프트 높이 몰라도 됨). 줄자 값이면 `--zed-height 1.15`.
- ② 찾을 말은 영어 (`pumpkin`, `"a toy corn"`, `"a red pepper"`). 안 잡히면 표현을 바꿈.
- ③ 은 시뮬로 먼저 보려면 `--no-sim` 빼기 (노트북).
- 속도: `play_plan.py --speed 1.0` = 팔 한계 30°/s (지금 기본, 한 동작 약 8 초).

### 4-3. 합칠 때 모양 (계획)
Jetson (두뇌): 장소 도착 → 리프트 올림 → ② 찾기 → 라파에 "이 점 집어라 (x,y,z, 책상 높이, 앞 모서리)" →
라파 팔 서버: ① + ③ + ④ → "잡힘 / 빈손 / 멈춤" 회신 → Jetson: 리프트 내림 → 다음 장소 (3. 명령 보내기).
남은 것: 라파 상주 팔 서버, ② 를 ROS 영상으로 (J1 안 끄고), 바구니 넣기·확인 (`checks.in_basket`).

---

## 높이 주의 (리프트·카메라)
1. 리프트 최저면 ZED 약 0.7 m — 책상 윗면 1.0 m 는 1.25 m 밖에서부터, 선반 위 판은 약 2.5 m 밖에서부터 보임 → 매핑 때 멀리서 한 번씩.
2. 내비는 카메라 높이를 리프트 최저 (`cam_z 0.675`) 로 가정 → **리프트 올린 채 주행 금지.** `place_mark --view` 사진도 리프트 최저에서.
3. 팔 작업 때만 리프트 올림 → 끝나면 내리고 이동.

## 끄기
Jetson: J5 → J4 → J3 → J2 (매핑이면 **저장 끝날 때까지 기다림**) → J1 (Ctrl+C).
라파: 6 (패드) → 5 (리프트, 최하단까지 내려가며 꺼짐 — 기다림) → 4 → 3 → 2 → 1.

## 알아 둘 것
| 것 | 내용 |
|---|---|
| 경로 | Jetson `~/capstone_ws/...` (`$NAV`), 지도 `~/maps/` (`unable to open database file` → `mkdir -p ~/maps`). 라파 패드 `~/pad`, 팔 `~/arm` |
| 인터넷 | Jetson 은 없음. 설치는 노트북을 붙이고 터널 (`ssh -N -R 127.0.0.1:1080 jetson`), apt 에 `-o Acquire::http::Proxy=socks5h://127.0.0.1:1080 -o Acquire::https::Proxy=socks5h://127.0.0.1:1080` |
| 게임패드 | 라파에 꽂음 (커널이 X-Box 를 바로 앎). Jetson 에 꽂을 땐 `pad` (Jetson 은 직접 빌드한 드라이버 `~/xpad_build`) — 패드 프로그램은 **하나만** |
| ZED 한 대 | 내비 (J1) 가 ZED 를 쓰는 동안 `svo_find --live` 는 못 엶 |
| 선반 서버 | OWLv2 선반 서버 (5577) 는 Jetson 에 안 띄움 → `--view` 는 깊이만으로 "키 큰 덩어리" 를 찾음 (옆 책상이 붙어 있으면 실패할 수 있음) |
| 라파→Jetson 키 | Jetson `authorized_keys` 의 라파 키가 `from="192.168.1.12"` 제한 → 대역이 0.x 가 되면 pick 의 Jetson 접속이 막힘 (그땐 `,192.168.0.12` 추가) |
| 리프트 경고 | `[lift] 경고: Pico mm/count 0.10980 ≠ 브리지 0.21574` = 높이 단위 불일치 (라파 리프트 설정). 주행·조그엔 영향 없음 |
