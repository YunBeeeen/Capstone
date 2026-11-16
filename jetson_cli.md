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

### RViz (선택)
- Jetson 모니터 앞: `rviz2 -d ~/capstone_ws/install/capstone_navigation/share/capstone_navigation/config/nav.rviz`
- 노트북 (랜선으로 iDLE): `export ROS_DOMAIN_ID=21 && source /opt/ros/humble/setup.bash && rviz2 -d ~/capstone/capstone_ws/install/capstone_navigation/share/capstone_navigation/config/nav.rviz`

---

## 끄기

Jetson: J5 → J3 (Nav2) → J2 (매핑이면 **저장 끝날 때까지 기다림**) → J1 → J4 순서로 Ctrl+C.
라파: 5 (리프트, 최하단까지 내려가며 꺼짐 — 기다림) → 4 → 3 → 2 → 1.

---

## 알아 둘 것

| 것 | 내용 |
|---|---|
| 경로 | 노트북 `~/capstone/capstone_ws/...` 대신 Jetson `~/capstone_ws/...` (`$NAV`). 지도 `~/maps/` (노트북 지도는 없음) |
| 인터넷 | Jetson 은 인터넷 없음. 설치가 더 필요하면 노트북을 랜선으로 꽂고 터널 (`ssh -N -R 127.0.0.1:1080 jetson`), apt 에 `-o Acquire::http::Proxy=socks5h://127.0.0.1:1080 -o Acquire::https::Proxy=socks5h://127.0.0.1:1080` |
| 게임패드 | Jetson 커널에 X-Box 드라이버를 직접 빌드해 넣음 (`~/xpad_build`, 부팅 때 자동). JetPack 업데이트하면 다시 빌드 |
| ZED 한 대 | 매핑·Nav2 가 ZED 를 쓰는 동안 물체 찾기 `svo_find --live` 는 ZED 를 못 엶 (ROS 영상으로 찾는 건 다음 작업) |
| 선반 글자 찾기 | OWLv2 선반 서버 (5577) 는 Jetson 에 아직 안 띄움 → place_mark/place_go 는 예전 방식으로 동작 |
| 리프트 경고 | `pad` 의 `[lift] 경고: Pico mm/count 0.10980 ≠ 브리지 0.21574` = 리프트 높이 단위 불일치 (라파 리프트 설정). 주행·조그엔 영향 없음, 높이 mm 표시가 틀림 |
| 패드 오류 | `source ~/nav_env.sh` 에서 `syntax error near '('` 가 나오면 옛 `pad` 별칭이 남은 터미널 — 고쳐 둠, 새 터미널에서 다시 |
