# 캡스톤 계획

> 설계 근거·수식·실험 세팅 자세히: Obsidian `03_Projects/Capstone/계층 기억 검색 & 위상 경로.md`
> 실행 명령: [`CLI.md`](CLI.md), 대회 당일 순서: [`CHALLENGE.md`](CHALLENGE.md)

## 최종 목표

"3층 행정실에서 서류 가져와" — 처음 간 건물을 매핑·기억하고, 층을 옮겨, 기억한 곳에서 물건을 가져온다.
상위 구조는 아래 다섯 가지를 **한 시스템으로** 합친 것.

1. **계층 기억** — 건물 → 층 → 방 → 물체 (지금 `places.yaml` 이 맨 아래 층)
2. **익숙함** — 로봇 자신의 방문·작업 성공 기록으로 만든 사전확률 (사람 설문 습관이 아님) + λ 실험 분석
3. **탐색 순서** — 후보를 P(있음) ÷ 경로 비용 순으로 방문, 도착 후 확인하고 기억 갱신 (`place_go` / `shelf_check`)
4. **다층 그래프 플래너** — 층·방·문·엘베 그래프를 직접 구현. 문은 열기 n회 실패 → 우회, 실패·막힘은 에지별로 기억(만료 있음). Nav2 는 같은 지도 안의 한 구간만
5. **실기** — Orin 온보드, 스워브 이동조작기, 실제 대학 건물

```
명령 → [의미 검색: 계층 attention(방 → 물체) + 익숙함 bias λ] → 후보 + 확률
     → [그래프 플래너: 층/방/문/엘베, 실패 기억] → 거쳐 갈 지점 순서
     → [Nav2: 같은 지도 안 구간] → [MPPI → 스워브]
     → 도착 후 확인 → 기억 갱신 → 없으면 다음 후보
```

- 검색 본선은 **학습 없음** (SigLIP 임베딩 + τ, λ). 간접 질의("서류" → 행정실)는 작은 LLM 이 graph 를 텍스트로 읽음.
- 학습된 attention 은 **조건부 확장**: 실험에서 학습 없는 방식이 특정 질의 유형에서 무너지고 LLM 이 Orin 에서 너무 느릴 때만. LLM 증류 우선.
- 같은 건물 시뮬로 학습하지 않음 (가중치가 위치를 외움 = 기억이 할 일). 우리 건물 Gazebo 는 시스템 시험(문 잠금·엘베 고장 재현)용.

## 단계

| # | 할 일 | 끝났다는 기준 |
|---|---|---|
| 0 | 기반: 구동 CAN 복구, depth 보정 후 주행 확인, 선반 기억 2단계 실기 | 아래 "오늘" 참고 |
| 1 | **지도 두 장을 문에서 갈아타기** — `lab_in1` ↔ `lab_robot1`, 문 앞 `lab_door` 를 양쪽 지도에 표시 = 포털 에지 (층 이동 리허설) | 갈아탄 뒤 재위치 성공률·시간·위치 오차 측정 |
| 2 | `places.yaml` → 계층 기억: `floor`, `room` 필드, 방문·성공 기록 자동 저장 | 기록이 쌓이는 것 확인 |
| 3 | 그래프 플래너: 포털·문 에지, 비용 = 과거 평균 시간, 막힘 기억, 문 n회 실패 → 우회 | Gazebo 문 잠금 시나리오에서 우회 |
| 4 | 학습 없는 계층 검색 + **λ 실험** | 결과 표·그래프 (아래 실험) |
| 5 | 확률 ÷ 비용 순서로 찾아다니기 (선반 기억 3단계와 합침) | 없으면 다음 후보로 가서 찾음 |
| 6 | (조건부) 학습된 attention | 4 의 실험 결과로 판단 |
| 7 | 엘베로 실제 층 이동 (버튼·탑승은 리더암 시연으로 배운 스킬) | 다른 층 목표 도착 |

나란히 진행: 전신 IL (리더암 시연 → ACT/π0 계열), 바퀴 odom + EKF 실기 비교, Orin 이전.

## λ 실험 세팅

| 칸 | 내용 |
|---|---|
| 고정 조건 | 우리 건물을 로봇이 실제로 만든 graph, 임베딩 모델 |
| 입력·정답 | 질의 + 정답 노드. (a) 명확 (b) 애매 — 습관이 정답 (c) 익숙하지 않은 곳이 정답 |
| 독립변수 | λ ∈ {0, 0.1, 0.3, 1, 3} (τ 함께) |
| 비교군 | 평평한 검색 / 계층 / 계층 + 익숙함 / LLM 에게 graph 텍스트 / (참고) 물건별로 개인화 여부를 정하는 PerSim 식 |
| 지표 | top-1, top-3, 찾을 때까지 이동 거리 |
| 분리 | 질의 60 % 로 λ 선택, 40 % 로 보고 |

## 오늘 (2026-11-13)

순서: CAN → 주행 확인 → 선반 (둘 다 `lab_in1` 재위치 상태에서) → 문 에지 (지도 바꿔 가며)

### ① 구동 CAN
라파에서. 조향 CAN 은 정상, 구동 CAN 만 ERROR-PASSIVE + CRC 에러 프레임 (노드를 다 꺼도 들어옴 → 하드웨어 쪽).
- [ ] `ip -details -statistics link show can_drive` — 상태, 에러 카운터
- [ ] `candump -e can_drive` — 에러 프레임 종류 (CRC = 누가 보내는데 깨짐)
- [ ] 전원 끄고 H–L 저항 ≈ 60 Ω (120 Ω 이면 종단 하나 빠짐, 40 Ω 이면 하나 더 붙음)
- [ ] 커넥터·꼬임선, 배터리 전압, 구동 드라이버 쪽 CAN 보드 설정(비트레이트가 `can_up.sh` 와 같은지)
- [ ] 복구 후 `./can_up.sh` → 구동 브리지 → `ros2 topic echo --once /wheel_state` (alive·ready)

### ② depth 보정 후 버벅거림
같은 길을 두 설정으로 왕복해 비교 (A/B).
```bash
# T2 재위치 — 보정 켬 (B). 비교용 A 는 cam_pitch_offset:=0 cam_roll_offset:=0
ros2 launch capstone_navigation slam.launch.py odom_source:=zed localization:=true database_path:=$HOME/maps/lab_in1.db cam_x:=0.167 cam_z:=0.675 cam_pitch_offset:=1.3 cam_roll_offset:=0.7
# T3 Nav2 — B 는 floor_clear 기본(켬), A 는 floor_clear:=false
ros2 launch capstone_navigation nav.launch.py odom_topic:=/zed_base_tf/odom cmd_vel_topic:=/nav/cmd_vel
# T6
python3 ~/capstone/capstone_ws/src/navigation/tools/zed_floor_level.py        # 먼저 바닥 평평한지
python3 ~/capstone/capstone_ws/src/navigation/tools/place_tour.py shelf_A desk_1 --loops 3
```
- [ ] 구간별 시간·실패 횟수 (`place_tour` 출력), 머뭇거린 횟수(눈으로)
- [ ] 사람이 지나간 뒤 RViz 에 핑크 자국이 남는지
- [ ] A 대비 B 가 시간·머뭇거림이 줄었으면 B 를 기본값으로 확정

### ③ 선반 살짝 옮겨도 잡는지 (실기)
`lab_in1` 의 `shelf_A` 는 11/12 저장이라 "보는 자리" 가 없음 → 먼저 추가.
```bash
cp ~/maps/lab_in1.places.yaml ~/maps/lab_in1.places.yaml.bak2
python3 ~/capstone/capstone_ws/src/navigation/tools/nav_goal_ahead.py --name shelf_A
# 0.8~1.2 m 뒤로 물러나서
python3 ~/capstone/capstone_ws/src/navigation/tools/place_mark.py shelf_A --view
python3 ~/capstone/capstone_ws/src/navigation/tools/shelf_check.py shelf_A -n 20     # 안 옮긴 상태 흩어짐
```
- [ ] 안 옮김: 흩어짐(표준편차)이 시뮬보다 얼마나 큰지 — 4 cm·3° 문턱이 실기에서도 맞는지
- [ ] 선반을 5~10 cm·몇 도 옮기고 줄자로 잼 → `shelf_check.py shelf_A -n 20` 값과 비교
- [ ] `place_go.py shelf_A --no-update` 로 설 자리에 서는지, 테이프로 정지 오차 측정 → 되면 `--no-update` 빼고 기억 갱신 확인

### ④ 문 기준 에지 연결 (단계 1 첫 시도)
1. `lab_in1` 재위치 상태에서 연구실 문 앞(복도를 보는 방향)에 세움 → `place_mark.py lab_door --kind door` (`lab_in1.places.yaml` 에 저장)
2. **움직이지 않고** T2 를 끄고 `database_path:=$HOME/maps/lab_robot1.db` 로 재위치 → `rtabmap_loc_monitor.py` 로 "위치 찾음" → `place_mark.py lab_door --kind door` (`lab_robot1.places.yaml` 에 저장). 복도 끝에도 목표 하나 `place_mark.py corridor_A --kind spot`
3. 시험: 다시 연구실 안에서 `lab_in1` 로 시작 → `nav_goal_ahead.py --name lab_door` → T2 를 `lab_robot1` 로 바꿈 → 재위치 → `nav_goal_ahead.py --name lab_door --dry` 의 직선 거리 = **갈아탈 때 위치 오차** → `nav_goal_ahead.py --name corridor_A`
   - Nav2 는 그대로 두고 지도가 바뀌는지 보고, 안 바뀌면 같이 재시작
- [ ] 재위치까지 걸린 시간, 성공/실패, 갈아탈 때 위치 오차 (3~5 회)
- [ ] 문 앞에서 어느 방향을 봐야 두 지도 다 재위치가 잘 되는지

## 꼭 읽을 논문

| 순서 | 논문 | 왜 | 읽을 곳 |
|---|---|---|---|
| 1 | **HOV-SG** (RSS 2024) | 층 → 방 → 물체 계층 + 층 사이 경로 그래프, 실기 다층. 목표 1·4 의 기준선 | 계층 만드는 법, cross-floor 그래프, 질의 방식 |
| 2 | **Affordance RAG** (RA-L/ICRA 2026) | 건물 → 구역 → 물체 계층 기억에서 자유 문장으로 물건 찾기, 이동조작, 학습 없음. 가장 최근·가장 가까움 | 계층 검색, 재순위, 실기 실험 |
| 3 | **SayPlan** (CoRL 2023) | 3층 36방 graph 를 접었다 펼치며 검색 + 경로 계획 + 실패 시 재계획. 목표 1·3·4 | semantic search, iterative replanning |
| 4 | **UcON** (ICRA 2026) | 사용자 습관이 물체 찾기 성능을 크게 바꾼다는 벤치마크. 목표 2 의 근거 | 습관 표현, Habit Retrieval Module, 결과 |
| 5 | **Embodied-RAG** (2024) | 위상 그래프를 계층으로 묶어 트리 검색 → 주행 전역 플래너. 구조가 거의 같음 | 의미 숲 만들기, 트리 검색, 주행 연결 |

같이 보면 좋은 것 (필요할 때)
- **PerSim** (2026) — 물건별로 개인화할지 말지 정함, 탐색 비용 = 방 방문 + 방 안 확인 → λ 실험 비교군·목표 3
- **HitMem** (2026) — 기억 노드 시간 감쇠 (`last_seen` 신뢰도)
- **Clio** (2024) — 과업에 따라 물체·영역 단위를 정함, 온보드 실시간, 코드 공개
- **ConceptGraphs** (ICRA 2024) — HOV-SG 의 바탕 (물체 graph 만들기)
- **엘베를 에지로 둔 다층 경로** (Applied Sciences 2023) — 목표 4 의 그래프 구조
- 이미 Obsidian 에 있음: FindAnything (온보드 open-vocab 지도), MoMa-LLM (동적 scene graph 로 물체 찾기)
- 학습할 때만: Scene Memory Transformer (2019), HOZ graph (2021), Task-driven graph attention (2023)
