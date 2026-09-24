# Navigation Accuracy 시험 계획

## 1. 목적과 완료 기준

이 문서는 TurtleBot3 Burger 실기체의 Navigation 도착 정확도를 개선하고,
변경 전후 차이를 물리 측정값으로 검증하기 위한 공통 시험 기준을 정의한다.

최종 목표 후보는 물리 GOAL과 실제 `base_footprint` 사이의 직선거리 오차
1 cm 이하이다. 다만 다음 두 조건을 만족하기 전에는 1 cm 달성을 판정하지 않는다.

1. 물리 GOAL과 `base_footprint` 기준점 사이 거리를 1 cm보다 충분히 작은 눈금으로
   반복 측정할 수 있어야 한다.
2. 같은 조건을 반복했을 때 평균뿐 아니라 분산과 95백분위도 허용 범위에 들어온다.

이 프로젝트의 완료 기준은 다음과 같다.

- Goal 판정, localization, odometry, controller, Route Server를 분리해 시험한다.
- 각 시험에서 주요 변수는 하나만 변경한다.
- 물리 측정값과 ROS 추정값을 구분해 저장한다.
- 변경 전후의 평균, 표준편차, 95백분위, 최대 오차, 성공률을 비교한다.
- 실기체에서 재현 가능한 실행 명령, 설정, 환경 조건을 함께 기록한다.

## 2. 시험 대상

| 항목 | 값 | 상태 |
|---|---|---|
| ROS 2 | Jazzy | 소스 및 실행 환경 확인 |
| Robot | TurtleBot3 Burger | 실행 환경 확인 |
| `turtle1` | `ROS_DOMAIN_ID=30` | 실기체 연결 확인 |
| `turtle2` | `ROS_DOMAIN_ID=31` | 2026-09-21 Goal Tolerance 시험에 사용 |
| `turtle3` | `ROS_DOMAIN_ID=33` | 사용자 제공, 미검증 |
| 1차 시험 로봇 | `turtle2` | Goal Tolerance 시험 완료 |
| 작업 저장소 | `/home/woozoo/dev/Logistics_AMR_logitle` | 확인 |
| 기준 브랜치 | `woozoo/logitle` | 확인 |

## 3. 2026-09-20 기준 확인된 런타임 구조

`turtle1`, `ROS_DOMAIN_ID=30`에서 실기체 bringup만 실행한 상태를 확인했다.
Nav2, AMCL, Route Server는 당시 실행되지 않았다.

```text
diff_drive_controller
  └─ /wheel/odom ─┐
                  ├─ ekf_filter_node ── /odom
turtlebot3_node   │                    └─ odom → base_footprint TF
  └─ /imu ────────┘

robot_state_publisher
  └─ base_footprint → base_link → sensor frames
```

확인값:

| 항목 | 확인 결과 |
|---|---|
| `/wheel/odom` publisher | `/diff_drive_controller` |
| `/wheel/odom` subscriber | `/ekf_filter_node` |
| `/odom` publisher | `/ekf_filter_node` |
| `/imu` publisher | `/turtlebot3_node` |
| `/imu` subscriber | `/ekf_filter_node` |
| diff-drive `use_imu` | `false` |
| diff-drive `publish_tf` | `false` |
| EKF `publish_tf` | `true` |
| EKF 입력 | `/wheel/odom`, `/imu` |
| EKF 융합 변수 | wheel `vx`, `vyaw`; IMU `vyaw` |
| EKF 설정 주기 | 30 Hz |
| 원격 측정 주기 | wheel 약 19.7 Hz, IMU 약 19.5 Hz, `/odom` 약 19.3 Hz |
| EKF 자체 진단 주기 | 약 30 Hz |
| `/cmd_vel` 형식 | `geometry_msgs/msg/TwistStamped` |
| namespace | 빈 문자열 |

원격 `ros2 topic hz` 결과는 DDS 전송 및 측정 프로세스의 영향을 받을 수 있으므로,
EKF 자체 진단의 내부 발행 주기와 구분한다.

### 3.1 현재 발견된 위험

`/wheel/odom`과 `/imu`의 covariance는 모두 0으로 발행된다. 정지 상태에서 확인한
EKF `/odom`의 pose covariance는 시간에 따라 매우 큰 값으로 증가했다. 예를 들어
`y` 분산이 약 `8.29e8`까지 증가했지만 diagnostics는 필터가 정상이라고 보고했다.

따라서 diagnostics의 정상 메시지만으로 EKF 품질을 판정하지 않는다. 별도 EKF
시험에서 covariance 설정, 정지 drift, 직선거리 오차, 회전 오차를 확인한다.

## 4. 시험 항목과 질문

| 순서 | 문서 | 핵심 질문 | 주요 변수 |
|---|---|---|---|
| 1 | `01_goal_tolerance_test.md` | Nav2는 언제 도착으로 판정하는가? | `xy_goal_tolerance` |
| 1-1 | `01-1_xy_goal_tolerance_test_result.md` | 2026-09-21 실측 결과는 어떠했는가? | 실제 수행·결과·한계 |
| 2 | `02_amcl_localization_test.md` | 물리 마킹·map·AMCL이 일관되게 대응하는가? | 먼저 측정, 필요할 때만 AMCL 설정 1개씩 |
| 3 | `03_ekf_odometry_test.md` | 실제 이동량과 회전을 정확히 추정하는가? | odometry 구성 |
| 4 | `04_dwb_controller_test.md` | 선속도를 낮추면 물리 도착 오차가 줄어드는가? | DWB 선속도 상한 |
| 5 | `05_route_server_integration_test.md` | Graph에서 생성한 path가 정상 실행되는가? | Route 통합 구성 |
| 도구 | `06_experiment_tooling.md` | 시험을 같은 방식으로 실행·기록·분석하는가? | `logitle_experiments` |

### 4.1 현재 확정한 다음 시험 순서

```text
물리 GOAL–map–TF 정합 확인
  → 현재 AMCL의 정지 안정성·동일 경로 반복성 측정
  → 편향 원인 수정 또는 영향 설명
  → XY Goal Tolerance 교차 순서 재시험
  → 사용할 tolerance 하나 선택
  → tolerance와 AMCL을 고정한 선속도 상한 시험
  → 필요할 때만 감속·DWB 또는 AMCL 파라미터 개별 시험
```

다음 단계로 넘어가는 조건은 해당 문서에 정의한다. 특히 왼쪽 탄착군의 공통 편향을
확인하지 않은 상태에서는 속도 시험을 시작하지 않는다. AMCL과 속도를 같은 시험에서
동시에 변경하지 않는다.

## 5. 공통 시험 원칙

### 5.1 변경 통제

- 한 시험에서는 주요 변수 하나만 변경한다.
- 코드, 파라미터 파일, 런타임 변경값과 Git 커밋을 기록한다.
- 배터리, 적재물, 타이어, 바닥, START/GOAL, map을 가능한 한 고정한다.
- 조건별 실행 순서는 한 방향으로만 진행하지 않고 교차하거나 무작위화한다.
- 각 조건은 최소 5회 반복한다. 편차가 크면 반복 횟수를 늘린다.
- 실패·취소·비상 정지도 삭제하지 않고 실패 사유와 함께 기록한다.

### 5.2 측정값 구분

다음 세 값은 서로 대체할 수 없다.

1. `estimated_error`: AMCL 또는 TF로 계산한 ROS 추정 오차
2. `physical_error`: 물리 GOAL 중심과 정지한 로봇 기준점 사이에서 직접 측정한 거리
3. `goal_tolerance`: Nav2가 성공을 판정하는 허용 범위

정확도 개선 판정에는 `physical_error`를 사용한다. 절대 물리 좌표는 측정하지 않고,
GOAL과 정지점 사이의 상대 직선거리만 측정한다. RViz, `/odom`, AMCL pose는 원인
분석용이며 물리 오차를 대신하지 않는다.

### 5.3 물리 측정 기준점

평면 위치 기준점은 `base_footprint`의 바닥 투영점이다. 현재 URDF에서
`base_footprint`와 `base_link`는 x, y가 같고 z만 0.010 m 차이 난다.
실기체에서 반복 측정할 수 있도록 차체 또는 측정 지그에 기준점을 표시해야 한다.

각 trial에서는 물리 GOAL 마킹 중심과 정지한 `base_footprint` 기준점 사이의 평면
직선거리를 직접 측정해 `physical_position_error_mm`으로 기록한다. 별도의 물리
x, y 좌표, 연속적인 전후·좌우 거리 성분, 물리 yaw는 측정하지 않는다. 다만 계통
편향을 놓치지 않도록 정지 방향을 `left/right/front/back/center` 범주로 함께 기록한다.

이 방식은 도착 위치 오차의 크기를 비교하기에는 충분하지만 다음은 알 수 없다.

- 목표를 지나쳤는지 덜 갔는지
- 좌우·전후 편향의 정확한 mm 성분
- 실제 차체 yaw가 목표 yaw와 얼마나 다른지

## 6. 1차 직선 시험 좌표 후보

다음 좌표는 이전 실험 설정에서 가져온 후보이며, 현재 map과 바닥 마킹의 대응을
실측하기 전에는 확정값으로 사용하지 않는다.

```yaml
start:
  x: -0.0797
  y: -0.0167
  yaw: 0.1167

goal:
  x: 1.7000
  y: 0.1920
  yaw: 0.1167
```

- yaw: 약 6.69도
- quaternion: `z=0.05832`, `w=0.99830`
- START와 GOAL 사이 직선거리: 약 1.8 m

## 7. 공통 실행 전 확인

아래 예시는 현재 시험 대상인 `turtle2` 기준이다. 다른 로봇에서는 해당 로봇의
Domain ID만 사용하며, 여러 로봇 Domain을 한 터미널에서 섞지 않는다.

```bash
source /opt/ros/jazzy/setup.bash
cd logitle_ws
source install/local_setup.bash
export ROS_DOMAIN_ID=31

ros2 node list
ros2 topic list
ros2 topic info -v /wheel/odom
ros2 topic info -v /odom
ros2 topic info -v /imu
ros2 topic info -v /tf
ros2 topic info -v /tf_static
```

실기체 운용 시에는 `logitle_ws`의 `install/local_setup.bash`를 사용한다. 다른 AMR
workspace의 setup 파일을 같은 터미널에서 추가로 source하지 않는다. 이후 launch도
이 환경을 유지한 터미널에서 실행한다.

Nav2 실행 후에는 다음을 추가 확인한다.

```bash
ros2 lifecycle get /amcl
ros2 lifecycle get /controller_server
ros2 lifecycle get /velocity_smoother
ros2 lifecycle get /collision_monitor
ros2 lifecycle get /route_server

ros2 param dump /amcl
ros2 param dump /controller_server
ros2 param dump /velocity_smoother
ros2 param dump /collision_monitor
ros2 param dump /route_server
```

## 8. 공통 기록 형식

각 trial은 최소한 다음 필드를 기록한다.

```text
timestamp
robot_id
ros_domain_id
git_commit
experiment
condition
trial
estimated_start_x_m
estimated_start_y_m
estimated_start_yaw_rad
start_scan_alignment_ok
goal_x_m
goal_y_m
goal_yaw_rad
nav_result
navigation_time_sec
estimated_end_x_m
estimated_end_y_m
estimated_end_yaw_rad
physical_position_error_mm
stop_direction
measurement_method
measurement_resolution_mm
battery_percent
notes
```

조건별 요약에는 다음 값을 포함한다.

- 성공 횟수와 성공률
- 평균, 표준편차
- 중앙값
- 95백분위
- 최대 오차
- 평균 주행 시간

## 9. 안전 및 중단 기준

- 비상 정지 수단과 담당자를 시험 전에 정한다.
- 사람이나 이동 장애물이 시험 구역에 들어오면 즉시 중단한다.
- localization jump, TF 단절, 센서 timeout, 비정상 covariance 증가가 보이면 중단한다.
- 제어 진동, 반복 회전, 경로 이탈, 충돌 위험이 발생하면 해당 trial을 실패로 기록한다.
- 원인을 확인하기 전에는 같은 조건을 반복 실행하지 않는다.

## 10. 아직 필요한 확정 정보

- 실기체와 Main PC에서 실제 사용하는 전체 실행 명령
- Nav2 실행 시 실제 로드되는 launch와 params 파일
- 현재 map과 물리 START/GOAL 마킹의 대응
- GOAL과 `base_footprint` 사이 거리 측정 도구: 철제자, 최소 눈금 1 mm
- `base_footprint` 물리 기준점 표시 방법의 반복 재현성
- 배터리·적재물·바닥 상태의 허용 범위
- keepout mask와 Route graph의 실제 사용 여부
