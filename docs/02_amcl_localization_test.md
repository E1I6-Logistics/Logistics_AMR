# AMCL 기준점 정합·반복성 시험

## 1. 목적

이 시험은 속도 설정을 변경하기 전에 다음 질문에 답하기 위한 선행 시험이다.

1. 물리 START/GOAL 마킹과 `map` 좌표가 일관되게 대응하는가?
2. AMCL 추정 위치가 정지·반복 주행에서 안정적인가?
3. 기존 Goal Tolerance 시험의 왼쪽 탄착군이 localization 계통 편향과 관련 있는가?

AMCL 파라미터를 바로 조정하는 시험이 아니다. 먼저 현재 설정을 기준선으로 측정하고,
문제 유형이 확인된 경우에만 관련 파라미터를 하나씩 변경한다.

## 2. 속도 시험보다 먼저 하는 이유

`xy_goal_tolerance`와 DWB는 AMCL이 보고하는 `map` 좌표를 기준으로 동작한다. 물리
GOAL과 AMCL 사이에 공통된 좌우 편향이 있으면 속도를 낮춰도 같은 위치를 목표로
정지할 수 있다. 이 상태에서 속도까지 바꾸면 localization 편향과 제어 효과를 분리할
수 없으므로, 아래 판정 기준을 통과한 뒤 속도 시험으로 진행한다.

## 3. 현재 소스 설정

현재 저장소의 `turtlebot3/turtlebot3_navigation2/param/burger.yaml`에는 다음 값이 있다.
실제로 로드된 값은 시험 직전 `ros2 param get`과 `ros2 param dump`로 다시 확인한다.

| 구분 | 현재 소스 값 |
|---|---:|
| motion noise `alpha1`~`alpha5` | 각각 `0.2` |
| `max_particles` / `min_particles` | `2000` / `500` |
| `max_beams` | `60` |
| `laser_model_type` | `likelihood_field` |
| `sigma_hit` | `0.2` |
| `laser_likelihood_max_dist` | `2.0` |
| `update_min_d` | `0.25 m` |
| `update_min_a` | `0.2 rad` |
| `base_frame_id` | `base_footprint` |
| `odom_frame_id` / `global_frame_id` | `odom` / `map` |

`update_min_d`와 `update_min_a`보다 작은 움직임에서는 AMCL 측정 업데이트가 일어나지
않을 수 있다. 따라서 initial pose를 발행한 직후의 `/amcl_pose`만 보고 AMCL 정확도를
판정하지 않는다. 정지 중 동일한 TF가 반복 관측되는 것도 새 localization 계산이
아닐 수 있으므로 메시지 또는 TF timestamp를 함께 확인한다.

## 4. 공통 조건

| 항목 | 값 |
|---|---|
| 1차 시험 로봇 | `turtle2` |
| `ROS_DOMAIN_ID` | `31` |
| START | `x=-0.0797, y=-0.0167, yaw=0.1167 rad` 후보 |
| GOAL | `x=1.7000, y=0.1920, yaw=0.1167 rad` 후보 |
| 측정 기준 | 물리 마킹 중심 ↔ 물리 `base_footprint` 기준점 |
| 측정 도구 | 철제자, 최소 눈금 1 mm |
| Nav2 설정 | 시험 중 고정 |
| 속도 설정 | 현재 기준값으로 고정 |

START/GOAL 좌표는 바닥 마킹과의 대응을 이 시험에서 확인한 후 확정한다.

## 5. 시험 전 확인과 저장

```bash
source /opt/ros/jazzy/setup.bash
cd logitle_ws
source install/local_setup.bash
export ROS_DOMAIN_ID=31

ros2 lifecycle get /amcl
ros2 lifecycle get /controller_server
ros2 topic info -v /amcl_pose
ros2 topic info -v /scan
ros2 topic info -v /tf
ros2 param dump /amcl
```

추가로 다음을 기록한다.

- 실제 launch 명령과 `params_file` 경로
- map yaml과 이미지 파일의 이름 또는 해시
- Git commit
- LiDAR와 `base_footprint`의 물리 기준점 표시 사진
- 바닥 START/GOAL 마킹, 진행 방향, 좌우 기준이 보이는 사진
- 배터리, 적재물, 장애물 및 바닥 상태

## 6. 시험 A — 물리 마킹과 map/TF 정합 확인

이 시험은 AMCL 절대 정확도를 계산하는 시험이 아니라 물리 마킹, 지도, TF 및 센서
장착 방향의 공통 불일치를 찾는 시험이다.

1차 시험은 왼쪽 탄착군이 관찰된 GOAL에서 다음 절차를 3회 반복한다. GOAL 결과가
정상이고 지도 구간별 편향을 추가 확인해야 할 때만 START 또는 중간 기준점을 확장한다.

1. 로봇을 들어 물리 기준점이 바닥 마킹 중심에 오도록 놓는다.
2. 차체 방향선을 마킹된 목표 yaw에 맞춘다.
3. 해당 마킹의 좌표를 동일한 covariance로 initial pose에 발행한다.
4. RViz에서 map, LiDAR scan, footprint 및 TF 방향을 확인한다.
5. scan이 벽·기둥 등 지도 구조물과 어느 방향으로 어긋나는지 기록한다.
6. 로봇을 천천히 `update_min_a=0.2 rad`보다 크게 회전시켜 scan update가 발생하게
   한 뒤, 원래 방향선으로 돌아와 정지한다.
7. 3초 후 `/amcl_pose`와 `map → base_footprint`를 기록한다.
8. `left/right/front/back`, localization jump, scan-map 정합 여부를 기록한다.

initial pose를 정답 좌표로 넣었기 때문에 이 결과만으로 AMCL이 정확하다고 판정하지
않는다. 반대로 반복해서 같은 방향으로 scan이 어긋나면 다음을 우선 확인한다.

- 바닥 마킹과 map 좌표의 대응
- map 작성 후 환경 구조물의 변경
- `base_footprint → base_scan` TF
- LiDAR 장착 위치와 yaw
- map 해상도·origin 및 사용 중인 map 파일

## 7. 시험 B — 정지 상태 안정성

시험 A에서 정합된 상태를 만든 뒤 로봇을 건드리지 않고 30초간 관측한다.

1. `/amcl_pose`와 `map → base_footprint`를 1 Hz로 기록한다.
2. pose와 covariance, 각 메시지·TF timestamp를 함께 저장한다.
3. 새 timestamp가 있는 표본만 AMCL 갱신 표본으로 별도 표시한다.
4. 위치 jump, yaw jump 또는 covariance 급증 여부를 기록한다.

계산값:

- `x`, `y`, yaw의 최솟값·최댓값과 peak-to-peak 범위
- 위치 중심에서 각 표본까지의 최대 거리
- covariance의 시작값·종료값·최댓값
- 실제로 갱신된 표본 수

동일한 오래된 transform이 반복된 경우에는 정지 안정성이 좋다고 결론 내리지 않고
`no_new_amcl_update`로 기록한다.

## 8. 시험 C — 동일 경로 종료점 반복성

이 시험이 기존 왼쪽 탄착군과 AMCL 추정값을 연결하는 핵심 시험이다. AMCL 및 속도
파라미터는 변경하지 않고 최소 5회 반복한다.

1. 로봇을 물리 START에 놓고 동일한 START initial pose를 발행한다.
2. scan-map 정합과 정지 상태를 확인한다.
3. 시작 `/amcl_pose`와 TF를 저장한다.
4. 동일한 GOAL로 `NavigateToPose` action을 전송한다.
5. `SUCCEEDED` 후 3초간 기다린다.
6. 종료 `/amcl_pose`, TF, covariance와 action 시간을 저장한다.
7. GOAL 중심과 물리 `base_footprint` 기준점 사이 직선거리를 mm로 측정한다.
8. 정지 방향을 `left/right/front/back/center`로 기록한다.

자동 계산값:

```text
estimated_error_mm = 1000 * hypot(amcl_end_x_m - goal_x_m,
                                  amcl_end_y_m - goal_y_m)
estimated_dx_mm = 1000 * (amcl_end_x_m - goal_x_m)
estimated_dy_mm = 1000 * (amcl_end_y_m - goal_y_m)
```

물리 오차는 직선거리만으로 유지하되 방향 범주를 반드시 함께 적는다. 물리 x/y
성분을 측정하지 않으므로 `estimated_dx/dy`와 물리 좌우 오차를 같은 값으로 취급하지 않는다.

## 9. 기록 필드

```text
timestamp
robot_id
ros_domain_id
map_id
git_commit
test_stage
trial
reference_point
initial_pose_x_m
initial_pose_y_m
initial_pose_yaw_rad
initial_pose_covariance
scan_alignment_ok
amcl_x_m
amcl_y_m
amcl_yaw_rad
amcl_cov_x
amcl_cov_y
amcl_cov_yaw
tf_x_m
tf_y_m
tf_yaw_rad
source_timestamp
goal_x_m
goal_y_m
goal_yaw_rad
nav_result
navigation_time_sec
physical_position_error_mm
stop_direction
measurement_method
measurement_resolution_mm
localization_jump
notes
```

## 10. 판정과 다음 단계

다음 중 하나라도 있으면 속도 시험을 보류하고 원인을 먼저 확인한다.

- START 또는 GOAL에서 scan-map 정합이 반복해서 같은 방향으로 어긋난다.
- `map → odom → base_footprint` TF가 끊기거나 큰 localization jump가 발생한다.
- AMCL 추정 종료점은 GOAL 부근인데 물리 정지점만 반복해서 같은 방향으로 벗어난다.
- initial pose·map·환경을 고정했는데 반복 종료점 분산이 속도 조건 간 예상 차이와
  구분하기 어려울 정도로 크다.
- 왼쪽 탄착군의 원인이 물리 마킹, map 또는 센서 TF 문제로 확인됐다.

문제가 없거나 원인을 수정한 뒤에는 `01_goal_tolerance_test.md`로 XY Goal Tolerance를
교차 순서로 재시험한다. 그 결과로 사용할 tolerance 하나를 정한 다음
`04_dwb_controller_test.md`의 속도 시험을 수행한다.

AMCL 조정이 필요하면 문제 유형과 직접 관련된 묶음만 한 번에 하나씩 변경한다.

| 관찰된 문제 | 우선 검토 항목 |
|---|---|
| 주행할수록 pose가 퍼짐 | odometry/EKF, `alpha1`~`alpha5` |
| scan이 구조물에 넓게 분포 | TF·장착, `sigma_hit`, likelihood field 설정 |
| 초기 위치 수렴 실패 | particle 수, 초기 covariance, scan/map 대응 |
| pose update가 지나치게 드묾 | `update_min_d`, `update_min_a` |

여러 AMCL 파라미터를 동시에 바꾸지 않으며, 변경 후 시험 A~C의 관련 구간을 동일한
조건으로 반복한다.

시험 A와 B의 반자동 실행 및 기록은 `06_experiment_tooling.md`의 `amcl_validation`
명령을 사용한다. 동일 경로 종료점 반복성인 시험 C는 `navigation_sweep`으로 수행한다.
