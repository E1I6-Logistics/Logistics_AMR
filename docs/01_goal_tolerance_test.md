# XY Goal Tolerance 표준 시험 절차

## 1. 목적과 범위

이 시험은 Nav2 `SimpleGoalChecker`의 `goal_checker.xy_goal_tolerance`가 성공 판정과
실제 정지 위치 오차에 미치는 영향을 비교한다. 물리 GOAL 마킹 중심과 로봇의
`base_footprint` 기준점 사이 직선거리를 주 결과로 사용한다.

이 시험만으로 AMCL, odometry 또는 EKF 자체의 정확도가 개선됐다고 판단하지 않는다.
`xy_goal_tolerance`는 ROS가 추정한 `map` 좌표계 위치를 기준으로 하는 허용 반경이며,
물리 바닥에서 측정한 오차를 직접 보장하는 값이 아니다.

## 2. 통제 변수와 시험 조건

주요 변수는 `goal_checker.xy_goal_tolerance` 하나뿐이다. 다음 값은 시험 중 고정한다.

- `goal_checker.yaw_goal_tolerance`: `0.25 rad`
- DWB `FollowPath.xy_goal_tolerance`: 현재 소스값 `0.05 m`로 고정
- DWB `FollowPath.trans_stopped_velocity`: 현재 소스값 `0.25 m/s`로 고정
- GOAL pose, START 마킹과 초기화 pose
- map, Nav2 설정, AMCL·DWB·velocity smoother 설정
- 로봇, 적재물, 바닥 및 가능한 범위의 배터리 상태

| 조건 | `xy_goal_tolerance` | 최소 반복 횟수 |
|---|---:|---:|
| A | 0.25 m | 5회 |
| B | 0.10 m | 5회 |
| C | 0.05 m | 5회 |

조건은 `A-B-C-C-B-A...`처럼 교차하거나 무작위화하는 것이 원칙이다. 조건별로 묶어
실행하면 배터리, 시간 경과, 바닥 상태 및 localization 변화가 조건 효과와 섞일 수 있다.

재시험 전에는 `02_amcl_localization_test.md`의 물리 마킹–map–TF 정합 확인을 먼저
수행한다. 공통된 왼쪽 편향의 원인이 확인되지 않았으면 tolerance 비교 결과를 운영값
선정 근거로 사용하지 않는다.

## 3. 시험 환경 확인

실제 로봇에 맞는 Domain ID를 사용한다. 예를 들어 `turtle2`는 다음과 같다.

```bash
source /opt/ros/jazzy/setup.bash
cd logitle_ws
source install/local_setup.bash
export ROS_DOMAIN_ID=31
```

다른 AMR workspace를 같은 터미널에서 추가로 source하지 않는다. bringup과 Nav2가
실행된 뒤 다음 항목을 확인한다.

```bash
ros2 lifecycle get /amcl
ros2 lifecycle get /controller_server
ros2 action list | grep navigate_to_pose
ros2 param get /controller_server goal_checker.xy_goal_tolerance
ros2 param get /controller_server goal_checker.yaw_goal_tolerance
ros2 param get /controller_server FollowPath.xy_goal_tolerance
ros2 param get /controller_server FollowPath.trans_stopped_velocity
```

`/amcl`과 `/controller_server`가 `active`이고, scan-map 정합과
`map → odom → base_footprint` TF가 안정적인 상태에서만 시험한다.

## 4. 시험 전 기록

재현을 위해 다음을 시험 기록에 남긴다.

- 로봇 ID, `ROS_DOMAIN_ID`, 날짜와 작업자
- 저장소 Git commit, 사용한 launch와 params 파일
- map 이름, START/GOAL pose, 물리 마킹 방법
- 측정 도구와 최소 눈금
- 배터리, 적재물, 타이어 및 바닥 상태
- 시험 전 `/controller_server`와 `/amcl` parameter dump

현재 후보 좌표는 다음과 같다. 물리 마킹과 `map` 좌표의 대응을 먼저 검증해야 한다.

```yaml
start: {x: -0.0797, y: -0.0167, yaw: 0.1167}
goal:  {x:  1.7000, y:  0.1920, yaw: 0.1167}
```

## 5. 조건 적용

각 조건을 적용한 직후 반드시 다시 읽어 실제 반영값을 확인한다.

```bash
ros2 param set /controller_server goal_checker.xy_goal_tolerance 0.25
ros2 param get /controller_server goal_checker.xy_goal_tolerance

ros2 param set /controller_server goal_checker.xy_goal_tolerance 0.10
ros2 param get /controller_server goal_checker.xy_goal_tolerance

ros2 param set /controller_server goal_checker.xy_goal_tolerance 0.05
ros2 param get /controller_server goal_checker.xy_goal_tolerance
```

변경 또는 조회가 실패하면 해당 trial을 시작하지 않는다. 런타임 변경은 Nav2 재시작
시 사라질 수 있으므로 각 시험 시작 시 다시 확인한다.

`goal_checker.xy_goal_tolerance`는 action 성공 판정 반경이고,
`FollowPath.xy_goal_tolerance`는 DWB `RotateToGoal`의 최종 접근 구간에 쓰이는 별도
파라미터다. 두 값을 혼동하거나 같은 시험에서 함께 변경하지 않는다.

## 6. Trial 표준 절차

1. 적용할 `xy_goal_tolerance`와 고정된 `yaw_goal_tolerance`를 조회해 기록한다.
2. 로봇을 물리 START 마킹과 방향선에 맞춰 놓고 완전히 정지시킨다.
3. 동일한 START pose를 `/initialpose`로 발행한다. RViz `2D Pose Estimate`를 사용할
   경우에도 가능한 한 같은 pose와 covariance를 사용한다.
4. scan-map 정합, AMCL 안정 및 정지 상태를 확인한다.
5. 시작 시점 AMCL pose 또는 `map → base_footprint` TF를 기록한다.
6. 모든 trial에 아래와 같은 동일한 `NavigateToPose` goal을 전송한다.
7. action result와 주행 시간을 기록한다. 실패·취소 결과도 삭제하지 않는다.
8. `SUCCEEDED` 후 로봇을 건드리지 않고 3초간 기다린다.
9. 종료 시점 AMCL pose 또는 TF를 기록한다.
10. 물리 GOAL 마킹 중심에서 로봇의 물리 `base_footprint` 기준점까지 직선거리를
    mm 단위로 측정한다.
11. 정지 방향을 `left/right/front/back/center` 중 하나로 함께 기록한다. 가능하면
    목표 진행 방향 기준의 부호 있는 좌우 오차도 추가한다.
12. 로봇을 들어 START로 복귀하고 다음 trial을 준비한다.

Goal Tolerance 시험에서는 매회 로봇을 들어 START로 옮겨도 된다. 다만 IMU와 속도
추정이 안정된 뒤 initial pose를 발행하고 다음 주행을 시작한다. 이 방법은 odometry나
EKF drift 시험에는 사용하지 않는다.

### 6.1 고정 GOAL 전송 예시

```bash
ros2 action send_goal \
  /navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  "{
    pose: {
      header: {
        frame_id: 'map'
      },
      pose: {
        position: {
          x: 1.7000,
          y: 0.1920,
          z: 0.0
        },
        orientation: {
          x: 0.0,
          y: 0.0,
          z: 0.05832,
          w: 0.99830
        }
      }
    }
  }" \
  --feedback
```

표준 자동화에서는 goal에 현재 시각의 header stamp를 포함하고, 요청·feedback·result를
CSV와 함께 저장하는 방식을 권장한다.

## 7. 기록 필드

직접 입력하는 값과 자동 수집·계산할 값을 구분한다.

| 구분 | 필드 | 의미 |
|---|---|---|
| 직접 입력 | `robot_id`, `condition`, `trial` | 시험 대상과 반복 번호 |
| 직접 입력 | `physical_position_error_mm` | 물리 GOAL 중심과 로봇 기준점 사이 직선거리 |
| 직접 입력 | `stop_direction` | GOAL 기준 정지 방향의 범주 |
| 직접 입력 | `measurement_method`, `measurement_resolution_mm` | 측정 도구와 최소 눈금 |
| 직접 입력 | `start_scan_alignment_ok`, `notes` | 정합 여부와 특이사항 |
| 자동 수집 | `xy_goal_tolerance_m`, `yaw_goal_tolerance_rad` | controller server 실제 반영값 |
| 자동 수집 | `estimated_start_*`, `estimated_end_*` | AMCL 또는 TF의 시작·종료 pose |
| 자동 수집 | `goal_*`, `nav_result`, `navigation_time_sec` | goal, action 결과와 시간 |
| 자동 계산 | `estimated_position_error_mm` | ROS 추정 종료점과 GOAL 사이 거리 |

```text
estimated_position_error_mm = 1000 * hypot(estimated_end_x_m - goal_x_m,
                                             estimated_end_y_m - goal_y_m)
```

## 8. 결과 판정

조건별 성공률, 평균, 표본 표준편차, 중앙값, 95백분위, 최솟값과 최댓값을 계산한다.
더 작은 tolerance가 유효하다고 판단하려면 물리 오차가 반복적으로 줄고, 성공률·주행
시간·진동이 허용 범위에 있어야 한다.

물리 오차가 설정 tolerance보다 크더라도 곧바로 Goal Checker 오류로 판정하지 않는다.
Goal Checker는 AMCL이 추정한 위치를 사용하므로, 물리 마킹-`map` 대응 오차, AMCL
편향, TF 또는 센서 장착 오차가 있으면 `SUCCEEDED`와 물리 오차가 동시에 나타날 수 있다.

속도 시험에 사용할 tolerance는 이 재시험 결과로 하나만 선택한다. 선택값, 선택 이유,
성공률과 오차 통계를 `04_dwb_controller_test.md`의 시험 전 고정값에 기록한다. tolerance와
선속도를 동시에 바꿔 두 변수의 효과를 섞지 않는다.

## 9. 시험 종료

기본값을 복원하고 확인한다.

```bash
ros2 param set /controller_server goal_checker.xy_goal_tolerance 0.25
ros2 param get /controller_server goal_checker.xy_goal_tolerance
```

실측 결과를 검토하기 전에는 `burger.yaml`의 기본값을 변경하지 않는다.

반자동 실행과 CSV 기록은 `06_experiment_tooling.md`의 `navigation_sweep` 명령을
사용하고 `xy_tolerance_sweep.yaml`을 experiment config로 전달한다.
