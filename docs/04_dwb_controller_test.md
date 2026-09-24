# DWB 선속도 상한 시험

## 1. 목적과 선행 조건

이 시험은 localization과 Goal Tolerance를 고정한 상태에서 선속도 상한을 낮추면
물리 도착 오차, 정지 안정성 및 주행 시간이 어떻게 바뀌는지 확인한다.

다음 조건을 만족하기 전에는 시작하지 않는다.

1. `02_amcl_localization_test.md`의 map/TF 정합과 AMCL 반복성 검토가 끝났다.
2. 왼쪽 탄착군의 공통 좌표 편향이 해결됐거나 영향이 설명됐다.
3. `01_goal_tolerance_test.md`를 재시험해 속도 시험에 고정할
   `xy_goal_tolerance` 하나를 선택했다.
4. 실제 로드된 launch와 params 파일을 확인했다.

2026-09-21 결과에서 0.10 m가 가장 작은 평균을 보였지만 표본 수와 시험 순서의
한계가 있으므로, AMCL 확인 전에는 최종 운영값으로 확정하지 않는다.

## 2. 실제 소스 설정과 변경 변수

현재 저장소의 `turtlebot3/turtlebot3_navigation2/param/burger.yaml`에는 다음 값이 있다.

```yaml
controller_server:
  ros__parameters:
    FollowPath:
      max_vel_x: 0.3
      max_speed_xy: 0.3
      acc_lim_x: 3.0
      decel_lim_x: -2.5
      xy_goal_tolerance: 0.05
      trans_stopped_velocity: 0.25

velocity_smoother:
  ros__parameters:
    max_velocity: [0.5, 0.0, 2.5]
    max_accel: [2.5, 0.0, 3.2]
    max_decel: [-2.5, 0.0, -3.2]
```

DWB의 선속도는 `FollowPath.max_vel_x`와 `FollowPath.max_speed_xy` 중 더 작은 제한의
영향을 받으므로 두 값을 같은 값으로 설정한다. 이는 하나의 실험 변수인
`linear_speed_limit`를 구현하기 위한 묶음 변경이다.

1차 속도 시험에서는 다음을 변경하지 않는다.

- `acc_lim_x`, `decel_lim_x`
- velocity smoother의 속도·가감속 제한
- 각속도 제한
- DWB critics와 `sim_time`
- DWB `xy_goal_tolerance`와 `trans_stopped_velocity`
- AMCL 파라미터
- 선택한 `xy_goal_tolerance`와 `yaw_goal_tolerance`

## 3. 시험 조건

| 조건 | `FollowPath.max_vel_x` | `FollowPath.max_speed_xy` | 반복 |
|---|---:|---:|---:|
| V30 기준 | 0.30 m/s | 0.30 m/s | 5회 |
| V20 중속 | 0.20 m/s | 0.20 m/s | 5회 |
| V10 저속 | 0.10 m/s | 0.10 m/s | 5회 |

권장 15회 실행 순서는 다음과 같다.

```text
V30, V20, V10, V10, V20, V30, V20, V30, V10, V10, V30, V20, V30, V10, V20
```

각 조건이 시간대별로 섞이도록 한 순서다. 실행 전에 순서를 고정해 기록하고, 실패가
발생해도 순번을 삭제하거나 임의로 대체하지 않는다.

## 4. 시험 전 고정값 기록

```bash
source /opt/ros/jazzy/setup.bash
cd logitle_ws
source install/local_setup.bash
export ROS_DOMAIN_ID=31

ros2 lifecycle get /amcl
ros2 lifecycle get /controller_server
ros2 lifecycle get /velocity_smoother
ros2 lifecycle get /collision_monitor

ros2 param get /controller_server goal_checker.xy_goal_tolerance
ros2 param get /controller_server goal_checker.yaw_goal_tolerance
ros2 param get /controller_server FollowPath.max_vel_x
ros2 param get /controller_server FollowPath.max_speed_xy
ros2 param get /controller_server FollowPath.acc_lim_x
ros2 param get /controller_server FollowPath.decel_lim_x
ros2 param get /controller_server FollowPath.xy_goal_tolerance
ros2 param get /controller_server FollowPath.trans_stopped_velocity
ros2 param get /velocity_smoother max_velocity
ros2 param get /velocity_smoother max_accel
ros2 param get /velocity_smoother max_decel
```

추가 기록:

- 선택한 `xy_goal_tolerance`와 선택 근거 문서
- map, START/GOAL, Git commit, launch와 params 경로
- 배터리, 적재물, 타이어와 바닥 상태
- 물리 기준점과 마킹 사진

`trans_stopped_velocity=0.25 m/s`는 V20과 V10의 속도 상한보다 크다. DWB의
`RotateToGoal`은 목표 근처에서 이 값을 선형 이동이 충분히 작아졌는지 판단하는 데
사용한다. 따라서 이 값은 1차 속도 시험에서 고정하되, V20/V10에서 최종 접근 방식이나
재회전이 달라지는지 반드시 기록한다. 이상 동작이 확인되면 속도 효과와 섞어 결론
내리지 않고 `trans_stopped_velocity`를 별도 후속 시험 변수로 다룬다.

## 5. 속도 조건 적용

로봇이 정지하고 action이 실행 중이지 않을 때 두 값을 적용한다. 두 명령이 모두
성공하고 조회값이 일치할 때만 trial을 시작한다.

V30:

```bash
ros2 param set /controller_server FollowPath.max_vel_x 0.30
ros2 param set /controller_server FollowPath.max_speed_xy 0.30
ros2 param get /controller_server FollowPath.max_vel_x
ros2 param get /controller_server FollowPath.max_speed_xy
```

V20:

```bash
ros2 param set /controller_server FollowPath.max_vel_x 0.20
ros2 param set /controller_server FollowPath.max_speed_xy 0.20
ros2 param get /controller_server FollowPath.max_vel_x
ros2 param get /controller_server FollowPath.max_speed_xy
```

V10:

```bash
ros2 param set /controller_server FollowPath.max_vel_x 0.10
ros2 param set /controller_server FollowPath.max_speed_xy 0.10
ros2 param get /controller_server FollowPath.max_vel_x
ros2 param get /controller_server FollowPath.max_speed_xy
```

한 값만 변경되거나 `param set`이 실패하면 원래 값으로 복구하고 해당 trial을 시작하지
않는다. 런타임 변경은 Nav2 재시작 시 사라질 수 있으므로 재시작 후 다시 확인한다.

## 6. Trial 실행 절차

1. 계획표에서 이번 trial의 속도 조건을 확인한다.
2. 로봇이 정지한 상태에서 두 속도 파라미터를 적용하고 조회한다.
3. 로봇을 물리 START에 놓고 동일한 START initial pose를 발행한다.
4. scan-map 정합, AMCL 안정 및 정지 상태를 확인한다.
5. 시작 AMCL pose, TF, covariance와 적용 파라미터를 저장한다.
6. 동일한 GOAL로 `NavigateToPose` action을 전송한다.
7. action feedback, result, 주행 시간과 `/cmd_vel`을 저장한다.
8. `SUCCEEDED` 후 로봇을 건드리지 않고 3초간 기다린다.
9. 종료 AMCL pose, TF와 covariance를 저장한다.
10. GOAL 중심과 물리 `base_footprint` 기준점 사이 직선거리를 mm로 측정한다.
11. `left/right/front/back/center`, overshoot, 진동, 재회전 및 특이사항을 기록한다.
12. 로봇을 START로 옮기고 IMU·속도 추정이 안정된 뒤 다음 trial을 시작한다.

`/cmd_vel`은 실제 구동기로 전달되는 최종 명령인지 publisher 연결을 시험 전에
`ros2 topic info -v /cmd_vel`로 확인한다. 현재 소스 설정에서는 collision monitor가
`cmd_vel_smoothed`를 입력받아 `cmd_vel`을 출력하도록 되어 있다.

## 7. 기록 필드

```text
timestamp
robot_id
ros_domain_id
git_commit
condition
trial
xy_goal_tolerance_m
yaw_goal_tolerance_rad
max_vel_x_mps
max_speed_xy_mps
acc_lim_x_mps2
decel_lim_x_mps2
follow_path_xy_goal_tolerance_m
trans_stopped_velocity_mps
velocity_smoother_max_x_mps
estimated_start_x_m
estimated_start_y_m
estimated_start_yaw_rad
goal_x_m
goal_y_m
goal_yaw_rad
nav_result
navigation_time_sec
observed_cmd_vel_max_x_mps
estimated_end_x_m
estimated_end_y_m
estimated_end_yaw_rad
estimated_position_error_mm
physical_position_error_mm
stop_direction
overshoot_observed
controller_oscillation
localization_jump
measurement_method
measurement_resolution_mm
battery_percent
notes
```

## 8. 결과 계산과 판정

조건별로 다음을 계산한다.

- action 성공률
- 물리 오차의 평균, 중앙값, 표본 표준편차, 95백분위, 최댓값
- AMCL 추정 오차의 같은 통계
- 평균·최대 주행 시간
- 실제 관측된 최대 `/cmd_vel.twist.linear.x` (`TwistStamped` 기준)
- overshoot, 재회전 및 진동 발생 횟수
- 정지 방향 분포

속도를 낮추는 것이 유효하려면 다음을 함께 만족해야 한다.

1. 물리 오차 또는 산포가 반복적으로 감소한다.
2. action 성공률이 악화되지 않는다.
3. 주행 시간 증가가 운영상 허용 범위 안이다.
4. 진동·재회전·비정상적으로 긴 최종 접근이 늘지 않는다.
5. 왼쪽 탄착군과 같은 공통 편향을 속도 효과로 잘못 해석하지 않는다.

0.10 m/s가 가장 느리다는 이유만으로 선택하지 않는다. 오차 개선이 측정 반복성보다
작거나 0.20 m/s와 실질적으로 같으면 주행 시간이 더 짧은 조건을 우선 검토한다.

## 9. 후속 시험

선속도 상한을 낮춰도 오차가 줄지 않지만 overshoot나 급정지가 관찰되면 그다음에
`decel_lim_x`, `trans_stopped_velocity`, DWB의 최종 접근 동작 또는 velocity smoother를
각각 별도 시험한다.
1차 속도 시험과 가감속 시험을 동시에 수행하지 않는다.

## 10. 시험 종료 및 복구

현재 소스 기준값으로 복구하고 확인한다.

```bash
ros2 param set /controller_server FollowPath.max_vel_x 0.30
ros2 param set /controller_server FollowPath.max_speed_xy 0.30
ros2 param get /controller_server FollowPath.max_vel_x
ros2 param get /controller_server FollowPath.max_speed_xy
```

결과를 검토하기 전에는 `burger.yaml`의 기본값을 변경하지 않는다.

반자동 실행과 `/cmd_vel` trace 기록은 `06_experiment_tooling.md`의
`navigation_sweep` 명령을 사용하고 `speed_sweep.yaml`을 experiment config로 전달한다.
