# 실험 데이터 사전

## 1. 기록 주체 구분

`trials.csv`는 trial당 한 행을 저장한다. 빈 값은 0이 아니라 미측정 또는 해당 없음을
뜻한다. 각 열의 기록 주체는 다음 네 종류로 구분한다.

- **자동**: ROS topic, TF, action 또는 실행 환경에서 runner가 읽는다.
- **계산**: 자동 기록값과 설정값으로 runner가 계산한다.
- **설정**: robot, scenario, experiment YAML에서 가져온다.
- **사용자**: 물리 측정이나 육안 관찰이 필요해 작업자가 입력한다.

## 2. 식별·진행 상태

| 열 | 주체 | 단위·형식 | 의미 |
|---|---|---|---|
| `schema_version` | 자동 | 정수 | CSV 스키마 버전 |
| `run_id` | 자동 | 문자열 | 한 번의 실험 실행을 구분하는 ID |
| `sequence_index` | 설정 | 정수 | 실제 실행 순서, 1부터 시작 |
| `timestamp_start` | 자동 | ISO 8601 | trial 시작 시각 |
| `timestamp_end` | 자동 | ISO 8601 | trial 종료·실패·중단 시각 |
| `robot_id` | 설정 | 문자열 | `turtle1`, `turtle2`, `turtle3` 등 로봇 이름 |
| `ros_domain_id` | 설정 | 정수 | 해당 로봇의 ROS Domain ID |
| `experiment_name` | 설정 | 문자열 | 실험 종류 이름 |
| `scenario_id` | 설정 | 문자열 | START·GOAL 시나리오 이름 |
| `condition_id` | 설정 | 문자열 | 분석용 조건 ID |
| `condition_label` | 설정 | 문자열 | 그래프에 표시할 조건명 |
| `trial` | 설정 | 정수 | 같은 조건 안에서의 반복 번호 |
| `status` | 자동 | 문자열 | `COMPLETED`, `FAILED`, `ERROR`, `INVALID_START`, `INTERRUPTED` 등 |
| `failure_reason` | 자동 | 문자열 | 실패·중단 사유, 정상 완료면 빈 값 |

## 3. 실제 적용 파라미터

아래 값은 YAML에 적힌 예정값이 아니라 parameter service로 다시 읽은 실제 값을
저장한다.

| 열 | 주체 | 단위 | 의미 |
|---|---|---:|---|
| `xy_goal_tolerance_m` | 자동 | m | Simple Goal Checker의 위치 허용 반경 |
| `yaw_goal_tolerance_rad` | 자동 | rad | Simple Goal Checker의 yaw 허용 오차 |
| `max_vel_x_mps` | 자동 | m/s | DWB `FollowPath.max_vel_x` 실제 값 |
| `max_speed_xy_mps` | 자동 | m/s | DWB `FollowPath.max_speed_xy` 실제 값 |
| `follow_path_xy_goal_tolerance_m` | 자동 | m | DWB RotateToGoal critic의 XY tolerance |
| `trans_stopped_velocity_mps` | 자동 | m/s | DWB가 정지로 간주하는 선속도 기준 |

## 4. START와 GOAL

| 열 | 주체 | 단위 | 의미 |
|---|---|---:|---|
| `start_expected_x_m` | 설정 | m | `/initialpose`로 보낼 START x |
| `start_expected_y_m` | 설정 | m | `/initialpose`로 보낼 START y |
| `start_expected_yaw_rad` | 설정 | rad | `/initialpose`로 보낼 START yaw |
| `start_amcl_x_m` | 자동 | m | 출발 직전 AMCL x |
| `start_amcl_y_m` | 자동 | m | 출발 직전 AMCL y |
| `start_amcl_yaw_rad` | 자동 | rad | 출발 직전 AMCL yaw |
| `start_amcl_cov_x` | 자동 | m² | 출발 직전 AMCL x 분산 |
| `start_amcl_cov_y` | 자동 | m² | 출발 직전 AMCL y 분산 |
| `start_amcl_cov_yaw` | 자동 | rad² | 출발 직전 AMCL yaw 분산 |
| `start_amcl_stamp_sec` | 자동 | ROS time s | 출발 AMCL 메시지의 원본 timestamp |
| `start_tf_x_m` | 자동 | m | 출발 직전 `map → base_footprint` TF x |
| `start_tf_y_m` | 자동 | m | 출발 직전 `map → base_footprint` TF y |
| `start_tf_yaw_rad` | 자동 | rad | 출발 직전 TF yaw |
| `goal_x_m` | 설정 | m | NavigateToPose GOAL x |
| `goal_y_m` | 설정 | m | NavigateToPose GOAL y |
| `goal_yaw_rad` | 설정 | rad | NavigateToPose GOAL yaw |

## 5. Action과 종료 추정값

| 열 | 주체 | 단위·형식 | 의미 |
|---|---|---|---|
| `nav_result` | 자동 | 문자열 | NavigateToPose 최종 상태, AMCL 시험은 `NOT_APPLICABLE` |
| `navigation_time_sec` | 자동 | s | Goal 전송부터 action 종료까지 걸린 시간 |
| `observed_cmd_vel_max_x_mps` | 계산 | m/s | 해당 trial에서 관측한 `abs(cmd_vel.linear.x)`의 최댓값 |
| `end_amcl_x_m` | 자동 | m | 종료 대기 후 AMCL x |
| `end_amcl_y_m` | 자동 | m | 종료 대기 후 AMCL y |
| `end_amcl_yaw_rad` | 자동 | rad | 종료 대기 후 AMCL yaw |
| `end_amcl_cov_x` | 자동 | m² | 종료 AMCL x 분산 |
| `end_amcl_cov_y` | 자동 | m² | 종료 AMCL y 분산 |
| `end_amcl_cov_yaw` | 자동 | rad² | 종료 AMCL yaw 분산 |
| `end_amcl_stamp_sec` | 자동 | ROS time s | 종료 AMCL 메시지의 원본 timestamp |
| `end_tf_x_m` | 자동 | m | 종료 시점 `map → base_footprint` TF x |
| `end_tf_y_m` | 자동 | m | 종료 시점 TF y |
| `end_tf_yaw_rad` | 자동 | rad | 종료 시점 TF yaw |

## 6. 자동 계산값

| 열 | 주체 | 단위 | 의미 |
|---|---|---:|---|
| `estimated_position_error_mm` | 계산 | mm | GOAL과 종료 AMCL pose 사이 평면 직선거리 |
| `estimated_dx_mm` | 계산 | mm | `end_amcl_x - goal_x`, 부호가 있는 x 차이 |
| `estimated_dy_mm` | 계산 | mm | `end_amcl_y - goal_y`, 부호가 있는 y 차이 |
| `estimated_yaw_error_deg` | 계산 | deg | GOAL과 종료 AMCL yaw 사이의 최소 각도 차이 |

이 값들은 ROS가 추정한 오차다. 철제자로 측정한 물리 오차를 대신하지 않는다.

## 7. 작업자 입력값

| 열 | 주체 | 단위·선택지 | 의미 |
|---|---|---|---|
| `physical_position_error_mm` | 사용자 | mm | GOAL 마킹 중심과 물리 `base_footprint` 기준점 사이 직선거리 |
| `stop_direction` | 사용자 | left/right/front/back/center/unknown | 목표 yaw 방향을 바라볼 때 정지점이 벗어난 방향 |
| `scan_alignment_ok` | 사용자 | true/false | RViz에서 scan과 map 정합이 정상인지 여부 |
| `overshoot_observed` | 사용자 | true/false | GOAL을 지나쳤다가 돌아오는 움직임 관찰 여부 |
| `controller_oscillation` | 사용자 | true/false | 제어 진동 또는 반복 재회전 관찰 여부 |
| `localization_jump_observed` | 사용자 | true/false | RViz pose가 불연속적으로 이동한 현상 관찰 여부 |
| `external_intervention` | 사용자 | true/false | 사람·장애물·수동 정지 등 외부 개입 여부 |
| `battery_percent` | 사용자 | % | trial 종료 시 확인한 배터리 잔량, 모르면 빈 값 |
| `measurement_method` | 설정 | 문자열 | 물리 거리 측정 도구, 기본값 `steel_ruler` |
| `measurement_resolution_mm` | 설정 | mm | 측정 도구 최소 눈금, 현재 1 mm |
| `photo_refs` | 사용자 | 문자열 | `photos/`에 둔 사진 파일명, 여러 개면 세미콜론으로 구분 |
| `notes` | 사용자 | 문자열 | 다른 열로 표현되지 않는 특이사항 |

`physical_position_error_mm`와 `battery_percent`에서 ENTER를 누르면 빈 값으로 남는다.
실측하지 않은 값을 0으로 입력하지 않는다.

## 8. `trace.csv` 필드

`trace.csv`는 분석용 고주파 자료이며 기본적으로 Git에 포함하지 않는다.

| 열 | 주체 | 단위·형식 | 의미 |
|---|---|---|---|
| `schema_version` | 자동 | 정수 | trace 스키마 버전 |
| `run_id` | 자동 | 문자열 | 상위 실행 ID |
| `sequence_index` | 자동 | 정수 | `trials.csv`와 연결되는 실행 순서 |
| `trial_elapsed_sec` | 자동 | s | 해당 trial의 trace 시작 후 경과 시간 |
| `source` | 자동 | 문자열 | `amcl`, `amcl_snapshot`, `cmd_vel` 중 자료 출처 |
| `source_stamp_sec` | 자동 | ROS time s | 원본 ROS 메시지 timestamp |
| `amcl_x_m`, `amcl_y_m` | 자동 | m | 해당 표본의 AMCL 위치 |
| `amcl_yaw_rad` | 자동 | rad | 해당 표본의 AMCL yaw |
| `amcl_cov_x`, `amcl_cov_y` | 자동 | m² | 해당 표본의 AMCL 위치 분산 |
| `amcl_cov_yaw` | 자동 | rad² | 해당 표본의 AMCL yaw 분산 |
| `cmd_vel_x_mps` | 자동 | m/s | 최종 `/cmd_vel` 선속도 명령 |
| `cmd_vel_yaw_rps` | 자동 | rad/s | 최종 `/cmd_vel` 각속도 명령 |

`source`와 무관한 열은 빈 값으로 저장한다. 동일한 `source_stamp_sec`가 반복되면 새
AMCL 계산 없이 같은 최신 표본을 다시 관측했을 가능성이 있으므로 별도로 구분한다.
