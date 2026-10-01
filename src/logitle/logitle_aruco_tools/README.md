# logitle_aruco_tools

TurtleBot3에서 4 cm ArUco 마커(ID 24, 25 / `DICT_5X5_50`)를 이용해 카메라 기준 pose를 발행하고, 마커 기반 자동정렬과 `map -> odom` TF 보정 dry-run/publish를 수행하는 ROS 2 Python 패키지입니다.

## Build

```bash
cd ~/turtlebot3_E1i6
colcon build --packages-select logitle_aruco_msgs logitle_aruco_tools
source install/setup.bash
```

## Marker Map Check

```bash
ros2 run logitle_aruco_tools logitle_marker_map \
  src/logitle/logitle_aruco_tools/config/logitle_marker_map.yaml
```

현재 `config/logitle_marker_map.yaml`에는 실측 계산값 기준 ID 24, 25만 들어 있습니다.
두 마커 모두 검은 외곽이 한쪽 축 39 mm, 다른 축 40 mm로 측정되어 현재 `size`는 평균값인 `0.0395`를 사용합니다. 정밀 TF 보정 전에는 40 mm 정사각형으로 재출력하는 편이 가장 좋습니다.

## Pose Topics For ID 24 And 25

```bash
ros2 launch logitle_aruco_tools logitle_aruco_pose_viewer.launch.py
```

기본값은 `marker_ids:=24,25`이며, 각 마커가 보이면 다음 토픽을 발행합니다.

```text
/aruco/id24/pose_camera
/aruco/id25/pose_camera
```

## Auto Align

기본은 dry-run입니다.

```bash
ros2 launch logitle_aruco_tools logitle_aruco_auto_align.launch.py
```

실제 `/cmd_vel` 발행은 바퀴를 든 상태에서 먼저 확인한 뒤 켭니다.

```bash
ros2 launch logitle_aruco_tools logitle_aruco_auto_align.launch.py enable_motion:=true
```

## Pickup Alignment And Pose Correction

5번/6번 노드처럼 로봇팔 앞 작업 위치에 도착한 뒤, 마커 기준으로 미세 정렬하고 이어서 pose correction까지 수행할 때 사용합니다.

```bash
ros2 launch logitle_aruco_tools logitle_aruco_align_and_correct.launch.py
```

관제 호출 예시:

```bash
ros2 action send_goal /aruco_align_and_correct logitle_aruco_msgs/action/AlignAndCorrectWithAruco \
"{marker_id: 24, apply_correction: true}" \
--feedback
```

동작 순서:

```text
Nav2로 5번/6번 노드 근처 이동
-> ArUco 마커 기준으로 전진/후진 및 회전 미세 정렬
-> CorrectPoseWithAruco 호출
-> /initialpose로 AMCL/Nav2 위치 보정
```

기본 정렬 목표는 `target_z=0.32m`, `x_tolerance=0.025m`, `z_tolerance=0.035m`입니다. 마커가 보이는 범위 안에서만 정렬할 수 있으며, pose가 끊기면 즉시 정지합니다.

## Pose Correction Action

관제 연동은 `/aruco_correct_pose` action을 사용합니다. 이 Action은 도킹이나 이동 제어를 하지 않고, ArUco 관측으로 현재 `map -> base_link` pose를 계산합니다.

Action 서버는 기본적으로 대기 중에는 카메라 이미지 처리를 하지 않습니다. 관제에서 goal이 들어온 동안에만 `/camera/image_raw`, `/camera/camera_info`를 구독하고 ArUco 검출/pose 계산을 수행한 뒤, goal이 끝나면 구독을 해제합니다.

기본 실행은 AMCL/Nav2와 같이 쓰기 안전한 모드입니다. `publish_tf: true` goal이 성공하면 직접 `map -> odom` TF를 발행하지 않고 `/initialpose`를 publish해서 AMCL/Nav2가 계속 `map -> odom`의 단일 publisher로 남게 합니다.

```bash
ros2 launch logitle_aruco_tools logitle_aruco_pose_corrector_action.launch.py
```

이미 별도 pose publisher를 켜서 `/aruco/id{marker_id}/pose_camera`를 받을 때만 다음처럼 topic 모드로 전환합니다.

```bash
ros2 launch logitle_aruco_tools logitle_aruco_pose_corrector_action.launch.py \
  pose_source:=topic
```

호출 예시:

계산만 확인하는 dry-run:

```bash
ros2 action send_goal /aruco_correct_pose logitle_aruco_msgs/action/CorrectPoseWithAruco \
"{marker_id: 24, publish_tf: false}" \
--feedback
```

보정 적용:

```bash
ros2 action send_goal /aruco_correct_pose logitle_aruco_msgs/action/CorrectPoseWithAruco \
"{marker_id: 24, publish_tf: true}" \
--feedback
```

Goal에 생략된 값은 Action 기본값을 사용합니다. 현장 기준 yaw 검사를 끄거나 값을 바꾸려면 `check_yaw`, `expected_base_yaw_deg`, `yaw_tolerance_deg`를 명시합니다.
`publish_tf: false`는 dry-run이며 계산 결과만 Result로 반환합니다.

서버는 accepted sample의 흔들림도 확인합니다. 기본 기준은 `max_xy_std:=0.03`, `max_yaw_std_deg:=3.0`이며, 이를 넘으면 `RESULT_UNSTABLE_SAMPLES`로 실패하고 `/initialpose`나 `map -> odom`을 적용하지 않습니다. 이 경우 marker map 좌표/yaw, marker size, 카메라 extrinsic, 카메라 보정값을 먼저 확인합니다.

AMCL/Nav2를 사용하지 않는 단독 TF 실험에서만 직접 `map -> odom` 발행을 켭니다.

```bash
ros2 launch logitle_aruco_tools logitle_aruco_pose_corrector_action.launch.py \
  allow_tf_publish:=true
```

`allow_tf_publish:=true` 상태에서도 이미 `map -> odom` TF가 보이면 서버는 충돌을 피하기 위해 `/initialpose`로 우회합니다. 정말 단독 실험 환경에서 Action 서버가 `map -> odom`을 강제로 소유해야 할 때만 다음처럼 실행합니다.

```bash
ros2 launch logitle_aruco_tools logitle_aruco_pose_corrector_action.launch.py \
  allow_tf_publish:=true \
  force_tf_publish:=true
```

`force_tf_publish:=true`는 AMCL/Nav2 등 다른 `map -> odom` publisher와 동시에 사용하지 않습니다.

## TF Correction

기본은 dry-run입니다. 현재 ID 24 현장 기준값은 launch 기본값에 반영되어 있습니다.

```bash
ros2 launch logitle_aruco_tools logitle_aruco_tf_corrector.launch.py
```

dry-run의 `T_map_base`, `T_map_odom`이 실제 로봇 위치와 맞을 때만 TF 발행을 켭니다.

```bash
ros2 launch logitle_aruco_tools logitle_aruco_tf_corrector.launch.py \
  publish_tf:=true
```

현재 기본값:

```text
camera_x=0.045
camera_y=0.0
camera_z=0.115
camera_pitch=-5.0
camera_yaw=0.0
camera_roll=0.0
outlier_z_max=0.15
outlier_tilt_deg=25
expected_base_yaw_deg=-90
yaw_tolerance_deg=4
```

AMCL/Nav2 등 다른 `map -> odom` publisher와 동시에 실행하지 않습니다.

## TF Continuity Check

`aruco_tf_corrector`가 `map -> odom`을 발행 중일 때, 로봇을 천천히 움직이며 `map -> base_link`가 튀지 않고 연속적으로 변하는지 확인합니다.

```bash
ros2 launch logitle_aruco_tools logitle_tf_continuity_monitor.launch.py \
  duration_sec:=30 \
  csv:=/tmp/tf_continuity_id24.csv
```

다른 터미널에서 아주 작은 속도로 teleop 이동 또는 손으로 살짝 이동시키고, monitor 로그의 `jumps=0` 및 종료 summary의 `summary=OK`를 확인합니다.

기본 판정 기준:

```text
sample_period=0.1s
jump_xy_threshold=0.08m/sample
jump_yaw_threshold_deg=10deg/sample
stale_threshold=0.5s
```

30초 테스트에서 `summary=OK`, `missing=0`, `jumps=0`이면 현재 조건에서는 `tf2_echo map base_link` 값이 연속적으로 변한다고 봐도 됩니다. `jump?` 경고가 나오면 CSV의 해당 행 주변에서 `dxy`, `dyaw_deg`, `dt`를 보고 실제 급격한 이동인지 TF 끊김/보정값 튐인지 분리해서 확인합니다.
