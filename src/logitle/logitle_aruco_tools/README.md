# logitle_aruco_tools

TurtleBot3에서 4 cm ArUco 마커(ID 24, 25 / `DICT_5X5_1000`)를 이용해 카메라 기준 pose를 발행하고, 마커 기반 자동정렬과 `map -> odom` TF 보정 dry-run/publish를 수행하는 ROS 2 Python 패키지입니다.

## Build

```bash
cd ~/turtlebot3_E1i6
colcon build --packages-select turtlebot3_bringup logitle_aruco_msgs logitle_aruco_tools logitle_bringup
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
현재는 `logitle_bringup`의 `logitle_robot.launch.py` 안에 포함되어 있으므로, 일반 운용에서는 별도 ArUco launch를 추가로 실행하지 않아도 됩니다.

```bash
ros2 launch logitle_bringup logitle_robot.launch.py
```

`logitle_robot.launch.py`는 기본적으로 아래 ArUco Action 서버를 함께 실행합니다.

```text
logitle_align_and_correct_action_server
logitle_aruco_pose_corrector_action_server
```

ArUco 기능만 끄고 기본 bringup을 실행해야 할 때는 다음처럼 실행합니다.

```bash
ros2 launch logitle_bringup logitle_robot.launch.py use_aruco:=false
```

`logitle_aruco_pose_viewer`는 포함하지 않습니다. 두 서버 모두 기본 `pose_source:=camera` 모드로 대기하며,
Action goal이 들어올 때 Action 서버가 `camera.launch.py`를 시작하고 `/camera/image_raw`,
`/camera/camera_info`를 구독해서 ArUco 검출을 수행합니다. goal이 끝나면 Action 서버가 시작한
카메라 프로세스와 구독을 함께 종료하므로, bringup만 켜둔 상태에서는 카메라 연산을 하지 않습니다.

카메라가 연결된 로봇이면 robot1/2/3 모두 같은 구조로 사용할 수 있습니다.
`use_camera` 기본값은 `false`이며, 카메라는 ArUco Action 요청 시 자동으로 켜집니다.
카메라를 계속 켜두고 pose viewer나 별도 검사를 할 때만 다음처럼 `use_camera:=true`를 전달합니다.

```bash
ros2 launch logitle_bringup logitle_robot.launch.py use_camera:=true
```

기본 경로가 아닌 calibration 파일을 사용할 때는 다음처럼 경로를 전달합니다.

```bash
ros2 launch logitle_bringup logitle_robot.launch.py \
  camera_info_url:=file:///home/turtlebot3/camera/imx219_320x240.yaml
```

### 카메라 영상이 안 나올 때 (`Failed to call start`)

Align goal이 매번 정렬 제한 시간(25초)에 끝나고 카메라 로그에 다음 에러가 있으면, Raspberry Pi용이
아닌 libcamera가 로드된 것입니다.

```text
ERROR IPCPipe ipc_pipe_unixsocket.cpp:131 Call timeout!
ERROR IPAProxy raspberrypi_ipa_proxy.cpp:316 Failed to call start: -110
```

로봇에는 `/usr/local/lib/aarch64-linux-gnu`(Raspberry Pi용 소스 빌드, 로그에 `libcamera v0.7.2+rpt...`)와
`/opt/ros/jazzy/lib`(`ros-jazzy-libcamera`, 로그에 `libcamera v0.7.2`) 두 libcamera가 있습니다. ROS를
source하면 `/opt/ros/jazzy/lib`가 먼저 잡혀 Raspberry Pi IPA가 시작되지 않습니다. `camera.launch.py`가
카메라 컨테이너에만 `/usr/local/lib/aarch64-linux-gnu`를 `LD_LIBRARY_PATH` 앞에 붙이므로, 별도
`export LD_LIBRARY_PATH=...` 없이 실행하면 됩니다. 카메라 로그에 `+rpt`가 없으면 `turtlebot3_bringup`이
최신(dev `6f73534` 이후)으로 빌드됐는지 확인합니다.

현장 측정처럼 전체 운용 스택이 필요 없는 경우에는 필요한 노드만 켜서 부하를 줄입니다.

ArUco 마커 x/z 값만 확인할 때:

```bash
ros2 launch turtlebot3_bringup camera.launch.py
ros2 launch logitle_aruco_tools logitle_aruco_pose_viewer.launch.py marker_ids:=24,25,28,29
```

로봇 TF yaw만 확인할 때:

```bash
ros2 launch logitle_bringup logitle_robot.launch.py \
  use_camera:=false \
  use_aruco:=false \
  use_nav2:=false \
  use_logitle_pose:=false \
  use_docking:=false
```

Nav2/AMCL 기준 `map -> base_footprint` yaw까지 필요하지만 docking/ArUco/camera는 필요 없을 때:

```bash
ros2 launch logitle_bringup logitle_robot.launch.py \
  use_camera:=false \
  use_aruco:=false \
  use_docking:=false
```

운용 기준 실행 순서는 다음과 같습니다.

```bash
source ~/shellscript/logitlebash.sh
sl
ros_local
ros2 launch logitle_bringup logitle_robot.launch.py
```

관제 호출 예시:

```bash
ros2 action send_goal /aruco_align_and_correct logitle_aruco_msgs/action/AlignAndCorrectWithAruco \
"{marker_id: 25, apply_correction: true}" \
--feedback
```

ArUco Action 이름과 로봇 내부 토픽/TF 이름은 robot1/2/3 모두 동일하게 사용합니다.

실제 로봇 내부 제어/TF는 현재 bringup 구조에 맞춰 namespace 없이 사용합니다.

```text
align/correct Action: /aruco_align_and_correct
pose correction Action: /aruco_correct_pose
pose topic: /aruco/id{marker_id}/pose_camera
cmd_vel: /cmd_vel
initialpose: /initialpose
TF: map -> odom -> base_footprint
camera: /camera/image_raw, /camera/camera_info
```

동작 순서:

```text
Nav2로 5번/6번 노드 근처 이동
-> 관제가 /aruco_align_and_correct Action goal 전송
-> Action 서버가 카메라 프로세스를 시작하고 topic 구독 및 ArUco 검출 시작
-> ArUco 마커 기준으로 전진/후진 및 회전 미세 정렬
-> CorrectPoseWithAruco 호출
-> /initialpose로 AMCL/Nav2 위치 보정
-> Action 종료 후 Action 서버가 시작한 카메라 프로세스와 topic 구독 해제
```

Action goal에서 `target_x`, `target_z`를 생략하면 `marker_id`에 맞는 현장 측정 preset이 자동 적용됩니다.
예를 들어 6번 좌표의 ID25는 `target_x=-0.157m`, `target_z=0.384m`,
`expected_base_yaw_deg=-87deg` preset을 사용합니다.
Action 기본 정렬 기준은 `x_tolerance=0.005m`, `z_tolerance=0.007m`,
`wall_yaw_tolerance_deg=2deg`, `yaw_tolerance_deg=3deg`입니다.
단, preset이 적용되는 marker는 Goal이 기본값을 보낼 때 다음 preset 값을 우선 사용합니다.

- 모든 preset: `z_tolerance=0.010m`. 약 0.38m 거리, 320x240 해상도에서 마커 거리 추정이
  약 0.7cm 단위로 바뀌므로, 0.007m는 경계에서 정렬 완료와 재정렬을 반복할 수 있습니다.
- N5/N6 preset: `yaw_tolerance_deg=8deg`. 이 값은 **pose correction 단계의 허용 범위**이며,
  로봇 방향을 -87deg로 정렬하는 값이 아닙니다. 정렬은 마커 하나의 x/z만 맞추므로 노드에서
  옆으로 치우쳐 시작하면 정렬 후 방향이 몇 도 틀어질 수 있습니다(로봇1 N5: 약 -93deg).
  두 마커 방식의 yaw는 정확하므로 `/initialpose`에는 실제 방향이 반영됩니다. 물리적으로
  방향까지 맞춰야 하는 작업이라면 두 마커 기반 방향 정렬을 후속으로 추가해야 합니다.
`check_wall_yaw` 기본값은 `false`입니다. 단일 마커의 벽 방향 추정이 두 방향으로
튀는 현장에서 정렬이 발산하는 것을 막기 위한 설정입니다. 필요한 경우에만 Goal에서
`check_wall_yaw: true`를 명시해 벽 방향 정렬을 활성화합니다. `check_yaw`는 정렬 후
pose correction 단계에서 AMCL 초기 자세 기준을 검사하는 별도 조건이며 서로 독립적입니다.
회전 제어는 `max_angular=0.04rad/s`, `min_angular=0.020rad/s`를 사용합니다.
계산된 회전 명령이 0이 아닌 경우에는 모터 데드밴드에 걸리지 않도록 최소값을 적용하고,
최대값은 계속 `0.04rad/s`로 제한합니다. 이 값은 현재의 임시 피드백 제어용이며,
추후 PID 제어로 전환할 때 재튜닝 대상입니다.
`target_x`는 카메라 기준에서 마커가 보이는 정상 위치이며, 마커를 화면 정중앙(`x=0`)으로
맞추는 값이 아닙니다. 마커가 보이는 범위 안에서만 정렬할 수 있으며, pose가 끊기면 즉시 정지합니다.
`check_wall_yaw=true`인 경우에만 벽 방향과 위치를 한 번에 합산하지 않고, 먼저 벽 방향
오차를 보정한 뒤 x/z 위치를 보정합니다. 기본값에서는 벽 방향 체크를 생략하고 x/z 위치
정렬을 수행한 뒤, marker preset의 `check_yaw` 조건으로 pose correction 결과를 검사합니다.
OpenCV ArUco 검출은 기본 10Hz로 제한하고 카메라 스트림 자체는 유지합니다.
필요하면 `detection_rate_hz:=15.0`으로 높일 수 있습니다.

실주행 전에 다음 항목을 확인합니다.

```bash
ros2 topic info /cmd_vel -v
ros2 node list | grep -E 'align|aruco'
```

정렬 중에는 의도한 Action 서버와 `/cmd_vel` 제어 발행자가 각각 하나인지 확인합니다.
Nav2, 수동 조종 노드 또는 이전에 남은 Action 서버가 동시에 `/cmd_vel`을 발행하면
정렬 명령이 덮어써질 수 있습니다. CPU 부하가 높은 경우에는 Nav2/도킹/pose viewer를
분리한 가벼운 구성으로 먼저 동작을 확인합니다.

현장 측정값을 기준으로 marker별 정렬 preset을 코드에 포함합니다. 관제가 `target_x`, `target_z`,
`expected_base_yaw_deg`를 생략하면 아래 preset이 적용됩니다. Action 정의의 `target_x=0.0`,
`target_z=0.0`은 생략값을 의미하며, 실제 정렬 목표가 아닙니다.

```text
N3 / ID27
target_x = 0.001
target_z = 0.397
expected_base_yaw_deg = 0.0      (오른쪽 벽, 마커 하나: yaw fallback)
yaw_tolerance_deg = 3.0

N4 / ID26
target_x = -0.009
target_z = 0.385
expected_base_yaw_deg = 0.0      (오른쪽 벽, 마커 하나: yaw fallback)
yaw_tolerance_deg = 3.0

N5 / ID24
target_x = -0.189
target_z = 0.376
expected_base_yaw_deg = -87.0    (두 마커: ID24 + ID29)
yaw_tolerance_deg = 8.0

N5 / ID29
target_x = 0.185
target_z = 0.372
expected_base_yaw_deg = -87.0
yaw_tolerance_deg = 8.0

N6 / ID25
target_x = -0.157
target_z = 0.384
expected_base_yaw_deg = -87.0    (두 마커: ID25 + ID28)
yaw_tolerance_deg = 8.0

N6 / ID28
target_x = 0.153
target_z = 0.361                  (잠정값: 로봇2·3 평균)
expected_base_yaw_deg = -87.0
yaw_tolerance_deg = 8.0

모든 preset: z_tolerance = 0.010
```

preset의 target 값은 로봇1·2·3에서 같은 노드에 놓고 측정한 값의 중앙값입니다(2026-10-06).
로봇별로 일정한 오프셋은 없었고 차이는 주로 로봇 배치 위치에서 생겨, 로봇별 preset 대신
공통 preset을 사용합니다.

예를 들어 N4의 ID26은 다음처럼 `marker_id`만 보내도 N4 preset이 적용됩니다.

```bash
ros2 action send_goal /aruco_align_and_correct logitle_aruco_msgs/action/AlignAndCorrectWithAruco \
"{marker_id: 26, apply_correction: true}" \
--feedback
```

### N5/N6 벽 정렬 (라이다 + 마커 방향각)

N5(ID24/ID29), N6(ID25/ID28) 마커로 goal을 보내고 `target_x`, `target_z`를 생략하면
x/z preset 정렬 대신 벽 정렬을 사용합니다. 마커 하나의 x/z만 맞추면 로봇이 조금 회전하면서
옆으로 밀린 위치에서도 같은 x/z가 나와 중앙이 벗어날 수 있기 때문입니다.

- 방향, 벽까지 거리: `/scan`에서 로봇 앞 벽을 직선으로 피팅합니다.
- 좌우: 마커의 방향각(화면 좌우 위치)과 라이다 벽 직선으로 계산합니다. 두 마커가 보이면 두 마커
  사이를 보간하고, 하나만 보이면 그 마커로 계산합니다. 마커 tvec의 거리값은 거리에 따라 오차가
  커져서 사용하지 않습니다.
- 동작: 정지 후 측정하고, 한 번에 하나만 움직입니다. 방향 정렬(최대 8°), 중심 보정(8° 회전,
  3cm 이내 직진, 원래 방향으로 회전), 거리 보정(3cm 이내) 중 하나를 수행합니다. 오차가 방향 1°,
  좌우 1cm, 거리 1cm 안에 두 번 연속 들어오면 완료합니다.
- 목표는 바닥 노드 표시 실측값입니다(2026-10-07). N5는 ID24 중심에서 오른쪽 18.0cm, 벽까지
  40.0cm이고, N6은 ID25 중심에서 오른쪽 17.5cm, 벽까지 41.0cm입니다.
- 정렬 후 pose corrector 대신 같은 측정값으로 `/initialpose`를 직접 발행합니다.
- 끄려면 align 서버에 `--wall-align false`를 줍니다. 한 번에 움직이는 한도는
  `--pair-max-turn-deg`, `--pair-max-drive`로 조절합니다.
- 로봇 앞 0.25~0.60m, 좌우 ±0.35m에 벽이 라이다로 보여야 합니다. 벽 직선 피팅이 안 되면
  정렬을 바로 실패로 끝냅니다. 마커가 하나도 안 보이면 정지한 채로 기다립니다.

주요 파라미터(align 서버 인자):

| 파라미터 | 기본값 | 설명 |
| --- | --- | --- |
| `--wall-align` | true | N5/N6 벽 정렬 사용 (false면 x/z preset 정렬) |
| `--wall-initialpose` | true | 벽 정렬 측정값으로 `/initialpose` 발행 (pose corrector 미사용) |
| `--pair-yaw-tolerance-deg` | 1.0 | 방향 허용 오차 |
| `--pair-lateral-tolerance` | 0.010 | 좌우 허용 오차 [m] |
| `--pair-distance-tolerance` | 0.010 | 거리 허용 오차 [m] |
| `--pair-max-turn-deg` | 8.0 | 한 번 회전 한도 |
| `--pair-max-drive` | 0.03 | 한 번 직진 한도 [m] |
| `--pair-max-centering` | 10 | 중심 보정 최대 횟수 |
| `--pair-timeout-sec` | 60 | 벽 정렬 제한 시간 |

목표값을 다시 잴 때는 로봇 없이 바닥 노드 표시만 기준으로 두 값을 잽니다.
`WALL_PAIR_TARGETS`(`logitle_align_and_correct_action_server.py`)의 `center_offset`, `wall_distance`에 넣습니다.

- `center_offset`: 왼쪽 마커(N5 ID24, N6 ID25) 중심에서 벽을 따라 오른쪽으로, 노드 표시 바로 앞 지점까지 거리
- `wall_distance`: 노드 표시에서 벽까지 수직 거리
- 정렬 후 확인: 왼쪽 바퀴 중앙은 로봇 중심에서 왼쪽 8.0cm이므로, 왼쪽 마커 중심에서
  왼쪽 바퀴 중앙까지 가로 거리가 `center_offset - 8.0cm`이면 정상입니다.

N5는 노드 표시 중앙에서 ID29가 화면 오른쪽 끝에 걸려 ID24 하나로 좌우를 계산합니다. 두 마커가 모두
보이면 두 마커 사이를 보간하므로 더 정확합니다(방향각 배율 오차 상쇄).

## Pose Correction Action

관제 연동은 `/aruco_correct_pose` action을 사용합니다. 이 Action은 도킹이나 이동 제어를 하지 않고, ArUco 관측으로 현재 `map -> base_footprint` pose를 계산합니다.

Action 서버는 기본적으로 대기 중에는 카메라 이미지 처리를 하지 않습니다. 관제에서 goal이 들어온 동안에만 `/camera/image_raw`, `/camera/camera_info`를 구독하고 ArUco 검출/pose 계산을 수행한 뒤, goal이 끝나면 구독을 해제합니다. TF listener는 pose-corrector와 분리된 단일 스레드 Node에서 동작하므로, 대기 중 TF 수신이 Action 서버의 멀티스레드 처리와 서로 간섭하지 않습니다.

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

`aruco_tf_corrector`가 `map -> odom`을 발행 중일 때, 로봇을 천천히 움직이며 `map -> base_footprint`가 튀지 않고 연속적으로 변하는지 확인합니다.

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
