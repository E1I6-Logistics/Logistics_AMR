# ArUco ID25 기반 map -> base_link TF 연속성 테스트 정리

## 1. 목적

ArUco 마커 기반 TF 보정(`map -> odom`)을 발행한 상태에서 TurtleBot3가 실제로 움직일 때,
`tf2_echo map base_link` 값이 갑자기 튀지 않고 연속적으로 변하는지 확인한다.

이번 테스트에서는 Action 구현은 제외하고, ID25 마커 기준 TF 보정 및 이동 중 TF 연속성만 확인했다.

## 2. 테스트 환경

| 항목 | 내용 |
| --- | --- |
| 로봇 | TurtleBot3 Burger |
| 호스트 | `turtlebot2` |
| ROS | ROS 2 Jazzy |
| 카메라 | IMX219 CSI, `camera_ros` |
| 작업 경로 | `~/aruco_test` |
| 사용 마커 | ArUco ID25 |
| 마커 사전 | `DICT_5X5_50` |
| 마커 크기 | `0.0395m` |
| 카메라 pitch | `-5.0deg` |
| TF 확인 대상 | `map -> base_link` |

## 3. ID25 marker_map 기준

ID25 마커는 기존 기록된 위치를 그대로 사용했다.

```yaml
id: 25
x: 2.0093385363
y: -0.3999184167
z: 0.12475
yaw_deg: 90.0
roll_deg: 0.0
size: 0.0395
```

주의: 테스트 중 언급된 "6번 좌표 기준 14cm 옆"은 마커 위치가 아니라 로봇/카메라 위치에 대한 설명이다. 따라서 `marker_map.yaml`의 ID25 좌표는 변경하지 않는다.

## 4. 실행 구성

아래 노드들이 동시에 실행되어야 한다.

| 구성 | 역할 |
| --- | --- |
| TurtleBot3 bringup | `odom -> base_link` 제공 및 `/cmd_vel` 수신 |
| camera node | 카메라 이미지 및 camera info 제공 |
| `aruco_pose_viewer.py` | `/aruco/id25/pose_camera` 발행 |
| `aruco_tf_corrector.py --publish-tf` | `map -> odom` 발행 |
| `tf2_echo map base_link` | 최종 로봇 pose 확인 |

TF 연결 구조:

```text
map -> odom -> base_link
```

## 5. pose viewer 실행

```bash
cd ~/aruco_test
source /opt/ros/jazzy/setup.bash
source ~/camera_ws/install/setup.bash
source ~/turtlebot3_ws/install/setup.bash

python3 aruco_pose_viewer.py \
  --no-gui \
  --image-topic /camera/image_raw \
  --camera-info-topic /camera/camera_info \
  --marker-ids 24,25 \
  --marker-map marker_map.yaml
```

## 6. TF corrector 실행 조건

ID25는 dry-run 상태에서 세 가지 pose 그룹을 오갔다.

| 그룹 | x | y | yaw |
| --- | ---: | ---: | ---: |
| A | `2.0133` | `0.0736` | `-89.13deg` |
| B | `2.0520` | `0.0561` | `-94.21deg` |
| C | `2.0929` | `0.0561` | `-98.61deg` |

위 상태에서 바로 `--publish-tf`를 켜면 TF가 튈 가능성이 있으므로 yaw gate를 적용했다.

```bash
python3 aruco_tf_corrector.py \
  --marker-map marker_map.yaml \
  --marker-id 25 \
  --pose-topic /aruco/id25/pose_camera \
  --camera-x 0.045 \
  --camera-y 0.0 \
  --camera-z 0.115 \
  --camera-pitch -5.0 \
  --camera-yaw 0.0 \
  --camera-roll 0.0 \
  --outlier-z-max 0.15 \
  --outlier-tilt-deg 25 \
  --expected-base-yaw-deg -90 \
  --yaw-tolerance-deg 4 \
  --publish-tf
```

Yaw gate 적용 후 정상 그룹만 통과했다.

```text
yaw=-89.13deg -> accept
yaw=-94.2deg  -> reject
yaw=-98.6deg  -> reject
```

정지 상태 publish 결과:

```text
map -> base_link
x   = 2.013
y   = 0.074
z   = 0.010
yaw = -89.13deg
```

정지 상태에서는 여러 초 동안 값이 거의 변하지 않아 TF 튐은 확인되지 않았다.

## 7. 회전 테스트 결과

회전 명령은 `/cmd_vel`의 실제 타입이 `geometry_msgs/msg/TwistStamped`임을 확인한 뒤 실행했다.

```bash
ros2 topic info /cmd_vel --verbose
```

확인 결과:

```text
Type: geometry_msgs/msg/TwistStamped
Subscription count: 1
Node name: turtlebot3_node
```

회전 명령:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped \
"{header: {frame_id: base_link}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.08}}}" \
-r 10
```

회전 중에는 로봇 yaw가 `-90deg`에서 벗어나기 때문에 yaw gate가 정상 회전도 reject했다.

예:

```text
reject sample: yaw=-97.3deg expected=-90.0deg error=7.3deg
reject sample: yaw=-102.7deg expected=-90.0deg error=12.7deg
reject sample: yaw=-117.2deg expected=-90.0deg error=27.2deg
```

따라서 현재 yaw gate 조건(`-90 ± 4deg`)에서는 회전 테스트가 TF 연속성 검증에 적합하지 않다.
회전 검증을 하려면 회전 후 기대 yaw를 바꿔 corrector를 재실행하거나, yaw gate 범위를 테스트 목적에 맞게 조정해야 한다.

정지 명령은 한 번만 보내면 놓칠 수 있으므로 반복 발행한다.

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped \
"{header: {frame_id: base_link}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}" \
-r 20
```

## 8. 후진 테스트 결과

후진 명령:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped \
"{header: {frame_id: base_link}, twist: {linear: {x: -0.02, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}" \
-r 10
```

`tf2_echo map base_link` 관측 결과:

```text
y: 0.323 -> 0.361 -> 0.383 -> 0.395 -> 0.462 -> 0.482
   -> 0.500 -> 0.523 -> 0.551 -> 0.572 -> 0.590
   -> 0.614 -> 0.632 -> 0.657 -> 0.666
```

`x`는 거의 유지되었다.

```text
x ≈ 2.013 ~ 2.025
```

Yaw도 안정적으로 유지되었다.

```text
yaw ≈ -90.0 ~ -91.1deg
```

판정:

```text
후진 중 map -> base_link 값은 연속적으로 변함.
큰 위치 점프나 yaw 점프는 확인되지 않음.
```

단, pose viewer 기준으로 마커 거리가 멀어지면서 검출 품질이 떨어졌다.

```text
distance: 0.63m -> 0.80m -> 0.95m -> 1.03m
no marker 발생
```

4cm 마커 기준으로 1m 근처는 검출이 약해지는 구간으로 판단된다.

## 9. 직진 테스트 결과

직진 명령:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped \
"{header: {frame_id: base_link}, twist: {linear: {x: 0.02, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}" \
-r 10
```

`tf2_echo map base_link` 관측 결과:

```text
y: 0.247 -> 0.228 -> 0.206 -> 0.188 -> 0.173
   -> 0.150 -> 0.130 -> 0.107 -> 0.091 -> 0.080
   -> 0.059 -> 0.033 -> 0.018 -> -0.009 -> -0.042
   -> -0.078 -> -0.102 -> -0.121 -> -0.132
```

`x`는 큰 점프 없이 유지되었다.

```text
x ≈ 1.990 ~ 2.014
```

Yaw는 대부분 안정권에 있었다.

```text
yaw ≈ -90 ~ -92deg
```

중간에 `yaw=-87.48deg` 수준의 값이 한 번 보였지만, 기존 문제였던 `-98deg` 또는 `-84deg` 방향으로 크게 튀는 현상은 보이지 않았다.

pose viewer 기준으로 가까워질수록 마커 거리가 줄었다.

```text
distance: 0.48m -> 0.30m -> 0.23m
```

0.23m 근처에서는 너무 가까워 pose 품질이 흔들릴 수 있고, corrector reject가 증가했다.

판정:

```text
직진 중 map -> base_link 값은 대체로 연속적으로 변함.
큰 위치 점프나 큰 yaw 점프는 확인되지 않음.
```

## 10. 최종 판정

| 항목 | 결과 |
| --- | --- |
| 정지 상태 TF 안정성 | OK |
| 후진 중 `map -> base_link` 연속성 | OK |
| 직진 중 `map -> base_link` 연속성 | OK |
| ID25 yaw gate 동작 | OK |
| 회전 테스트 | 현재 yaw gate 조건에서는 부적합 |
| 1m 근처 마커 검출 | 불안정, `no marker` 발생 |
| 0.23m 근처 마커 검출 | 너무 가까워 reject 증가 |

최종 결론:

```text
ID25 + yaw gate(-90 ± 4deg) 조건에서 로봇이 직진/후진할 때
tf2_echo map base_link 값은 갑자기 튀지 않고 연속적으로 변하는 것을 확인했다.
```

## 11. 권장 운용 범위

4cm ID25 마커 기준 권장 사용 거리는 아래와 같다.

```text
권장 거리: 약 0.3m ~ 0.8m
너무 가까움: 약 0.23m 이하
너무 멂: 약 1.0m 이상
```

실사용 시에는 다음 조건을 권장한다.

1. `--expected-base-yaw-deg -90`
2. `--yaw-tolerance-deg 4`
3. 마커 거리 `0.3m ~ 0.8m` 유지
4. 회전 중에는 현재 yaw gate 기준으로 corrector publish 테스트를 하지 않음
5. 더 먼 거리나 넓은 동작 범위가 필요하면 8~10cm 마커 사용 검토

## 12. 남은 이슈 및 후속 작업

| 이슈 | 내용 |
| --- | --- |
| 4cm 마커 한계 | pose estimation이 여러 자세 해 사이에서 튀는 현상이 있음 |
| 거리 의존성 | 1m 근처에서는 `no marker`, 0.23m 근처에서는 reject 증가 |
| 회전 중 보정 | 현재 yaw gate 조건에서는 회전 중 corrector가 정상 샘플도 reject |
| 개선 방향 | 큰 마커 재출력, 단단한 판 부착, 거리 조건 제한, yaw gate 상황별 조정 |

후속으로는 ID24/ID25 모두 같은 절차로 비교하거나, 8~10cm 마커로 재출력 후 동일 테스트를 반복하는 것이 좋다.
