# 2026-09-30 ArUco 아래쪽 벽 ID24/ID25 현장 테스트 정리

## 1. 한 줄 결론

아래쪽 벽의 ID24/ID25 마커는 4cm 새 출력물 기준으로 최종 좌표를 확정했고, 두 마커 모두 정지 상태 `publish-tf`는 안정적으로 확인했다. 실제 운용 거리는 두 마커 모두 대략 `0.35m ~ 0.55m` 구간을 권장하며, 너무 가까운 구간과 먼 구간은 TF 보정 샘플에서 제외한다.

## 2. 오늘 확정한 내용

| 항목 | 결과 |
| --- | --- |
| 마커 재출력 | 네 변 모두 4cm인 새 ArUco 마커로 교체 |
| 부착 위치 | 아래쪽 벽 ID24/ID25 재부착 완료 |
| 좌표 기준 | `logitle_route.geojson`의 5번, 6번 좌표 기준 |
| 좌표 단위 | 마커 모서리가 아니라 마커 중심 기준 |
| 높이 | 바닥부터 마커 아래 변까지 `12.2cm`, 중심 높이 `0.142m` |
| 마커 크기 | `size=0.0400m` |
| ID24 정지 보정 | `publish-tf` 기준 OK |
| ID25 정지 보정 | `publish-tf` 기준 OK |
| 권장 거리 | ID24/ID25 모두 약 `0.35m ~ 0.55m` |

## 3. 최종 marker_map 값

측정 기준:

| Marker | 기준 좌표 | 중심 오프셋 | 벽까지 거리 | 중심 높이 |
| --- | --- | --- | --- | --- |
| ID24 | route 5번 좌표 | 왼쪽 `16.0cm` | `40.5cm` | `14.2cm` |
| ID25 | route 6번 좌표 | 왼쪽 `17.5cm` | `40.5cm` | `14.2cm` |

최종 좌표:

```yaml
markers:
  - id: 24
    x: 0.9685766435
    y: -0.6782363343
    z: 0.142
    yaw_deg: 90.0
    roll_deg: 0.0
    size: 0.0400

  - id: 25
    x: 1.7693385363
    y: -0.6699184167
    z: 0.142
    yaw_deg: 90.0
    roll_deg: 0.0
    size: 0.0400
```

반영 파일:

```text
src/logitle/logitle_aruco_tools/config/logitle_marker_map.yaml
test/marker_map.yaml
현장 로봇: ~/aruco_test/marker_map.yaml
```

## 4. 현장 실행 환경 이슈

| 이슈 | 증상 | 조치/판정 |
| --- | --- | --- |
| `TURTLEBOT3_MODEL` 미설정 | `KeyError: 'TURTLEBOT3_MODEL'` | `export TURTLEBOT3_MODEL=burger` 필요 |
| `LDS_MODEL` 미설정 | `KeyError: 'LDS_MODEL'` | `export LDS_MODEL=LDS-01` 필요 |
| ROS 패키지 미설치 | `Package 'logitle_aruco_tools' not found` | 로봇에서는 `~/aruco_test`의 standalone script로 실행 |
| YAML 파싱 오류 | `marker_map.yaml` block collection parser error | `marker_map.yaml` 들여쓰기/중복 key 수정 필요 |
| TurtleBot node segfault | `turtlebot3_ros exit code -11` | bringup 재시작 필요. 재현 시 USB/포트/전원 상태 확인 |
| `tf2_echo` 첫 줄 warning | `map` frame not found | corrector가 TF publish하기 전이면 정상적으로 발생 가능 |

## 5. 공통 실행 명령

pose viewer:

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

TF corrector 기본 옵션:

```bash
python3 aruco_tf_corrector.py \
  --marker-map marker_map.yaml \
  --marker-id <24 또는 25> \
  --pose-topic /aruco/id<24 또는 25>/pose_camera \
  --camera-x 0.045 \
  --camera-y 0.0 \
  --camera-z 0.115 \
  --camera-pitch -5.0 \
  --camera-yaw 0.0 \
  --camera-roll 0.0 \
  --outlier-z-max 0.15 \
  --outlier-tilt-deg 25 \
  --expected-base-yaw-deg <마커별 값> \
  --yaw-tolerance-deg 3
```

`publish-tf` 실험 시 마지막에 추가:

```bash
--publish-tf
```

TF 확인:

```bash
ros2 run tf2_ros tf2_echo map base_link
```

## 6. ID25 테스트 결과

### 6.1 정면 정지 pose viewer

| 항목 | 값 |
| --- | --- |
| `x` | `-0.002m ~ -0.003m` |
| `y` | 약 `-0.065m` |
| `z` | `0.391m ~ 0.395m` |
| distance | `0.397m ~ 0.400m` |
| size | `0.0400m` |

판정: 카메라 중앙에 거의 정면으로 잡혔고, 거리도 권장 운용 구간 안에 있었다.

### 6.2 dry-run yaw gate 조정

처음 조건은 `expected_base_yaw_deg=-90`, `yaw_tolerance_deg=8`이었다. 이 조건에서는 `-80.2deg`, `-84.2deg`, `-90.9deg`, `-95.8deg`, `-101.5deg`처럼 여러 yaw 해가 섞였다.

최종 조건:

| 항목 | 값 |
| --- | --- |
| `expected_base_yaw_deg` | `-96` |
| `yaw_tolerance_deg` | `3` |
| `outlier_z_max` | `0.15` |
| `outlier_tilt_deg` | `25` |

대표 dry-run 결과:

```text
T_map_base(latest): x=+1.8131 y=-0.2025 z=+0.0000 yaw=-95.81deg
```

판정: 정상 클러스터는 통과하고, `-80deg`, `-84deg`, `-90.9deg` 계열의 튀는 해는 yaw gate에서 reject되었다.

### 6.3 publish-tf 정지 테스트

`tf2_echo map base_link` 대표 결과:

```text
Translation: [1.813, -0.203, 0.010]
Rotation RPY degree: [0.000, 0.000, -95.81]
```

여러 출력 동안 값은 아래 범위로 안정적이었다.

| 항목 | 값 |
| --- | --- |
| `x` | 약 `1.813` |
| `y` | 약 `-0.203` |
| `z` | 약 `0.010` |
| yaw | `-95.805deg ~ -95.813deg` |

판정: ID25 정지 상태 `publish-tf`는 OK.

### 6.4 직진/후진 거리 테스트

| 구간 | pose viewer 거리 | corrector 경향 | 판정 |
| --- | --- | --- | --- |
| 너무 가까움 | `0.250m ~ 0.262m` | yaw reject 급증, accepted 거의 정체 | 제외 |
| 권장/제한 운용 | `0.392m ~ 0.548m` | reject는 있으나 accepted 증가 | OK |
| 너무 멂 | `0.683m ~ 0.771m` | z/yaw reject 급증, accepted 거의 정체 | 제외 |

ID25 운용 결론:

| 항목 | 값 |
| --- | --- |
| 권장 거리 | 약 `0.35m ~ 0.55m` |
| 주의 거리 | 약 `0.55m ~ 0.65m` |
| 제외 거리 | 약 `0.30m 이하`, 약 `0.65m 이상` |
| 정면 배치 기준 yaw gate | `expected=-96`, `tolerance=3` |
| 6번 좌표 실제 도착 자세 yaw gate | `expected=-87`, `tolerance=3` |

### 6.5 ID25 6번 좌표 실제 도착 자세 기준

2026-10-01 로봇을 6번 좌표에 둔 상태에서 ID25가 안정적으로 검출되는지 확인했다. 로봇팔 간섭 가능성 때문에 마커를 더 안쪽으로 옮기지 않고, 현재 부착 위치를 유지한 채 실제 도착 자세 기준 yaw gate를 별도로 잡는다.

pose viewer 결과:

```text
id=25 x=-0.166 ~ -0.167m
id=25 y=-0.043m
id=25 z=0.386 ~ 0.391m
distance=0.423 ~ 0.427m
size=0.0400m
```

판정:

```text
거리와 z는 권장 구간 안에 있음.
x=-0.166m 근처라 화면 중앙에서는 벗어나지만, 값이 거의 고정되어 검출 안정성은 OK.
마커를 더 안쪽으로 옮기면 로봇팔과 겹칠 가능성이 있어 현재 위치를 유지한다.
```

기존 `expected_base_yaw_deg=-96`, `yaw_tolerance_deg=3` 조건에서는 실제 관측 yaw가 약 `-86deg ~ -87deg`로 들어와 모두 yaw reject되었다.

6번 좌표 실제 도착 자세 기준으로 아래 조건을 적용하면 accepted sample이 안정적으로 증가했다.

```text
expected_base_yaw_deg=-87
yaw_tolerance_deg=3
```

dry-run 결과:

```text
accepted=24 -> 241
rejected: z=0 tilt=0
T_map_base(latest): x=+1.5730 y=-0.2271 z=+0.0000 yaw=-86.76deg
T_map_odom(median x15): x=+1.5730 y=-0.2272 yaw=-86.79deg
```

판정:

```text
ID25 @ 6번 좌표 실제 도착 자세는 expected=-87, tolerance=3 조건으로 운용한다.
이는 마커 좌표가 바뀐 것이 아니라, 6번 좌표 도착 시 로봇이 마커를 보는 자세가 정면 테스트 자세와 다르기 때문이다.
```

## 7. ID24 테스트 결과

### 7.1 정면 정지 pose viewer

로봇을 5번 좌표와 같은 선에 두고, ID24 마커를 최대한 정면으로 보도록 배치했다.

| 항목 | 값 |
| --- | --- |
| `x` | `+0.015m ~ +0.016m` |
| `y` | `-0.062m ~ -0.063m` |
| `z` | `0.377m ~ 0.382m` |
| distance | `0.383m ~ 0.387m` |
| size | `0.0400m` |

판정: ID24도 정면 정지 상태에서 pose viewer 검출은 안정적이었다.

### 7.2 dry-run yaw gate 조정

초기 관찰에서는 정상 클러스터와 튀는 yaw 해가 함께 보였다.

| 구분 | 관찰 yaw |
| --- | --- |
| 정상 클러스터 | 약 `-94deg ~ -96.4deg` |
| reject 대상 | `-79.6deg`, `-83.6deg`, `-88.2deg`, `-98.2deg` 등 |

최종 조건:

| 항목 | 값 |
| --- | --- |
| `expected_base_yaw_deg` | `-95` |
| `yaw_tolerance_deg` | `3` |
| `outlier_z_max` | `0.15` |
| `outlier_tilt_deg` | `25` |

대표 dry-run 결과:

```text
T_map_base(latest): x=+1.0299 y=-0.2304 z=+0.0000 yaw=-94.79deg
T_map_odom(median x15): x=+1.0299 y=-0.2304 yaw=-94.12deg
```

판정: z/tilt reject 없이 accepted sample이 증가했고, 튀는 yaw 해는 yaw gate에서 제외되었다. `-98.2deg`는 정상 클러스터 근처지만 tolerance `3deg` 기준에서는 error `3.2deg`로 reject되었다.

### 7.3 publish-tf 정지 테스트

`tf2_echo map base_link` 대표 결과:

```text
Translation: [1.030, -0.230, 0.010]
Rotation RPY degree: [0.000, 0.000, -94.78]
```

여러 출력 동안 값은 대부분 아래 범위에 있었다.

| 항목 | 값 |
| --- | --- |
| `x` | 약 `1.030` |
| `y` | 약 `-0.230` |
| `z` | 약 `0.010` |
| yaw | 약 `-94.78deg` |

판정: ID24 정지 상태 `publish-tf`는 OK.

### 7.4 직진/후진 거리 테스트

| 구간 | pose viewer 거리 | corrector/TF 경향 | 판정 |
| --- | --- | --- | --- |
| 직진 접근 | 약 `0.38m -> 0.30m` | `map -> base_link` 연속성 유지 | OK |
| 너무 가까움 | 약 `0.30m 이하` | obstacle stop 이후 accepted 정체, yaw reject 증가 | 제외 |
| 후진 | 약 `0.38m -> 0.55m` | accepted 증가, TF 연속성 대체로 유지 | OK |
| 먼 구간 | 약 `0.55m 초과`, 특히 `0.60m 이상` | z/yaw reject 증가 | 주의/제외 |

장애물에 강제로 멈춘 이후의 `tf2_echo` 뒷부분은 실제 보정 품질 판단에서 제외한다. 해당 구간은 로봇 이동이 막힌 상태에서 새 accepted TF가 충분히 들어오지 않아, 같은 timestamp가 반복되거나 extrapolation 성격의 출력이 섞인 것으로 본다.

ID24 운용 결론:

| 항목 | 값 |
| --- | --- |
| 권장 거리 | 약 `0.35m ~ 0.55m` |
| 제외 거리 | 약 `0.30m 이하`, 약 `0.60m 이상` |
| yaw gate | `expected=-95`, `tolerance=3` |

## 8. 아래쪽 벽 기준 권장 설정

| Marker | expected yaw | yaw tolerance | 권장 거리 | 제외 거리 |
| --- | ---: | ---: | --- | --- |
| ID24 | `-95deg` | `3deg` | `0.35m ~ 0.55m` | `0.30m 이하`, `0.60m 이상` |
| ID25 정면 배치 기준 | `-96deg` | `3deg` | `0.35m ~ 0.55m` | `0.30m 이하`, `0.65m 이상` |
| ID25 6번 좌표 도착 자세 | `-87deg` | `3deg` | `0.35m ~ 0.55m` | `0.30m 이하`, `0.65m 이상` |

공통 outlier 조건:

```text
outlier_z_max: 0.15
outlier_tilt_deg: 25
marker_size: 0.0400m
```

## 9. 오늘 테스트의 최종 판정

| 항목 | 판정 |
| --- | --- |
| ID24/ID25 최종 좌표 | 확정 |
| 4cm marker size 반영 | 완료 |
| ID24 pose viewer 검출 | OK |
| ID25 pose viewer 검출 | OK |
| ID24 dry-run yaw gate | OK |
| ID25 dry-run yaw gate | OK |
| ID24 정지 publish-tf | OK |
| ID25 정지 publish-tf | OK |
| ID24 직진/후진 제한 구간 | OK |
| ID25 직진/후진 제한 구간 | OK |
| 너무 가까운 구간 | TF 보정 제외 |
| 너무 먼 구간 | TF 보정 제외 |

## 10. 수정 일정표

### 사전 준비

1. 4cm 정사각 ArUco 마커 재출력
2. 4cm 정사각형 기준 재단

### 1일차: 아래쪽 벽 ID24/ID25 최종 좌표 및 운용 조건 확정

1. 아래쪽 벽 ID24/ID25 재부착
2. `marker_map.yaml` 중심 좌표 재측정
3. PC 프로젝트의 `marker_map.yaml` 수정 및 현장 로봇 `~/aruco_test/marker_map.yaml` 동기화
4. ID24/ID25 pose viewer 검출 확인
5. ID24/ID25 dry-run 재검증
6. ID24/ID25 yaw gate / outlier 조건 1차 재조정
7. ID24/ID25 `publish-tf` 상태에서 정지 테스트
8. 직진/후진 중 `map -> base_link` 연속성 재확인
9. 권장 거리 범위 재정리
10. 아래쪽 벽 기준 보정 조건 문서화
11. 현장 실행 이슈 정리
12. 추가 가능 작업
    - 권장 거리 조건을 코드/launch 설정의 distance gate 후보로 반영
    - 마커 선택 기준 초안에 ID24/ID25 권장 거리와 yaw gate 반영: `docs/aruco_marker_selection_policy_draft_20260930.md`
    - 로봇 앞면/카메라 렌즈/마커 중심 기준 거리 환산표를 운용 문서에 추가
    - 현장 실행 명령을 체크리스트 형태로 정리

### 2일차: 오른쪽 벽 마커 추가

1. 오른쪽 벽 마커 2개 부착
2. 오른쪽 벽 마커 `marker_map.yaml` 좌표 추가
3. 오른쪽 벽 마커별 yaw 기준값 측정
4. 오른쪽 벽 기준 dry-run / publish-tf 테스트

### 3일차: 복수 마커 운용 규칙 정리

1. 여러 마커 중 사용할 마커 선택 기준 정리
2. 마커 lost / no marker 상황 테스트
3. 로봇팔 또는 구조물 가림 상황 테스트
4. fallback 운용 규칙 정리

### 4일차: Action / Nav2 흐름 통합

1. `CorrectPoseWithAruco` Action과 보정 흐름 통합
2. `/initialpose` publish 방식 테스트
3. `map -> odom` publish 실험 옵션 정리
4. Nav2 / 관제 호출 흐름 점검

### 5일차: 설정 정리 및 반복 테스트

1. launch/config 기본값 정리
2. 거리 gate, yaw gate, outlier 조건의 기본값 후보 정리
3. 반복 테스트 및 실패 케이스 정리
4. 아래쪽 벽/오른쪽 벽 마커 설정 비교
5. Confluence 문서 업데이트

### 6일차: 최종 시나리오 점검 및 공유

1. `/initialpose` 기반 보정 후 Nav2 주행 연결 확인
2. 보정 실패 시 fallback 흐름 확인
3. 관제 호출 시나리오 점검
4. 실물 반복 주행 전 최종 체크리스트 작성
5. Slack 공유용 요약 정리
