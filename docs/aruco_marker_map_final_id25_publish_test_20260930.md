# ArUco ID24/ID25 최종 부착 좌표 및 publish-tf 현장 테스트

## 1. 목적

2026-09-30 아래쪽 벽에 ID24/ID25 마커를 재부착한 뒤, 더 이상 변경하지 않을 최종 `marker_map.yaml` 좌표와 ID24/ID25 기준 `publish-tf` 정지 및 짧은 직진/후진 테스트 결과를 기록한다.

## 2. 최종 marker_map 기준

마커는 네 변이 모두 4cm인 새 출력물로 교체했다. 따라서 `size`는 `0.0400m`이며, 중심 높이는 아래 변 높이 `12.2cm`에 마커 절반 `2.0cm`를 더한 `0.142m`를 사용한다.

측정 기준:

```text
ID24: logitle_route 5번 좌표 기준, 마커 중심이 왼쪽(map +X) 16.0cm, 벽 방향(map -Y) 40.5cm
ID25: logitle_route 6번 좌표 기준, 마커 중심이 왼쪽(map +X) 17.5cm, 벽 방향(map -Y) 40.5cm
```

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

이 값은 아래 파일에 반영되어 있다.

```text
src/logitle/logitle_aruco_tools/config/logitle_marker_map.yaml
test/marker_map.yaml
```

로봇 현장 테스트용 `~/aruco_test/marker_map.yaml`에도 같은 값을 복사해서 사용한다.

## 3. ID25 pose viewer 확인

실행:

```bash
cd ~/aruco_test

python3 aruco_pose_viewer.py \
  --no-gui \
  --image-topic /camera/image_raw \
  --camera-info-topic /camera/camera_info \
  --marker-ids 24,25 \
  --marker-map marker_map.yaml
```

확인된 pose viewer 상태:

```text
marker_sizes={24: 0.04, 25: 0.04}
id=25 x=-0.002 ~ -0.003m
id=25 y=-0.065m
id=25 z=0.391 ~ 0.395m
distance=0.397 ~ 0.400m
size=0.0400m
```

판정:

```text
ID25는 카메라 중앙에 거의 정면으로 잡힘.
거리도 4cm 마커의 현장 안정 구간으로 확인된 약 0.35m ~ 0.55m 안에 있음.
```

## 4. ID25 dry-run 재조정

처음 `expected_base_yaw_deg=-90`, `yaw_tolerance_deg=8` 조건에서는 다음 yaw 해들이 섞였다.

```text
-80.2deg
-84.2deg
-90.9deg
-95.8deg
-101.5deg
```

실제 안정 클러스터는 `-95.81deg`였으므로, gate를 아래처럼 조정했다.

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
  --expected-base-yaw-deg -96 \
  --yaw-tolerance-deg 3
```

dry-run 결과:

```text
T_map_base(latest): x=+1.8131 y=-0.2025 z=+0.0000 yaw=-95.81deg
```

판정:

```text
expected_base_yaw_deg=-96, yaw_tolerance_deg=3 조건에서 정상 클러스터만 통과했다.
-80deg, -84deg, -90.9deg 등 튀는 해는 yaw gate에서 reject되었다.
```

## 5. ID25 publish-tf 정지 테스트

실행:

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
  --expected-base-yaw-deg -96 \
  --yaw-tolerance-deg 3 \
  --publish-tf
```

`tf2_echo map base_link` 확인 결과:

```text
Translation: [1.813, -0.203, 0.010]
Rotation RPY degree: [0.000, 0.000, -95.81]
```

여러 출력 동안 값은 거의 고정되어 있었다.

```text
x   = 1.813
y   = -0.203
z   = 0.010
yaw = -95.805 ~ -95.813deg
```

corrector 로그:

```text
tf T_map_base(latest): x=+1.8131 y=-0.2025 z=+0.0000 yaw=-95.81deg
T_map_odom(median x15): x=+1.8131 y=-0.2025 yaw=-101.09 ~ -101.13deg
rejected: z=0, tilt=16~24, accepted=271~457
```

의도대로 reject된 샘플:

```text
yaw=-80.2deg expected=-96.0deg error=15.8deg
yaw=-90.9deg expected=-96.0deg error=5.1deg
yaw=-84.2deg expected=-96.0deg error=11.8deg
```

## 6. ID25 직진/후진 거리 구간 테스트

정지 publish-tf 확인 후 같은 조건으로 짧은 직진/후진 테스트를 수행했다.

조건:

```text
expected_base_yaw_deg=-96
yaw_tolerance_deg=3
outlier_z_max=0.15
outlier_tilt_deg=25
```

### 6.1 직진: 너무 가까운 구간

직진 중 pose viewer 기준 거리가 약 `0.25m`까지 줄어들었다.

```text
id=25 distance=0.250 ~ 0.262m
x=-0.004 ~ -0.006m
y=-0.050 ~ -0.052m
z=0.245 ~ 0.257m
```

이 구간에서 corrector는 대부분 yaw gate에서 reject했다.

```text
accepted=89 -> 92 정도로 거의 증가하지 않음
yaw rejected=578 -> 1213까지 급증
대표 reject yaw=-84 ~ -90deg, expected=-96deg, tolerance=3deg
```

판정:

```text
0.25m 근처는 너무 가까워 TF 보정용 샘플로 부적합하다.
ID25 운용 구간에서 제외한다.
```

### 6.2 후진: 0.39m ~ 0.55m 구간

후진 중 pose viewer 기준 거리가 약 `0.39m`에서 `0.55m`까지 증가했다.

```text
id=25 distance=0.392 ~ 0.548m
x=+0.017 ~ +0.026m
y=-0.065 ~ -0.079m
z=0.386 ~ 0.542m
```

corrector는 튀는 yaw 해를 계속 reject했지만, accepted sample도 꾸준히 증가했다.

```text
accepted=21 -> 232
z rejected=0 -> 57
tilt rejected=4 -> 27
대표 accepted T_map_base yaw=-93.0 ~ -98.3deg
```

판정:

```text
0.39m ~ 0.55m 구간은 reject가 많더라도 accepted sample이 계속 들어오므로 제한적 운용 가능 구간으로 본다.
```

### 6.3 후진: 0.68m ~ 0.77m 구간

후진을 더 진행하면 pose viewer 기준 거리 `0.68m ~ 0.77m`에서도 마커 검출 자체는 유지되었다.

```text
id=25 distance=0.683 ~ 0.771m
x=+0.002 ~ +0.009m
y=-0.087 ~ -0.093m
z=0.677 ~ 0.766m
```

그러나 같은 구간에서 corrector는 yaw/z reject가 급증했다.

```text
accepted=242 -> 252 정도로 거의 정체
z rejected=78 -> 295
yaw rejected 계속 증가
대표 reject yaw=-91.1deg, expected=-96deg, error=4.9deg
대표 z reject: z=-0.154, z=-0.202, z=+0.170
```

판정:

```text
0.68m ~ 0.77m 구간은 detection은 OK지만 TF correction sample stability는 NG/보류다.
ID25 publish-tf 운용 구간에서는 제외한다.
```

## 7. ID24 정지 및 직진/후진 테스트

ID24는 로봇을 `logitle_route` 5번 좌표와 같은 선에 두고, 마커를 가능한 정면으로 보도록 배치한 상태에서 테스트했다.

pose viewer 정지 상태:

```text
id=24 x=+0.015 ~ +0.016m
id=24 y=-0.062 ~ -0.063m
id=24 z=0.377 ~ 0.382m
distance=0.383 ~ 0.387m
size=0.0400m
```

최종 gate 조건:

```text
marker_id: 24
expected_base_yaw_deg: -95
yaw_tolerance_deg: 3
outlier_z_max: 0.15
outlier_tilt_deg: 25
```

dry-run / publish-tf 대표 결과:

```text
T_map_base(latest): x=+1.0299 y=-0.2304 z=+0.0000 yaw=-94.79deg
T_map_odom(median x15): x=+1.0299 y=-0.2304 yaw=-94.12deg

tf2_echo map base_link:
Translation: [1.030, -0.230, 0.010]
Rotation RPY degree: [0.000, 0.000, -94.78]
```

판정:

```text
ID24 정지 publish-tf는 안정적이다.
정상 클러스터는 약 -94deg ~ -96.4deg에 형성되며, -79.6deg, -83.6deg, -88.2deg 계열은 reject 대상이다.
-98.2deg는 정상 클러스터 근처지만 tolerance 3deg 기준에서는 error 3.2deg로 reject되었다.
```

직진/후진 테스트 판정:

```text
직진: 약 0.38m -> 0.30m 구간에서는 map -> base_link 연속성 OK.
직진: 약 0.30m 이하와 장애물 강제 정지 이후 구간은 품질 판단에서 제외.
후진: 약 0.38m -> 0.55m 구간은 accepted sample 증가 및 TF 연속성 OK.
후진: 약 0.55m 초과부터 주의, 0.60m 이상은 제외.
```

ID24 아래쪽 벽 기준 현장 권장 조건:

```text
marker_id: 24
marker_size: 0.0400m
expected_base_yaw_deg: -95
yaw_tolerance_deg: 3
outlier_z_max: 0.15
outlier_tilt_deg: 25
권장 거리: 약 0.35m ~ 0.55m
제외 거리: 약 0.30m 이하, 약 0.60m 이상
```

## 8. 최종 판정

| 항목 | 결과 |
| --- | --- |
| ID24/ID25 최종 좌표 확정 | OK |
| 4cm 마커 size 반영 | OK |
| ID24 pose viewer 검출 | OK |
| ID25 pose viewer 검출 | OK |
| ID24 dry-run yaw gate | OK |
| ID25 dry-run yaw gate | OK |
| ID24 정지 publish-tf | OK |
| ID25 정지 publish-tf | OK |
| `map -> base_link` 정지 안정성 | OK |
| ID24 0.35m ~ 0.55m 전후진 제한 구간 | OK |
| ID25 0.39m ~ 0.55m 전후진 제한 구간 | OK |
| ID24 0.30m 이하 | 제외 |
| ID24 0.60m 이상 | 제외 |
| ID25 0.25m 근처 | 제외 |
| ID25 0.68m ~ 0.77m 구간 | 제외 |

ID25 아래쪽 벽 기준 현장 권장 조건:

```text
marker_id: 25
marker_size: 0.0400m
expected_base_yaw_deg: -96
yaw_tolerance_deg: 3
outlier_z_max: 0.15
outlier_tilt_deg: 25
권장 거리: 약 0.35m ~ 0.55m
주의 거리: 약 0.55m ~ 0.65m
제외 거리: 약 0.30m 이하, 약 0.65m 이상
```

남은 작업:

```text
1. ID24/ID25 거리 제한을 Action/운용 규칙에 반영
2. 두 마커 운용 시 마커 선택/우선순위 규칙 정리
3. 오른쪽 벽 마커 추가 시 같은 절차로 marker별 yaw gate와 거리 gate 측정
```
