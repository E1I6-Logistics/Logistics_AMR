# ArUco 복수 마커 선택 기준 초안

## 1. 목적

여러 ArUco 마커가 동시에 보이거나, 일부 마커가 불안정하게 검출될 때 어떤 마커의 보정값을 사용할지 정하는 기준 초안이다.

현재 초안은 2026-09-30 아래쪽 벽 ID24/ID25 현장 테스트 결과를 기준으로 작성했다. 오른쪽 벽 마커를 추가하면 같은 형식으로 marker별 조건을 확장한다.

## 2. 기본 원칙

1. 보정 후보는 반드시 yaw gate, 거리 gate, z/tilt outlier gate를 통과해야 한다.
2. 필수 gate를 통과하지 못한 마커는 점수 계산 전에 제외한다.
3. 여러 마커가 동시에 통과하면 거리, 화면 중심성, 최근 안정성, reject 비율을 기준으로 우선순위를 정한다.
4. 안정적인 후보가 없으면 보정을 적용하지 않고 기존 odom/AMCL 상태를 유지한다.
5. Nav2/AMCL 통합 시 기본 적용 방식은 `map -> odom` 직접 publish가 아니라 `/initialpose` publish를 우선한다.

## 3. 아래쪽 벽 ID24/ID25 1차 조건

| Marker / 상황 | expected yaw | yaw tolerance | 권장 거리 | 주의 거리 | 제외 거리 |
| --- | ---: | ---: | --- | --- | --- |
| ID24 | `-95deg` | `3deg` | `0.35m ~ 0.55m` | `0.55m ~ 0.60m` | `0.30m 이하`, `0.60m 이상` |
| ID25 정면 배치 기준 | `-96deg` | `3deg` | `0.35m ~ 0.55m` | `0.55m ~ 0.65m` | `0.30m 이하`, `0.65m 이상` |
| ID25 6번 좌표 도착 자세 | `-87deg` | `3deg` | `0.35m ~ 0.55m` | `0.55m ~ 0.65m` | `0.30m 이하`, `0.65m 이상` |

공통 조건:

```text
marker_size=0.0400m
outlier_z_max=0.15
outlier_tilt_deg=25
```

거리 기준은 pose viewer의 `distance` 또는 정면 배치 시 `z` 값을 기준으로 본다. 현장 자 측정에서는 카메라 렌즈에서 마커 중심/벽면까지의 거리를 대략 같은 값으로 보면 된다.

ID25의 `-96deg`는 정면 배치 테스트 기준이고, `-87deg`는 2026-10-01 로봇이 실제 6번 좌표에 도착한 자세 기준이다. 6번 좌표 실제 도착 자세에서는 pose viewer `distance=0.423~0.427m`, `x=-0.166~-0.167m`로 검출되었고, `expected=-87`, `tolerance=3` 조건에서 accepted sample이 안정적으로 증가했다.

## 4. 필수 탈락 조건

아래 조건 중 하나라도 해당하면 해당 marker sample은 보정 후보에서 제외한다.

| 조건 | 제외 기준 |
| --- | --- |
| marker id 불일치 | 현재 사용 가능한 marker map에 없는 ID |
| 거리 너무 가까움 | ID24/ID25 공통 `0.30m 이하` |
| 거리 너무 멂 | ID24 `0.60m 이상`, ID25 `0.65m 이상` |
| yaw gate 실패 | marker별 expected yaw에서 `3deg` 초과 |
| z outlier | `outlier_z_max=0.15` 초과 |
| tilt outlier | `outlier_tilt_deg=25` 초과 |
| 샘플 부족 | median 계산에 필요한 accepted sample 수 부족 |
| TF 불연속 | 직전 accepted pose 대비 x/y/yaw가 비정상적으로 점프 |

주의:

```text
ID25는 0.68m ~ 0.77m에서도 detection은 되었지만 z/yaw reject가 급증했다.
따라서 "검출됨"과 "보정 후보로 사용 가능"은 분리해서 판단한다.
```

## 5. 후보 점수화 기준

필수 탈락 조건을 통과한 후보가 여러 개면 아래 순서로 점수를 준다.

| 우선순위 | 기준 | 좋은 상태 |
| ---: | --- | --- |
| 1 | 거리 | 권장 거리 `0.35m ~ 0.55m` 안에 있음 |
| 2 | yaw error | expected yaw와의 차이가 작음 |
| 3 | 화면 중심성 | pose viewer 기준 `x`가 0에 가까움 |
| 4 | 최근 안정성 | 최근 N frame에서 accepted가 꾸준히 증가 |
| 5 | reject 비율 | yaw/z/tilt reject 증가가 적음 |
| 6 | 마커 우선순위 | 같은 조건이면 사전 정의한 marker priority 사용 |

초기 priority 제안:

```text
1순위: 권장 거리 안에 있는 마커
2순위: yaw error가 더 작은 마커
3순위: 화면 중앙에 더 가까운 마커
4순위: 최근 accepted sample이 더 많은 마커
5순위: marker id priority
```

ID24/ID25만 있는 현재 아래쪽 벽에서는 marker id 자체보다 거리와 yaw 안정성을 우선한다.

## 6. 의사결정 흐름

```text
1. 현재 frame에서 감지된 marker 목록을 가져온다.
2. marker_map.yaml에 없는 marker는 제외한다.
3. marker별 거리 gate를 적용한다.
4. marker별 yaw gate를 적용한다.
5. z/tilt outlier gate를 적용한다.
6. accepted sample window를 marker별로 갱신한다.
7. 후보가 0개면 보정하지 않는다.
8. 후보가 1개면 해당 marker를 사용한다.
9. 후보가 2개 이상이면 점수화 기준으로 best marker를 선택한다.
10. 선택된 marker의 median pose를 사용해 보정값을 계산한다.
11. 보정 적용 직전 TF jump guard를 한 번 더 확인한다.
```

## 7. 권장 파라미터 초안

```yaml
marker_selection:
  min_distance_m: 0.35
  preferred_distance_min_m: 0.35
  preferred_distance_max_m: 0.55
  max_yaw_error_deg: 3.0
  outlier_z_max_m: 0.15
  outlier_tilt_deg: 25.0
  sample_window: 15
  require_tf_jump_guard: true

markers:
  24:
    expected_base_yaw_deg: -95.0
    yaw_tolerance_deg: 3.0
    preferred_distance_m: [0.35, 0.55]
    caution_distance_m: [0.55, 0.60]
    reject_distance_m:
      near_lte: 0.30
      far_gte: 0.60

  25:
    expected_base_yaw_deg: -96.0
    yaw_tolerance_deg: 3.0
    preferred_distance_m: [0.35, 0.55]
    caution_distance_m: [0.55, 0.65]
    reject_distance_m:
      near_lte: 0.30
      far_gte: 0.65

marker_contexts:
  id25_front_facing_test:
    marker_id: 25
    expected_base_yaw_deg: -96.0
    yaw_tolerance_deg: 3.0

  id25_route_6_arrival_pose:
    marker_id: 25
    expected_base_yaw_deg: -87.0
    yaw_tolerance_deg: 3.0
    observed_pose_viewer:
      x_m: [-0.167, -0.166]
      z_m: [0.386, 0.391]
      distance_m: [0.423, 0.427]
```

## 8. Fallback 규칙 초안

| 상황 | 동작 |
| --- | --- |
| 모든 마커가 gate 실패 | 보정 적용 안 함 |
| 마커 lost | 마지막 보정값을 새로 publish하지 않음 |
| yaw reject 급증 | 해당 marker를 일시 제외하고 다음 frame에서 재평가 |
| 거리 gate 실패 | 보정 적용 안 함. 권장 거리로 재접근 유도 |
| ID24/ID25 모두 안정 | 점수화 기준으로 하나만 선택 |
| 선택 marker가 frame마다 흔들림 | hysteresis를 둬서 일정 frame 동안 기존 marker 유지 |

hysteresis 초안:

```text
현재 선택 marker가 정상 gate를 통과 중이면 바로 바꾸지 않는다.
다른 marker가 5 frame 이상 더 좋은 점수를 유지할 때만 switching한다.
```

## 9. 구현 시 확인할 항목

1. pose viewer의 `distance`와 `z` 중 코드에서 어떤 값을 distance gate로 사용할지 결정한다.
2. marker별 gate 설정을 launch argument로 둘지, YAML config로 둘지 결정한다.
3. `CorrectPoseWithAruco` Action Result에 선택된 marker id와 reject reason을 포함할지 검토한다.
4. no marker / all rejected / unstable marker를 관제가 구분할 수 있도록 result code를 정리한다.
5. 오른쪽 벽 마커 추가 후 같은 표에 ID26/ID27 조건을 추가한다.
