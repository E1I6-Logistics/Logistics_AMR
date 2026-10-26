# ArUco 위치 보정 다음 작업 계획

이 문서는 새 채팅이나 새 작업을 시작할 때 ArUco 위치 보정 작업의 다음 일정을 바로 이어가기 위한 기준 문서이다.

## 현재 결론

2026-09-30 기준으로 아래쪽 벽 ID24/ID25는 네 변 4cm 마커로 재출력/재부착했고, 중심 좌표를 최종 확정했다.

```text
ID24: x=0.9685766435, y=-0.6782363343, z=0.142, yaw_deg=90, size=0.0400
ID25: x=1.7693385363, y=-0.6699184167, z=0.142, yaw_deg=90, size=0.0400
```

ID24/ID25는 새 부착 기준에서 정지 `publish-tf` 안정성을 확인했다.

```text
ID24 map -> base_link: x=1.030, y=-0.230, z=0.010, yaw=-94.78deg
ID25 map -> base_link: x=1.813, y=-0.203, z=0.010, yaw=-95.81deg
```

2026-10-01에는 로봇이 실제 6번 좌표에 도착한 자세 기준으로 ID25를 다시 확인했다. 이 자세에서는 ID25가 화면 중앙에서는 벗어나지만 안정적으로 검출되었고, yaw gate는 별도 운용값 `expected_base_yaw_deg=-87`, `yaw_tolerance_deg=3`을 사용한다.

```text
ID25 @ 6번 좌표 실제 도착 자세:
pose_viewer x=-0.166~-0.167m, z=0.386~0.391m, distance=0.423~0.427m
dry-run accepted=24 -> 241
median yaw=-86.7~-86.8deg
```

이후 짧은 직진/후진 거리 테스트 결과, 아래쪽 벽 ID24/ID25 현장 권장 사용 거리는 약 `0.35m ~ 0.55m`로 좁혀 잡는다.

```text
ID24 권장: 0.35m ~ 0.55m
ID24 제외: 0.30m 이하, 0.60m 이상
ID25 권장: 0.35m ~ 0.55m
ID25 주의: 0.55m ~ 0.65m
ID25 제외: 0.30m 이하, 0.65m 이상
```

상세 테스트 로그와 판정은 아래 문서에 정리되어 있다.

```text
docs/aruco_marker_map_final_id25_publish_test_20260930.md
docs/aruco_confluence_test_summary_20260930.md
docs/aruco_marker_selection_policy_draft_20260930.md
docs/aruco_tf_continuity_test_id25_20260929.md
```

## 사전 준비

아래 작업은 완료되었다.

1. 네 변이 모두 정확히 4cm인 ArUco 마커 재출력
2. 4cm 정사각형 기준으로 재단

4cm 마커를 유지하는 이유:

```text
- 로봇팔 또는 구조물에 가려질 가능성 고려
- 벽면 부착 공간 고려
- 큰 마커 1개보다 작은 마커 여러 개를 분산 배치하는 전략이 현재 환경에 더 적합
```

## 1일차: 아래쪽 벽 ID24/ID25 최종 좌표 및 운용 조건 확정

1. 아래쪽 벽 ID24/ID25 재부착 - 완료
2. `marker_map.yaml` 중심 좌표 재측정 - 완료
3. PC 프로젝트의 `marker_map.yaml` 수정 및 현장 로봇 `~/aruco_test/marker_map.yaml` 동기화 - 완료
4. ID24/ID25 pose viewer 검출 확인 - 완료
5. ID24/ID25 dry-run 재검증 - 완료
6. ID24/ID25 yaw gate / outlier 조건 1차 재조정 - 완료
7. ID24/ID25 `--publish-tf` 상태에서 정지 테스트 - 완료
8. ID24/ID25 직진/후진 중 `map -> base_link` 연속성 확인 - 완료
9. ID24/ID25 권장 거리 범위 재정리 - 완료
10. 아래쪽 벽 기준 보정 조건 문서화 - 완료
11. 현장 실행 이슈 정리 - 완료
    - `TURTLEBOT3_MODEL=burger`
    - `LDS_MODEL=LDS-01`
    - 로봇에서는 `~/aruco_test` standalone script 사용
    - `marker_map.yaml` YAML 문법 오류 발생 시 들여쓰기/중복 key 확인
12. 추가로 할 수 있는 일
    - 권장 거리 조건을 코드/launch 설정의 distance gate 후보로 반영
    - 마커 선택 기준 초안에 ID24/ID25 권장 거리와 yaw gate 반영 - 완료
    - 로봇 앞면/카메라 렌즈/마커 중심 기준 거리 환산표를 운용 문서에 추가
    - 현장 실행 명령을 체크리스트 형태로 정리

주의:

```text
- marker_map.yaml의 x/y/z는 마커 모서리가 아니라 중심 좌표 기준
- z는 바닥부터 마커 중심까지의 높이 기준
- yaw_deg는 마커가 바라보는 벽 법선 방향 기준
```

1일차 판정 기준:

```text
- tf2_echo map base_link의 x/y/yaw가 큰 점프 없이 연속적으로 변해야 함
- 기존 문제였던 yaw -98deg, -84deg 방향의 순간 점프가 없어야 함
- no marker와 reject가 급증하는 거리 구간 기록
```

1일차 최종 조건:

```text
ID24:
  expected_base_yaw_deg=-95
  yaw_tolerance_deg=3
  권장 거리: 0.35m ~ 0.55m
  제외 거리: 0.30m 이하, 0.60m 이상

ID25:
  expected_base_yaw_deg=-96  # 정면 배치 기준
  expected_base_yaw_deg=-87  # 6번 좌표 실제 도착 자세 기준
  yaw_tolerance_deg=3
  권장 거리: 0.35m ~ 0.55m
  주의 거리: 0.55m ~ 0.65m
  제외 거리: 0.30m 이하, 0.65m 이상

공통:
  marker_size=0.0400m
  outlier_z_max=0.15
  outlier_tilt_deg=25
```

## 2일차: 오른쪽 벽 마커 추가

1. 오른쪽 벽 마커 2개 부착
2. 오른쪽 벽 마커 `marker_map.yaml` 좌표 추가
3. 오른쪽 벽 마커별 yaw 기준값 측정
4. 오른쪽 벽 기준 dry-run / publish-tf 테스트

오른쪽 벽 마커는 아래쪽 벽 마커와 기대 yaw가 다를 수 있다. 배치 후 실제 dry-run 결과를 보고 마커별 `expected_base_yaw_deg`를 정한다.

예시:

```yaml
id: 24
expected_base_yaw_deg: -90
yaw_tolerance_deg: 4

id: 25
expected_base_yaw_deg: -90
yaw_tolerance_deg: 4

id: 26
expected_base_yaw_deg: TBD
yaw_tolerance_deg: 4

id: 27
expected_base_yaw_deg: TBD
yaw_tolerance_deg: 4
```

## 3일차: 복수 마커 운용 규칙 정리

1. 여러 마커 중 사용할 마커 선택 기준 정리
2. 마커 lost / no marker 상황 테스트
3. 로봇팔 또는 구조물 가림 상황 테스트
4. fallback 운용 규칙 정리

마커 선택 기준이란, 여러 마커가 동시에 보일 때 어떤 마커의 보정값을 사용할지 정하는 규칙이다.
초안은 `docs/aruco_marker_selection_policy_draft_20260930.md`에 작성했다.

예시 기준:

```text
1. yaw gate를 통과한 마커만 사용
2. 거리 조건이 좋은 마커 우선: 아래쪽 벽 ID24/ID25는 약 0.35m ~ 0.55m, 다른 마커는 마커별 테스트 결과 기준
3. 화면 중앙에 가까운 마커 우선: pose viewer 기준 x가 0에 가까움
4. 최근 몇 프레임 동안 안정적으로 검출된 마커 우선
5. reject가 적은 마커 우선
6. 모두 안정적이면 사전에 정한 ID 우선순위 사용
```

## 4일차: Action / Nav2 흐름 통합

1. `CorrectPoseWithAruco` Action과 보정 흐름 통합
2. `/initialpose` publish 방식 테스트
3. `map -> odom` publish 실험 옵션 정리
4. Nav2 / 관제 호출 흐름 점검

기본 보정 방식은 AMCL/Nav2와 TF 충돌이 나지 않도록 `/initialpose` publish를 우선한다.
`map -> odom` TF publish는 단독 실험용 옵션으로 유지한다.

## 5일차: 설정 정리 및 반복 테스트

1. launch/config 기본값 정리
2. 거리 gate, yaw gate, outlier 조건의 기본값 후보 정리
3. 반복 테스트 및 실패 케이스 정리
4. 아래쪽 벽/오른쪽 벽 마커 설정 비교
5. Confluence 문서 업데이트

## 6일차: 최종 시나리오 점검 및 공유

1. `/initialpose` 기반 보정 후 Nav2 주행 연결 확인
2. 보정 실패 시 fallback 흐름 확인
3. 관제 호출 시나리오 점검
4. 실물 반복 주행 전 최종 체크리스트 작성
5. Slack 공유용 요약 정리

## 남은 핵심 의사결정

| 항목 | 현재 방향 |
| --- | --- |
| 마커 크기 | 4cm 유지 |
| 안정화 전략 | 큰 마커보다 4cm 복수 마커 + yaw gate + 거리 제한 |
| ID24 권장 거리 | 약 0.35m ~ 0.55m |
| ID24 제외 거리 | 약 0.30m 이하, 약 0.60m 이상 |
| ID25 권장 거리 | 약 0.35m ~ 0.55m |
| ID25 제외 거리 | 약 0.30m 이하, 약 0.65m 이상 |
| 기본 보정 방식 | `/initialpose` publish |
| `map -> odom` publish | 단독 실험용 옵션 |
| 회전 중 보정 | 현재 yaw gate 조건에서는 별도 설계 필요 |
| ID24 아래쪽 벽 yaw gate | `expected_base_yaw_deg=-95`, `yaw_tolerance_deg=3` |
| ID25 아래쪽 벽 yaw gate | 정면 배치 기준 `expected_base_yaw_deg=-96`, 6번 좌표 도착 자세 기준 `expected_base_yaw_deg=-87`, `yaw_tolerance_deg=3` |

## 새 채팅에서 바로 확인할 파일

```text
docs/PROJECT_STATUS.md
docs/aruco_marker_map_final_id25_publish_test_20260930.md
docs/aruco_confluence_test_summary_20260930.md
docs/aruco_marker_selection_policy_draft_20260930.md
docs/aruco_next_work_plan_20260929.md
docs/aruco_tf_continuity_test_id25_20260929.md
```
