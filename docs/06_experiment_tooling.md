# 실기체 실험 실행·기록·분석 도구

## 1. 구성과 책임

`logitle_experiments`는 Main PC에서 실행하는 ROS 2 Python 패키지다. 로봇 bringup,
Nav2 launch 및 운영 설정 파일은 수정하지 않는다.

| 구성 | 책임 |
|---|---|
| `amcl_validation` | 물리 기준점, scan-map, AMCL 및 TF 정합·정지 안정성 기록 |
| `navigation_sweep` | Goal Tolerance 또는 선속도 조건 적용, Goal 전송, 실측 입력 |
| `analyze_run` | `summary.csv`와 조건별 PNG 그래프 생성 |
| Notebook | 같은 분석 함수를 이용한 추가 탐색과 시각화 |

## 2. 빌드

```bash
cd /home/woozoo/dev/Logistics_AMR_logitle
source /opt/ros/jazzy/setup.bash
colcon build --packages-select logitle_experiments --symlink-install
source install/local_setup.bash
```

다른 AMR workspace를 같은 터미널에서 함께 source하지 않는다.

## 3. turtle2 공통 환경

```bash
export ROS_DOMAIN_ID=31

ros2 topic echo /amcl_pose --once
ros2 action list | grep navigate_to_pose
ros2 param get /controller_server goal_checker.xy_goal_tolerance
```

실제 시험 전에 `--check-only`를 붙여 AMCL publisher, `/initialpose` subscriber와 해당
runner에 필요한 action 및 parameter service 연결을 검증한다. 이 단계에서는 initial
pose를 발행하지 않으므로 미리 `2D Pose Estimate`를 줄 필요가 없다. TF와 실제
`/amcl_pose` 수신은 trial의 initial pose 발행 직후 검증한다. Domain ID 환경값과
robot config가 다르면 runner는 시작하지 않는다.

## 4. AMCL 기준점 시험

```bash
ros2 run logitle_experiments amcl_validation -- \
  --robot-config logitle_experiments/config/robots/turtle2.yaml \
  --scenario-config logitle_experiments/config/scenarios/goal_alignment.yaml \
  --experiment-config logitle_experiments/config/experiments/amcl_validation.yaml \
  --output-root experiment_data \
  --repo-path /home/woozoo/dev/Logistics_AMR_logitle
```

현재 1차 시험은 왼쪽 탄착군이 관찰된 GOAL에서 3회 반복한다.

1. 작업자가 로봇 기준점을 물리 GOAL 마킹에 맞춘다.
2. runner가 동일한 reference pose를 `/initialpose`로 발행한다.
3. 작업자가 RViz scan-map 정합 여부를 입력한다.
4. 작업자가 0.2 rad보다 크게 회전한 후 원래 방향으로 돌아온다.
5. runner가 AMCL, covariance, TF와 30초 정지 표본을 기록한다.

initial pose를 정답 좌표로 넣으므로 이 결과만으로 AMCL 절대 정확도를 판정하지 않는다.
물리 마킹–map–scan–TF의 반복적인 방향 불일치와 정지 갱신 상태를 확인하는 시험이다.

## 5. XY Goal Tolerance 시험

```bash
ros2 run logitle_experiments navigation_sweep -- \
  --robot-config logitle_experiments/config/robots/turtle2.yaml \
  --scenario-config logitle_experiments/config/scenarios/straight_goal.yaml \
  --experiment-config logitle_experiments/config/experiments/xy_tolerance_sweep.yaml \
  --output-root experiment_data \
  --repo-path /home/woozoo/dev/Logistics_AMR_logitle
```

runner가 `/initialpose`, parameter 적용·확인, Goal 전송, 성공 후 3초 대기, AMCL·TF
기록을 수행한다. 작업자는 START 배치, scan-map 정합 확인, 물리 오차(mm), 정지 방향,
진동·overshoot·외부 개입과 사진 파일명을 입력한다.

## 6. 선속도 시험

먼저 `speed_sweep.yaml`의 `fixed_parameters`에 재시험으로 선택한 Goal Tolerance를
기록한다. 현재 `0.10 m`는 1차 결과를 바탕으로 한 후보값이며 최종 운영값이 아니다.

```bash
ros2 run logitle_experiments navigation_sweep -- \
  --robot-config logitle_experiments/config/robots/turtle2.yaml \
  --scenario-config logitle_experiments/config/scenarios/straight_goal.yaml \
  --experiment-config logitle_experiments/config/experiments/speed_sweep.yaml \
  --output-root experiment_data \
  --repo-path /home/woozoo/dev/Logistics_AMR_logitle
```

`FollowPath.max_vel_x`와 `FollowPath.max_speed_xy`를 하나의 선속도 조건으로 함께
적용한다. `/cmd_vel` trace는 로컬에 저장하지만 Git에는 포함하지 않는다.

## 7. 안전과 복구

- parameter 변경 전에 원래 값을 저장한다.
- 적용 후 실제 조회값이 다르면 trial을 시작하지 않는다.
- Ctrl+C 시 실행 중 Goal을 취소하고 parameter 복구를 시도한다.
- 실패·취소·중단 trial도 삭제하지 않는다.
- 복구 실패는 `metadata.yaml`과 `events.jsonl`에 남긴다.
- `--resume`은 기존 `--run-id`를 함께 지정해야 한다.

## 8. 결과 구조

```text
experiment_data/<run_id>/
├── metadata.yaml
├── config_snapshot.yaml
├── trials.csv
├── trace.csv
├── events.jsonl
├── summary.csv
├── amcl_stability_summary.csv
├── figures/
└── photos/
```

- `metadata.yaml`: Git commit, 로봇, 실행 상태, 실제 파일 경로와 parameter 원본값
- `config_snapshot.yaml`: 시험에 사용한 세 YAML의 병합 스냅샷
- `trials.csv`: trial당 한 행의 자동 기록·실측·계산 결과
- `trace.csv`: AMCL 정지 표본 또는 최종 `/cmd_vel` 시계열
- `summary.csv`: 조건별 평균, 중앙값, 표본 표준편차, P95, 최솟값·최댓값, 성공률
- `amcl_stability_summary.csv`: AMCL 시험의 새 timestamp 수, pose 변화폭과 covariance 변화
- `runs_index.csv`: `experiment_data` 아래 실행 전체 목록

각 CSV 열의 기록 주체, 단위와 의미는 `07_experiment_data_dictionary.md`를 따른다.

단위는 필드명에 포함하며 물리 거리와 계산된 위치 오차는 mm로 저장한다. 미측정값은
0이 아닌 빈 값으로 저장한다.

## 9. 분석과 Notebook

실험이 정상 종료되면 runner가 요약과 그래프 생성을 자동 시도한다. 다시 생성하려면
다음 명령을 사용한다.

```bash
ros2 run logitle_experiments analyze_run -- experiment_data/<run_id>
```

`notebooks/navigation_experiment_analysis.ipynb`는 같은 `build_report()` 함수를 호출한다.
Notebook 안에서 통계식을 중복 구현하지 않으므로 CLI와 결과가 일치한다.

기본 그래프:

- 조건별 물리 오차 box plot과 전체 trial 점
- 실행 순서별 물리 오차
- AMCL 추정 오차와 물리 오차 비교
- 조건별 주행 시간
- 정지 방향 빈도
- AMCL 정지 pose 변화
- 최종 `/cmd_vel` 시계열

## 10. Git 관리

Git에 포함할 기본 결과:

- `metadata.yaml`
- `config_snapshot.yaml`
- `trials.csv`
- `summary.csv`
- `amcl_stability_summary.csv`
- 확정 그래프
- 문서용 대표 사진

`trace.csv`, rosbag, 전체 원본 사진과 Notebook 캐시는 `.gitignore`로 제외한다.
