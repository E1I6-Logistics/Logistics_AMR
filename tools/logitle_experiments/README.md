# logitle_experiments

Main PC에서 TurtleBot3 실기체의 AMCL 정합, XY Goal Tolerance, 선속도 상한 시험을
반복 실행하고 결과를 CSV로 저장하는 ROS 2 Python 패키지다.

## 원칙

- 로봇 bringup과 Nav2는 기존 방식으로 별도 실행한다.
- 이 패키지는 운용 파라미터 파일을 수정하지 않고 런타임 파라미터만 임시 변경한다.
- 변경 전 파라미터를 저장하고 종료·중단 시 복구한다.
- START 복귀와 물리 측정은 작업자가 수행한다.
- AMCL·TF·Action·속도 명령은 프로그램이 기록한다.
- trial은 성공·실패·중단 여부와 관계없이 즉시 CSV에 저장한다.

## 빌드

```bash
cd /home/woozoo/dev/Logistics_AMR_logitle
source /opt/ros/jazzy/setup.bash
colcon build --packages-select logitle_experiments --symlink-install
source install/local_setup.bash
```

다른 AMR workspace를 같은 터미널에서 함께 source하지 않는다.

## turtle2 사전 확인

```bash
export ROS_DOMAIN_ID=31
ros2 topic info /amcl_pose
ros2 action list | grep navigate_to_pose
ros2 param get /controller_server goal_checker.xy_goal_tolerance
```

`--check-only`와 위 확인 단계에서는 initial pose를 미리 줄 필요가 없다. 실제 trial에서
runner가 물리 START/GOAL의 설정 좌표를 `/initialpose`로 발행한 뒤 `/amcl_pose`와 TF를
검증한다.

## AMCL 기준점 정합·안정성 시험

먼저 통신과 설정만 확인한다.

```bash
ros2 run logitle_experiments amcl_validation -- \
  --robot-config logitle_experiments/config/robots/turtle2.yaml \
  --scenario-config logitle_experiments/config/scenarios/goal_alignment.yaml \
  --experiment-config logitle_experiments/config/experiments/amcl_validation.yaml \
  --check-only
```

실험 실행:

```bash
ros2 run logitle_experiments amcl_validation -- \
  --robot-config logitle_experiments/config/robots/turtle2.yaml \
  --scenario-config logitle_experiments/config/scenarios/goal_alignment.yaml \
  --experiment-config logitle_experiments/config/experiments/amcl_validation.yaml \
  --output-root experiment_data \
  --repo-path /home/woozoo/dev/Logistics_AMR_logitle
```

## XY Goal Tolerance 시험

```bash
ros2 run logitle_experiments navigation_sweep -- \
  --robot-config logitle_experiments/config/robots/turtle2.yaml \
  --scenario-config logitle_experiments/config/scenarios/straight_goal.yaml \
  --experiment-config logitle_experiments/config/experiments/xy_tolerance_sweep.yaml \
  --output-root experiment_data \
  --repo-path /home/woozoo/dev/Logistics_AMR_logitle
```

조건은 설정 파일의 `condition_order` 순서로 실행한다. 기본값은 각 조건을 시간대별로
섞은 15회 순서다.

## 선속도 상한 시험

`speed_sweep.yaml`의 `fixed_parameters`에 있는 Goal Tolerance를 AMCL 검증과 XY
재시험에서 선택한 값으로 먼저 수정한다.

```bash
ros2 run logitle_experiments navigation_sweep -- \
  --robot-config logitle_experiments/config/robots/turtle2.yaml \
  --scenario-config logitle_experiments/config/scenarios/straight_goal.yaml \
  --experiment-config logitle_experiments/config/experiments/speed_sweep.yaml \
  --output-root experiment_data \
  --repo-path /home/woozoo/dev/Logistics_AMR_logitle
```

## 결과 분석

```bash
ros2 run logitle_experiments analyze_run -- \
  experiment_data/<run_id>
```

생성 결과:

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

`trace.csv`, rosbag, 전체 원본 사진은 Git에서 제외한다. metadata, config snapshot,
trials, summary와 확정 그래프는 Git에 포함할 수 있다.

## Notebook

`notebooks/navigation_experiment_analysis.ipynb`의 `RUN_DIR`만 실제 실행 폴더로 바꾼다.
Notebook은 패키지의 동일한 분석 함수를 호출하므로 CLI와 통계 계산식이 같다.

자동 기록값, 계산값과 작업자 입력값의 구분은
`docs/07_experiment_data_dictionary.md`에 정리돼 있다.
