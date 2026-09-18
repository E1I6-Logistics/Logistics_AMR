# Multi-Robot Goal Dispatcher

## 목적

RViz 하나에서 선택한 로봇 한 대에만 Nav2 `NavigateToPose` Goal을 전달한다.
이 노드는 수동 멀티로봇 주행 검증용이며 Fleet Manager나 Traffic Manager를
대체하지 않는다.

```text
선택 로봇: /multi_robot/selected_robot
목표 자세: /multi_robot/goal_pose
                    │
                    ▼
      multi_robot_goal_dispatcher
                    │
       ┌────────────┼────────────┐
       ▼            ▼            ▼
/robot1/...   /robot2/...   /robot3/...
navigate_to_pose Action
```

## 패키지 구분

```text
logistics_amr_goal_dispatcher
└─ Goal 검증과 NavigateToPose Action 전달 로직

logistics_amr_rviz_plugins
└─ 로봇 선택, Goal 상태 확인, Goal 취소 RViz 패널

logistics_amr_bringup
└─ Gazebo, Nav2, Dispatcher, RViz를 한 번에 실행

turtlebot3_navigation2
└─ 기존 로봇별 Nav2 실행 기능
```

상위 bringup은 기존 TurtleBot3 런처를 `Include`한다. Dispatcher 때문에 기존
TurtleBot3 Nav2 구현을 수정하지 않는다.

## 빌드

```bash
cd /home/woozoo/dev/Logistics_AMR
source /opt/ros/jazzy/setup.bash

colcon build --packages-select \
  turtlebot3_gazebo \
  turtlebot3_navigation2 \
  logistics_amr_goal_dispatcher \
  logistics_amr_rviz_plugins \
  logistics_amr_bringup \
  --symlink-install

source install/setup.bash
```

## 전체 실행

```bash
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=96

ros2 launch logistics_amr_bringup multi_robot_sim.launch.py
```

기존 명령도 유지된다. 기존 명령은 Dispatcher 없이 이전 방식으로 실행된다.

```bash
ros2 launch turtlebot3_navigation2 multi_robot_navigation2.launch.py
```

## 사용 방법

### 1. RViz 패널에서 로봇 선택

RViz의 `Multi-Robot Control` 패널에서 `Target Robot`을 고르고
`Select Robot` 버튼을 누른다. 로봇 목록은 Dispatcher가 `robots_file`에서
읽어 패널에 자동으로 제공하므로 패널에 `robot1`, `robot2`, `robot3`을
하드코딩하지 않는다.

이후 RViz 상단의 `2D Goal Pose`로 목표를 지정한다. 패널에서는 현재 선택된
로봇의 Goal 상태와 남은 거리를 확인하고 `Cancel Goal`로 주행을 취소할 수 있다.

### 2. 상태 확인 터미널

```bash
ros2 topic echo /multi_robot/goal_status
```

### 3. 대상 로봇 선택(터미널 대체 방법)

```bash
ros2 topic pub --once \
  /multi_robot/selected_robot \
  std_msgs/msg/String \
  "{data: robot2}"
```

선택 가능한 이름은 `multi_robot.yaml`의 로봇 목록으로 제한된다.

처음부터 로봇 하나를 선택해서 실행할 수도 있다.

```bash
ros2 launch logistics_amr_bringup multi_robot_sim.launch.py \
  selected_robot:=robot1
```

### 4. RViz에서 Goal 지정

RViz 상단의 `2D Goal Pose`를 선택하고 지도 위에서 드래그한다.

```text
클릭 위치  -> 목표 x, y
화살표 방향 -> 목표 yaw
```

RViz는 `/multi_robot/goal_pose`에 `PoseStamped`를 발행하고 Dispatcher는 현재
선택된 로봇의 `/robotN/navigate_to_pose` Action으로 전달한다.

`Publish Point`는 `/clicked_point`에 위치만 발행하므로 Goal 전송용이 아니라
좌표 확인용이다.

### 5. Goal 취소

```bash
ros2 topic pub --once \
  /multi_robot/cancel_goal \
  std_msgs/msg/String \
  "{data: robot2}"
```

메시지의 로봇 이름을 비우면 현재 선택된 로봇의 Goal 취소를 시도한다.

## 인터페이스

| 이름 | 타입 | 방향 | 역할 |
|---|---|---|---|
| `/multi_robot/selected_robot` | `std_msgs/String` | 입력 | Goal을 받을 로봇 선택 |
| `/multi_robot/robots` | `std_msgs/String` | 출력 | RViz 패널에 제공하는 로봇 목록 |
| `/multi_robot/goal_pose` | `geometry_msgs/PoseStamped` | 입력 | RViz가 만든 map 기준 목표 자세 |
| `/multi_robot/cancel_goal` | `std_msgs/String` | 입력 | 지정 로봇 Goal 취소 |
| `/multi_robot/goal_status` | `std_msgs/String` | 출력 | JSON 형태의 선택·수락·결과 상태 |
| `/robotN/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | 출력 | 선택 로봇의 Nav2 Action |

상태 메시지 예시:

```json
{"state": "accepted", "robot": "robot2", "detail": ""}
```

## 안전 조건

- 로봇이 선택되지 않았으면 Goal을 거부한다.
- `robots_file`에 없는 로봇 이름을 거부한다.
- Goal의 `frame_id`가 `map`이 아니면 거부한다.
- 위치에 NaN 또는 무한대가 있으면 거부한다.
- orientation quaternion이 유효하지 않으면 거부한다.
- 선택한 로봇의 Action Server가 준비되지 않았으면 거부한다.
- 같은 로봇에 Goal이 진행 중이면 새 Goal을 거부한다.
- Goal을 모든 로봇에 동시에 broadcast하지 않는다.

## RViz 패널

기본 RViz 설정에는 다음 패널이 자동으로 포함된다.

```text
Target Robot: [robot2 ▼] [Select Robot]
Selected: robot2
Goal State: ACTIVE
Detail: 1.25 m remaining
[Cancel Goal]
```

패널은 선택과 표시만 담당한다. Goal 전달·검증 로직은 Dispatcher 노드에
유지하므로 RViz가 종료되어도 주행 중인 Nav2 Goal은 Dispatcher가 계속 관리한다.
