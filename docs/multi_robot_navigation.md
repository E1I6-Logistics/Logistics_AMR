# Gazebo 멀티로봇 + Nav2 + RViz 구성

## 한 줄 요약

Gazebo에는 로봇 3대를 한 번만 띄우고, ROS 2에서는 로봇마다 namespace가 다른 Nav2를 하나씩 실행한다. RViz는 기본적으로 한 창만 열어서 선택한 로봇을 조작한다.

## 전체 구조

```mermaid
flowchart LR
    CONFIG["multi_robot.yaml<br/>로봇 이름 · 스폰 위치 · 초기 자세"]

    subgraph SIM["Gazebo Sim · amr_workcell.sdf"]
        GZ1["robot1"]
        GZ2["robot2"]
        GZ3["robot3"]
    end

    BRIDGE["ros_gz_bridge<br/>/clock은 공용<br/>나머지는 로봇별 namespace"]

    subgraph ROS["ROS 2 / Nav2"]
        N1["/robot1<br/>AMCL · Planner · Controller"]
        N2["/robot2<br/>AMCL · Planner · Controller"]
        N3["/robot3<br/>AMCL · Planner · Controller"]
    end

    RVIZ["RViz 1개<br/>기본: robot1"]
    FMS["FMS / Zenoh Gateway<br/>추후 연결"]

    CONFIG --> SIM
    CONFIG --> ROS
    SIM <--> BRIDGE
    BRIDGE <--> ROS
    N1 <--> RVIZ
    ROS <--> FMS
```

## 왜 이 구조인가?

### 1. 로봇 설정을 한 파일에서 관리

로봇 이름과 시작 위치는 `multi_robot.yaml` 하나만 수정한다.

```yaml
robots:
  - name: robot1
    x: 2.37
    y: -1.30
    z: 0.01
    yaw: 3.141592653589793
```

Gazebo spawn과 AMCL 초기 위치가 같은 파일을 사용하므로 좌표를 두 군데에 중복 입력하지 않는다.

### 2. ROS 토픽은 namespace로 분리

```text
/robot1/scan       /robot2/scan       /robot3/scan
/robot1/odom       /robot2/odom       /robot3/odom
/robot1/cmd_vel    /robot2/cmd_vel    /robot3/cmd_vel
/robot1/tf         /robot2/tf         /robot3/tf
```

`/clock`만 모든 로봇이 공유한다.

### 3. TF 프레임 이름은 표준 이름 유지

각 로봇의 TF 토픽이 이미 분리되어 있으므로 프레임 이름까지 `robot1/base_link`처럼 바꾸지 않는다.

```text
/robot1/tf 안: map → odom → base_footprint → base_link
/robot2/tf 안: map → odom → base_footprint → base_link
/robot3/tf 안: map → odom → base_footprint → base_link
```

이 방식이면 TurtleBot3와 Nav2의 기본 프레임 파라미터인 `map`, `odom`, `base_link`를 그대로 사용할 수 있다.

### 4. Nav2는 로봇마다 독립 실행

각 로봇은 서로 다른 Nav2 노드를 갖는다.

```text
/robot1/amcl
/robot1/planner_server
/robot1/controller_server
/robot1/bt_navigator

/robot2/amcl
/robot2/planner_server
...
```

한 로봇의 경로 계획이나 복구 동작이 다른 로봇의 Nav2 상태에 직접 영향을 주지 않는다.

### 5. RViz는 기본적으로 한 창만 실행

RViz 세 창을 동시에 열면 CPU와 GPU 사용량이 커진다. 기본값은 `robot1` 한 창이며 실행할 때 변경할 수 있다.

```bash
ros2 launch turtlebot3_navigation2 multi_robot_navigation2.launch.py \
  rviz_robot:=robot2
```

Nav2 세 개는 모두 실행 중이고 RViz가 바라보는 로봇만 변경된다.

## 실행 방법

### 빌드

```bash
cd /home/woozoo/dev/Logistics_AMR
source /opt/ros/jazzy/setup.bash
colcon build --packages-select turtlebot3_gazebo turtlebot3_navigation2 --symlink-install
source install/setup.bash
```

### Gazebo + 로봇 3대 + Nav2 3개 + RViz 실행

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 multi_robot_navigation2.launch.py
```

### RViz 없이 실행

```bash
ros2 launch turtlebot3_navigation2 multi_robot_navigation2.launch.py \
  use_rviz:=false
```

### Gazebo는 이미 실행 중이고 Nav2만 실행

```bash
ros2 launch turtlebot3_navigation2 multi_robot_navigation2.launch.py \
  use_simulator:=false
```

### Gazebo GUI 없이 실행

```bash
ros2 launch turtlebot3_navigation2 multi_robot_navigation2.launch.py \
  gui:=false
```

## RViz에서 주행시키기

1. RViz의 Fixed Frame이 `map`인지 확인한다.
2. 지도, LaserScan, RobotModel이 표시되는지 확인한다.
3. AMCL 초기 위치는 `multi_robot.yaml`의 spawn 좌표로 자동 설정된다.
4. `Nav2 Goal` 버튼으로 목표 위치를 지정한다.
5. 선택한 로봇 namespace의 `navigate_to_pose` 액션으로 명령이 전달된다.

## 주요 파일

| 파일 | 역할 |
|---|---|
| `turtlebot3_gazebo/params/multi_robot.yaml` | 로봇 목록과 초기 위치의 단일 관리 지점 |
| `turtlebot3_gazebo/launch/multi_robot_my_map_world.launch.py` | Gazebo, robot_state_publisher, ros_gz_bridge 실행 |
| `turtlebot3_navigation2/launch/multi_robot_navigation2.launch.py` | Nav2 3개와 RViz 실행을 총괄 |
| `turtlebot3_navigation2/param/burger.yaml` | Nav2 동작 파라미터 |
| `turtlebot3_navigation2/map/my_map.yaml` | AMCL과 planner가 사용하는 공용 지도 |

## FMS / Zenoh 연결 위치

Nav2까지 정상 동작하면 FMS Gateway는 다음 경계에 연결한다.

```text
Zenoh robot1/goal
        ↓
/robot1/navigate_to_pose 액션

/robot1/odom + Nav2 상태
        ↓
Zenoh robot1/telemetry
```

Gazebo와 Nav2는 실제 로봇과 동일한 ROS 인터페이스를 제공하고, Gateway만 시뮬레이션과 실제 AMR에서 공통으로 사용한다.

## 확인 명령

```bash
ros2 node list | grep robot
ros2 topic list | grep robot
ros2 action list | grep navigate_to_pose
ros2 topic hz /robot1/scan
ros2 topic hz /robot1/odom
```

예상되는 핵심 액션은 다음과 같다.

```text
/robot1/navigate_to_pose
/robot2/navigate_to_pose
/robot3/navigate_to_pose
```
