# Workcell SLAM 매핑

## 목적

`amr_workcell.sdf`에 로봇 한 대만 스폰하고 SLAM Toolbox로 Nav2용 2D 지도를
생성한다. 매핑 중에는 AMCL과 기존 정적 `map_server`를 실행하지 않는다.

## 실행 구조

```text
mapping_robot.yaml
        ↓
Gazebo + mapping robot 1대
        ↓ odom/scan 실제 수신 확인
TF Aggregator + SLAM Toolbox
        ↓ SLAM Toolbox ACTIVE 확인
매핑 전용 RViz
```

`mapping_robot.yaml`에는 반드시 로봇 한 대만 있어야 한다. 런치파일은 YAML에서
로봇 이름을 읽어 `robotN/odom`, `robotN/base_footprint`, `/robotN/scan`과 RViz
설정을 자동으로 만든다.

```yaml
robots:
  - name: robot1
    spawn_pose_world:
      x: 2.48
      y: -1.27
      z: 0.01
      yaw: 0.0
    initial_pose_map:
      x: 0.0
      y: 0.0
      z: 0.0
      yaw: 0.0
```

매핑 실행에서 Gazebo는 `spawn_pose_world`만 사용한다. SLAM을 시작하면 이 로봇
위치가 새 지도의 `(0, 0)` 부근이 되므로 `initial_pose_map`은 이후 저장된
지도를 AMCL로 다시 불러올 때 사용할 기준값이다. 두 값은 한 파일에서 함께
관리하지만 서로 다른 좌표계의 값이다.

## 빌드

```bash
cd /home/woozoo/dev/Logistics_AMR
source /opt/ros/jazzy/setup.bash
colcon build --packages-select \
  turtlebot3_gazebo \
  turtlebot3_navigation2 \
  logistics_amr_bringup \
  --symlink-install
source install/setup.bash
```

## 매핑 실행

기존 멀티로봇 Nav2 launch를 먼저 종료한 후 실행한다.

```bash
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=96

ros2 launch logistics_amr_bringup workcell_mapping.launch.py
```

정상 기동 로그는 다음 순서로 출력된다.

```text
robot1 mapping topics: ready
Starting SLAM Toolbox for robot1
SLAM Toolbox is ACTIVE
SLAM Toolbox: ready
```

Gazebo가 이미 `mapping_robot.yaml`로 실행 중이면 다음처럼 SLAM과 RViz만 붙인다.

```bash
ros2 launch logistics_amr_bringup workcell_mapping.launch.py \
  use_simulator:=false
```

## 키보드 주행

키보드 입력이 필요하므로 Teleop은 별도 터미널에서 실행한다.

```bash
source /opt/ros/jazzy/setup.bash
source /home/woozoo/dev/Logistics_AMR/install/setup.bash
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=96

ros2 run turtlebot3_teleop teleop_keyboard \
  --ros-args \
  -r cmd_vel:=/robot1/cmd_vel
```

천천히 이동하고, 회전할 때는 제자리에서 빠르게 돌기보다 주변 벽이 계속
LiDAR에 보이도록 완만하게 회전한다. 출발 지점으로 다시 돌아오면 Loop Closure
검증에 도움이 된다.

## 지도 저장

지도 모양을 확인한 뒤 별도 터미널에서 명시적으로 저장한다. 기존 맵을
덮어쓰지 않도록 버전 이름을 사용한다.

```bash
ros2 run nav2_map_server map_saver_cli \
  -f /home/woozoo/dev/Logistics_AMR/src/turtlebot3/turtlebot3_navigation2/map/amr_workcell_slam_v1
```

다음 두 파일이 생성된다.

```text
amr_workcell_slam_v1.pgm
amr_workcell_slam_v1.yaml
```

저장 후에는 새 YAML을 멀티로봇 Navigation 런치의 `map` 인자로 전달해 AMCL
초기 위치와 실제 스폰 위치가 일치하는지 검증한다.
