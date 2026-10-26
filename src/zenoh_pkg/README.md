# zenoh_pkg

`zenoh_pkg`는 로봇의 ROS 2 통신을 Main PC의 Zenoh Router에 연결하기 위한 ROS 2 Python 패키지입니다.

Launch 파일을 실행하면 다음 두 프로세스가 함께 시작됩니다.

- 전달받은 `robot_id`를 표시하는 `zenoh_bridge_node`
- 로봇 namespace를 적용한 `zenoh-bridge-ros2dds`

## 사전 준비

- ROS 2 Jazzy
- `zenoh-bridge-ros2dds`
- 실행 중인 Main PC의 Zenoh Router(기본 포트 `7447`)

## 빌드

Logistics_AMR workspace 루트에서 빌드합니다.

```bash
cd ~/Logistics_FMS/robots_ws/Logistics_AMR
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select zenoh_pkg
source install/setup.bash
```

## 실행

`robot_id`에는 현재 로봇의 ID를, `router_ip`에는 Main PC의 IP 주소를 입력합니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/Logistics_FMS/robots_ws/Logistics_AMR/install/setup.bash

# ros_local 필수
ros_local

ros2 launch zenoh_pkg zenoh.launch.py \
  robot_id:=robot1 \
  router_ip:=Main Pc zenoh router IP
```

다른 로봇에서는 `robot_id`를 `robot2`, `robot3`처럼 변경합니다. `router_ip`도 실제 Main PC 주소에 맞게 변경해야 합니다.

실행이 완료되면 Bridge는 다음 endpoint에 client로 연결됩니다.

```text
tcp/<router_ip>:7447
```

종료하려면 실행한 터미널에서 `Ctrl+C`를 누릅니다.

## 주요 파일

```text
zenoh_pkg/
├── config/bridge.json5          # Bridge의 ROS 2 route 필터와 discovery 설정
├── launch/zenoh.launch.py       # ROS 노드와 Bridge 통합 실행
└── zenoh_pkg/zenoh_bridge_node.py
```

## 확인

```bash
ros2 node list
ros2 topic list
```

연결되지 않으면 Main PC의 Zenoh Router 실행 여부, `router_ip`, 7447 포트, `ROS_DOMAIN_ID`를 확인합니다.
