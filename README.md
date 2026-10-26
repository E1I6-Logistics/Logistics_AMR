# Logistics_AMR

TurtleBot3 Burger 3대를 사용하는 물류 로봇 시스템의 **로봇 측 ROS 2 프로젝트**입니다. 각 로봇에서 센서·구동부를 실행하고, Nav2로 이동하며, ArUco 정렬과 정밀 도킹을 수행합니다. 메인 PC의 [Logistics_FMS](https://github.com/E1I6-Logistics/Logistics_AI)와는 **Zenoh ROS2DDS Bridge**로 통신합니다.

## 1. 프로젝트 구조

```text
Logistics_AMR/
├── README.md
├── .gitmodules                     # LDS-02 드라이버 Submodule
├── src/                            # ROS 2 소스 패키지
│   ├── logitle/
│   │   ├── logitle_bringup/        # 로봇 통합 실행 Launch
│   │   ├── logitle_docking/        # 정밀 도킹 Action, 로봇별 YAML
│   │   ├── logitle_aruco_tools/    # ArUco 마커 정렬·위치 보정
│   │   └── logitle_pose/           # 로봇 위치 발행
│   ├── turtlebot3/                 # TurtleBot3 구동·Navigation 관련 패키지
│   │   └── turtlebot3_navigation2/ # Nav2 Launch / 지도 / 파라미터
│   ├── nav2_graph_waypoint_bt/     # 경유지 기반 커스텀 Behavior Tree
│   ├── nav2_straight_first_planner/# 직선 우선 Global Planner
│   ├── zenoh_pkg/                  # Zenoh Bridge Launch 및 필터 설정
│   ├── ld08_driver/                # LDS-02 드라이버 (Git Submodule)
│   └── ...                         # 기타 ROS 패키지 및 외부 의존 소스
├── docs/                           # 주행·위치 추정 실험 문서
├── experiment_data/                # 실험 결과
└── notebooks/                      # 실험 데이터 분석
```

위 구조는 **실행에 필요한 주요 폴더만 추린 것**입니다. `src/` 아래에는 추가 TurtleBot3 패키지와 외부 라이브러리도 포함되어 있습니다. `build/`, `install/`, `log/`는 `colcon build` 과정에서 생성됩니다.

## 2. 구성과 실행 환경

```text
메인 PC: Logistics_FMS + Zenoh Router (TCP 7447)
                        │
                     Zenoh
                        │
TurtleBot3 #1~3: zenoh-bridge-ros2dds (/robot1~3)
                        │
                  ROS 2 Jazzy
                        │
          TurtleBot3 / LDS-02 / EKF / AMCL
                        │
        Nav2 / ArUco / Precision Docking
```

- **하드웨어:** TurtleBot3 Burger, Raspberry Pi 4, OpenCR, LDS-02; ArUco 사용 시 카메라
- **OS / ROS:** Ubuntu 24.04, ROS 2 Jazzy
- **주요 패키지:** `logitle_bringup`(통합 실행), `turtlebot3_navigation2`(주행), `logitle_aruco_tools`(마커 정렬), `logitle_docking`(도킹), `logitle_pose`(위치 발행), `zenoh_pkg`(FMS 통신)
- **추가 Nav2 플러그인:** `nav2_graph_waypoint_bt`, `nav2_straight_first_planner`

> 이 저장소는 로봇에서 실행하는 코드입니다. FMS 서버·웹 대시보드·Zenoh Router는 별도 [Logistics_FMS](https://github.com/E1I6-Logistics/Logistics_FMS)에서 준비합니다.

## 3. 설치 및 빌드

ROS 2 Jazzy와 ROS 개발 도구(`colcon`, `rosdep`)가 설치된 로봇에서 실행합니다. 로봇 장치 드라이버와 `zenoh-bridge-ros2dds` 실행 파일은 환경에 맞게 설치되어 있어야 합니다.

**처음 설치하는 경우** (`~/logitle_ws`가 아직 없을 때):

```bash
source /opt/ros/jazzy/setup.bash
git clone -b dev https://github.com/E1I6-Logistics/Logistics_AMR.git ~/logitle_ws
cd ~/logitle_ws
# src/ld08_driver 등 외부 Git Submodule 내려받기
# LDS-02 드라이버 소스가 누락되지 않게 최초 설치 시 실행
git submodule update --init --recursive

# package.xml에 선언된 ROS/Ubuntu 의존 패키지 설치
# -r: 일부 의존성 오류가 있어도 가능한 항목 계속 처리, -y: 설치 질문 자동 승인
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --executor sequential
source install/setup.bash
```

**이미 설치된 경우:**

```bash
cd ~/logitle_ws
git pull --ff-only origin dev
git submodule update --init --recursive
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --executor sequential
source install/setup.bash
```

**두 명령어의 의미**

- `git submodule update --init --recursive`: 저장소 안에서 **별도 Git 저장소를 참조하는 폴더**를 내려받습니다. 현재 `.gitmodules`에는 `src/ld08_driver`(LDS-02 LiDAR 드라이버)가 등록되어 있습니다. 일반 `git clone`만으로 해당 드라이버 소스가 채워지지 않을 수 있어 필요합니다.
- `rosdep install --from-paths src --ignore-src -r -y`: `src/`에 있는 ROS 패키지들의 `package.xml`을 읽고 **Ubuntu 패키지·ROS 의존성**을 설치합니다. `--ignore-src`는 소스에 이미 존재하는 패키지를 설치 대상에서 제외합니다. 기존 로봇에 필요한 의존성이 전부 설치돼 있다면 매번 실행할 필요는 없습니다. **Zenoh Bridge 바이너리나 개별 하드웨어 환경 설정까지 전부 설치해 주지는 않습니다.**

> `git pull` 전에 현재 브랜치가 `dev`인지 `git branch --show-current`로 확인하세요. 로컬 수정사항이 있다면 먼저 보존해야 합니다. 라즈베리파이의 메모리 부담을 줄이기 위해 순차 빌드를 예시로 사용했습니다.

## 4. 로봇 실행

**터미널 1 — 로봇 구동 및 Nav2**

```bash
source /opt/ros/jazzy/setup.bash
source ~/logitle_ws/install/setup.bash

export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-02
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

ros2 launch logitle_bringup logitle_robot.launch.py
```

위 통합 Launch는 기본적으로 TurtleBot3 구동부·LiDAR·EKF, AMCL·Nav2, 위치 발행, ArUco Action 서버, 정밀 도킹 서버를 실행합니다. 카메라는 기본적으로 상시 실행되지 않고 ArUco Action에서 필요할 때 사용하도록 구성되어 있습니다.

주요 선택 옵션:

```bash
# Nav2를 제외하고 실행
ros2 launch logitle_bringup logitle_robot.launch.py use_nav2:=false

# ArUco 또는 도킹 기능 제외
ros2 launch logitle_bringup logitle_robot.launch.py use_aruco:=false
ros2 launch logitle_bringup logitle_robot.launch.py use_docking:=false

# 실행 가능한 인자 확인
ros2 launch logitle_bringup logitle_robot.launch.py --show-args
```

**로봇별 설정:** Launch는 Linux 사용자명(`$USER`) 마지막 글자가 `1`, `2`, `3`인지 확인해 초기 위치와 `docking_turtlebot1.yaml` ~ `docking_turtlebot3.yaml`을 자동 선택합니다. 사용자명이 다르면 기본 설정으로 대체되므로 확인이 필요합니다. `docking_params_file:=<YAML 경로>`로 도킹 설정을 직접 지정할 수도 있습니다.

Nav2는 `NavigateThroughPoses`를 사용하며, 현재 설정에 Graph Waypoint BT·StraightFirst/ThetaStar Planner·RPP Controller·Keepout Filter·Collision Monitor가 포함됩니다. 세부 설정은 `src/turtlebot3/turtlebot3_navigation2/param/burger.yaml`을 참고하세요.

## 5. Zenoh 연결 (FMS 통신)

메인 PC에서 **Zenoh Router가 TCP 7447 포트로 실행 중**이어야 합니다. 로봇에서는 별도 터미널을 열고 Bridge를 실행합니다.

**터미널 2 — Robot 3 예시**

```bash
source /opt/ros/jazzy/setup.bash
source ~/logitle_ws/install/setup.bash

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

ros2 launch zenoh_pkg zenoh.launch.py \
  robot_id:=robot3 router_ip:=192.168.0.16
```

- `robot_id`: 각 로봇에 맞게 `robot1`, `robot2`, `robot3`로 변경
- `router_ip`: **실제 메인 PC IP로 변경** (`192.168.0.16`은 예시)
- Bridge는 `/robot1` ~ `/robot3` 네임스페이스로 데이터를 중계하며, Router의 `tcp/<IP>:7447`에 Client로 접속
- 로봇 내부 ROS 노드와 Bridge의 `ROS_DOMAIN_ID`, RMW 및 Discovery 설정을 일치시켜야 함

**중계 대상:** `src/zenoh_pkg/config/bridge.json5`에서 설정합니다.

| 방향 | 현재 허용된 인터페이스 |
| --- | --- |
| 로봇 → FMS | `/battery_state`, `/logitle_pose`, `/map`, `/plan` |
| FMS → 로봇 | `/cmd_vel` |
| FMS → 로봇 Action | `/navigate_through_poses`, `/precision_dock`, `/aruco_align_and_correct` |

다른 ROS 토픽이나 Action을 추가로 중계하려면 Bridge 설정의 허용 목록을 확인해야 합니다.
