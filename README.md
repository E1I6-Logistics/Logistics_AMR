# Logistics AMR

ROS 2 Jazzy 기반 TurtleBot3 멀티 AMR 물류 시스템의 **Robot Runtime 및 Navigation 저장소**입니다.

본 프로젝트는 크게 두 개의 시스템으로 구성됩니다.

- **Logistics_AMR**
  - 각 TurtleBot3에서 실행되는 로봇 측 시스템
  - TurtleBot3 Bringup
  - Sensor / Odometry
  - EKF
  - AMCL Localization
  - Nav2
  - Route Server
  - Docking
  - FMS 통신
  - Navigation 성능 실험

- **Logistics_FMS**
  - 여러 AMR을 통합 관리하는 관제 시스템
  - Robot Monitoring
  - Map / Route Visualization
  - Task / Goal Management
  - Operator Dashboard
  - ROS 2 / Zenoh 통신 환경

FMS 저장소:

https://github.com/E1I6-Logistics/Logistics_FMS

> 현재 FMS 저장소의 Backend 일부는 mock robot data를 사용하고 있습니다.
> `real` 모드로 변경하는 것만으로 실제 로봇 통신이 완성되는 것은 아니며,
> ROS 2 / Zenoh 기반 실기체 연동은 프로젝트 통합 과정에서 별도로 검증합니다.

---

# 1. 시스템 구성

전체 시스템은 **FMS가 Fleet-level 관제**, 각 **AMR이 Robot-level 자율주행**을 담당하는 구조입니다.

```text
                           Main PC
                    ┌─────────────────────┐
                    │    Logistics_FMS    │
                    │                     │
                    │  FMS Backend        │
                    │  Dashboard          │
                    │  Map / Route        │
                    │  Task Management    │
                    │  Robot Monitoring   │
                    │  Traffic Management │
                    │  Zenoh Router       │
                    └──────────┬──────────┘
                               │
                             Zenoh
                               │
            ┌──────────────────┼──────────────────┐
            │                  │                  │
            ▼                  ▼                  ▼

      TurtleBot #1        TurtleBot #2        TurtleBot #3
    ┌──────────────┐    ┌──────────────┐    ┌──────────────┐
    │ Logistics_AMR│    │ Logistics_AMR│    │ Logistics_AMR│
    │              │    │              │    │              │
    │ Bringup      │    │ Bringup      │    │ Bringup      │
    │ EKF          │    │ EKF          │    │ EKF          │
    │ AMCL         │    │ AMCL         │    │ AMCL         │
    │ Nav2         │    │ Nav2         │    │ Nav2         │
    │ Route Server │    │ Route Server │    │ Route Server │
    │ Controller   │    │ Controller   │    │ Controller   │
    │ Docking      │    │ Docking      │    │ Docking      │
    └──────────────┘    └──────────────┘    └──────────────┘
```

핵심 역할은 다음과 같이 구분합니다.

```text
FMS
├── 여러 로봇 상태 관리
├── 작업(Task) 관리
├── 목적지 / Goal 관리
├── Node / Edge 기반 교통 관리
├── 충돌 및 병목 방지를 위한 Fleet-level Coordination
└── 사용자 관제 화면

AMR
├── 센서 처리
├── Odometry / EKF
├── Localization
├── Nav2
├── Route Server
├── Path Following
├── Local Obstacle Avoidance
├── Goal 도착
└── Docking / 정밀 정차
```

즉 FMS가 로봇 바퀴를 직접 제어하는 구조가 아니라,
FMS는 **어디로 갈지 / 어떤 작업을 수행할지**를 관리하고,
실제 경로 추종과 로컬 제어는 각 TurtleBot의 Nav2가 수행하는 방향으로 구성합니다.

---

# 2. 개발 환경

주요 개발 환경은 다음과 같습니다.

```text
OS          : Ubuntu 24.04
ROS         : ROS 2 Jazzy
Robot       : TurtleBot3 Burger
Navigation  : Nav2
Localization: AMCL
Odometry    : EKF / robot_localization
LiDAR       : LDS-02
LiDAR Driver: ld08_driver
Language    : C++, Python
Middleware  : ROS 2 DDS / Zenoh
```

---

# 3. Repository 구조

```text
Logistics_AMR/
├── src/
│   ├── DynamixelSDK/
│   │
│   ├── ld08_driver/
│   │   └── LDS-02 LiDAR Driver
│   │       Git Submodule
│   │
│   ├── logitle_docking/
│   │   └── Docking 기능
│   │
│   ├── logitle_ekf/
│   │   └── EKF / robot_localization
│   │
│   ├── tools/
│   │   ├── logitle_experiments/
│   │   │   └── Navigation 실험 도구
│   │   │
│   │   └── staging_pose_manager/
│   │       └── Station Pose 측정 및 Calibration
│   │
│   ├── turtlebot3/
│   │   ├── turtlebot3_bringup/
│   │   ├── turtlebot3_description/
│   │   ├── turtlebot3_navigation2/
│   │   └── turtlebot3_node/
│   │
│   ├── turtlebot3_msgs/
│   ├── turtlebot3_my_msg/
│   └── zenoh_pkg/
│
├── docs/
│   └── Navigation / Localization / Controller 실험 문서
│
├── experiment_data/
│   └── Navigation 실험 결과 데이터
│
├── notebooks/
│   └── 실험 데이터 분석 Notebook
│
├── .gitmodules
├── .gitignore
└── README.md
```

ROS 2의 빌드 결과인 아래 디렉터리는 Git에서 관리하지 않습니다.

```text
build/
install/
log/
```

---

# 4. 주요 ROS 2 Package

| Package | 역할 |
|---|---|
| `turtlebot3_bringup` | TurtleBot3 하드웨어 및 센서 Bringup |
| `turtlebot3_node` | TurtleBot3 Base Node / Odometry |
| `turtlebot3_description` | Robot URDF 및 모델 |
| `turtlebot3_navigation2` | Nav2, Route Server, Map, Route Graph |
| `turtlebot3_msgs` | TurtleBot3 ROS Interface |
| `turtlebot3_my_msg` | 프로젝트 Custom ROS Interface |
| `logitle_ekf` | EKF / robot_localization |
| `logitle_docking` | Docking 기능 |
| `zenoh_pkg` | Robot ↔ FMS 통신 |
| `staging_pose_manager` | Station Pose Calibration |
| `logitle_experiments` | Navigation 성능 실험 |
| `ld08_driver` | LDS-02 LiDAR ROS 2 Driver |

---

# 5. ld08_driver Submodule

본 프로젝트에서는 TurtleBot3 LDS-02 LiDAR를 사용하기 위해
ROBOTIS의 `ld08_driver`를 Git Submodule로 관리합니다.

Submodule 설정:

```text
src/ld08_driver
└── https://github.com/ROBOTIS-GIT/ld08_driver.git
```

사용 Branch:

```text
jazzy
```

따라서 Repository를 처음 Clone할 때 Submodule도 함께 받아야 합니다.

---

# 6. 처음 Clone하기

## 6.1 일반 PC

```bash
cd ~/dev

git clone --recurse-submodules \
  https://github.com/E1I6-Logistics/Logistics_AMR.git

cd Logistics_AMR
```

`dev` Branch에서 개발할 경우:

```bash
git switch dev
```

Submodule 상태 확인:

```bash
git submodule status
```

---

## 6.2 TurtleBot

TurtleBot에 SSH로 접속하거나 직접 Terminal을 실행합니다.

예:

```bash
ssh turtlebot2@<ROBOT_IP>
```

TurtleBot Home에서 Clone합니다.

```bash
cd ~

git clone --recurse-submodules \
  -b dev \
  https://github.com/E1I6-Logistics/Logistics_AMR.git

cd ~/Logistics_AMR
```

확인:

```bash
git status -sb
git submodule status
```

---

## 6.3 이미 Clone한 Repository에서 Submodule 받기

기존에 `git clone`만 수행했다면:

```bash
cd ~/Logistics_AMR

git submodule update --init --recursive
```

확인:

```bash
ls src/ld08_driver
```

정상적으로 `package.xml`, `launch`, source 파일 등이 보여야 합니다.

---

# 7. ROS 2 환경 설정

새 Terminal에서 빌드하기 전에 먼저 ROS 2 Jazzy 환경을 불러옵니다.

```bash
source /opt/ros/jazzy/setup.bash
```

확인:

```bash
echo $ROS_DISTRO
```

정상:

```text
jazzy
```

ROS 2 기본 Package 확인:

```bash
ros2 pkg prefix ament_cmake
```

정상이라면:

```text
/opt/ros/jazzy
```

가 출력됩니다.

---

# 8. Dependency 설치

Workspace Root에서 실행합니다.

```bash
cd ~/Logistics_AMR
```

또는 개발 PC의 경우:

```bash
cd ~/dev/Logistics_AMR
```

ROS 환경을 먼저 Source합니다.

```bash
source /opt/ros/jazzy/setup.bash
```

그리고:

```bash
rosdep install \
  --from-paths src \
  --ignore-src \
  -r \
  -y
```

`package.xml`을 기준으로 필요한 ROS 및 System Dependency를 설치합니다.

---

# 9. Build

## 9.1 PC 개발 환경

개발 PC에서는 전체 Workspace를 빌드하는 것을 기본으로 합니다.

```bash
cd ~/dev/Logistics_AMR

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install \
  --allow-overriding \
  turtlebot3_bringup \
  turtlebot3_description \
  turtlebot3_msgs \
  turtlebot3_navigation2 \
  turtlebot3_node
```

빌드 성공 후:

```bash
source install/setup.bash
```

---

## 9.2 TurtleBot

TurtleBot에서는 Robot Runtime에 필요한 Package를 중심으로 빌드할 수 있습니다.

```bash
cd ~/Logistics_AMR

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install \
  --packages-up-to \
  turtlebot3_bringup \
  turtlebot3_navigation2 \
  logitle_ekf \
  zenoh_pkg \
  logitle_docking \
  --allow-overriding \
  turtlebot3_bringup \
  turtlebot3_description \
  turtlebot3_msgs \
  turtlebot3_navigation2 \
  turtlebot3_node
```

빌드 후:

```bash
source install/setup.bash
```

---

# 10. Build 결과 확인

Package가 정상적으로 설치되었는지 확인할 수 있습니다.

예:

```bash
ros2 pkg prefix turtlebot3_bringup
```

```bash
ros2 pkg prefix turtlebot3_navigation2
```

```bash
ros2 pkg prefix ld08_driver
```

TurtleBot에서 `ld08_driver`가 정상 빌드되었다면 대략 다음과 같은 경로가 출력됩니다.

```text
/home/turtlebot2/Logistics_AMR/install/ld08_driver
```

---

# 11. TurtleBot 환경 변수

실기체 실행 전 Robot별 환경 값을 설정해야 합니다.

예:

```bash
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-02
```

ROS Domain ID는 팀의 네트워크 구성에 맞게 설정합니다.

```bash
export ROS_DOMAIN_ID=<DOMAIN_ID>
```

현재 환경 확인:

```bash
echo $TURTLEBOT3_MODEL
echo $LDS_MODEL
echo $ROS_DOMAIN_ID
```

환경 변수는 필요에 따라 `~/.bashrc` 또는 프로젝트 Shell Script에서 관리할 수 있습니다.

---

# 12. TurtleBot Bringup

새 Terminal에서:

```bash
cd ~/Logistics_AMR

source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Robot Bringup:

```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

현재 Bringup 구성에서는 LDS-02 사용 시 `ld08_driver`가 필요합니다.

따라서 다음 오류가 발생한다면:

```text
PackageNotFoundError: package 'ld08_driver' not found
```

다음을 확인합니다.

```bash
git submodule status
```

```bash
ls src/ld08_driver
```

```bash
ros2 pkg prefix ld08_driver
```

---

# 13. Navigation

Navigation 관련 Package는 다음 위치에 있습니다.

```text
src/turtlebot3/turtlebot3_navigation2/
```

현재 주요 구조:

```text
turtlebot3_navigation2/
├── launch/
│   ├── navigation2.launch.py
│   ├── navigation2_robot.launch.py
│   └── navigation2_route.launch.py
│
├── graphs/
│   └── logitle_route.geojson
│
├── map/
├── param/
└── rviz/
```

---

# 14. Navigation Launch

예:

```bash
ros2 launch turtlebot3_navigation2 navigation2_robot.launch.py
```

Route Server를 포함한 Route 기반 Navigation:

```bash
ros2 launch turtlebot3_navigation2 navigation2_route.launch.py
```

실제 실행 시 사용하는 Launch File과 Argument는 프로젝트 개발 상태에 따라 변경될 수 있습니다.

---

# 15. 새로운 Launch File 추가

새로운 Launch File은 다음 위치에 추가합니다.

```text
src/turtlebot3/turtlebot3_navigation2/launch/
```

예:

```text
my_navigation.launch.py
```

현재 `turtlebot3_navigation2/CMakeLists.txt`는 다음 디렉터리를 install 대상으로 포함하고 있습니다.

```cmake
install(
  DIRECTORY launch map param rviz graphs
  DESTINATION share/${PROJECT_NAME}
)
```

따라서 새 Launch File을 추가한 뒤 Package를 다시 빌드합니다.

```bash
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install \
  --packages-select turtlebot3_navigation2 \
  --allow-overriding turtlebot3_navigation2
```

그다음:

```bash
source install/setup.bash
```

실행:

```bash
ros2 launch turtlebot3_navigation2 my_navigation.launch.py
```

---

# 16. Route Server

본 프로젝트에서는 일반적인 Free-space Navigation뿐 아니라
Nav2 Route Server를 이용한 **Node / Edge 기반 물류 경로 주행**을 사용합니다.

Route Graph:

```text
src/turtlebot3/turtlebot3_navigation2/graphs/logitle_route.geojson
```

기본 흐름:

```text
FMS
 │
 │ start_node / goal_node
 ▼
Route Server
 │
 │ ComputeRoute
 ▼
Node / Edge Route
 │
 ▼
nav_msgs/Path
 │
 ▼
FollowPath
 │
 ▼
Nav2 Controller
 │
 ▼
cmd_vel
 │
 ▼
TurtleBot
```

Route Server는 물류센터의 고정 통로 구조를 Node / Edge Graph로 표현합니다.

FMS는 이 Graph 정보를 활용해 여러 AMR의 이동 경로와 교통 상황을 관리할 수 있습니다.

---

# 17. Node 종류

Route Graph의 Node는 용도에 따라 개념적으로 구분할 수 있습니다.

```text
Transit Node
└── 이동 경로를 연결하기 위한 일반 Node

Station Node
└── Picking / Packing / Charging / Docking 등
    실제 작업이 수행되는 위치

Intersection Node
└── 여러 Edge가 만나는 교차 지점
    FMS Traffic Management의 중요 관리 지점
```

Transit Node는 반드시 정밀 정차할 필요가 없으며,
Nav2 Controller가 전체 Path를 따라 자연스럽게 주행할 수 있습니다.

반면 Station Node는 작업 수행을 위해 정밀한 최종 Pose가 필요할 수 있습니다.

---

# 18. Station Pose

Station에서는 단순한 `(x, y)` 좌표 외에 방향 `yaw`까지 포함한 Pose가 필요할 수 있습니다.

```text
Station Pose
├── x
├── y
└── yaw
```

본 프로젝트에서는 `staging_pose_manager`를 사용하여
실제 Station 위치에서 `map -> base_footprint` Pose를 반복 측정하고
대표 Pose를 계산하는 방식으로 Station Pose Calibration을 수행합니다.

개념적인 흐름:

```text
Route Server
    ↓
Station 근처까지 접근
    ↓
정밀 Station Pose
(x, y, yaw)
    ↓
Final Navigation
    ↓
Docking / Marker Correction
```

---

# 19. Navigation Accuracy

본 프로젝트에서는 단순히 `Goal에 도착했는가`뿐 아니라
실제 물리적 도착 정확도를 별도로 측정합니다.

정확도에 영향을 주는 주요 요소:

```text
Wheel Encoder
      ↓
Odometry
      ↓
EKF
      ↓
Localization
      ↓
AMCL
      ↓
Route / Global Path
      ↓
Controller
      ↓
Goal Checker
      ↓
Station Pose
      ↓
Physical Arrival Accuracy
```

Navigation 관련 실험 문서는 `docs/`에서 관리합니다.

```text
docs/
├── 00_navigation_accuracy_test_plan.md
├── 01_goal_tolerance_test.md
├── 01-1_xy_goal_tolerance_test_result.md
├── 02_amcl_localization_test.md
├── 04_dwb_controller_test.md
├── 06_experiment_tooling.md
└── 07_experiment_data_dictionary.md
```

실험 데이터:

```text
experiment_data/
```

분석 Notebook:

```text
notebooks/
```

---

# 20. FMS

Fleet Management System은 별도 Repository에서 관리합니다.

```text
E1I6-Logistics/Logistics_FMS
```

Repository:

https://github.com/E1I6-Logistics/Logistics_FMS

현재 FMS는 다음 구성 요소를 포함합니다.

```text
Logistics_FMS
├── FastAPI Backend
├── React / Vite Dashboard
├── Map
├── Route Graph
├── Robot API
├── Command API
├── WebSocket
├── Zenoh Router
├── zenoh-bridge-ros2dds
└── ROS 2 / Zenoh 관련 Script
```

현재 FMS 실행 구조:

```text
Map / Route
     │
     ▼
FastAPI Backend
     │
     ├───────────────┐
     │               │
     ▼               ▼
Dashboard        WebSocket
                     │
                     │
ROS 2 Robot
     │
     ▼
zenoh-bridge-ros2dds
     │
     ▼
Zenoh Router
```

---

# 21. FMS 실행

FMS는 Main PC에서 별도로 실행합니다.

FMS Repository Clone 및 설치 방법은
`Logistics_FMS` Repository의 README를 기준으로 합니다.

기본 실행:

```bash
cd ~/Logistics_FMS

./start_fms.sh
```

현재 `start_fms.sh`는 다음 구성 요소를 시작합니다.

```text
1. Zenoh Router
        ↓
2. zenoh-bridge-ros2dds
        ↓
3. FastAPI Backend
        ↓
4. React / Vite Frontend
```

FMS 종료:

```bash
./stop_fms.sh
```

FMS Dashboard 기본 개발 주소:

```text
http://127.0.0.1:5173
```

Backend Health:

```text
http://127.0.0.1:8000/health
```

FastAPI API 문서:

```text
http://127.0.0.1:8000/docs
```

> FMS의 상세 설치 및 실행 방법은 Logistics_FMS README를 기준으로 합니다.

---

# 22. AMR ↔ FMS 통신

AMR과 FMS는 ROS 2 / Zenoh 기반 통신을 사용합니다.

개념적인 구조:

```text
TurtleBot
┌──────────────────────┐
│ ROS 2                │
│                      │
│ Nav2                 │
│ Robot State          │
│ Pose                 │
│ Task State           │
│ Command Interface    │
└──────────┬───────────┘
           │
           │ ROS 2 / Zenoh
           ▼
   zenoh-bridge-ros2dds
           │
           ▼
       Zenoh Router
           │
           ▼
┌──────────────────────┐
│ Logistics_FMS        │
│                      │
│ Backend              │
│ Dashboard            │
│ Fleet Management     │
└──────────────────────┘
```

최종적으로 FMS ↔ AMR 간 Interface에는 다음과 같은 데이터가 포함될 수 있습니다.

```text
FMS → AMR
├── robot_id
├── task
├── start_node
├── goal_node
├── movement command
└── stop / resume

AMR → FMS
├── robot_id
├── current pose
├── current node
├── robot state
├── navigation state
├── task state
├── velocity
└── error / alarm
```

단, 실제 Interface 형식은 현재 프로젝트 통합 과정에서 확정합니다.

---

# 23. 전체 실행 순서

실기체 운용 시 전체 시스템은 다음 순서로 실행하는 것을 기본 흐름으로 합니다.

```text
Main PC
│
├── Logistics_FMS 실행
│   │
│   ├── Zenoh Router
│   ├── ROS2-Zenoh Bridge
│   ├── Backend
│   └── Dashboard
│
│
TurtleBot #1
│
├── ROS 환경 설정
├── Logistics_AMR Workspace Source
├── TurtleBot Bringup
├── Nav2
├── Route Server
└── FMS Communication
│
│
TurtleBot #2
│
├── ROS 환경 설정
├── Logistics_AMR Workspace Source
├── TurtleBot Bringup
├── Nav2
├── Route Server
└── FMS Communication
│
│
└── FMS에서 Multi-AMR 상태 및 작업 관리
```

현재 프로젝트 개발 단계에 따라
Bringup / Nav2 / Zenoh 관련 실행 순서는 변경될 수 있습니다.

---

# 24. TurtleBot 기본 실행 예시

Terminal 1:

```bash
cd ~/Logistics_AMR

source /opt/ros/jazzy/setup.bash
source install/setup.bash

export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-02
export ROS_DOMAIN_ID=<DOMAIN_ID>

ros2 launch turtlebot3_bringup robot.launch.py
```

Terminal 2:

```bash
cd ~/Logistics_AMR

source /opt/ros/jazzy/setup.bash
source install/setup.bash

export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=<DOMAIN_ID>

ros2 launch turtlebot3_navigation2 navigation2_robot.launch.py
```

Route Server 기반 Navigation을 사용하는 경우:

```bash
ros2 launch turtlebot3_navigation2 navigation2_route.launch.py
```

---

# 25. Repository 업데이트

`dev`에 새로운 코드가 Merge되었을 경우 TurtleBot 또는 PC에서:

```bash
cd ~/Logistics_AMR
```

현재 Branch 확인:

```bash
git status -sb
```

`dev` 사용:

```bash
git switch dev
```

최신 코드:

```bash
git pull --ff-only
```

Submodule 갱신:

```bash
git submodule update --init --recursive
```

그다음:

```bash
source /opt/ros/jazzy/setup.bash
```

Dependency 갱신:

```bash
rosdep install \
  --from-paths src \
  --ignore-src \
  -r \
  -y
```

다시 Build:

```bash
colcon build --symlink-install \
  --allow-overriding \
  turtlebot3_bringup \
  turtlebot3_description \
  turtlebot3_msgs \
  turtlebot3_navigation2 \
  turtlebot3_node
```

그리고:

```bash
source install/setup.bash
```

---

# 26. 개발 Branch Workflow

기본 개발 흐름:

```text
feature / developer branch
          │
          ▼
         dev
          │
          ▼
         main
```

`main`

```text
안정화된 통합 버전
```

`dev`

```text
팀 개발 Integration Branch
```

개인 또는 기능 Branch:

```text
새로운 기능 개발 및 테스트
```

예:

```bash
git switch dev
git pull --ff-only

git switch -c feature/<feature-name>
```

작업 후:

```bash
git add .
git commit -m "<type>: <description>"
git push -u origin feature/<feature-name>
```

---

# 27. 새 Terminal에서 Source 순서

새로운 Terminal을 열었다면 기본적으로:

```bash
source /opt/ros/jazzy/setup.bash
```

프로젝트 Build 결과까지 사용하려면:

```bash
source ~/Logistics_AMR/install/setup.bash
```

개발 PC의 경우:

```bash
source ~/dev/Logistics_AMR/install/setup.bash
```

즉:

```text
ROS 2 Jazzy
     ↓
/opt/ros/jazzy/setup.bash
     ↓
Project Workspace
     ↓
install/setup.bash
```

순서입니다.

---

# 28. Troubleshooting

## 28.1 `ament_cmake`를 찾지 못하는 경우

에러:

```text
Could not find a package configuration file provided by "ament_cmake"
```

또는:

```text
ament_cmakeConfig.cmake
ament_cmake-config.cmake
```

원인:

```text
ROS 2 Jazzy 환경이 Source되지 않음
```

해결:

```bash
source /opt/ros/jazzy/setup.bash
```

확인:

```bash
ros2 pkg prefix ament_cmake
```

---

## 28.2 `action_msgs` 등 기본 ROS Package를 찾지 못하는 경우

예:

```text
Could not find action_msgs
```

먼저:

```bash
source /opt/ros/jazzy/setup.bash
```

다시 Build합니다.

---

## 28.3 `ld08_driver`를 찾지 못하는 경우

에러:

```text
PackageNotFoundError:
package 'ld08_driver' not found
```

Submodule 확인:

```bash
git submodule status
```

실제 Source 확인:

```bash
ls src/ld08_driver
```

없다면:

```bash
git submodule update --init --recursive
```

그다음:

```bash
source /opt/ros/jazzy/setup.bash
```

Build:

```bash
colcon build --symlink-install
```

Source:

```bash
source install/setup.bash
```

확인:

```bash
ros2 pkg prefix ld08_driver
```

---

## 28.4 Package를 새로 추가했는데 `ros2 launch`에서 보이지 않는 경우

예를 들어:

```text
src/turtlebot3/turtlebot3_navigation2/launch/
```

에 새로운 Launch File을 추가했다면 다시 Build합니다.

```bash
colcon build --symlink-install \
  --packages-select turtlebot3_navigation2 \
  --allow-overriding turtlebot3_navigation2
```

그리고:

```bash
source install/setup.bash
```

확인:

```bash
ls install/turtlebot3_navigation2/share/turtlebot3_navigation2/launch
```

---

## 28.5 Package 위치 변경 후 CMake Cache 오류

Package를 다른 디렉터리로 이동한 뒤 다음과 같은 오류가 발생할 수 있습니다.

```text
source directory does not match the source directory
used to generate cache
```

기존 Build Cache 삭제:

```bash
rm -rf build install log
```

다시:

```bash
source /opt/ros/jazzy/setup.bash
```

```bash
colcon build --symlink-install
```

---

## 28.6 Git Submodule이 비어 있는 경우

```bash
ls src/ld08_driver
```

했는데 내용이 없거나 Submodule 관련 오류가 발생하면:

```bash
git submodule update --init --recursive
```

현재 Submodule 상태:

```bash
git submodule status
```

---

## 28.7 Git Branch가 원격보다 뒤처진 경우

확인:

```bash
git status -sb
```

예:

```text
## dev...origin/dev [behind 1]
```

일반적인 경우:

```bash
git pull --ff-only
```

이미 `origin/dev`의 최신 Commit을 Fetch해둔 상태라면:

```bash
git merge --ff-only origin/dev
```

를 사용할 수도 있습니다.

---

# 29. 개발 / 실험 문서

Navigation 성능 관련 테스트와 실험 절차는 README에 전부 넣지 않고
`docs/` 아래에 별도로 관리합니다.

현재 주요 문서:

```text
docs/
├── 00_navigation_accuracy_test_plan.md
├── 01_goal_tolerance_test.md
├── 01-1_xy_goal_tolerance_test_result.md
├── 02_amcl_localization_test.md
├── 04_dwb_controller_test.md
├── 06_experiment_tooling.md
└── 07_experiment_data_dictionary.md
```

README는 전체 시스템의 **설치 / 구조 / 실행 / 운영 진입점**으로 사용하고,
세부 실험 방법과 분석 내용은 `docs/`를 참고합니다.

---

# 30. 프로젝트 Repository

AMR:

https://github.com/E1I6-Logistics/Logistics_AMR

FMS:

https://github.com/E1I6-Logistics/Logistics_FMS

두 Repository는 하나의 Multi-AMR 물류 시스템을 구성합니다.

```text
Logistics_FMS
     │
     │ Fleet Management
     │ Monitoring
     │ Task / Traffic Control
     │
     ▼
ROS 2 / Zenoh
     │
     ▼
Logistics_AMR
     │
     ├── TurtleBot #1
     ├── TurtleBot #2
     └── TurtleBot #N
```

---

# 31. 현재 개발 방향

본 프로젝트는 다음 구조를 목표로 개발합니다.

```text
Fleet Level
─────────────────────────────────
Logistics_FMS
│
├── Task Allocation
├── Fleet Monitoring
├── Route / Traffic Management
├── Node / Edge Reservation
├── Priority Management
└── Deadlock / Recovery Handling

                │
                │ Zenoh
                ▼

Robot Level
─────────────────────────────────
Logistics_AMR
│
├── TurtleBot Bringup
├── Sensor
├── Odometry
├── EKF
├── AMCL
├── Nav2
├── Route Server
├── Local Controller
├── Collision Avoidance
├── Station Pose
└── Docking
```

Fleet-level 판단은 FMS에서 수행하고,
실제 Robot Motion 및 Navigation은 각 AMR에서 수행하는 구조를 기본으로 합니다.