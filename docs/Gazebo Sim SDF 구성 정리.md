# Gazebo Sim SDF 구성 정리

## 1. 핵심 개념

Gazebo Sim 구성은 크게 `world SDF`와 `model SDF`로 나눈다.

```text
world SDF
├─ 물리 엔진, 중력, 조명, GUI 설정
├─ 모델을 어디에 배치할지 결정
└─ model SDF를 include

model SDF
├─ 하나의 로봇이나 설비를 정의
├─ visual, collision, link, joint, sensor 정의
└─ STL·DAE 같은 mesh 참조
```

`SDF 파일을 두 개만 만든다`는 뜻은 아니다. 책임을 다음과 같이 나눈다는 뜻이다.

```text
작업장 전체 설정       → worlds/amr_workcell.sdf
팔레트 하나의 형상     → models/pallet/model.sdf
도킹 스테이션 하나     → models/docking_station/model.sdf
컨베이어 하나           → models/conveyor/model.sdf
```

따라서 월드 SDF는 보통 하나이고, 모델 SDF는 재사용할 물체의 종류만큼 생길 수 있다.

---

## 2. 같은 모델을 여러 번 배치하는 방법

팔레트처럼 모양은 같고 위치만 다른 물체는 `model.sdf`를 여러 번 복사하지 않는다. 모델은 하나만 정의하고, 월드에서 `name`과 `pose`를 바꾸어 여러 번 배치한다.

```xml
<include>
  <uri>model://pallet</uri>
  <name>pallet_01</name>
  <pose>-2.5 3.4 0 0 0 0</pose>
</include>

<include>
  <uri>model://pallet</uri>
  <name>pallet_02</name>
  <pose>-0.5 3.4 0 0 0 0</pose>
</include>
```

`pose`의 값은 다음 순서다.

```text
x y z roll pitch yaw
```

이 방식을 사용하면 팔레트 모양을 수정할 때 `models/pallet/model.sdf`만 바꾸면 모든 팔레트에 같은 수정이 적용된다.

### 분리 기준

> 같이 생성되고, 같이 움직이고, 같이 삭제되는 구성은 하나의 모델로 묶는다.

| 상황 | 권장 |
|---|---|
| 항상 고정된 바닥·외벽 | `workcell_shell` 모델로 통합 |
| 모양이 같은 팔레트 여러 개 | 모델 하나를 여러 번 배치 |
| 각자 움직이는 물체 | 별도 모델로 분리 |
| 센서나 플러그인이 필요한 설비 | 별도 모델로 분리 |
| 여러 장소에 반복 배치할 설비 | 재사용 모델로 분리 |

---

## 3. 현재 프로젝트의 실행 구조

```text
multi_robot_workcell.launch.py
├─ worlds/amr_workcell.sdf              작업장 월드
├─ models/turtlebot3_burger/model.sdf   Gazebo 로봇 모델
├─ urdf/turtlebot3_burger.urdf          ROS TF 구조
└─ params/multi_robot.yaml               로봇 이름과 초기 위치
```

기본 실행:

```bash
cd /home/woozoo/dev/Logistics_AMR

source /opt/ros/jazzy/setup.bash
colcon build --packages-select turtlebot3_gazebo --symlink-install
source install/setup.bash

export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo multi_robot_workcell.launch.py
```

GUI 없이 실행:

```bash
ros2 launch turtlebot3_gazebo multi_robot_workcell.launch.py gui:=false
```

다른 월드 사용:

```bash
ros2 launch turtlebot3_gazebo multi_robot_workcell.launch.py \
  world:=/절대경로/다른_world.sdf
```

---

## 4. 현재 STL 연결 방식

현재 `amr_workcell.sdf`는 `models/my_workbench_map/model.sdf`를 include하지 않는다. 월드 SDF에서 STL 조각을 직접 참조하는 구조다.

```xml
<visual name="wall_0">
  <geometry>
    <mesh>
      <uri>../models/my_workbench_map/meshes/part_00.stl</uri>
    </mesh>
  </geometry>
</visual>
```

| STL | 용도 |
|---|---|
| `part_00 ~ 05.stl` | 벽 |
| `part_06 ~ 11.stl` | 도킹 구역 |
| `part_12.stl` | 컨베이어 |
| `part_13 ~ 14.stl` | 작업대 |
| `part_15 ~ 16.stl` | 로봇 암 |
| `part_17 ~ 20.stl` | 팔레트 |
| `part_21 ~ 22.stl` | 작업대 |
| `part_23.stl` | 바닥, 현재 주석 처리 |
| `my_map.stl` | 통합 모델, 현재 충돌체 사용은 주석 처리 |

```xml
<visual>화면에 보이는 모양</visual>
<collision>실제로 부딪히는 물리 영역</collision>
```

현재는 표시용 형상에 STL을 사용하고, 충돌 계산에는 단순한 Box나 Cylinder를 사용한다.

```text
STL mesh      → 시각적 표현
Box/Cylinder  → 충돌 계산
```

복잡한 STL을 충돌체로 직접 사용하는 것보다 시뮬레이션 속도와 충돌 안정성에 유리하다.

---

## 5. model 폴더 구조

Gazebo에서 재사용할 모델은 다음 구조로 관리한다.

```text
models/pallet/
├─ model.config
├─ model.sdf
└─ meshes/
   └─ pallet.stl
```

| 파일 | 역할 |
|---|---|
| `model.config` | 모델 이름, 버전, 사용할 SDF 정보 |
| `model.sdf` | visual, collision, sensor, plugin 등 모델 정의 |
| `meshes/*.stl` | 3D 형상 |

월드에서 `model://pallet`처럼 불러오려면 `models` 폴더가 `GZ_SIM_RESOURCE_PATH`에 포함되어야 한다. 현재 런치 파일은 다음 경로를 등록한다.

```text
turtlebot3_gazebo/models
/opt/ros/jazzy/share
```

---

## 6. SDF에서 실행하는 Gazebo 기능

### 월드 시스템 플러그인

| 플러그인 | 기능 |
|---|---|
| `Physics` | 물리와 충돌 계산 |
| `UserCommands` | 모델 생성·이동·삭제 |
| `SceneBroadcaster` | 서버의 월드 상태를 GUI로 전달 |
| `Sensors` | LiDAR와 Camera 등 센서 처리 |
| `Imu` | IMU 데이터 생성 |

### Gazebo GUI 도구

`amr_workcell.sdf`의 `<gui>`에서 다음 도구를 표시한다.

| 도구 | 기능 |
|---|---|
| `MinimalScene` | 3D 화면 |
| `InteractiveViewControl` | 카메라 이동·회전·확대 |
| `CameraTracking` | 선택한 로봇 추적 |
| `SelectEntities` | 모델 선택 |
| `Shapes` | Box, Sphere 등 기본 도형 삽입 |
| `TransformControl` | 모델 이동·회전 |
| `WorldControl` | 실행, 일시정지, 한 스텝 진행 |
| `WorldStats` | 시간, RTF, 반복 횟수 표시 |
| `ComponentInspector` | 선택한 엔티티의 속성 확인 |
| `EntityTree` | 월드·모델·링크 계층 확인 |

GUI 플러그인은 화면의 도구를 만들고, Physics·Sensors·DiffDrive 같은 시스템 플러그인은 실제 시뮬레이션 동작을 담당한다.

---

## 7. 로봇 model.sdf의 역할

`turtlebot3_burger/model.sdf`는 다음 내용을 정의한다.

- 차체와 바퀴 형상
- 질량과 관성
- 충돌체와 마찰
- 바퀴 조인트
- IMU와 GPU LiDAR
- 차동주행 플러그인
- 조인트 상태 발행 플러그인

멀티로봇 런치는 각 로봇의 이름에 맞게 토픽을 변경한다.

```text
/robot1/cmd_vel
/robot1/odom
/robot1/scan
/robot1/imu
/robot1/tf
/robot1/joint_states
```

`ros_gz_bridge`는 Gazebo 토픽과 ROS 2 토픽을 연결한다.

---

## 8. SDF로 설정할 수 있는 범위

| 영역 | 설정 가능한 내용 |
|---|---|
| 월드 | 물리 엔진, 중력, 조명, 배경, 바람 |
| 모델 | 로봇, 설비, 팔레트, 장애물 |
| 위치 | `pose`를 이용한 위치와 회전 |
| 형상 | Box, Sphere, Cylinder, Mesh, Heightmap |
| 외형 | 색상, 재질, 텍스처, 투명도 |
| 물리 | 질량, 관성, 마찰, 충돌 |
| 관절 | Fixed, Revolute, Prismatic, Ball |
| 센서 | LiDAR, IMU, Camera, GPS, Contact 등 |
| 센서 품질 | 주기, 범위, 해상도, 노이즈 |
| 동작 | DiffDrive와 사용자 정의 플러그인 |
| GUI | 카메라 위치와 표시할 도구 |

SDF가 직접 담당하지 않는 영역:

- Nav2 실행
- RViz 실행
- FMS 서버
- 작업 배차
- 경로 계획
- ROS 노드 실행 순서

```text
SDF       = Gazebo 내부의 월드·모델·물리·센서·GUI
STL       = 3D 형상
URDF      = ROS의 로봇 링크와 TF 구조
launch.py = Gazebo·ROS 노드·브리지 실행
YAML      = 로봇 위치와 파라미터
Plugin    = 주행·센서·설비의 동작 로직
```

---

## 9. 실무적인 폴더 구조 검토

현재의 `launch`, `worlds`, `models`, `urdf`, `params`, `rviz` 구조는 ROS 2와 Gazebo 패키지에서 일반적인 구조다. 다만 현재 프로젝트는 다음 부분을 개선할 필요가 있다.

1. `amr_workcell.sdf`가 약 2,000줄로 커서 수정하기 어렵다.
2. 월드에 모델의 세부 visual과 collision이 함께 들어 있다.
3. `amr_workcell.sdf`와 `warehouse.sdf`의 내용이 중복된다.
4. `my_workbench_map/model.sdf`가 있지만 실제 월드는 STL을 직접 참조한다.
5. `part_00.stl`처럼 이름만으로 용도를 알기 어려운 파일이 많다.
6. 프로젝트 전용 물류 환경이 TurtleBot3 원본 패키지 안에 있다.
7. `multi_robot_workcell.launch.py`가 Gazebo 실행, SDF 수정, 로봇 스폰, 브리지를 모두 담당한다.

---

## 10. 권장 프로젝트 구조

프로젝트 전용 자산을 TurtleBot3 원본 패키지에서 분리한다.

```text
src/
├─ turtlebot3/
├─ turtlebot3_simulations/
│
└─ logistics_gazebo/
   ├─ CMakeLists.txt
   ├─ package.xml
   ├─ launch/
   │  └─ simulation.launch.py
   ├─ config/
   │  ├─ robots.yaml
   │  └─ bridge.yaml
   ├─ worlds/
   │  └─ amr_workcell.sdf
   ├─ models/
   │  ├─ workcell_shell/
   │  ├─ docking_station/
   │  ├─ conveyor/
   │  ├─ pallet/
   │  └─ robot_arm_station/
   ├─ gui/
   │  └─ development.config
   └─ test/
      └─ test_sdf_files.py
```

이 구조에서는 사용자가 다음 명령 하나만 알면 된다.

```bash
ros2 launch logistics_gazebo simulation.launch.py
```

내부적으로는 책임을 다음과 같이 분리한다.

```text
world SDF  → 물리, 조명, 플러그인, 모델 배치
model SDF  → 개별 물체의 형상, 충돌, 센서, 동작
launch.py  → Gazebo, ROS 노드, 브리지 실행
config     → 로봇 목록, 좌표, 토픽, 실행 옵션
```

### STL 이름도 용도가 드러나게 변경한다

```text
part_00.stl → wall_north.stl
part_06.stl → dock_left.stl
part_12.stl → conveyor_belt.stl
part_17.stl → pallet.stl
```

모양이 같은 팔레트는 `pallet_01.stl`, `pallet_02.stl`처럼 복사하지 않고 `pallet.stl` 하나만 유지한다.

---

## 11. 리팩터링 순서

기존 실행을 유지하면서 다음 순서로 정리한다.

1. `warehouse.sdf`의 사용 여부를 확인하고 중복을 제거한다.
2. `logistics_gazebo` 패키지를 생성한다.
3. 기존 `amr_workcell.sdf`를 옮겨 기존 실행을 먼저 보존한다.
4. 바닥과 외벽을 `workcell_shell` 모델로 분리한다.
5. 팔레트와 도킹 스테이션을 재사용 모델로 분리한다.
6. 컨베이어와 로봇 암을 독립 모델로 분리한다.
7. `part_XX.stl`을 의미 있는 이름으로 변경한다.
8. 런치 파일의 SDF 처리와 브리지 로직을 분리한다.
9. 리팩터링 전후의 로봇 위치와 ROS 토픽이 같은지 검증한다.

검증 예시:

```bash
gz sdf -k worlds/amr_workcell.sdf

ros2 launch logistics_gazebo simulation.launch.py gui:=false

ros2 topic list
ros2 topic hz /robot1/scan
ros2 topic hz /robot1/odom
```

---

## 12. 최종 정리

```text
동일한 물체를 여러 번 배치
→ model.sdf는 하나만 만들고 pose만 변경한다.

월드와 모델의 구분
→ world SDF는 전체 환경과 배치를 담당한다.
→ model SDF는 각 물체의 내부 구조를 담당한다.

폴더 구조
→ TurtleBot3 원본 패키지와 물류 프로젝트 자산을 분리한다.

사용자 경험
→ 실행 명령은 하나로 유지한다.
→ 내부 모델·설정·런치 로직은 책임별로 분리한다.
```

> 사용자는 하나의 launch 파일만 실행하고, 개발자는 모델·월드·설정 파일을 서로 영향 없이 수정할 수 있는 구조가 가장 좋다.
