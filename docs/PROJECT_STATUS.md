# Logistics AMR Project Status

Last updated: 2026-09-14

이 문서는 새 채팅이나 새 작업을 시작할 때 현재 프로젝트 상태를 빠르게 파악하기 위한 기준 문서입니다.

## Roadmap

### 선행 완료(~9/13)

- 가상 맵 및 Occupancy Map
- Mapping
- 멀티 로봇 Spawn
- Route Graph 작성
- 1대 주행 확인

### Sprint 1(9/14~9/20)

- ROS2 Namespace 분리
- FMS 명령 <-> Nav2/Route Server 연동
- Pose <-> Node/Edge 변환
- 1대 다중 목적지 검증

### Sprint 2(9/21~9/27)

- 3대 Route 동시 실행
- 장애물 감지 및 회피
- Free-space Replanning
- Docking Station 이동 기본 검증

### Sprint 3(9/28~10/4)

- 실제 테스트 맵 및 Mapping
- Keepout / Speed-restricted Zone
- 실물 TurtleBot 1~3대 동시 주행

### Final(10/5~10/7)

- 실물 반복 주행
- 교차로 및 교착 상황 테스트
- Nav2 / Route Graph 최종 보정

## Final Acceptance Criteria

- 실물 TurtleBot 3대 동시 주행 성공
- 장애물 회피 후 기존 Route 복귀
- Docking Station 접근 성공

## Current Status

현재 저장소 기준으로는 선행 완료 항목과 Sprint 1 일부가 진행되어 있습니다. Sprint 2 항목 중 장애물 감지/회피와 Docking 관련 설정 일부도 선반영되어 있으나, 실제 동시 Route 실행 및 실물 검증은 아직 별도 확인이 필요합니다.

### 완료 또는 거의 완료

- 가상 맵 / Occupancy Map
  - `src/turtlebot3/turtlebot3_navigation2/map/warehouse_map.yaml`
  - `src/turtlebot3/turtlebot3_navigation2/map/warehouse_map.pgm`
  - `src/turtlebot3/turtlebot3_navigation2/map/my_map.yaml`
  - `src/turtlebot3/turtlebot3_navigation2/map/my_map.pgm`
- 멀티 로봇 Gazebo Spawn
  - `src/turtlebot3_simulations/turtlebot3_gazebo/launch/my_map_multi_world.launch.py`
  - `tb3_0`, `tb3_1`, `tb3_2` 세 대 spawn
  - `/tb3_n/cmd_vel`, `/tb3_n/odom`, `/tb3_n/scan`, `/tb3_n/joint_states` remap
- ROS2 Namespace 분리
  - `src/turtlebot3/turtlebot3_navigation2/launch/multi_nav2_route_server.launch.py`
  - `tb3_0`, `tb3_1`, `tb3_2` 각각 Nav2 bringup
  - `burger_route_tb3_0.yaml`, `burger_route_tb3_1.yaml`, `burger_route_tb3_2.yaml`에 로봇별 frame 설정
- Route Graph 작성
  - `test2.geojson`
  - 2026-09-14 확인 기준 JSON 유효
  - Point 10개, MultiLineString edge 36개

### 부분 완료

- Nav2 / Route Server 연동
  - `src/turtlebot3/turtlebot3_navigation2/launch/navigation_launch.py`에 `route_server` lifecycle node 포함
  - `burger_route*.yaml`에 `route_server.graph_filepath`가 `test2.geojson`으로 설정됨
  - 단일 로봇용 `burger_route.yaml`은 커스텀 Route Behavior Tree를 사용함
  - 멀티 로봇용 `burger_route_tb3_*.yaml`은 아직 기본 Nav2 replanning Behavior Tree를 사용함
- 1대 Route 실행 보조 스크립트
  - `src/turtlebot3/turtlebot3_navigation2/scripts/route_goal_executor.py`
  - node id 기반 `ComputeRoute -> FollowPath` 실행 흐름 구현
  - 현재 action client가 `/compute_route`, `/follow_path` 절대 경로를 사용하므로 멀티 로봇 namespace 실행에는 수정 필요
- 장애물 감지/회피 설정
  - local/global costmap에 obstacle/voxel layer 설정 있음
  - collision monitor 설정 있음
  - 실제 장애물 회피 후 Route 복귀 검증은 아직 필요
- Docking 설정
  - `docking_server` 설정과 Gazebo world 내 dock visual/collision 요소가 있음
  - 실제 Docking Station 접근 성공 검증은 아직 필요

### 후순위 또는 미완료

- FMS 명령 <-> Nav2/Route Server 연동
  - FMS는 추후 FMS 팀에서 제공하는 인터페이스를 받은 뒤 병합 및 테스트 예정
  - 현재 우선순위는 낮음
- Pose <-> Node/Edge 변환
  - 별도 변환 계층 구현은 아직 확인되지 않음
  - 현재 route 실행기는 node id 기반
- 3대 Route 동시 실행
  - 세 대 Nav2 bringup 구조는 있으나, 로봇별 route command/executor와 동시 실행 검증이 필요
- Keepout / Speed-restricted Zone
  - 현재 저장소에서 명확한 costmap filter 설정은 확인되지 않음
- 실물 TurtleBot 1~3대 주행
  - 저장소 기준으로 실물 검증 로그/문서는 아직 확인되지 않음

## Current Priority

FMS 연동은 FMS 팀 산출물이 들어온 뒤 처리한다. 지금 우선순위는 다음 순서로 둔다.

1. 멀티 로봇 Nav2/Route Server 실행 구조 검증
2. `route_goal_executor.py`를 namespace 대응 가능하게 수정
3. 1대 로봇 Route Graph 기반 다중 목적지 주행 검증
4. 3대 Route 동시 실행 검증
5. 장애물 회피 후 기존 Route 복귀 검증
6. Docking Station 접근 검증
7. 이후 FMS 인터페이스 병합 및 end-to-end 테스트

## Useful Launch Files

- Gazebo 3대 spawn:
  - `ros2 launch turtlebot3_gazebo my_map_multi_world.launch.py`
- Multi Nav2 + Route Server:
  - `ros2 launch turtlebot3_navigation2 multi_nav2_route_server.launch.py`
- Single Nav2 + Route Server:
  - `ros2 launch turtlebot3_navigation2 nav2_route_server.launch.py`

## Verification Notes

2026-09-14에 확인한 내용:

- `test.geojson`, `test2.geojson` JSON 문법 정상
- 주요 launch/script 파일 Python 문법 체크 통과
- git working tree clean

실제 Gazebo/Nav2 runtime 검증은 별도로 수행해야 한다.
