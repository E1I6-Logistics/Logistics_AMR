# TurtleBot3 Odometry와 EKF/UKF 위치추정 정리 초안

> 상태: 초안  
> 대상 환경: ROS 2 Jazzy, TurtleBot3 Burger, Gazebo Harmonic  
> 목적: 센서에서 만들어진 값이 `/odom`, TF, EKF/UKF, SLAM·AMCL, Nav2까지 전달되는 전체 흐름을 이해하고 이후 실험 설계의 기준으로 사용한다.

---

## 먼저 보는 전체 흐름 — plain text

```text
[전체 시스템을 한눈에 보기]

                         [ Hardware Layer ]

 Left Wheel Motor / Encoder                 Right Wheel Motor / Encoder
        │ present position                         │ present position
        │ present velocity                         │ present velocity
        └──────────────────────┬────────────────────┘
                               │
                               │ wheel 상태
                               ▼
                            OpenCR
                               ▲
                               │ IMU 상태
                               │
                         IMU processing
                               ▲
                  ┌────────────┼────────────┐
                  │            │            │
             Gyroscope    Accelerometer  Magnetometer
                  ▲            ▲            ▲
                  └────────────┼────────────┘
                         IMU Sensor

 OpenCR이 제공하는 값
   ├─ wheel position / velocity
   ├─ orientation quaternion
   ├─ angular velocity
   ├─ linear acceleration
   └─ magnetic field
                               │
                               ▼
                    OpenCR Control Table
                               │
                               │ USB
                               ▼

                    [ Raspberry Pi / SBC ]

                         turtlebot3_node
                               │
              ┌────────────────┼────────────────┐
              │                │                │
              ▼                ▼                ▼
       /joint_states          /imu       /magnetic_field
              │                │
              └───────┬────────┘
                      ▼
                 odometry.cpp
                      │
                      ▼
                 raw /odom
                      │
          ┌───────────┴───────────┐
          │                       │
          │ EKF/UKF 미사용         │ EKF/UKF 사용
          ▼                       ▼
 odom -> base_footprint    robot_localization <──── /imu
    기본 odometry TF           EKF 또는 UKF
                                  │
                         ┌────────┴────────┐
                         ▼                 ▼
               /odometry/filtered  odom -> base_footprint
                                  filtered TF
                                          │
 /scan + 지도 ───────────────> SLAM 또는 AMCL
                                          │
                                          ▼
                                    map -> odom TF
                                          │
                                          ▼
                                   TF 전체 연결 완성
                                          │
                                          ▼
                                  Nav2 / RViz / 관제


[1. 바퀴가 회전한다]

왼쪽·오른쪽 모터에 붙은 엔코더는 바퀴가 얼마나 회전했는지 측정한다.
엔코더가 처음부터 x, y 좌표를 측정하는 것은 아니다.
엔코더의 원본 정보는 회전 위치에 대응하는 tick/count와 회전 속도에 해당하는 값이다.

왼쪽 엔코더 ── present position / velocity ──┐
                                               ├──> OpenCR
오른쪽 엔코더 ─ present position / velocity ──┘

OpenCR은 모터와 엔코더를 직접 제어하고 읽는다.
OpenCR이 보유한 엔코더 값을 USB 통신을 통해 Raspberry Pi/SBC에서 읽을 수 있도록
Control Table 형태로 제공한다.

Raspberry Pi에서 실행되는 turtlebot3_node는 이 값을 읽는다.
현재 소스에서는 누적 엔코더 차이를 rad 단위의 wheel joint position으로 변환하고,
모터 속도를 wheel velocity로 변환한 뒤 sensor_msgs/JointState 메시지를 만든다.

OpenCR의 엔코더 값
  -> turtlebot3_node의 JointState 처리
  -> 왼쪽·오른쪽 바퀴의 position / velocity
  -> /joint_states


[2. IMU가 로봇의 움직임을 측정한다]

IMU는 하나의 위치 센서가 아니다. 여러 종류의 관성 정보를 측정한다.

Gyroscope     -> 각 축을 중심으로 얼마나 빠르게 회전하는지: angular velocity
Accelerometer -> 각 축 방향으로 측정되는 가속도: linear acceleration
Magnetometer  -> 각 축에서 측정되는 자기장: magnetic field

gyro 값만 적분해서 방향을 계산하면 bias 때문에 시간이 지날수록 오차가 누적된다.
accel은 중력 방향을 이용해 roll/pitch 보정에 도움을 줄 수 있지만 진동과 선형가속의 영향을 받는다.
magnetometer는 방위 보정에 도움을 줄 수 있지만 모터·철 구조물·전류의 영향을 받는다.

따라서 OpenCR 쪽에서는 raw gyro/accel/mag 값 외에도 이들을 처리한 자세 정보를
quaternion 형태로 제공한다.

주의: quaternion은 raw 센서값이 아니다.
      여러 측정값을 이용해 계산된 orientation 결과이다.
      정확히 어떤 자세 필터를 사용하는지는 OpenCR 펌웨어 소스를 추가 확인해야 한다.

IMU 센서
  -> gyro / accel / mag 측정
  -> OpenCR 내부 IMU 처리
  -> orientation quaternion 생성
  -> OpenCR Control Table
  -> USB
  -> Raspberry Pi의 turtlebot3_node

turtlebot3_node는 다음 ROS 메시지를 발행한다.

/imu
  orientation          : quaternion (x, y, z, w)
  angular_velocity     : x, y, z 회전속도
  linear_acceleration  : x, y, z 가속도

/magnetic_field
  magnetic_field       : x, y, z 자기장


[3. 기본 TurtleBot3 odometry가 만들어진다]

odometryf란
**시간에 따른 바퀴 회전과 방향 변화를 누적해서 로봇의 상대적인 위치와 속도를 추정하는 과정**이다.

turtlebot3_node의 odometry.cpp는 /joint_states를 구독한다.
use_imu=true이면 /imu도 함께 구독하고 두 메시지를 시간 동기화하여 처리한다.

매 주기마다 이전 wheel position과 현재 wheel position의 차이를 구한다.

delta_phi_left  = 현재 왼쪽 바퀴 각도 - 이전 왼쪽 바퀴 각도
delta_phi_right = 현재 오른쪽 바퀴 각도 - 이전 오른쪽 바퀴 각도

두 바퀴의 평균 회전량과 wheel radius를 이용해 전진거리 delta_s를 계산한다.

delta_s = wheel_radius * (delta_phi_right + delta_phi_left) / 2

회전량 delta_theta를 만드는 방법은 use_imu 설정에 따라 달라진다.

use_imu=false:
  왼쪽·오른쪽 바퀴 회전량의 차이와 wheel separation으로 delta_theta를 계산한다.
  즉, 전진과 회전을 모두 wheel encoder로 계산한다.

use_imu=true:
  /imu.orientation quaternion을 yaw로 변환한다.
  현재 yaw와 이전 yaw의 차이로 delta_theta를 계산한다.
  즉, 전진거리 delta_s는 wheel, 회전량 delta_theta는 IMU orientation에서 얻는다.

계산한 delta_s와 delta_theta를 이전 pose에 계속 누적한다.

x_next     = x + delta_s * cos(theta + delta_theta / 2)
y_next     = y + delta_s * sin(theta + delta_theta / 2)
theta_next = theta + delta_theta

그 결과를 nav_msgs/Odometry 메시지로 만들어 /odom에 발행한다.

/joint_states ──> wheel delta ──> delta_s ───────────┐
                                                     ├──> pose 적분 ──> /odom
/imu.orientation ─> quaternion -> yaw -> delta_theta ┘   (use_imu=true)

또는

/joint_states ──> wheel delta ──> delta_s + delta_theta ─> pose 적분 ─> /odom
                                                               (use_imu=false)

/odom에는 대략 다음 내용이 들어간다.

header.frame_id = odom
child_frame_id  = base_footprint
pose            = odom 원점에서 본 로봇의 x, y, orientation
twist           = 로봇의 linear.x, angular.z 속도

publish_tf=true이면 같은 계산 결과를 odom -> base_footprint TF로도 발행한다.


[4. 기본 odometry는 EKF가 아니다]

Wheel 정보와 IMU 정보를 둘 다 사용하더라도 자동으로 EKF가 되는 것은 아니다.
현재 TurtleBot3 odometry.cpp는 정해진 방식으로 delta_s와 delta_theta를 선택하고 적분한다.

기본 odometry:
  sensor value -> 정해진 수식 -> 누적 pose

EKF/UKF:
  이전 state와 uncertainty로 prediction
  -> sensor measurement와 measurement uncertainty 입력
  -> 확률적으로 state 보정
  -> state와 covariance 출력

현재 odometry.cpp에는 Kalman gain을 계산하여 센서별 신뢰도를 가중하는 과정이 없다.
따라서 Wheel + IMU 기반 dead-reckoning이라고 부르는 것이 적절하다.


[5. robot_localization을 추가하면]

robot_localization의 ekf_localization_node 또는 ukf_localization_node가
기존 센서·odometry 메시지를 입력으로 받는다.

/odom ────────────────┐
                      ├──> robot_localization ──> /odometry/filtered
/imu ─────────────────┘       EKF 또는 UKF              + TF(선택)

filter는 x, y, yaw, 선속도, 각속도 등 설정에서 선택한 상태만 사용한다.
메시지의 모든 필드를 무조건 융합하는 것이 아니다.

가장 중요한 주의점:
use_imu=true로 만든 /odom의 yaw에는 이미 /imu.orientation 정보가 들어 있다.
그 상태에서 /odom yaw와 /imu orientation yaw를 서로 독립적인 측정처럼 동시에 융합하면
동일한 IMU 정보를 두 번 반영하는 correlated measurement 문제가 생길 수 있다.

실험 구조를 명확하게 하려면 우선 다음 구성을 검토한다.

turtlebot3 odometry: use_imu=false
  -> /odom에서는 wheel 기반 정보만 사용

robot_localization:
  -> wheel-only /odom
  -> 별도의 /imu
  -> 두 입력을 EKF 또는 UKF로 융합
  -> /odometry/filtered 출력

단, 어떤 pose/twist/yaw/yaw-rate 필드를 사용할지는 covariance와 실제 메시지를 확인한 뒤 결정한다.


[6. TF와 Nav2까지 연결된다]

로봇의 지역적이고 연속적인 움직임은 odom 좌표계에서 표현한다.
SLAM 또는 AMCL은 map과 odom의 관계를 계산한다.
robot_state_publisher는 base_link 아래의 센서·바퀴 링크 관계를 발행한다.

map
  └── robot1/odom                 SLAM 또는 AMCL 담당
        └── robot1/base_footprint 기본 odometry 또는 EKF/UKF 담당
              └── robot1/base_link
                    ├── robot1/imu_link
                    ├── robot1/base_scan
                    ├── robot1/wheel_left_link
                    └── robot1/wheel_right_link

Nav2와 RViz는 이 TF 연결을 이용해 지도 위에서 로봇의 위치를 해석한다.

같은 TF를 기본 odometry와 EKF가 동시에 발행하면 충돌한다.
EKF/UKF를 적용할 때는 누가 odom -> base_footprint TF를 소유할지 하나로 정해야 한다.


[7. Gazebo에서는]

실제 로봇에서는 OpenCR과 실제 encoder/IMU가 데이터를 만든다.
Gazebo에서는 물리 시뮬레이터와 센서·DiffDrive 시스템이 비슷한 역할의 데이터를 만든다.

실제 로봇:
  encoder/IMU -> OpenCR -> USB -> turtlebot3_node -> ROS topics

Gazebo:
  simulated wheel/IMU/pose -> Gazebo systems -> bridge -> ROS topics

ROS 토픽 이름이 비슷해도 데이터 생성 주체는 다르다.
따라서 실험 문서에는 실제 로봇 실험인지 Gazebo 실험인지 항상 표시해야 한다.


[8. 전체를 한 문장으로 정리]

엔코더는 바퀴 회전량을, IMU는 회전속도·가속도·자기장과 처리된 자세를 제공하고,
turtlebot3_node는 이 값으로 상대 pose인 /odom을 계산한다.
필요하면 robot_localization이 /odom과 /imu를 uncertainty와 함께 다시 융합하여
/odometry/filtered를 만들며, SLAM/AMCL이 map -> odom을 연결하고 Nav2가 이 전체 TF를 사용한다.
```

---

## 목차

1. [문서의 범위와 핵심 용어](#1-문서의-범위와-핵심-용어)
2. [센서가 실제로 주는 값](#2-센서가-실제로-주는-값)
3. [OpenCR의 역할](#3-opencr의-역할)
4. [Raspberry Pi와 turtlebot3_node의 역할](#4-raspberry-pi와-turtlebot3_node의-역할)
5. [기본 Odometry 계산 과정](#5-기본-odometry-계산-과정)
6. [`/odom` 메시지와 TF의 차이](#6-odom-메시지와-tf의-차이)
7. [`use_imu` 설정에 따른 차이](#7-use_imu-설정에-따른-차이)
8. [EKF와 UKF가 추가되는 위치](#8-ekf와-ukf가-추가되는-위치)
9. [센서 전처리와 정보 중복 문제](#9-센서-전처리와-정보-중복-문제)
10. [TF 소유권 설계](#10-tf-소유권-설계)
11. [SLAM·AMCL·Nav2와의 관계](#11-slamamclnav2와의-관계)
12. [실제 로봇과 Gazebo의 차이](#12-실제-로봇과-gazebo의-차이)
13. [권장 실험 구조](#13-권장-실험-구조)
14. [rosbag과 Ground Truth](#14-rosbag과-ground-truth)
15. [평가 지표와 실험 시나리오](#15-평가-지표와-실험-시나리오)
16. [현재 저장소에서 확인된 사실](#16-현재-저장소에서-확인된-사실)
17. [추가 확인이 필요한 항목](#17-추가-확인이-필요한-항목)
18. [자주 헷갈리는 질문](#18-자주-헷갈리는-질문)
19. [용어 요약](#19-용어-요약)

---

## 1. 문서의 범위와 핵심 용어

이 문서는 다음 세 가지를 구분하는 것을 첫 번째 목표로 한다.

| 구분 | 의미 |
|---|---|
| 센서 측정 | encoder tick, angular velocity, acceleration처럼 센서가 관측한 값 |
| Odometry | 센서값을 시간에 따라 누적해 계산한 상대 위치·자세 추정 |
| Localization | 지도 또는 여러 센서를 이용해 로봇이 어디 있는지 추정하는 과정 |

`odometry`는 센서 부품이 아니라 계산 결과 또는 계산 과정이다. `/odom`은 그 결과를 전달하는 ROS 토픽의 관례적인 이름이다. `odom`은 해당 결과가 표현되는 TF 좌표 프레임 이름으로도 사용된다. 세 단어가 비슷하지만 동일한 대상은 아니다.

## 2. 센서가 실제로 주는 값

### 2.1 Wheel encoder

Wheel encoder는 바퀴가 회전한 정도를 측정한다. 엔코더가 직접 제공하는 것은 일반적으로 다음과 같은 값이다.

- 누적 회전 위치에 대응하는 tick/count
- 단위 시간당 회전량 또는 모터 회전속도
- 장치에 따라 전류, 토크 추정치 등의 부가 정보

엔코더는 다음을 직접 측정하지 않는다.

- 로봇의 전역 `x`, `y`
- 지도 위의 위치
- 바닥에서 실제로 이동한 거리
- wheel slip으로 손실된 이동량

따라서 바퀴가 헛돌아도 엔코더는 바퀴가 회전했다고 보고한다. 이것이 wheel odometry 오차의 대표적인 원인이다.

현재 TurtleBot3 소스에서는 OpenCR의 왼쪽·오른쪽 present position과 velocity를 읽고 다음 형태로 변환해 `/joint_states`에 담는다.

```text
position: 누적 tick 차이 × TICK_TO_RAD
velocity: motor velocity × RPM_TO_MS
```

여기서 `position`은 wheel joint의 회전각이며 로봇의 `x`, `y` 위치가 아니다.

### 2.2 Gyroscope

Gyroscope는 각 축을 중심으로 얼마나 빠르게 회전하는지 측정한다.

```text
angular_velocity.x: roll 축 회전속도
angular_velocity.y: pitch 축 회전속도
angular_velocity.z: yaw 축 회전속도
```

단위는 ROS `sensor_msgs/Imu` 기준으로 `rad/s`다. 평면 이동 로봇에서는 주로 `angular_velocity.z`가 중요하다.

Gyroscope의 대표적인 문제는 bias다. 작은 영점 오차를 계속 적분하면 자세 오차가 시간에 따라 커진다.

### 2.3 Accelerometer

Accelerometer는 각 축에서 측정되는 가속도를 제공한다.

```text
linear_acceleration.x
linear_acceleration.y
linear_acceleration.z
```

단위는 ROS `sensor_msgs/Imu` 기준으로 `m/s²`다. 중력의 영향도 포함될 수 있으므로 가속도 값을 두 번 적분한다고 안정적인 위치가 바로 만들어지는 것은 아니다. 작은 bias와 진동도 이중 적분 과정에서 큰 위치 오차가 된다.

### 2.4 Magnetometer

Magnetometer는 자기장 벡터를 측정한다. 방위 추정에 도움을 줄 수 있지만 다음 영향에 민감하다.

- 모터와 전류
- 철제 선반과 구조물
- 자석
- 실내 자기장 왜곡

현재 TurtleBot3 노드는 자기장 값을 `/imu`가 아니라 별도의 `/magnetic_field` 메시지로 발행한다.

### 2.5 Orientation quaternion

Quaternion은 센서의 raw 값이 아니다. 자세추정 알고리즘의 결과다.

```text
orientation.x
orientation.y
orientation.z
orientation.w
```

Quaternion은 roll, pitch, yaw 자세를 singularity 문제를 줄인 형태로 표현한다. TurtleBot3 SBC 코드는 OpenCR Control Table에 이미 저장된 quaternion을 읽어 `/imu.orientation`에 복사한다.

현재 문서 단계에서는 OpenCR이 quaternion을 만들 때 정확히 어떤 필터를 쓰는지 단정하지 않는다. 해당 내용은 OpenCR 펌웨어에서 `updateIMU()`, `getOrientation()`과 연결된 구현을 확인한 뒤 확정한다.

## 3. OpenCR의 역할

OpenCR은 로봇 하부 하드웨어와 SBC 사이의 실시간 인터페이스다.

```text
모터 명령 수신
  -> Dynamixel 모터 제어
  -> encoder 상태 읽기
  -> IMU 상태 읽기·처리
  -> Control Table 갱신
  -> USB로 SBC가 읽을 수 있게 제공
```

OpenCR이 담당하는 주요 정보는 다음과 같다.

| 정보 | 예시 |
|---|---|
| Wheel 상태 | 왼쪽·오른쪽 present position, velocity |
| IMU orientation | quaternion `w, x, y, z` |
| IMU angular velocity | `x, y, z` |
| IMU linear acceleration | `x, y, z` |
| Magnetic field | `x, y, z` |

중요한 구분은 OpenCR이 값을 제공하는 것과 ROS `/odom`을 최종 계산하는 것이 다르다는 점이다. 현재 실제 로봇 구조에서 `/odom` 계산은 SBC의 `turtlebot3_node/src/odometry.cpp`가 담당한다.

## 4. Raspberry Pi와 turtlebot3_node의 역할

Raspberry Pi/SBC의 `turtlebot3_node`는 OpenCR Control Table을 읽어서 ROS 메시지로 변환한다.

```text
                         OpenCR
                            │
                  Control Table over USB
                            │
                            ▼
                    turtlebot3_node
                            │
          ┌─────────────────┼─────────────────┐
          │                 │                 │
          ▼                 ▼                 ▼
   JointState 처리       IMU 처리        Odometry 처리
          │                 │                 ▲
          ▼                 ▼                 │
   /joint_states           /imu ──────────────┘
          │                                   │
          └───────────────────────────────────┘
                                              ▼
                                            /odom
```

| 처리 객체 | 입력 | ROS 출력 |
|---|---|---|
| JointState 처리 | wheel position, velocity | `/joint_states` |
| IMU 처리 | quaternion, angular velocity, acceleration | `/imu` |
| MagneticField 처리 | magnetic field | `/magnetic_field` |
| Odometry 처리 | `/joint_states`, 선택적으로 `/imu` | `/odom`, 선택적으로 TF |

즉 OpenCR 데이터가 USB로 전달됐다고 곧바로 다른 ROS 노드가 사용할 수 있는 것은 아니다. `turtlebot3_node`가 장치 데이터를 ROS 메시지 규격으로 포장하고 토픽에 발행해야 한다.

## 5. 기본 Odometry 계산 과정

```text
                         /joint_states
                               │
                  ┌────────────┴────────────┐
                  ▼                         ▼
         Left wheel delta          Right wheel delta
                  │                         │
                  └────────────┬────────────┘
                               ▼
                 평균 회전량 × wheel radius
                               │
                               ▼
                           delta_s
                               │
              ┌────────────────┴────────────────┐
              │                                 │
       use_imu=false                     use_imu=true
              │                                 │
   wheel 좌우 차이 사용                   /imu.orientation
              │                                 │
              ▼                                 ▼
         delta_theta              quaternion -> yaw 차이
              │                                 │
              └────────────────┬────────────────┘
                               ▼
                     x, y, theta 누적 적분
                               │
                 ┌─────────────┴─────────────┐
                 ▼                           ▼
              /odom               odom -> base_footprint TF
```

### 5.1 Wheel position 차이

매 주기마다 현재 wheel joint position에서 이전 값을 뺀다.

$$
\Delta\phi_L=\phi_{L,k}-\phi_{L,k-1}
$$

$$
\Delta\phi_R=\phi_{R,k}-\phi_{R,k-1}
$$

### 5.2 전진거리 계산

Wheel radius를 $r$이라고 하면 중심의 이동거리 근사는 다음과 같다.

$$
\Delta s=r\frac{\Delta\phi_R+\Delta\phi_L}{2}
$$

### 5.3 회전량 계산

`use_imu=false`이면 wheel separation $L$을 이용한다.

$$
\Delta\theta=\frac{r(\Delta\phi_R-\Delta\phi_L)}{L}
$$

`use_imu=true`이면 `/imu.orientation` quaternion을 yaw로 변환하고 이전 yaw와의 차이를 사용한다.

$$
\Delta\theta=\operatorname{wrap}(yaw_k-yaw_{k-1})
$$

현재 구현은 각도 차이에 `remainder(..., 2π)`를 사용하여 $-π$와 $π$ 경계를 지날 때 큰 점프가 생기는 것을 줄인다.

### 5.4 Pose 적분

현재 소스는 회전 구간의 중간 방향을 사용해 다음과 같이 pose를 누적한다.

$$
x_{k+1}=x_k+\Delta s\cos(\theta_k+\Delta\theta/2)
$$

$$
y_{k+1}=y_k+\Delta s\sin(\theta_k+\Delta\theta/2)
$$

$$
\theta_{k+1}=\theta_k+\Delta\theta
$$

### 5.5 속도 계산

샘플 간 시간차를 $\Delta t$라고 하면:

$$
v=\frac{\Delta s}{\Delta t}
$$

$$
\omega=\frac{\Delta\theta}{\Delta t}
$$

이 값은 `/odom.twist.twist.linear.x`와 `/odom.twist.twist.angular.z`에 들어간다.

## 6. `/odom` 메시지와 TF의 차이

`/odom`은 토픽이고 `odom`은 좌표 프레임이다.

```text
/odom
  타입: nav_msgs/msg/Odometry
  역할: pose, twist, covariance 등을 다른 노드에 전달

odom -> base_footprint
  타입: TF 변환 관계
  역할: TF tree에서 두 좌표계의 상대 위치를 제공
```

같은 계산 결과가 `/odom` 메시지와 TF 양쪽으로 표현될 수 있지만 소비 방식이 다르다. Nav2, RViz와 센서 변환은 TF 연결을 필요로 하고, 상태추정 노드는 `/odom` 메시지의 pose·twist·covariance를 입력으로 사용할 수 있다.

현재 `odometry.cpp`는 `odometry.publish_tf=true`인 경우 `odom -> base_footprint`도 발행한다.

## 7. `use_imu` 설정에 따른 차이

| 설정 | 전진거리 | 회전량 | 성격 |
|---|---|---|---|
| `use_imu=false` | Wheel | Wheel 차이 | Wheel-only dead-reckoning |
| `use_imu=true` | Wheel | IMU orientation yaw 차이 | Wheel translation + IMU rotation |

`use_imu=true`가 항상 더 좋다고 단정할 수는 없다. IMU orientation 품질, 자기장 교란, 진동, 시간 동기화와 wheel slip 조건에 따라 결과가 달라질 수 있다.

또한 어떤 설정이 실제로 적용됐는지는 설치된 파일만 보고 추정하지 말고 실행 중 파라미터를 확인해야 한다.

```bash
ros2 param get /diff_drive_controller odometry.use_imu
```

멀티로봇 namespace를 사용한다면 실제 노드 이름에 맞춰 확인한다.

```bash
ros2 param get /robot1/diff_drive_controller odometry.use_imu
```

## 8. EKF와 UKF가 추가되는 위치

`robot_localization`은 보통 SBC에서 별도 ROS 노드로 실행한다.

```text
Wheel Encoder                      IMU
      │                             │
      ▼                             ▼
wheel-only /odom                  /imu
      │                             │
      └──────────────┬──────────────┘
                     ▼
              robot_localization
              ┌──────┴──────┐
              │             │
              ▼             ▼
             EKF           UKF
              │             │
              └──────┬──────┘
                     ▼
            /odometry/filtered
                     │
                     ├─ filtered pose
                     ├─ filtered velocity
                     ├─ covariance
                     └─ odom -> base_footprint TF (선택)
                     │
                     ▼
                 Nav2 / RViz
```

### 8.1 EKF

EKF는 비선형 motion model과 measurement model을 현재 추정값 주변에서 Jacobian으로 선형화한다.

```text
이전 state + covariance
  -> Prediction
  -> 측정값과 measurement covariance 입력
  -> Innovation 계산
  -> Kalman gain 계산
  -> state와 covariance 보정
```

### 8.2 UKF

UKF는 비선형 분포를 근사하기 위해 여러 sigma point를 생성하고 비선형 함수에 통과시킨다. Jacobian을 직접 사용하는 EKF와 접근 방식이 다르다.

UKF가 언제나 EKF보다 정확한 것은 아니다. 모델, 센서 품질, 파라미터, 연산량에 따라 결과가 달라지므로 같은 rosbag과 Ground Truth를 사용한 비교가 필요하다.

### 8.3 Covariance의 의미

Covariance는 해당 값이 얼마나 불확실한지를 표현한다.

```text
작은 covariance -> 상대적으로 신뢰도가 높다고 해석
큰 covariance   -> 상대적으로 불확실하다고 해석
```

단순히 값을 작게 설정한다고 센서가 실제로 정확해지는 것은 아니다. 잘못된 covariance는 filter가 나쁜 측정을 과신하거나 좋은 측정을 무시하게 만들 수 있다.

현재 TurtleBot3 `odometry.cpp`에는 covariance 할당 코드가 주석 처리되어 있다. 실제 `/odom`의 covariance가 어떤 값으로 발행되는지 반드시 런타임 메시지에서 확인해야 한다.

## 9. 센서 전처리와 정보 중복 문제

### 9.1 전처리된 센서값을 EKF/UKF에 넣어도 되는가?

가능하며 일반적인 구조다. EKF/UKF가 반드시 ADC count 같은 완전한 raw 값만 받아야 하는 것은 아니다. 보통 물리 단위로 변환되고 기본 보정이 적용된 측정값을 입력으로 사용한다.

```text
물리적인 움직임
      ↓
센서의 전기 신호 / ADC count
      ↓
Scale·offset·bias 보정
      ↓
단위 변환
      ↓
센서 내부 low-pass filter
      ↓
ROS에서 사용할 수 있는 측정값
      ↓
EKF / UKF 상태추정
```

앞단의 전처리와 EKF/UKF는 목적이 다르다.

| 단계 | 주요 목적 |
|---|---|
| 센서 내부 필터 | 전기적 노이즈, 진동, 고주파 노이즈 감소 |
| Calibration·단위 변환 | bias와 scale 보정, ROS 표준 단위로 표현 |
| Attitude filter | gyro·accel·mag를 이용해 orientation 계산 |
| Wheel odometry | wheel 변화량을 상대 pose와 velocity로 변환 |
| EKF/UKF | 여러 측정과 uncertainty를 이용해 robot state 추정 |

따라서 다음은 정상적인 입력 구조다.

```text
Encoder tick
  -> rad 변환
  -> Wheel velocity 또는 Wheel odometry
  -> EKF/UKF

Gyro ADC 값
  -> bias·scale 보정
  -> rad/s 변환
  -> EKF/UKF
```

문제는 값이 가공됐다는 사실 자체가 아니라, 이미 같은 센서에서 파생된 정보를 서로 독립적인 측정처럼 반복해서 넣는 것이다.

### 9.2 Encoder 경로에서의 가공

현재 확인된 SBC 측 코드는 다음 순서로 처리한다.

```text
Motor Encoder
  -> present position / present velocity
  -> OpenCR Control Table
  -> USB
  -> turtlebot3_node
  -> tick을 wheel joint rad로 변환
  -> motor velocity를 ROS에서 사용할 값으로 변환
  -> /joint_states
```

`tick -> rad`는 sensor fusion이 아니라 단위 변환이다. `odometry.cpp`는 wheel position의 시간 차이를 이용해 pose를 적분한다. 현재 SBC의 이 과정에는 EKF/UKF가 없다.

다만 motor 또는 OpenCR이 present velocity를 만들 때 내부 smoothing을 적용하는지는 장치 펌웨어까지 확인해야 확정할 수 있다. 기본 pose 계산은 velocity보다 wheel position 차이를 주로 사용하므로 이 두 경로도 구분해서 본다.

### 9.3 IMU 경로에서의 가공

`/imu` 메시지 안에서도 필드의 처리 수준이 다르다.

| 필드 | 의미 | 처리 수준 |
|---|---|---|
| `angular_velocity` | Gyroscope 기반 회전속도 | 센서 측정에 가까우나 scale·bias·내부 필터가 적용될 수 있음 |
| `linear_acceleration` | Accelerometer 기반 가속도 | 센서 측정에 가까우나 scale·bias·내부 필터가 적용될 수 있음 |
| `orientation` | Quaternion 자세 | gyro·accel·mag 등으로 계산된 추정 결과 |
| `/magnetic_field` | Magnetometer 기반 자기장 | 센서 측정에 가까우나 calibration이 필요할 수 있음 |

```text
Gyroscope ─────┐
Accelerometer ─┼──> OpenCR의 자세 처리 ──> orientation quaternion
Magnetometer ──┘
```

따라서 orientation quaternion은 raw IMU 값이라고 부르기 어렵다. 현재 SBC 코드는 OpenCR Control Table에서 이미 계산된 quaternion을 읽는다. OpenCR이 정확히 어떤 attitude filter를 사용하는지는 펌웨어 소스를 확인하기 전까지 단정하지 않는다.

### 9.4 실제로 문제가 되는 중복 경로

`use_imu=true`인 기본 `/odom`은 다음 관계를 가진다.

```text
/imu.orientation
      ├──────────────> EKF의 imu 입력
      └─> odometry.cpp -> /odom yaw -> EKF의 odom 입력
```

이 두 값은 서로 독립적인 센서가 아니다. 같은 IMU orientation에서 파생된 값이다. EKF에 독립적인 측정 두 개처럼 넣으면 정보량을 실제보다 크게 평가할 수 있다.

```text
문제가 아닌 것:
  보정된 gyro rad/s를 EKF에 입력
  wheel tick을 rad로 변환한 뒤 EKF에 입력

주의할 것:
  /imu.orientation yaw
  + 같은 orientation으로 만든 /odom yaw
  두 값을 독립적인 센서처럼 동시 융합
```

해결 방향은 여러 가지다.

1. `/odom`에서는 IMU에서 파생된 yaw를 융합하지 않고 wheel 기반 속도·위치 일부만 선택한다.
2. `use_imu=false`로 wheel-only odometry를 만들고 `/imu`를 별도 입력으로 사용한다.
3. 기본 구조를 유지하되 `/imu`에서는 중복되지 않는 angular velocity 등 필요한 항목만 신중하게 사용한다.

이 프로젝트의 비교 실험에서는 2번이 입력 출처를 가장 명확히 분리한다. 다만 실제 적용 전에는 메시지 covariance, 시간 동기화, TF 발행 주체를 함께 검증해야 한다.

### 9.5 IMU 내부에서도 상관관계가 있다

`/imu.angular_velocity.z`와 `/imu.orientation.yaw`도 완전히 독립적이지 않을 수 있다. Orientation 계산에 같은 gyroscope 값이 사용됐을 가능성이 높기 때문이다.

```text
같은 Gyroscope
   ├──> angular_velocity.z
   └──> attitude filter ──> orientation.yaw
```

두 값을 함께 사용할 수는 있지만, EKF/UKF가 이 상관관계를 자동으로 완벽하게 아는 것은 아니다. 입력 covariance, orientation 생성 방식과 실험 결과를 함께 확인해야 한다.

### 9.6 과도한 전처리의 문제

전처리 자체는 정상적이지만 지나치게 강한 low-pass filter는 지연을 만들 수 있다.

```text
과도한 smoothing
      ↓
측정값의 시간 지연
      ↓
Encoder와 IMU가 서로 다른 시점의 움직임을 표현
      ↓
EKF innovation 증가 또는 추정 진동
```

EKF/UKF 입력을 정할 때 다음을 함께 확인한다.

- 적용된 calibration과 filter 종류
- 출력 단위
- timestamp와 센서 간 시간 동기화
- pose/twist/orientation covariance
- 동일 정보의 중복 여부
- 필터 지연
- 센서 dropout과 이상값 처리

### 9.7 이 프로젝트의 권장 단계별 실험

가장 해석하기 쉬운 순서로 입력을 하나씩 추가한다.

```text
[실험 A]
Wheel-only /odom의 linear velocity x, angular velocity z
+ IMU angular velocity z

[실험 B]
Wheel-only /odom
+ IMU orientation yaw

[실험 C]
Wheel-only /odom
+ IMU angular velocity z
+ IMU orientation yaw

[실험 D]
필요성과 품질을 확인한 뒤 IMU acceleration 추가 검토
```

실험 A에서는 wheel 회전속도와 gyroscope 회전속도가 서로 다른 물리 센서에서 나오므로 역할을 비교하기 쉽다. 실험 B와 C를 통해 OpenCR orientation을 추가했을 때 실제 정확도가 좋아지는지 확인한다.

초기에는 acceleration을 제외하는 편이 해석하기 쉽다. Acceleration은 중력 제거, 장착 방향, bias, 진동과 covariance를 먼저 검증해야 한다.

이 장의 결론은 다음과 같다.

> 보정되거나 저역통과 필터를 거친 센서값을 EKF/UKF에 넣는 것은 일반적이다. 주의해야 할 부분은 이미 융합되거나 적분된 동일 정보를 여러 입력으로 중복 반영하는 것과, 전처리로 생긴 지연·covariance를 무시하는 것이다.

## 10. TF 소유권 설계

정상적인 2D 로봇 TF는 대략 다음과 같다.

```text
map
  └── odom
        └── base_footprint
              └── base_link
                    └── sensors / wheels
```

각 구간의 일반적인 소유자는 다음과 같다.

| TF | 일반적인 발행 주체 |
|---|---|
| `map -> odom` | SLAM Toolbox 또는 AMCL |
| `odom -> base_footprint` | 기본 odometry 또는 robot_localization 중 하나 |
| `base_footprint -> base_link -> sensors` | robot_state_publisher |

EKF가 `odom -> base_footprint`를 발행하도록 설정했다면 기본 TurtleBot3 odometry의 동일 TF 발행은 꺼야 한다. `/odom` 토픽 발행 자체와 TF 발행 여부는 별도로 생각해야 한다.

```text
허용 가능:
기본 노드 -> raw /odom topic
EKF       -> filtered /odometry/filtered + odom -> base_footprint TF

피해야 함:
기본 노드 -> odom -> base_footprint TF
EKF       -> odom -> base_footprint TF
두 노드가 같은 TF를 동시에 발행
```

## 11. SLAM·AMCL·Nav2와의 관계

Wheel odometry나 EKF만으로는 지도 위의 절대 위치가 만들어지지 않는다. 이들은 주로 로봇의 연속적인 지역 이동을 `odom` 좌표계에서 제공한다.

```text
                         저장된 지도
                             │
                             ▼
 /scan ───────────────────> AMCL
                             │
                             │ map -> odom
                             ▼
 map ─────────────────────> odom
                             │
                             │ odom -> base_footprint
                             ▼
                       base_footprint
                             │
                             ▼
                          base_link
                             │
                ┌────────────┼────────────┐
                ▼            ▼            ▼
             IMU link     Laser link    Wheel links

 Nav2가 보는 로봇 위치:
 map -> odom -> base_footprint -> base_link

 RViz가 그리는 로봇 위치:
 같은 TF chain을 따라 계산한 map 기준 위치
```

### SLAM 중

```text
scan + odom/filtered odometry
  -> SLAM Toolbox
  -> 지도 생성
  -> map -> odom 보정
```

### 저장된 지도에서 Navigation 중

```text
저장된 map + scan + odom/filtered odometry
  -> AMCL
  -> map -> odom 보정
```

Nav2는 다음을 함께 사용한다.

- `map` 기준의 목표 위치
- `map -> odom -> base_link` TF
- `/scan` 장애물 정보
- odometry와 속도
- costmap과 planner/controller 결과

EKF/UKF는 일반적으로 `map -> odom`을 대신 계산하는 SLAM/AMCL과 역할이 다르다. 설정에 따라 전역 센서를 융합할 수도 있지만, 현재 프로젝트의 1차 목표는 지역 odometry 품질 비교다.

## 12. 실제 로봇과 Gazebo의 차이

| 구분 | 실제 TurtleBot3 | Gazebo Harmonic |
|---|---|---|
| Wheel 데이터 생성 | 실제 motor encoder | 물리 시뮬레이션의 joint/DiffDrive system |
| IMU 데이터 생성 | 실제 IMU와 OpenCR 처리 | Gazebo IMU sensor |
| 장치 전달 | OpenCR Control Table와 USB | Gazebo Transport와 ROS bridge |
| Ground Truth | 별도 외부 장치 필요 | 시뮬레이터의 실제 model pose 활용 가능 |
| 오차 특성 | slip, bias, 진동, 통신 지연 | 설정한 noise와 물리 모델 오차 |

Gazebo 결과가 좋다고 실제 로봇에서도 같은 성능이 보장되는 것은 아니다. 반대로 Gazebo는 알고리즘 구조와 TF, 설정 오류를 안전하게 검증하고 Ground Truth를 얻기에 유리하다.

## 13. 권장 실험 구조

### 13.1 Baseline A — 현재 기본 설정

```text
현재 launch와 parameter를 변경하지 않은 TurtleBot3 /odom
```

먼저 실제 `use_imu` 값과 covariance, TF publisher를 기록한다. 이것이 비교 기준이다.

### 13.2 Baseline B — Wheel-only odometry

```text
use_imu=false
/joint_states -> wheel-only /odom
```

Wheel slip과 회전 오차가 어떤 형태로 나타나는지 확인한다.

### 13.3 EKF

```text
wheel-only /odom + /imu
  -> ekf_localization_node
  -> /odometry/filtered
```

### 13.4 UKF

```text
동일한 wheel-only /odom + 동일한 /imu
  -> ukf_localization_node
  -> /odometry/filtered
```

EKF와 UKF에 서로 다른 주행 데이터를 넣으면 공정한 비교가 어렵다. 하나의 rosbag을 재생하거나 동일한 자동 주행 명령을 반복해야 한다.

## 14. rosbag과 Ground Truth

기본 기록 후보는 다음과 같다.

```bash
ros2 bag record \
  /odom \
  /imu \
  /joint_states \
  /cmd_vel \
  /odometry/filtered \
  /tf \
  /tf_static
```

멀티로봇 환경에서는 namespace가 포함된 실제 토픽으로 바꿔야 한다.

```bash
ros2 bag record \
  /robot1/odom \
  /robot1/imu \
  /robot1/joint_states \
  /robot1/cmd_vel \
  /robot1/odometry/filtered \
  /tf \
  /tf_static
```

rosbag은 메시지를 재현할 수 있게 저장하지만 Ground Truth를 자동으로 제공하지 않는다. `/odom`과 `/odometry/filtered`는 둘 다 추정값이다.

Ground Truth 후보:

- Gazebo model pose
- Motion Capture
- 외부 카메라와 AprilTag
- 정밀하게 측정한 시작·종료 위치
- 바닥 마킹과 줄자·각도 측정

## 15. 평가 지표와 실험 시나리오

### 15.1 Position error

$$
e_{position}=\sqrt{(x_{est}-x_{gt})^2+(y_{est}-y_{gt})^2}
$$

### 15.2 비교 지표

- Position RMSE
- Yaw RMSE
- 최종 위치 drift
- 최종 yaw error
- 반복 실험의 평균과 표준편차
- 센서 dropout 후 복구 특성
- CPU 사용량과 처리 지연

### 15.3 기본 시나리오

1. 2~5 m 직선 주행
2. 360도 제자리 회전
3. 사각 경로 주행 후 시작점 복귀
4. 빠른 회전
5. wheel slip 조건
6. IMU noise 증가
7. encoder noise 증가
8. sensor dropout

각 조건은 최소 5회 이상 반복하고 가능한 한 동일한 명령과 초기조건을 사용한다.

## 16. 현재 저장소에서 확인된 사실

현재 `/home/woozoo/dev/Logistics_AMR` 소스에서 확인된 내용이다.

1. `turtlebot3_node/src/sensors/joint_state.cpp`
   - OpenCR의 left/right present position과 velocity를 읽는다.
   - position을 rad 기반 joint position으로 변환한다.
   - `/joint_states`에 발행한다.

2. `turtlebot3_node/src/sensors/imu.cpp`
   - OpenCR Control Table에서 orientation quaternion을 읽는다.
   - angular velocity와 linear acceleration을 읽는다.
   - `/imu`에 발행한다.
   - magnetic field를 별도 메시지로 발행한다.

3. `turtlebot3_node/src/odometry.cpp`
   - `/joint_states`를 입력으로 받는다.
   - `use_imu=true`이면 `/imu`를 함께 시간 동기화하여 받는다.
   - wheel 평균 회전량으로 `delta_s`를 계산한다.
   - `use_imu=true`이면 IMU quaternion의 yaw 차이로 `delta_theta`를 계산한다.
   - `use_imu=false`이면 wheel 차이로 `delta_theta`를 계산한다.
   - `/odom`과 선택적으로 `odom -> base_footprint` TF를 발행한다.
   - covariance 설정 부분은 현재 주석 처리되어 있다.

4. 파라미터 파일 상태
   - `turtlebot3_node/param/burger.yaml`에는 `use_imu: true`가 있다.
   - `turtlebot3_bringup/param/burger.yaml`에는 `use_imu: false`가 있다.
   - 따라서 실행 launch가 실제로 어느 파일을 로드하는지 확인해야 최종 동작을 확정할 수 있다.

## 17. 추가 확인이 필요한 항목

다음 내용은 추정으로 확정하지 않고 소스 또는 런타임에서 확인한다.

- TurtleBot3 Burger에 장착된 정확한 IMU 모델과 펌웨어 버전
- OpenCR의 `updateIMU()` 내부 처리
- `getOrientation()` quaternion 생성 알고리즘
- Madgwick, Mahony, complementary filter 등의 실제 사용 여부
- magnetometer가 orientation 계산에 실제로 사용되는지
- `/imu` orientation/angular velocity/acceleration covariance 값
- `/odom` pose/twist covariance 값
- 실행 중 실제 `odometry.use_imu` 값
- 현재 TF publisher 목록과 중복 여부
- Gazebo에서 `/odom`과 `/imu`를 생성하는 정확한 plugin/system 설정
- Ground Truth 토픽 또는 Gazebo pose 획득 방법

## 18. 자주 헷갈리는 질문

### Q. Odom은 어떤 부품인가?

부품이 아니다. Wheel encoder와 IMU 등의 값을 시간에 따라 누적해서 만든 상대 위치추정 결과다.

### Q. IMU가 위치 `x`, `y`를 알려주는가?

직접 알려주지 않는다. IMU는 주로 angular velocity와 acceleration을 측정하고, 처리된 orientation을 제공할 수 있다. 가속도의 이중 적분만으로 안정적인 위치를 구하기는 어렵다.

### Q. `/odom`과 `odom`은 같은 것인가?

아니다. `/odom`은 메시지가 흐르는 토픽이고 `odom`은 좌표 프레임 이름이다.

### Q. Wheel과 IMU를 같이 사용하면 EKF인가?

아니다. 센서가 두 개라는 사실과 Kalman filter 사용 여부는 별개다. Prediction covariance, measurement covariance, Kalman gain을 이용한 확률적 update가 있어야 EKF 구조라고 할 수 있다.

### Q. EKF를 사용하면 SLAM이나 AMCL이 필요 없는가?

일반적인 현재 구성에서는 필요하다. EKF는 연속적인 지역 motion estimate를 개선하고, SLAM/AMCL은 지도 기준 `map -> odom` 관계를 제공한다.

### Q. `/odom`과 `/imu`를 모두 EKF에 넣으면 되는가?

메시지 전체를 무조건 넣으면 안 된다. `/odom`이 이미 IMU에서 파생된 yaw를 포함하는지, 어떤 covariance를 가지는지, 어떤 state field를 선택할지 검토해야 한다.

### Q. Filtered odometry가 실제 위치인가?

아니다. 더 체계적으로 융합된 추정값이다. 정확도를 평가하려면 별도의 Ground Truth가 필요하다.

## 19. 용어 요약

| 용어 | 쉬운 의미 |
|---|---|
| Raw measurement | 센서가 직접 측정한 값 |
| Encoder tick | 바퀴 회전량을 나타내는 카운트 |
| Joint state | 관절 이름과 position/velocity 상태를 담는 ROS 메시지 |
| IMU | 각속도와 가속도 등을 측정하는 관성 센서 |
| Quaternion | 3차원 orientation을 표현하는 네 값 |
| Odometry | 이동량을 누적해 구한 상대 위치추정 |
| Dead-reckoning | 이전 위치와 이동량을 계속 누적하는 추정 방식 |
| Covariance | 추정 또는 측정의 불확실성 표현 |
| Sensor fusion | 여러 측정정보를 함께 사용해 상태를 추정하는 과정 |
| EKF | 비선형 모델을 국소 선형화하는 확장 칼만 필터 |
| UKF | Sigma point를 이용하는 비선형 칼만 필터 |
| TF | ROS 좌표 프레임 사이의 변환 관계 |
| `map` | 지도 기준의 전역 좌표 프레임 |
| `odom` | 연속적이지만 drift가 누적될 수 있는 지역 좌표 프레임 |
| `base_footprint` | 바닥면에 투영한 로봇 기준 프레임 |
| `base_link` | 로봇 본체 기준 프레임 |
| SLAM | 지도 작성과 자기 위치추정을 함께 수행 |
| AMCL | 저장된 지도에서 로봇 위치를 추정 |
| Ground Truth | 성능 비교 기준이 되는 실제 위치·자세 |

---

## 초안 이후의 다음 작업

1. 실제 실행 launch가 사용하는 Burger 파라미터 파일을 추적한다.
2. 실행 중 `/odom`, `/imu`, covariance와 TF publisher를 확인한다.
3. OpenCR 펌웨어에서 orientation quaternion 생성 경로를 추적한다.
4. Baseline을 보존한 상태로 EKF/UKF 패키지 구조를 설계한다.
5. `odom0_config`, `imu0_config`, TF ownership 초안을 작성한다.
6. Gazebo Ground Truth 기록 방법과 rosbag 실험 절차를 확정한다.
