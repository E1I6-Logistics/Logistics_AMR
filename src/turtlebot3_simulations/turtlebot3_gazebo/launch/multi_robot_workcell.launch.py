#!/usr/bin/env python3

import os
import xml.etree.ElementTree as ET

import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


# ============================================================
# Timing
# ============================================================

# Gazebo가 먼저 뜰 시간
FIRST_ROBOT_DELAY = 5.0

# 이전 robot spawn 성공 프로세스 종료 후
# 다음 robot 시작까지 대기
NEXT_ROBOT_DELAY = 2.0


# ============================================================
# multi_robot.yaml
# ============================================================

def _load_robots(config_path):

    with open(config_path, 'r', encoding='utf-8') as config_file:
        config = yaml.safe_load(config_file) or {}

    robots = config.get('robots', [])

    if not robots:
        raise RuntimeError(
            f'No robots configured in {config_path}'
        )

    names = [
        robot.get('name')
        for robot in robots
    ]

    if any(not name for name in names):
        raise RuntimeError(
            'Every robot must have a name.'
        )

    if len(names) != len(set(names)):
        raise RuntimeError(
            'Robot names must be unique.'
        )

    required_pose_fields = {
        'x',
        'y',
        'z',
        'yaw',
    }

    for robot in robots:

        missing = (
            required_pose_fields
            - robot.keys()
        )

        if missing:
            raise RuntimeError(
                f"Robot {robot['name']} "
                f"is missing pose fields: "
                f"{sorted(missing)}"
            )

    return robots


# ============================================================
# Topic namespace
# ============================================================

def _prefix_topic(robot_name, topic):

    if topic is None:
        return topic

    topic = topic.strip()

    if not topic:
        return topic

    if topic.startswith(
        f'/{robot_name}/'
    ):
        return topic

    return (
        f'/{robot_name}/'
        f'{topic.strip("/")}'
    )


# ============================================================
# XML helper
# ============================================================

def _set_or_create(
    parent,
    tag,
    value,
):

    element = parent.find(tag)

    if element is None:
        element = ET.SubElement(
            parent,
            tag,
        )

    element.text = value


# ============================================================
# Robot별 Gazebo SDF
# ============================================================

def _make_scoped_sdf(
    model_path,
    robot_name,
):

    tree = ET.parse(
        model_path
    )

    root = tree.getroot()

    model = root.find(
        'model'
    )

    if model is None:
        raise RuntimeError(
            f'No <model> element found '
            f'in {model_path}'
        )

    # --------------------------------------------------------
    # Gazebo Entity 이름
    # --------------------------------------------------------

    model.set(
        'name',
        robot_name,
    )

    # --------------------------------------------------------
    # Gazebo topic namespace
    #
    # scan
    # -> /robot1/scan
    #
    # odom
    # -> /robot1/odom
    # --------------------------------------------------------

    for element in model.iter():

        tag = (
            element.tag
            .split('}')[-1]
        )

        if tag in {
            'topic',
            'odom_topic',
            'tf_topic',
        }:

            if element.text:

                element.text = (
                    _prefix_topic(
                        robot_name,
                        element.text,
                    )
                )

    # --------------------------------------------------------
    # DiffDrive
    #
    # TF frame 자체도 고유하게 한다.
    #
    # robot1/odom
    #     ↓
    # robot1/base_footprint
    # --------------------------------------------------------

    for plugin in model.findall(
        'plugin'
    ):

        plugin_id = (
            plugin.get('filename', '')
            +
            ' '
            +
            plugin.get('name', '')
        ).lower()

        if (
            'diff-drive' in plugin_id
            or
            'diffdrive' in plugin_id
        ):

            _set_or_create(
                plugin,
                'topic',
                f'/{robot_name}/cmd_vel',
            )

            _set_or_create(
                plugin,
                'odom_topic',
                f'/{robot_name}/odom',
            )

            _set_or_create(
                plugin,
                'tf_topic',
                f'/{robot_name}/tf',
            )

            _set_or_create(
                plugin,
                'frame_id',
                f'{robot_name}/odom',
            )

            _set_or_create(
                plugin,
                'child_frame_id',
                (
                    f'{robot_name}/'
                    f'base_footprint'
                ),
            )

    # --------------------------------------------------------
    # Sensor frame ID
    #
    # base_scan
    # -> robot1/base_scan
    #
    # imu_link
    # -> robot1/imu_link
    # --------------------------------------------------------

    for link in model.findall(
        'link'
    ):

        link_name = link.get(
            'name'
        )

        if not link_name:
            continue

        for sensor in link.findall(
            'sensor'
        ):

            frame = sensor.find(
                'gz_frame_id'
            )

            if frame is None:

                frame = ET.SubElement(
                    sensor,
                    'gz_frame_id',
                )

            frame.text = (
                f'{robot_name}/'
                f'{link_name}'
            )

    return ET.tostring(
        root,
        encoding='unicode',
    )


# ============================================================
# Gazebo <-> ROS bridge
# ============================================================

def _bridge_arguments(
    robot_names,
):

    arguments = [

        '/clock'
        '@rosgraph_msgs/msg/Clock'
        '[gz.msgs.Clock',

    ]

    for robot_name in robot_names:

        prefix = (
            f'/{robot_name}'
        )

        arguments.extend([

            (
                f'{prefix}/joint_states'
                '@sensor_msgs/msg/JointState'
                '[gz.msgs.Model'
            ),

            (
                f'{prefix}/odom'
                '@nav_msgs/msg/Odometry'
                '[gz.msgs.Odometry'
            ),

            (
                f'{prefix}/tf'
                '@tf2_msgs/msg/TFMessage'
                '[gz.msgs.Pose_V'
            ),

            (
                f'{prefix}/cmd_vel'
                '@geometry_msgs/msg/TwistStamped'
                ']gz.msgs.Twist'
            ),

            (
                f'{prefix}/imu'
                '@sensor_msgs/msg/Imu'
                '[gz.msgs.IMU'
            ),

            (
                f'{prefix}/scan'
                '@sensor_msgs/msg/LaserScan'
                '[gz.msgs.LaserScan'
            ),
        ])

    return arguments


# ============================================================
# Spawn Scheduler
# ============================================================

def _launch_robots(
    context,
    model_path,
    urdf_path,
    robots_file,
    use_sim_time,
):

    config_path = (
        context.perform_substitution(
            robots_file
        )
    )

    robots = _load_robots(
        config_path
    )

    # --------------------------------------------------------
    # 기본 URDF
    #
    # frame_prefix는 RSP parameter로 처리
    # --------------------------------------------------------

    with open(
        urdf_path,
        'r',
        encoding='utf-8',
    ) as urdf_file:

        robot_description = (
            urdf_file.read()
        )

    actions = []

    # ========================================================
    # Bridge
    # ========================================================

    robot_names = [
        robot['name']
        for robot in robots
    ]

    actions.append(

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',

            name=(
                'multi_robot_'
                'parameter_bridge'
            ),

            arguments=(
                _bridge_arguments(
                    robot_names
                )
            ),

            output='screen',
        )
    )

    # ========================================================
    # 각 Robot action 생성
    # ========================================================

    spawn_nodes = []
    robot_groups = []

    for robot in robots:

        robot_name = (
            robot['name']
        )

        scoped_sdf = (
            _make_scoped_sdf(
                model_path,
                robot_name,
            )
        )

        # ----------------------------------------------------
        # robot_state_publisher
        #
        # /robot1/tf
        # /robot1/tf_static
        #
        # frame:
        # robot1/base_footprint
        # robot1/base_link
        # robot1/base_scan
        # ...
        # ----------------------------------------------------

        state_publisher = Node(

            package=(
                'robot_state_publisher'
            ),

            executable=(
                'robot_state_publisher'
            ),

            namespace=
                robot_name,

            name=
                'robot_state_publisher',

            output='screen',

            parameters=[{

                'use_sim_time':
                    use_sim_time,

                'robot_description':
                    robot_description,

                'frame_prefix':
                    f'{robot_name}/',
            }],

            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ],

        )

        # ----------------------------------------------------
        # Gazebo Spawn
        # ----------------------------------------------------

        spawn_node = Node(

            package='ros_gz_sim',

            executable='create',

            name=(
                f'spawn_{robot_name}'
            ),

            output='screen',

            arguments=[

                '-name',
                robot_name,

                '-string',
                scoped_sdf,

                '-x',
                str(robot['x']),

                '-y',
                str(robot['y']),

                '-z',
                str(robot['z']),

                '-Y',
                str(robot['yaw']),
            ],
        )

        spawn_nodes.append(
            spawn_node
        )

        robot_groups.append(

            GroupAction([

                state_publisher,

                spawn_node,
            ])
        )

    # ========================================================
    # 먼저 이벤트 핸들러 등록
    #
    # robot1 spawn process 종료
    # -> 2초
    # -> robot2
    #
    # robot2 종료
    # -> 2초
    # -> robot3
    # ========================================================

    for index in range(
        len(robots) - 1
    ):

        current_spawn = (
            spawn_nodes[index]
        )

        next_robot_group = (
            robot_groups[
                index + 1
            ]
        )

        actions.append(

            RegisterEventHandler(

                OnProcessExit(

                    target_action=
                        current_spawn,

                    on_exit=[

                        TimerAction(

                            period=
                                NEXT_ROBOT_DELAY,

                            actions=[
                                next_robot_group
                            ],
                        )
                    ],
                )
            )
        )

    # ========================================================
    # 첫 로봇만 Gazebo 시작 5초 후 실행
    # ========================================================

    if robot_groups:

        actions.append(

            TimerAction(

                period=
                    FIRST_ROBOT_DELAY,

                actions=[
                    robot_groups[0]
                ],
            )
        )

    return actions


# ============================================================
# Main
# ============================================================

def generate_launch_description():

    pkg_turtlebot3_gazebo = (
        get_package_share_directory(
            'turtlebot3_gazebo'
        )
    )

    ros_gz_sim = (
        get_package_share_directory(
            'ros_gz_sim'
        )
    )

    model_name = os.environ.get(
        'TURTLEBOT3_MODEL',
        'burger',
    )

    # ========================================================
    # Paths
    # ========================================================

    model_path = os.path.join(

        pkg_turtlebot3_gazebo,

        'models',

        f'turtlebot3_'
        f'{model_name}',

        'model.sdf',
    )

    urdf_path = os.path.join(

        pkg_turtlebot3_gazebo,

        'urdf',

        f'turtlebot3_'
        f'{model_name}.urdf',
    )

    world_path = os.path.join(

        pkg_turtlebot3_gazebo,

        'worlds',

        'amr_workcell.sdf',
    )

    robots_path = os.path.join(

        pkg_turtlebot3_gazebo,

        'params',

        'multi_robot.yaml',
    )

    if not os.path.isfile(
        model_path
    ):

        raise RuntimeError(
            f'SDF not found: '
            f'{model_path}'
        )

    if not os.path.isfile(
        urdf_path
    ):

        raise RuntimeError(
            f'URDF not found: '
            f'{urdf_path}'
        )

    # ========================================================
    # LaunchConfiguration
    # ========================================================

    use_sim_time = (
        LaunchConfiguration(
            'use_sim_time'
        )
    )

    gui = (
        LaunchConfiguration(
            'gui'
        )
    )

    world = (
        LaunchConfiguration(
            'world'
        )
    )

    robots_file = (
        LaunchConfiguration(
            'robots_file'
        )
    )

    # ========================================================
    # Launch
    # ========================================================

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
        ),

        DeclareLaunchArgument(
            'gui',
            default_value='True',
        ),

        DeclareLaunchArgument(
            'world',
            default_value=
                world_path,
        ),

        DeclareLaunchArgument(
            'robots_file',
            default_value=
                robots_path,
        ),

        # ----------------------------------------------------
        # Gazebo resources
        # ----------------------------------------------------

        AppendEnvironmentVariable(

            'GZ_SIM_RESOURCE_PATH',

            os.path.join(
                pkg_turtlebot3_gazebo,
                'models',
            ),
        ),

        AppendEnvironmentVariable(

            'GZ_SIM_RESOURCE_PATH',

            '/opt/ros/jazzy/share',
        ),

        # ----------------------------------------------------
        # Gazebo Server
        # ----------------------------------------------------

        IncludeLaunchDescription(

            PythonLaunchDescriptionSource(

                os.path.join(

                    ros_gz_sim,

                    'launch',

                    'gz_sim.launch.py',
                )
            ),

            launch_arguments={

                'gz_args': [

                    '-r -s -v2 ',

                    world,
                ],

                'on_exit_shutdown':
                    'True',

            }.items(),
        ),

        # ----------------------------------------------------
        # Gazebo GUI
        # ----------------------------------------------------

        IncludeLaunchDescription(

            PythonLaunchDescriptionSource(

                os.path.join(

                    ros_gz_sim,

                    'launch',

                    'gz_sim.launch.py',
                )
            ),

            condition=
                IfCondition(gui),

            launch_arguments={

                'gz_args':
                    '-g -v2',

                'on_exit_shutdown':
                    'True',

            }.items(),
        ),

        # ----------------------------------------------------
        # YAML -> sequential robots
        # ----------------------------------------------------

        OpaqueFunction(

            function=
                _launch_robots,

            args=[

                model_path,

                urdf_path,

                robots_file,

                use_sim_time,
            ],
        ),
    ])
