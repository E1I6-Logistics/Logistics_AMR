#!/usr/bin/env python3

from launch import LaunchDescription

from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
)

from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)

from launch_ros.actions import Node

from launch_ros.substitutions import (
    FindPackageShare,
)


def generate_launch_description():

    robot_id = LaunchConfiguration(
        'robot_id'
    )

    router_ip = LaunchConfiguration(
        'router_ip'
    )

    # 환경변수에서 가져옴
    ros_domain_id = EnvironmentVariable(
        'ROS_DOMAIN_ID',
        default_value='0',
    )

    discovery_range = EnvironmentVariable(
        'ROS_AUTOMATIC_DISCOVERY_RANGE',
        default_value='LOCALHOST',
    )

    # robot3 → /robot3
    bridge_namespace = PythonExpression([
        "'/' + '",
        robot_id,
        "'.lstrip('/')",
    ])

    # 포트는 7447 고정
    router_endpoint = PythonExpression([
        "'tcp/' + '",
        router_ip,
        "' + ':7447'",
    ])

    config_file = PathJoinSubstitution([
        FindPackageShare(
            'zenoh_pkg'
        ),
        'config',
        'bridge.json5',
    ])

    return LaunchDescription([

        DeclareLaunchArgument(
            'robot_id',
            default_value='unknown_robot',
            description=(
                'Robot ID: robot1, robot2, robot3'
            ),
        ),

        DeclareLaunchArgument(
            'router_ip',
            default_value='10.10.141.15',
            description='Zenoh Router IP',
        ),

        # ----------------------------------------------
        # 빈 ROS 2 노드
        # ----------------------------------------------
        Node(
            package='zenoh_pkg',
            executable='zenoh_bridge_node',
            name='zenoh_bridge_node',
            output='screen',
            parameters=[
                {
                    'robot_id': robot_id,
                }
            ],
        ),

        # ----------------------------------------------
        # Zenoh ROS2DDS Bridge
        #
        # 기존 zenoh_start 함수와 동일한 옵션에
        # bridge.json5 설정만 추가
        # ----------------------------------------------
        ExecuteProcess(
            cmd=[
                'zenoh-bridge-ros2dds',

                '-c',
                config_file,

                '-d',
                ros_domain_id,

                '-n',
                bridge_namespace,

                '--ros-automatic-discovery-range',
                discovery_range,

                '-e',
                router_endpoint,

                'client',
            ],
            output='screen',
        ),
    ])