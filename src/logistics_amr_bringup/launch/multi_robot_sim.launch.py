#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('logistics_amr_bringup')
    pkg_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_navigation = get_package_share_directory('turtlebot3_navigation2')
    model_name = os.environ.get('TURTLEBOT3_MODEL', 'burger')

    robots_file = LaunchConfiguration('robots_file')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_composition = LaunchConfiguration('use_composition')
    use_simulator = LaunchConfiguration('use_simulator')
    gui = LaunchConfiguration('gui')
    use_rviz = LaunchConfiguration('use_rviz')
    use_dispatcher = LaunchConfiguration('use_dispatcher')
    selected_robot = LaunchConfiguration('selected_robot')
    rviz_config = LaunchConfiguration('rviz_config')
    topic_timeout = LaunchConfiguration('topic_timeout')
    nav2_timeout = LaunchConfiguration('nav2_timeout')

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_navigation,
                'launch',
                'multi_robot_navigation2.launch.py',
            )
        ),
        launch_arguments={
            'robots_file': robots_file,
            'map': map_file,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'use_composition': use_composition,
            'use_simulator': use_simulator,
            'gui': gui,
            'use_rviz': use_rviz,
            'rviz_config': rviz_config,
            'topic_timeout': topic_timeout,
            'nav2_timeout': nav2_timeout,
        }.items(),
    )

    dispatcher = Node(
        package='logistics_amr_goal_dispatcher',
        executable='multi_robot_goal_dispatcher',
        name='multi_robot_goal_dispatcher',
        condition=IfCondition(use_dispatcher),
        parameters=[{
            'robots_file': robots_file,
            'selected_robot': selected_robot,
            'goal_frame': 'map',
        }],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'robots_file',
            default_value=os.path.join(
                pkg_gazebo,
                'params',
                'multi_robot.yaml',
            ),
        ),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(
                pkg_navigation,
                'map',
                'amr_workcell.yaml',
            ),
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                pkg_navigation,
                'param',
                f'{model_name}.yaml',
            ),
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        # Nav2 Jazzy evaluates this value inside a PythonExpression, so keep
        # the Python boolean spelling rather than the usual ROS lowercase form.
        DeclareLaunchArgument('use_composition', default_value='True'),
        DeclareLaunchArgument('use_simulator', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_dispatcher', default_value='true'),
        DeclareLaunchArgument('selected_robot', default_value=''),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(
                pkg_bringup,
                'rviz',
                'multi_robot_control.rviz',
            ),
        ),
        DeclareLaunchArgument('topic_timeout', default_value='60.0'),
        DeclareLaunchArgument('nav2_timeout', default_value='90.0'),
        navigation,
        dispatcher,
    ])
