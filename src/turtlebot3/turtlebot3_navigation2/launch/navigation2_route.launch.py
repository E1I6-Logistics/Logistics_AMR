#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import Node

from nav2_common.launch import RewrittenYaml


TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'burger')
ROS_DISTRO = os.environ.get('ROS_DISTRO', 'jazzy')


def generate_launch_description():

    pkg_share = get_package_share_directory('turtlebot3_navigation2')

    # ============================================================
    # Launch Configurations
    # ============================================================

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')

    map_file = LaunchConfiguration('map')
    mask_file = LaunchConfiguration('mask')
    graph_file = LaunchConfiguration('graph')
    params_file = LaunchConfiguration('params_file')

    # ============================================================
    # Default Paths
    # ============================================================

    map_default = os.path.join(
        pkg_share,
        'map',
        'logitle_map_fin.yaml'
    )

    graph_default = os.path.join(
        pkg_share,
        'graphs',
        'logitle_route.geojson'
    )

    if ROS_DISTRO == 'humble':
        params_default = os.path.join(
            pkg_share,
            'param',
            ROS_DISTRO,
            TURTLEBOT3_MODEL + '_route.yaml'
        )
    else:
        params_default = os.path.join(
            pkg_share,
            'param',
            TURTLEBOT3_MODEL + '_route.yaml'
        )

    rviz_config = os.path.join(
        pkg_share,
        'rviz',
        'tb3_navigation2.rviz'
    )

    nav2_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch'
    )

    # ============================================================
    # Route Graph Path Injection
    #
    # burger_route.yaml:
    #
    # route_server:
    #   ros__parameters:
    #     graph_filepath: ""
    #
    # 여기에 graph:=... 값을 넣음
    # ============================================================

    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            'graph_filepath': graph_file,
        },
        convert_types=True,
    )

    # ============================================================
    # Keepout Filter Condition
    # mask가 비어있지 않을 때만 Keepout 관련 노드 실행
    # ============================================================

    has_mask = IfCondition(
        PythonExpression([
            "'", mask_file, "' != ''"
        ])
    )

    # ============================================================
    # Launch
    # ============================================================

    return LaunchDescription([

        # --------------------------------------------------------
        # Arguments
        # --------------------------------------------------------

        DeclareLaunchArgument(
            'map',
            default_value=map_default,
            description='Full path to occupancy map yaml'
        ),

        DeclareLaunchArgument(
            'mask',
            default_value='',
            description='Full path to keepout mask yaml (optional)'
        ),

        DeclareLaunchArgument(
            'graph',
            default_value=graph_default,
            description='Full path to Route Server GeoJSON graph'
        ),

        DeclareLaunchArgument(
            'params_file',
            default_value=params_default,
            description='Full path to Nav2 route parameter yaml'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'
        ),

        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically activate Nav2 lifecycle nodes'
        ),

        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz'
        ),

        # --------------------------------------------------------
        # Nav2 Bringup
        #
        # Jazzy의 navigation_launch.py 안에 이미
        # route_server가 포함되어 있음.
        #
        # 따라서 별도의 route_server Node를 만들지 않음.
        # --------------------------------------------------------

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    nav2_launch_dir,
                    'bringup_launch.py'
                )
            ),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': configured_params,
                'autostart': autostart,
            }.items(),
        ),

        # --------------------------------------------------------
        # Keepout Mask Server
        # --------------------------------------------------------

        Node(
            condition=has_mask,
            package='nav2_map_server',
            executable='map_server',
            name='filter_mask_server',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'use_sim_time': use_sim_time,
                'yaml_filename': mask_file,
            }],
            remappings=[
                ('/map', '/keepout_filter_mask')
            ],
        ),

        # --------------------------------------------------------
        # Costmap Filter Info Server
        # --------------------------------------------------------

        Node(
            condition=has_mask,
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'use_sim_time': use_sim_time,
                'type': 0,
                'filter_info_topic': '/costmap_filter_info',
                'mask_topic': '/keepout_filter_mask',
                'base': 0.0,
                'multiplier': 1.0,
            }],
        ),

        # --------------------------------------------------------
        # Keepout Lifecycle Manager
        # --------------------------------------------------------

        Node(
            condition=has_mask,
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_costmap_filters',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': [
                    'filter_mask_server',
                    'costmap_filter_info_server',
                ],
            }],
        ),

        # --------------------------------------------------------
        # RViz
        # --------------------------------------------------------

        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=[
                '-d',
                rviz_config
            ],
            parameters=[{
                'use_sim_time': use_sim_time
            }],
            output='screen',
        ),
    ])