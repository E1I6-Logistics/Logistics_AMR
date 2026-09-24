# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'burger')
ROS_DISTRO = os.environ.get('ROS_DISTRO', 'jazzy')


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    autostart = LaunchConfiguration('autostart', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='false')

    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('turtlebot3_navigation2'),
            'map',
            'map.yaml'))

    # [핵심 1] mask 기본값을 빈 문자열('')로 지정
    mask_yaml_file = LaunchConfiguration('mask', default='')

    param_file_name = TURTLEBOT3_MODEL + '.yaml'
    if ROS_DISTRO == 'humble':
        param_dir = LaunchConfiguration(
            'params_file',
            default=os.path.join(
                get_package_share_directory('turtlebot3_navigation2'),
                'param',
                ROS_DISTRO,
                param_file_name))
    else:
        param_dir = LaunchConfiguration(
            'params_file',
            default=os.path.join(
                get_package_share_directory('turtlebot3_navigation2'),
                'param',
                param_file_name))

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    # [핵심 2] mask 인자가 비어있지 않을 때만 True가 되는 조건식
    has_mask = IfCondition(PythonExpression(["'", mask_yaml_file, "' != ''"]))

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'mask',
            default_value='',
            description='Full path to filter mask yaml file (Optional: leave empty to disable)'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the nav2 stack'),

        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Whether to start RViz'),

        # 1. Nav2 Core Bringup (항상 실행)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir,
                'autostart': autostart,
                'use_rviz': use_rviz}.items(),
        ),

        # 2. Keepout Filter Mask Server (mask 인자가 있을 때만 실행)
        Node(
            condition=has_mask,
            package='nav2_map_server',
            executable='map_server',
            name='filter_mask_server',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'use_sim_time': use_sim_time,
                'yaml_filename': mask_yaml_file
            }],
            remappings=[('/map', '/keepout_filter_mask')]
        ),

        # 3. Costmap Filter Info Server (mask 인자가 있을 때만 실행)
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
                'multiplier': 1.0
            }]
        ),

        # 4. Filter Lifecycle Manager (mask 인자가 있을 때만 실행)
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
                'node_names': ['filter_mask_server', 'costmap_filter_info_server']
            }]
        ),
    ])