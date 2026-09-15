#!/usr/bin/env python3

import os
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_mapping_robot(config_path):
    with open(config_path, 'r', encoding='utf-8') as config_file:
        config = yaml.safe_load(config_file) or {}

    robots = config.get('robots', [])
    if len(robots) != 1:
        raise RuntimeError(
            'Mapping requires exactly one robot in the robots_file; '
            f'found {len(robots)} in {config_path}'
        )

    robot = robots[0]
    if not robot.get('name'):
        raise RuntimeError('The mapping robot must have a name')
    return robot


def _as_bool(value):
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


def _write_runtime_files(slam_params_path, rviz_config_path, robot, use_sim_time):
    robot_name = robot['name']

    with open(slam_params_path, 'r', encoding='utf-8') as params_file:
        slam_config = yaml.safe_load(params_file) or {}

    params = slam_config['slam_toolbox']['ros__parameters']
    params['use_sim_time'] = use_sim_time
    params['map_frame'] = 'map'
    params['odom_frame'] = f'{robot_name}/odom'
    params['base_frame'] = f'{robot_name}/base_footprint'
    params['scan_topic'] = f'/{robot_name}/scan'

    runtime_params = tempfile.NamedTemporaryFile(
        mode='w',
        prefix=f'slam_toolbox_{robot_name}_',
        suffix='.yaml',
        delete=False,
    )
    yaml.safe_dump(slam_config, runtime_params, sort_keys=False)
    runtime_params.close()

    with open(rviz_config_path, 'r', encoding='utf-8') as rviz_file:
        rviz_config = rviz_file.read().replace('__ROBOT_NAME__', robot_name)

    runtime_rviz = tempfile.NamedTemporaryFile(
        mode='w',
        prefix=f'workcell_mapping_{robot_name}_',
        suffix='.rviz',
        delete=False,
    )
    runtime_rviz.write(rviz_config)
    runtime_rviz.close()

    return runtime_params.name, runtime_rviz.name


def _cleanup_runtime_files(_context, paths):
    for path in paths:
        try:
            os.unlink(path)
        except FileNotFoundError:
            pass
    return []


def _continue_or_shutdown(event, _context, label, next_actions):
    if event.returncode == 0:
        return [LogInfo(msg=f'{label}: ready')] + list(next_actions)

    return [
        LogInfo(msg=f'ERROR: {label}: readiness check failed; stopping launch'),
        EmitEvent(event=Shutdown(reason=f'{label} readiness check failed')),
    ]


def _launch_setup(context):
    pkg_bringup = get_package_share_directory('logistics_amr_bringup')
    pkg_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    robots_file = LaunchConfiguration('robots_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    rviz_config = LaunchConfiguration('rviz_config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_simulator = LaunchConfiguration('use_simulator')
    use_rviz = LaunchConfiguration('use_rviz')
    gui = LaunchConfiguration('gui')
    topic_timeout = LaunchConfiguration('topic_timeout')
    slam_timeout = LaunchConfiguration('slam_timeout')

    robot = _load_mapping_robot(context.perform_substitution(robots_file))
    robot_name = robot['name']
    runtime_params, runtime_rviz = _write_runtime_files(
        context.perform_substitution(slam_params_file),
        context.perform_substitution(rviz_config),
        robot,
        _as_bool(context.perform_substitution(use_sim_time)),
    )
    runtime_files = [runtime_params, runtime_rviz]

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'multi_robot_workcell.launch.py')
        ),
        condition=IfCondition(use_simulator),
        launch_arguments={
            'robots_file': robots_file,
            'use_sim_time': use_sim_time,
            'gui': gui,
        }.items(),
    )

    tf_aggregator = Node(
        package='turtlebot3_navigation2',
        executable='multi_robot_tf_aggregator.py',
        name='mapping_tf_aggregator',
        parameters=[{'robot_names': [robot_name]}],
        output='screen',
    )

    topics_checker = Node(
        package='turtlebot3_navigation2',
        executable='readiness_checker.py',
        name=f'{robot_name}_mapping_topics_checker',
        arguments=[
            '--mode', 'topics',
            '--robot-name', robot_name,
            '--timeout', topic_timeout,
        ],
        output='screen',
    )

    slam_checker = Node(
        package='logistics_amr_bringup',
        executable='slam_readiness_checker.py',
        name='slam_toolbox_readiness_checker',
        arguments=['--timeout', slam_timeout],
        output='screen',
    )

    slam_group = GroupAction([
        LogInfo(msg=f'Starting SLAM Toolbox for {robot_name}'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    pkg_slam_toolbox,
                    'launch',
                    'online_async_launch.py',
                )
            ),
            launch_arguments={
                'slam_params_file': runtime_params,
                'use_sim_time': use_sim_time,
                'autostart': 'true',
                'use_lifecycle_manager': 'false',
            }.items(),
        ),
        slam_checker,
    ])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='workcell_mapping_rviz2',
        condition=IfCondition(use_rviz),
        arguments=['-d', runtime_rviz],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    topics_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=topics_checker,
            on_exit=lambda event, ctx: _continue_or_shutdown(
                event,
                ctx,
                f'{robot_name} mapping topics',
                [slam_group],
            ),
        )
    )
    slam_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=slam_checker,
            on_exit=lambda event, ctx: _continue_or_shutdown(
                event,
                ctx,
                'SLAM Toolbox',
                [rviz],
            ),
        )
    )
    cleanup_handler = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                OpaqueFunction(
                    function=_cleanup_runtime_files,
                    args=[runtime_files],
                )
            ]
        )
    )

    return [
        simulator,
        tf_aggregator,
        topics_handler,
        slam_handler,
        cleanup_handler,
        topics_checker,
    ]


def generate_launch_description():
    pkg_bringup = get_package_share_directory('logistics_amr_bringup')
    pkg_gazebo = get_package_share_directory('turtlebot3_gazebo')

    return LaunchDescription([
        DeclareLaunchArgument(
            'robots_file',
            default_value=os.path.join(
                pkg_gazebo,
                'params',
                'mapping_robot.yaml',
            ),
            description='YAML containing exactly one robot for mapping',
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(
                pkg_bringup,
                'config',
                'slam_toolbox_mapping.yaml',
            ),
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(
                pkg_bringup,
                'rviz',
                'workcell_mapping.rviz',
            ),
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_simulator', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('topic_timeout', default_value='60.0'),
        DeclareLaunchArgument('slam_timeout', default_value='60.0'),
        OpaqueFunction(function=_launch_setup),
    ])
