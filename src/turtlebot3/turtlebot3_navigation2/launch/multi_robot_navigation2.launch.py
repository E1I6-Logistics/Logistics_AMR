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


def _load_robots(config_path):
    with open(config_path, 'r', encoding='utf-8') as config_file:
        config = yaml.safe_load(config_file) or {}

    robots = config.get('robots', [])
    if not robots:
        raise RuntimeError(f'No robots configured in {config_path}')

    names = [robot.get('name') for robot in robots]
    if any(not name for name in names):
        raise RuntimeError('Every robot must have a name')
    if len(names) != len(set(names)):
        raise RuntimeError('Robot names must be unique')

    required_fields = {'x', 'y', 'z', 'yaw'}
    for robot in robots:
        missing = required_fields - robot.keys()
        if missing:
            raise RuntimeError(
                f"Robot {robot['name']} is missing fields: {sorted(missing)}"
            )

    return robots


def _set_parameter(config, node_name, parameter_name, value, nested_name=None):
    node_config = config[node_name]
    if nested_name is not None:
        node_config = node_config[nested_name]
    node_config['ros__parameters'][parameter_name] = value


def _write_robot_params(source_path, robot):
    with open(source_path, 'r', encoding='utf-8') as source_file:
        config = yaml.safe_load(source_file) or {}

    robot_name = robot['name']
    odom_frame = f'{robot_name}/odom'
    base_footprint_frame = f'{robot_name}/base_footprint'
    base_link_frame = f'{robot_name}/base_link'

    _set_parameter(config, 'amcl', 'base_frame_id', base_footprint_frame)
    _set_parameter(config, 'amcl', 'odom_frame_id', odom_frame)
    _set_parameter(config, 'amcl', 'set_initial_pose', True)
    _set_parameter(config, 'amcl', 'always_reset_initial_pose', True)
    amcl_params = config['amcl']['ros__parameters']
    amcl_params['initial_pose'] = {
        'x': float(robot['x']),
        'y': float(robot['y']),
        'z': float(robot['z']),
        'yaw': float(robot['yaw']),
    }

    _set_parameter(config, 'bt_navigator', 'robot_base_frame', base_link_frame)
    _set_parameter(
        config,
        'local_costmap',
        'global_frame',
        odom_frame,
        nested_name='local_costmap',
    )
    _set_parameter(
        config,
        'local_costmap',
        'robot_base_frame',
        base_link_frame,
        nested_name='local_costmap',
    )
    _set_parameter(
        config,
        'global_costmap',
        'robot_base_frame',
        base_link_frame,
        nested_name='global_costmap',
    )
    _set_parameter(config, 'behavior_server', 'local_frame', odom_frame)
    _set_parameter(config, 'behavior_server', 'robot_base_frame', base_link_frame)
    _set_parameter(
        config,
        'collision_monitor',
        'base_frame_id',
        base_footprint_frame,
    )
    _set_parameter(config, 'collision_monitor', 'odom_frame_id', odom_frame)

    params_file = tempfile.NamedTemporaryFile(
        mode='w',
        prefix=f'nav2_{robot_name}_',
        suffix='.yaml',
        delete=False,
    )
    yaml.safe_dump(config, params_file, sort_keys=False)
    params_file.close()
    return params_file.name


def _cleanup_temp_files(_context, temp_paths):
    for temp_path in temp_paths:
        try:
            os.unlink(temp_path)
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


def _make_checker(robot_name, mode, timeout):
    return Node(
        package='turtlebot3_navigation2',
        executable='readiness_checker.py',
        name=f'{robot_name}_{mode}_readiness_checker',
        arguments=[
            '--mode', mode,
            '--robot-name', robot_name,
            '--timeout', timeout,
        ],
        output='screen',
    )


def _launch_setup(context):
    pkg_navigation = get_package_share_directory('turtlebot3_navigation2')
    pkg_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    robots_file = LaunchConfiguration('robots_file')
    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_composition = LaunchConfiguration('use_composition')
    use_simulator = LaunchConfiguration('use_simulator')
    gui = LaunchConfiguration('gui')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    topic_timeout = LaunchConfiguration('topic_timeout')
    nav2_timeout = LaunchConfiguration('nav2_timeout')

    robots_path = context.perform_substitution(robots_file)
    params_path = context.perform_substitution(params_file)
    robots = _load_robots(robots_path)
    robot_names = [robot['name'] for robot in robots]

    temp_paths = [
        _write_robot_params(params_path, robot)
        for robot in robots
    ]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'multi_robot_workcell.launch.py')
        ),
        condition=IfCondition(use_simulator),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'gui': gui,
            'robots_file': robots_file,
        }.items(),
    )

    tf_aggregator = Node(
        package='turtlebot3_navigation2',
        executable='multi_robot_tf_aggregator.py',
        name='multi_robot_tf_aggregator',
        parameters=[{'robot_names': robot_names}],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='multi_robot_rviz2',
        condition=IfCondition(use_rviz),
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    topic_checkers = [
        _make_checker(robot_name, 'topics', topic_timeout)
        for robot_name in robot_names
    ]
    nav_checkers = [
        _make_checker(robot_name, 'nav2', nav2_timeout)
        for robot_name in robot_names
    ]

    nav_groups = []
    for robot, robot_params, nav_checker in zip(
        robots, temp_paths, nav_checkers
    ):
        nav_groups.append(
            GroupAction([
                LogInfo(msg=f"Starting Nav2 for {robot['name']}"),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            pkg_nav2_bringup,
                            'launch',
                            'bringup_launch.py',
                        )
                    ),
                    launch_arguments={
                        'namespace': robot['name'],
                        'use_namespace': 'True',
                        'map': map_file,
                        'use_sim_time': use_sim_time,
                        'params_file': robot_params,
                        'autostart': 'True',
                        'use_composition': use_composition,
                        'use_respawn': 'False',
                    }.items(),
                ),
                nav_checker,
            ])
        )

    handlers = []
    for index, checker in enumerate(topic_checkers):
        next_action = (
            [topic_checkers[index + 1]]
            if index + 1 < len(topic_checkers)
            else [nav_groups[0]]
        )
        handlers.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=checker,
                    on_exit=lambda event, ctx, index=index, actions=next_action: (
                        _continue_or_shutdown(
                            event,
                            ctx,
                            f'{robot_names[index]} topics',
                            actions,
                        )
                    ),
                )
            )
        )

    for index, checker in enumerate(nav_checkers):
        next_action = (
            [nav_groups[index + 1]]
            if index + 1 < len(nav_groups)
            else [rviz]
        )
        handlers.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=checker,
                    on_exit=lambda event, ctx, index=index, actions=next_action: (
                        _continue_or_shutdown(
                            event,
                            ctx,
                            f'{robot_names[index]} Nav2',
                            actions,
                        )
                    ),
                )
            )
        )

    cleanup_handler = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                OpaqueFunction(
                    function=_cleanup_temp_files,
                    args=[temp_paths],
                )
            ]
        )
    )

    return [
        gazebo,
        tf_aggregator,
        cleanup_handler,
        *handlers,
        topic_checkers[0],
    ]


def generate_launch_description():
    pkg_navigation = get_package_share_directory('turtlebot3_navigation2')
    pkg_gazebo = get_package_share_directory('turtlebot3_gazebo')
    model_name = os.environ.get('TURTLEBOT3_MODEL', 'burger')

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
            default_value=os.path.join(pkg_navigation, 'map', 'amr_workcell.yaml'),
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                pkg_navigation,
                'param',
                f'{model_name}.yaml',
            ),
        ),
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        DeclareLaunchArgument('use_composition', default_value='True'),
        DeclareLaunchArgument('use_simulator', default_value='True'),
        DeclareLaunchArgument('gui', default_value='True'),
        DeclareLaunchArgument('use_rviz', default_value='True'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(
                pkg_navigation,
                'rviz',
                'multi_robot_navigation.rviz',
            ),
        ),
        DeclareLaunchArgument('topic_timeout', default_value='60.0'),
        DeclareLaunchArgument('nav2_timeout', default_value='90.0'),
        OpaqueFunction(function=_launch_setup),
    ])
