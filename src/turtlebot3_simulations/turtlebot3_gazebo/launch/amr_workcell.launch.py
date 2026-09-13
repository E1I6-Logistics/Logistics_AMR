"""Launch the local warehouse with Gazebo Sim (ROS 2 Jazzy / Harmonic)."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


ROOT = Path(get_package_share_directory('turtlebot3_gazebo'))


def launch_sim(context):
    world = LaunchConfiguration('world').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(Path(get_package_share_directory('ros_gz_sim')) / 'launch/gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': f'-r {"-s " if headless else ""}-v 2 "{world}"',
                'on_exit_shutdown': 'true',
            }.items(),
        )
    ]

    if LaunchConfiguration('spawn_robot').perform(context).lower() == 'true':
        launch_dir = Path(get_package_share_directory('turtlebot3_gazebo')) / 'launch'
        for filename, args in [
            ('robot_state_publisher.launch.py', {'use_sim_time': LaunchConfiguration('use_sim_time')}),
            (
                'spawn_turtlebot3.launch.py',
                {
                    'x_pose': LaunchConfiguration('x_pose'),
                    'y_pose': LaunchConfiguration('y_pose'),
                },
            ),
        ]:
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(str(launch_dir / filename)),
                    launch_arguments=args.items(),
                )
            )
    return actions


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable(
            'TURTLEBOT3_MODEL',
            EnvironmentVariable('TURTLEBOT3_MODEL', default_value='waffle'),
        ),
        DeclareLaunchArgument(
            'gz_partition',
            default_value='amr_workcell_v3',
            description='Isolates this scene from previously running Gazebo worlds',
        ),
        DeclareLaunchArgument('world', default_value=str(ROOT / 'worlds/amr_workcell.sdf')),
        SetEnvironmentVariable('GZ_PARTITION', LaunchConfiguration('gz_partition')),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument(
            'spawn_robot',
            default_value='true',
            description='Requires Gazebo Sim compatible turtlebot3_gazebo',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.5'),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', str(ROOT / 'models')),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen',
            condition=UnlessCondition(LaunchConfiguration('spawn_robot')),
        ),
        OpaqueFunction(function=launch_sim),
    ])
