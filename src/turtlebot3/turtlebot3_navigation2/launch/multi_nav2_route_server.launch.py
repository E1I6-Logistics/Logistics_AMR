import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nav2_for_robot(namespace, params_file, map_file, graph_file, use_sim_time):
    nav2_bringup_launch = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'),
        'launch',
        'bringup_launch.py'
    )

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_bringup_launch),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': 'True',
            'map': map_file,
            'graph': graph_file,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'autostart': 'True',
            'slam': 'False',
            'use_localization': 'True',
            'use_composition': 'False',
        }.items()
    )


def generate_launch_description():
    pkg_nav = get_package_share_directory('turtlebot3_navigation2')

    map_file = LaunchConfiguration('map')
    graph_file = LaunchConfiguration('graph')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_namespace = LaunchConfiguration('rviz_namespace')

    map_default = os.path.join(pkg_nav, 'map', 'warehouse_map.yaml')
    graph_default = os.path.join(
        os.path.expanduser('~'),
        'turtlebot3_E1i6',
        'test2.geojson'
    )

    params_tb3_0 = os.path.join(pkg_nav, 'param', 'burger_route_tb3_0.yaml')
    params_tb3_1 = os.path.join(pkg_nav, 'param', 'burger_route_tb3_1.yaml')
    params_tb3_2 = os.path.join(pkg_nav, 'param', 'burger_route_tb3_2.yaml')

    rviz_config = os.path.join(
        pkg_nav,
        'rviz',
        'tb3_navigation2.rviz'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_multi',
        namespace=rviz_namespace,
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        # The saved single-robot RViz config uses absolute topic names.
        remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')] + [
            ('/' + topic, topic) for topic in (
                'initialpose', 'goal_pose', 'map', 'map_updates',
                'scan', 'robot_description', 'particle_cloud', 'plan',
                'local_plan', 'global_costmap/costmap',
                'global_costmap/costmap_updates',
                'global_costmap/published_footprint',
                'global_costmap/voxel_marked_cloud',
                'local_costmap/costmap', 'local_costmap/costmap_updates',
                'local_costmap/published_footprint',
                'local_costmap/voxel_marked_cloud',
            )
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_namespace',
            default_value='tb3_0',
            choices=['tb3_0', 'tb3_1', 'tb3_2'],
            description='Robot controlled by the RViz navigation panel'
        ),
        DeclareLaunchArgument(
            'map',
            default_value=map_default,
            description='Full path to occupancy map yaml'
        ),
        DeclareLaunchArgument(
            'graph',
            default_value=graph_default,
            description='Full path to Route Server GeoJSON graph'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock'
        ),

        nav2_for_robot('tb3_0', params_tb3_0, map_file, graph_file, use_sim_time),
        nav2_for_robot('tb3_1', params_tb3_1, map_file, graph_file, use_sim_time),
        nav2_for_robot('tb3_2', params_tb3_2, map_file, graph_file, use_sim_time),

        rviz,
    ])
