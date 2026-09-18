import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'burger')


def generate_launch_description():

    # ============================================================
    # Default paths
    # ============================================================
    map_default = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'),
        'map',
        'amr_workcell.yaml'
    )

    graph_default = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'),
        'graphs',
        'amr_workcell_graph.geojson'
    )

    params_default = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'),
        'param',
        TURTLEBOT3_MODEL + '_route.yaml'
    )

    rviz_config = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'),
        'rviz',
        'tb3_navigation2.rviz'
    )

    nav2_bringup_launch = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'bringup_launch.py'
    )

    behavior_tree = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'),
        'behavior_trees',
        'navigate_on_route_graph_w_recovery.xml'
    )

    # ============================================================
    # Launch configurations
    # ============================================================

    map_file = LaunchConfiguration('map')
    graph_file = LaunchConfiguration('graph')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            'graph_filepath': graph_file,
            'default_nav_to_pose_bt_xml': behavior_tree,
            'yaml_filename': map_file,
        },
        convert_types=True,
    )

    # ============================================================
    # Nav2 Bringup
    # ============================================================

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_bringup_launch),
        launch_arguments={
            'map': map_file,
            'params_file': configured_params,
            'use_sim_time': use_sim_time,
            'autostart': 'True',
            'slam': 'False',
            'use_localization': 'True',
        }.items()
    )

    # ============================================================
    # RViz
    # ============================================================

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(use_rviz),
        output='screen'
    )

    # ============================================================
    # Launch Description
    # ============================================================

    return LaunchDescription([

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
            'params_file',
            default_value=params_default,
            description='Full path to Nav2 parameter file'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock'
        ),

        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz'
        ),

        nav2_bringup,

        rviz,
    ])
