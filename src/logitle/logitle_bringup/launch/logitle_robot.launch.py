import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('turtlebot3_bringup')
    nav2_dir = get_package_share_directory('turtlebot3_navigation2')

    # ============================================================
    # Robot별 초기 위치
    # 계정명 마지막 숫자 기준
    # ============================================================

    username = os.environ.get('USER', 'default')

    initial_points = {
        '1': ['-0.2641', '0.3367', '0.1327'],
        '2': ['-0.2653', '-0.0186', '0.1327'],
        '3': ['-0.2653', '-0.4242', '0.1327'],
    }

    last_char = username[-1] if username else ''
    init_point = initial_points.get(
        last_char,
        ['0.0', '0.0', '0.0']
    )

    # ============================================================
    # Launch Configuration
    # ============================================================

    map_yaml_file = LaunchConfiguration('map')
    mask_yaml_file = LaunchConfiguration('mask')
    use_sim_time = LaunchConfiguration('use_sim_time')

    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')

    # ============================================================
    # Launch Arguments
    # ============================================================

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            nav2_dir,
            'map',
            'logitle_map_fin.yaml'
        ),
        description='Full path to logitle_map_fin yaml file to load'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    declare_mask_yaml_cmd = DeclareLaunchArgument(
        'mask',
        default_value=os.path.join(
            nav2_dir,
            'map',
            'logitle_map_fin_keepout.yaml'
        ),
        description='Full path to logitle_map_fin_keepout yaml file to load'
    )

    declare_initial_pose_x_cmd = DeclareLaunchArgument(
        'initial_pose_x',
        default_value=init_point[0],
        description='Initial pose X position'
    )

    declare_initial_pose_y_cmd = DeclareLaunchArgument(
        'initial_pose_y',
        default_value=init_point[1],
        description='Initial pose Y position'
    )

    declare_initial_pose_yaw_cmd = DeclareLaunchArgument(
        'initial_pose_yaw',
        default_value=init_point[2],
        description='Initial pose Yaw angle'
    )

    # ============================================================
    # 1. TurtleBot Bringup
    #
    # Base driver
    # LiDAR
    # Robot State Publisher
    # EKF
    # ============================================================

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bringup_dir,
                'launch',
                'robot.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # ============================================================
    # 2. Nav2
    #
    # Map Server
    # AMCL
    # Planner
    # Controller
    # Waypoint Follower
    # ============================================================

    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_dir,
                'launch',
                'navigation2_robot.launch.py'
            )
        ),
        launch_arguments={
            'map': map_yaml_file,
            'mask': mask_yaml_file,
            'use_sim_time': use_sim_time,
            'initial_pose_x': initial_pose_x,
            'initial_pose_y': initial_pose_y,
            'initial_pose_yaw': initial_pose_yaw,
        }.items()
    )

    # ============================================================
    # 3. Logitle Pose Publisher
    #
    # TF:
    #   map -> base_footprint
    #
    # Output:
    #   /logitle_pose
    #
    # Type:
    #   geometry_msgs/msg/PoseStamped
    #
    # Publish Rate:
    #   20 Hz
    # ============================================================

    logitle_pose_node = Node(
        package='logitle_pose',
        executable='logitle_pose_publisher',
        name='logitle_pose_publisher',
        output='screen',
        parameters=[
            {
                'target_frame': 'map',
                'source_frame': 'base_footprint',
                'publish_rate_hz': 20.0,
                'topic_name': 'logitle_pose',
                'use_sim_time': use_sim_time,
            }
        ]
    )

    # ============================================================
    # 4. Logitle Docking
    # ============================================================

    docking_node = Node(
        package='logitle_docking',
        executable='precision_docking_ICP_align_server_V2',
        name='logitle_docking_node',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'pose_topic': 'logitle_pose',
                'use_logitle_pose_topic': True,
                'global_frame': 'map',
                'base_frame': 'base_footprint',
            }
        ]
    )

    # ============================================================
    # Launch Description
    # ============================================================

    ld = LaunchDescription()

    # Arguments
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_mask_yaml_cmd)

    ld.add_action(declare_initial_pose_x_cmd)
    ld.add_action(declare_initial_pose_y_cmd)
    ld.add_action(declare_initial_pose_yaw_cmd)

    # Nodes / Launches
    ld.add_action(bringup_cmd)
    ld.add_action(nav2_cmd)
    ld.add_action(logitle_pose_node)
    ld.add_action(docking_node)

    return ld