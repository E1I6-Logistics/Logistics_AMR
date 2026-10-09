import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_dir = get_package_share_directory('turtlebot3_bringup')
    nav2_dir = get_package_share_directory('turtlebot3_navigation2')
    aruco_tools_dir = get_package_share_directory('logitle_aruco_tools')

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
    use_camera_default = 'false'
    camera_info_url_default = 'file://' + os.path.expanduser(
        '~/camera/imx219_320x240.yaml'
    )

    # ============================================================
    # Launch Configuration
    # ============================================================

    map_yaml_file = LaunchConfiguration('map')
    mask_yaml_file = LaunchConfiguration('mask')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_camera = LaunchConfiguration('use_camera')
    camera_info_url = LaunchConfiguration('camera_info_url')
    use_aruco = LaunchConfiguration('use_aruco')
    use_nav2 = LaunchConfiguration('use_nav2')
    use_logitle_pose = LaunchConfiguration('use_logitle_pose')
    use_docking = LaunchConfiguration('use_docking')
    docking_dry_run = LaunchConfiguration('docking_dry_run')

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

    declare_use_camera_cmd = DeclareLaunchArgument(
        'use_camera',
        default_value=use_camera_default,
        description='Launch camera_ros continuously. ArUco Action starts it on demand when false.'
    )

    declare_camera_info_url_cmd = DeclareLaunchArgument(
        'camera_info_url',
        default_value=camera_info_url_default,
        description='Camera calibration YAML URL used by the camera and ArUco Action.'
    )

    declare_use_aruco_cmd = DeclareLaunchArgument(
        'use_aruco',
        default_value='true',
        description='Launch ArUco alignment/correction action servers.'
    )

    declare_use_nav2_cmd = DeclareLaunchArgument(
        'use_nav2',
        default_value='true',
        description='Launch Nav2 navigation stack.'
    )

    declare_use_logitle_pose_cmd = DeclareLaunchArgument(
        'use_logitle_pose',
        default_value='true',
        description='Launch logitle pose publisher.'
    )

    declare_use_docking_cmd = DeclareLaunchArgument(
        'use_docking',
        default_value='true',
        description='Launch logitle docking action server.'
    )

    declare_docking_dry_run_cmd = DeclareLaunchArgument(
        'docking_dry_run',
        default_value='true',
        description='Verify Map staging/model poses without any robot motion.'
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

    camera_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bringup_dir,
                'launch',
                'camera.launch.py'
            )
        ),
        launch_arguments={
            'camera_info_url': camera_info_url,
        }.items(),
        condition=IfCondition(use_camera),
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
        }.items(),
        condition=IfCondition(use_nav2),
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
                'publish_rate_hz': 5.0,
                'topic_name': 'logitle_pose',
                'use_sim_time': use_sim_time,
            }
        ],
        condition=IfCondition(use_logitle_pose),
    )

    # ============================================================
    # 4. Logitle Docking
    # ============================================================

    docking_node = Node(
        package='logitle_docking',
        executable='precision_docking_server',
        name='logitle_docking_node',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'base_frame': 'base_footprint',
                'map_frame': 'map',
                'scan_frame': 'base_scan',
                'charger_width': 0.145,
                'wing_length': 0.10,
                'wing_angle_deg': 22.5,
                # 실측 V형 도크 형상과 Map 도킹 완료 위치
                'roi_y_limit': 0.20,
                'dock_pose_configured': last_char in initial_points,
                'dock_final_map_x': float(init_point[0]),
                'dock_final_map_y': float(init_point[1]),
                'dock_final_map_yaw': float(init_point[2]),
                'staging_offset_from_final': 0.35,
                'dock_model_rear_offset': 0.10,
                'recovery_max_retries': 2,
                'dry_run': ParameterValue(docking_dry_run, value_type=bool),
            }
        ],
        condition=IfCondition(use_docking),
    )

    # ============================================================
    # 5. ArUco Alignment / Pose Correction
    #
    # Action servers only.
    # Camera is started by the ArUco Action only when a goal is received.
    # ============================================================

    aruco_align_and_correct_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                aruco_tools_dir,
                'launch',
                'logitle_aruco_align_and_correct.launch.py'
            )
        ),
        launch_arguments={
            'camera_info_url': camera_info_url,
        }.items(),
        condition=IfCondition(use_aruco),
    )

    # ============================================================
    # Launch Description
    # ============================================================

    ld = LaunchDescription()

    # Arguments
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_camera_cmd)
    ld.add_action(declare_camera_info_url_cmd)
    ld.add_action(declare_use_aruco_cmd)
    ld.add_action(declare_use_nav2_cmd)
    ld.add_action(declare_use_logitle_pose_cmd)
    ld.add_action(declare_use_docking_cmd)
    ld.add_action(declare_docking_dry_run_cmd)
    ld.add_action(declare_mask_yaml_cmd)

    ld.add_action(declare_initial_pose_x_cmd)
    ld.add_action(declare_initial_pose_y_cmd)
    ld.add_action(declare_initial_pose_yaw_cmd)

    # Nodes / Launches
    ld.add_action(bringup_cmd)
    ld.add_action(camera_cmd)
    ld.add_action(nav2_cmd)
    ld.add_action(logitle_pose_node)
    ld.add_action(docking_node)
    ld.add_action(aruco_align_and_correct_cmd)

    return ld
