import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node  # <- 1. Node 임포트 추가


def generate_launch_description():
    bringup_dir = get_package_share_directory('turtlebot3_bringup')
    nav2_dir = get_package_share_directory('turtlebot3_navigation2')

    # 계정명 기반 초기 위치 설정
    username = os.environ.get('USER', 'default')
    initial_points = {
        '1': ['-0.2641', '0.3367', '0.1327'],
        '2': ['-0.2653', '-0.0186', '0.1327'],
        '3': ['-0.2653', '-0.4242', '0.1327'],
    }
    last_char = username[-1] if username else ''
    init_point = initial_points.get(last_char, ['0.0', '0.0', '0.0'])

    map_yaml_file = LaunchConfiguration('map')
    mask_yaml_file = LaunchConfiguration('mask')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(nav2_dir, 'map', 'logitle_map_fin.yaml'),
        description='Full path to logitle_map_fin yaml file to load'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    declare_mask_yaml_cmd = DeclareLaunchArgument(
        'mask',
        default_value=os.path.join(nav2_dir, 'map', 'logitle_map_fin_keepout.yaml'),
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

    # 1. Bringup
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'robot.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 2. Nav2
    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'navigation2_robot.launch.py')
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

    # 3. logitle_docking 노드 직접 실행 (ros2 run logitle_docking <실행파일명>)
    # executable 인자에 평소 ros2 run으로 실행하던 노드 바이너리/스크립트 이름을 적어주세요.
    docking_node = Node(
        package='logitle_docking',
        executable='precision_docking_ICP_align_server_V2',  # 예: 'docking_server' 또는 'docking_node'
        name='logitle_docking_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    ld = LaunchDescription()

    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_mask_yaml_cmd)
    ld.add_action(declare_initial_pose_x_cmd)
    ld.add_action(declare_initial_pose_y_cmd)
    ld.add_action(declare_initial_pose_yaw_cmd)

    ld.add_action(bringup_cmd)
    ld.add_action(nav2_cmd)
    ld.add_action(docking_node)  # <- 노드 등록

    return ld