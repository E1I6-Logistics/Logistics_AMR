import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_dir = get_package_share_directory('turtlebot3_bringup')
    nav2_dir = get_package_share_directory('turtlebot3_navigation2')
    docking_dir = get_package_share_directory('logitle_docking')

    # 1. 시스템 계정명 가져오기 (예: 'robot1' -> username = 'robot1')
    username = os.environ.get('USER', 'default')

    # 2. 번호별 초기 위치 정의 [x, y, yaw] (default_value용 문자열 형태)
    # 실제 환경 좌표에 맞게 수정하세요.
    initial_points = {
        '1': ['0.0', '0.0', '0.0'],
        '2': ['1.0', '0.5', '1.57'],
        '3': ['2.0', '-0.5', '-1.57'],
    }

    # 계정명의 마지막 한 글자 추출
    last_char = username[-1] if username else ''
    
    # 딕셔너리 매핑 (1, 2, 3이 아니면 기본 0.0 처리)
    init_point = initial_points.get(last_char, ['0.0', '0.0', '0.0'])

    # 3. LaunchConfiguration 정의
    map_yaml_file = LaunchConfiguration('map')
    mask_yaml_file = LaunchConfiguration('mask')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')

    # 4. Launch 인자 선언 (선택된 init_point가 default_value로 들어감)
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(nav2_dir, 'map', 'logitle_map_fin.yaml'),
        description='Full path to logitle_map_fin yaml file to load'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_mask_yaml_cmd = DeclareLaunchArgument(
        'mask',
        default_value=os.path.join(nav2_dir, 'map', 'logitle_map_fin_keepout.yaml'),
        description='Full path to logitle_map_fin_keepout yaml file to load'
    )

    declare_initial_pose_x_cmd = DeclareLaunchArgument(
        'initial_pose_x',
        default_value=init_point[0],
        description='Initial pose X position in map frame'
    )

    declare_initial_pose_y_cmd = DeclareLaunchArgument(
        'initial_pose_y',
        default_value=init_point[1],
        description='Initial pose Y position in map frame'
    )

    declare_initial_pose_yaw_cmd = DeclareLaunchArgument(
        'initial_pose_yaw',
        default_value=init_point[2],
        description='Initial pose Yaw angle (radians) in map frame'
    )

    # 5. 하위 Launch 포함
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'robot.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'navigation2.launch.py')
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

    docking_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(docking_dir, 'launch', 'docking.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 6. LaunchDescription 생성 및 반환
    ld = LaunchDescription()

    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_mask_yaml_cmd)
    ld.add_action(declare_initial_pose_x_cmd)
    ld.add_action(declare_initial_pose_y_cmd)
    ld.add_action(declare_initial_pose_yaw_cmd)

    ld.add_action(bringup_cmd)
    ld.add_action(nav2_cmd)
    ld.add_action(docking_cmd)

    return ld