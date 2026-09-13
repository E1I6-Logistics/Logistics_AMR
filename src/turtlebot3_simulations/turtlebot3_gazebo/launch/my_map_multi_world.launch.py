#!/usr/bin/env python3
import os
import math
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    launch_file_dir = os.path.join(pkg_turtlebot3_gazebo, 'launch')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # 월드 파일 지정
    world_file = os.path.join(
    pkg_turtlebot3_gazebo, 'worlds', 'amr_workcell.sdf')
    
    world = LaunchConfiguration('world', default=world_file)

    # 모델 SDF 파일 경로
    sdf_path_tb3_0 = os.path.join(
        pkg_turtlebot3_gazebo,
        'models',
        'turtlebot3_burger_tb3_0',
        'model.sdf'
    )
    sdf_path_tb3_1 = os.path.join(
        pkg_turtlebot3_gazebo,
        'models',
        'turtlebot3_burger_tb3_1',
        'model.sdf'
    )
    sdf_path_tb3_2 = os.path.join(
        pkg_turtlebot3_gazebo,
        'models',
        'turtlebot3_burger_tb3_2',
        'model.sdf'
    )

    # 1. Gazebo Sim 서버 & 클라이언트 실행
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v2 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v2 ', 'on_exit_shutdown': 'true'}.items()
    )

    # 2. 전역 시뮬레이션 클록 브리지 (모든 Nav2 노드 시간 동기화용)
    global_clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # ==========================================
    # [로봇 1: tb3_0] (좌표: x=2.50, y=-1.27, yaw=pi)
    # ==========================================
    robot1_group = GroupAction([
        PushRosNamespace('tb3_0'),

        # Gazebo 스폰
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'tb3_0',
                '-file', sdf_path_tb3_0,
                '-x', '2.50',
                '-y', '-1.27',
                '-z', '0.01',
                '-Y', f'{math.pi}'
            ],
            output='screen',
        ),

        # 로봇 1 전용 브리지 (/joint_states 추가)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/model/tb3_0/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/model/tb3_0/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                '/model/tb3_0/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/model/tb3_0/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                '/model/tb3_0/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model',
            ],
            remappings=[
                ('/model/tb3_0/cmd_vel', '/tb3_0/cmd_vel'),
                ('/model/tb3_0/odometry', '/tb3_0/odom'),
                ('/model/tb3_0/scan', '/tb3_0/scan'),
                ('/model/tb3_0/tf', '/tf'),
                ('/model/tb3_0/joint_state', '/tb3_0/joint_states'),
            ],
            output='screen',
        ),

        # 로봇 1 상태 및 TF 트리 발행 (/tf_static 생성)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
            ),
            launch_arguments={'use_sim_time': 'true',
                              'frame_prefix': 'tb3_0',}.items()
        )
    ])

    # ==========================================
    # [로봇 2: tb3_1] (좌표: x=2.50, y=0.0, yaw=0.0)
    # ==========================================
    robot2_group = GroupAction([
        PushRosNamespace('tb3_1'),

        # Gazebo 스폰
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'tb3_1',
                '-file', sdf_path_tb3_1,
                '-x', '2.50',
                '-y', '0.00',
                '-z', '0.01',
                '-Y', '0.0'
            ],
            output='screen',
        ),

        # 로봇 2 전용 브리지 (/joint_states 추가)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/model/tb3_1/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/model/tb3_1/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                '/model/tb3_1/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/model/tb3_1/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                '/model/tb3_1/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model',
            ],
            remappings=[
                ('/model/tb3_1/cmd_vel', '/tb3_1/cmd_vel'),
                ('/model/tb3_1/odometry', '/tb3_1/odom'),
                ('/model/tb3_1/scan', '/tb3_1/scan'),
                ('/model/tb3_1/tf', '/tf'),
                ('/model/tb3_1/joint_state', '/tb3_1/joint_states'),
            ],
            output='screen',
        ),

        # 로봇 2 상태 및 TF 트리 발행 (/tf_static 생성)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
            ),
            launch_arguments={'use_sim_time': 'true',
                              'frame_prefix': 'tb3_1',}.items()
        )
    ])

    # ==========================================
    # [로봇 3: tb3_2] (좌표: x=2.50, y=1.27, yaw=0.0)
    # ==========================================
    robot3_group = GroupAction([
        PushRosNamespace('tb3_2'),

        # Gazebo 스폰
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'tb3_2',
                '-file', sdf_path_tb3_2,
                '-x', '2.50',
                '-y', '1.27',
                '-z', '0.01',
                '-Y', '0.0'
            ],
            output='screen',
        ),

        # 로봇 3 전용 브리지 (/joint_states 추가)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/model/tb3_2/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/model/tb3_2/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                '/model/tb3_2/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/model/tb3_2/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                '/model/tb3_2/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model',
            ],
            remappings=[
                ('/model/tb3_2/cmd_vel', '/tb3_2/cmd_vel'),
                ('/model/tb3_2/odometry', '/tb3_2/odom'),
                ('/model/tb3_2/scan', '/tb3_2/scan'),
                ('/model/tb3_2/tf', '/tf'),
                ('/model/tb3_2/joint_state', '/tb3_2/joint_states'),
            ],
            output='screen',
        ),

        # 로봇 3 상태 및 TF 트리 발행 (/tf_static 생성)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
            ),
            launch_arguments={'use_sim_time': 'true',
                              'frame_prefix': 'tb3_2',}.items()
        )
    ])

    # 리소스 경로 설정
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_turtlebot3_gazebo, 'models')
    )
    set_env_vars_common_mesh = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        '/opt/ros/jazzy/share'
    )

    ld = LaunchDescription()

    ld.add_action(set_env_vars_resources)
    ld.add_action(set_env_vars_common_mesh)

    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(global_clock_bridge)

    # 세 대의 로봇 그룹 추가
    ld.add_action(robot1_group)
    ld.add_action(robot2_group)
    ld.add_action(robot3_group)

    return ld
