#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    ROS_DISTRO = os.environ.get('ROS_DISTRO')
    LDS_MODEL = os.environ['LDS_MODEL']
    LDS_LAUNCH_FILE = '/hlds_laser.launch.py'

    namespace = LaunchConfiguration('namespace', default='')

    usb_port = LaunchConfiguration('usb_port', default='/dev/ttyACM0')

    if ROS_DISTRO == 'humble':
        tb3_param_dir = LaunchConfiguration(
            'tb3_param_dir',
            default=os.path.join(
                get_package_share_directory('turtlebot3_bringup'),
                'param',
                ROS_DISTRO,
                TURTLEBOT3_MODEL + '.yaml'))
    else:
        tb3_param_dir = LaunchConfiguration(
            'tb3_param_dir',
            default=os.path.join(
                get_package_share_directory('turtlebot3_bringup'),
                'param',
                TURTLEBOT3_MODEL + '.yaml'))

    if LDS_MODEL == 'LDS-01':
        lidar_pkg_dir = LaunchConfiguration(
            'lidar_pkg_dir',
            default=os.path.join(get_package_share_directory('hls_lfcd_lds_driver'), 'launch'))
    elif LDS_MODEL == 'LDS-02':
        lidar_pkg_dir = LaunchConfiguration(
            'lidar_pkg_dir',
            default=os.path.join(get_package_share_directory('ld08_driver'), 'launch'))
        LDS_LAUNCH_FILE = '/ld08.launch.py'
    elif LDS_MODEL == 'LDS-03':
        lidar_pkg_dir = LaunchConfiguration(
            'lidar_pkg_dir',
            default=os.path.join(get_package_share_directory('coin_d4_driver'), 'launch'))
        LDS_LAUNCH_FILE = '/single_lidar_node.launch.py'
    else:
        lidar_pkg_dir = LaunchConfiguration(
            'lidar_pkg_dir',
            default=os.path.join(get_package_share_directory('hls_lfcd_lds_driver'), 'launch'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Launch 실행 시 사용자가 외부에서 인자로 전달할 수 있는 'robot_id' 값을 가져와 변수에 저장합니다. (예: robot1, robot2)
    robot_id = LaunchConfiguration('robot_id')

    # robot1 -> /robot1
    # PythonExpression을 사용하여 위에서 받은 robot_id 문자열 앞에 슬래시('/')를 붙여 네임스페이스 형태(예: '/robot1')로 만듭니다. 
    # 이 변수는 하단 Zenoh 브릿지 실행 시 통신 토픽들의 접두사(Prefix)로 사용되어 메인 PC가 어떤 로봇인지 식별할 수 있게 해줍니다.
    bridge_namespace = PythonExpression([
        "'/' + '", robot_id, "'"
    ])

    # 브릿지 설정 파일인 bridge.json5의 절대 경로를 동적으로 생성합니다. 
    # 패키지(zenoh_pkg)가 설치된 경로를 찾고, 그 아래의 config 폴더 내에 있는 파일을 지정하여 하드코딩된 경로 없이 유연하게 파일을 찾게 해줍니다.
    config_file = PathJoinSubstitution([
        FindPackageShare('turtlebot3_bringup'),
        'config',
        'bridge.json5'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'usb_port',
            default_value=usb_port,
            description='Connected USB port with OpenCR'),

        DeclareLaunchArgument(
            'tb3_param_dir',
            default_value=tb3_param_dir,
            description='Full path to turtlebot3 parameter file to load'),

        DeclareLaunchArgument(
            'namespace',
            default_value=namespace,
            description='Namespace for nodes'),

        PushRosNamespace(namespace),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/turtlebot3_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time,
                              'namespace': namespace}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([lidar_pkg_dir, LDS_LAUNCH_FILE]),
            launch_arguments={'port': '/dev/ttyUSB0',
                              'frame_id': 'base_scan',
                              'namespace': namespace}.items(),
        ),

        Node(
            package='turtlebot3_node',
            executable='turtlebot3_ros',
            parameters=[
                tb3_param_dir,
                {'namespace': namespace}],
            arguments=['-i', usb_port],
            output='screen'
        ),

        # launch 명령어 실행 시 사용자가 'robot_id:=원하는이름' 형태로 값을 넘길 수 있도록 인자를 선언합니다. 값을 넘기지 않으면 기본값 'robot1'이 사용됩니다.
        DeclareLaunchArgument(
            'robot_id',
            default_value='robot1'
        ),

        # =====================================================
        # Zenoh Bridge
        #
        # Robot1:
        #   내부 /odom <-> 외부 /robot1/odom
        #
        # Robot2:
        #   내부 /odom <-> 외부 /robot2/odom
        #
        # 다른 robot namespace는 bridge.json5에서 차단
        # =====================================================
        # ROS 2의 DDS 통신을 Zenoh 프로토콜로 변환하여 외부 네트워크(Main PC)로 보내주는 브릿지 프로세스를 실행합니다.
        ExecuteProcess(
            cmd=[
                'zenoh-bridge-ros2dds',
                '-c',
                config_file,          # 위에서 찾은 bridge.json5 설정 파일을 적용하여 불필요한 토픽 유입을 막습니다.
                '-e',
                'tcp/10.10.141.15:7447', # Main PC(또는 Zenoh 라우터)의 IP와 포트 번호로 연결(client 모드)을 시도합니다.
                '-n',
                bridge_namespace,     # 에이전트 노드가 발행하는 순수 토픽('/telemetry')에 이 네임스페이스('/robot1')를 붙여서 외부로 보냅니다. (결과: '/robot1/telemetry')
                'client'
            ],
            output='screen'
        ),
    ])

    
