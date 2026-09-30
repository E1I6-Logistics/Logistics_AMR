#!/usr/bin/env python3
"""Launch ArUco pose publishing and auto alignment together."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _auto_align_node(enable_motion):
    args = [
        "--pose-topic",
        LaunchConfiguration("pose_topic"),
        "--cmd-vel-topic",
        LaunchConfiguration("cmd_vel_topic"),
        "--x-tol",
        LaunchConfiguration("x_tol"),
        "--z-target",
        LaunchConfiguration("z_target"),
        "--z-tol",
        LaunchConfiguration("z_tol"),
        "--max-linear",
        LaunchConfiguration("max_linear"),
        "--max-angular",
        LaunchConfiguration("max_angular"),
        "--pose-timeout",
        LaunchConfiguration("pose_timeout"),
    ]
    if enable_motion:
        args.insert(0, "--enable-motion")

    return Node(
        package="logitle_aruco_tools",
        executable="logitle_aruco_auto_align",
        name="aruco_auto_align",
        output="screen",
        arguments=args,
        condition=IfCondition(LaunchConfiguration("enable_motion"))
        if enable_motion
        else UnlessCondition(LaunchConfiguration("enable_motion")),
    )


def generate_launch_description():
    default_marker_map = PathJoinSubstitution([
        FindPackageShare("logitle_aruco_tools"),
        "config",
        "logitle_marker_map.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera_info"),
        DeclareLaunchArgument("dictionary", default_value="5X5_50"),
        DeclareLaunchArgument("pose_topic", default_value="/aruco/id24/pose_camera"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("marker_id", default_value="24"),
        DeclareLaunchArgument("marker_size", default_value="0.0395"),
        DeclareLaunchArgument("marker_map", default_value=default_marker_map),
        DeclareLaunchArgument("x_tol", default_value="0.025"),
        DeclareLaunchArgument("z_target", default_value="0.32"),
        DeclareLaunchArgument("z_tol", default_value="0.035"),
        DeclareLaunchArgument("max_linear", default_value="0.015"),
        DeclareLaunchArgument("max_angular", default_value="0.08"),
        DeclareLaunchArgument("pose_timeout", default_value="0.3"),
        DeclareLaunchArgument("enable_motion", default_value="false"),
        Node(
            package="logitle_aruco_tools",
            executable="logitle_aruco_pose_viewer",
            name="aruco_pose_viewer",
            output="screen",
            arguments=[
                "--no-gui",
                "--image-topic",
                LaunchConfiguration("image_topic"),
                "--camera-info-topic",
                LaunchConfiguration("camera_info_topic"),
                "--dictionary",
                LaunchConfiguration("dictionary"),
                "--marker-id",
                LaunchConfiguration("marker_id"),
                "--marker-size",
                LaunchConfiguration("marker_size"),
                "--marker-map",
                LaunchConfiguration("marker_map"),
                "--pose-topic",
                LaunchConfiguration("pose_topic"),
            ],
        ),
        _auto_align_node(enable_motion=False),
        _auto_align_node(enable_motion=True),
    ])
