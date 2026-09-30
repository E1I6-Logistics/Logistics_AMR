#!/usr/bin/env python3
"""Publish ArUco marker poses from camera image topics."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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
        DeclareLaunchArgument("marker_ids", default_value="24,25"),
        DeclareLaunchArgument("marker_size", default_value="0.04"),
        DeclareLaunchArgument("marker_map", default_value=default_marker_map),
        DeclareLaunchArgument("print_period", default_value="0.5"),
        DeclareLaunchArgument("save_path", default_value="/tmp/aruco_pose_viewer.jpg"),
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
                "--marker-ids",
                LaunchConfiguration("marker_ids"),
                "--marker-size",
                LaunchConfiguration("marker_size"),
                "--marker-map",
                LaunchConfiguration("marker_map"),
                "--print-period",
                LaunchConfiguration("print_period"),
                "--save-path",
                LaunchConfiguration("save_path"),
            ],
        ),
    ])
