#!/usr/bin/env python3
"""Publish ArUco marker poses from camera image topics."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def default_robot_namespace():
    return {
        "1": "tb3_0",
        "2": "tb3_1",
        "3": "tb3_2",
    }.get(os.environ.get("USER", "")[-1:], "")


def scoped_topic(namespace, name):
    return f"/{namespace}/{name}" if namespace else f"/{name}"


def scoped_frame(namespace, name):
    return f"{namespace}/{name}" if namespace else name


def generate_launch_description():
    robot_namespace = default_robot_namespace()
    default_marker_map = PathJoinSubstitution([
        FindPackageShare("logitle_aruco_tools"),
        "config",
        "logitle_marker_map.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera_info"),
        DeclareLaunchArgument("dictionary", default_value="5X5_1000"),
        DeclareLaunchArgument("marker_ids", default_value="24,25"),
        DeclareLaunchArgument("marker_size", default_value="0.04"),
        DeclareLaunchArgument("marker_map", default_value=default_marker_map),
        DeclareLaunchArgument(
            "pose_topic_template",
            default_value=scoped_topic(robot_namespace, "aruco/id{marker_id}/pose_camera"),
        ),
        DeclareLaunchArgument("pose_frame_id", default_value=scoped_frame(robot_namespace, "camera_optical_frame")),
        DeclareLaunchArgument("print_period", default_value="0.5"),
        DeclareLaunchArgument("save_path", default_value="/tmp/aruco_pose_viewer.jpg"),
        Node(
            package="logitle_aruco_tools",
            executable="logitle_aruco_pose_viewer",
            name="aruco_pose_viewer",
            namespace=robot_namespace,
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
                "--pose-topic-template",
                LaunchConfiguration("pose_topic_template"),
                "--pose-frame-id",
                LaunchConfiguration("pose_frame_id"),
                "--print-period",
                LaunchConfiguration("print_period"),
                "--save-path",
                LaunchConfiguration("save_path"),
            ],
        ),
    ])
