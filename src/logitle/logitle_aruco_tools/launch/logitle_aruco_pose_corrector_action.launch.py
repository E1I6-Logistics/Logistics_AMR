#!/usr/bin/env python3
"""Launch the on-demand ArUco pose-correction action server."""

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
        DeclareLaunchArgument("marker_id", default_value="25"),
        DeclareLaunchArgument("marker_size", default_value="0.04"),
        DeclareLaunchArgument("marker_map", default_value=default_marker_map),
        DeclareLaunchArgument("pose_source", default_value="camera"),
        DeclareLaunchArgument(
            "pose_topic_template",
            default_value=scoped_topic(robot_namespace, "aruco/id{marker_id}/pose_camera"),
        ),
        DeclareLaunchArgument("pose_frame_id", default_value=scoped_frame(robot_namespace, "camera_optical_frame")),
        DeclareLaunchArgument("action_name", default_value=scoped_topic(robot_namespace, "aruco_correct_pose")),
        DeclareLaunchArgument("approx_camera_info", default_value="false"),
        DeclareLaunchArgument("approx_horizontal_fov_deg", default_value="62.2"),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("odom_frame", default_value=scoped_frame(robot_namespace, "odom")),
        DeclareLaunchArgument("base_frame", default_value=scoped_frame(robot_namespace, "base_link")),
        DeclareLaunchArgument("initialpose_topic", default_value=scoped_topic(robot_namespace, "initialpose")),
        DeclareLaunchArgument("initialpose_xy_std", default_value="0.05"),
        DeclareLaunchArgument("initialpose_yaw_std_deg", default_value="5.0"),
        DeclareLaunchArgument("allow_tf_publish", default_value="false"),
        DeclareLaunchArgument("force_tf_publish", default_value="false"),
        DeclareLaunchArgument("max_xy_std", default_value="0.03"),
        DeclareLaunchArgument("max_yaw_std_deg", default_value="3.0"),
        DeclareLaunchArgument("camera_x", default_value="0.045"),
        DeclareLaunchArgument("camera_y", default_value="0.0"),
        DeclareLaunchArgument("camera_z", default_value="0.115"),
        DeclareLaunchArgument("camera_pitch", default_value="-5.0"),
        DeclareLaunchArgument("camera_yaw", default_value="0.0"),
        DeclareLaunchArgument("camera_roll", default_value="0.0"),
        Node(
            package="logitle_aruco_tools",
            executable="logitle_aruco_pose_corrector_action_server",
            name="aruco_pose_corrector_action_server",
            namespace=robot_namespace,
            output="screen",
            arguments=[
                "--action-name",
                LaunchConfiguration("action_name"),
                "--pose-source",
                LaunchConfiguration("pose_source"),
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
                "--pose-topic-template",
                LaunchConfiguration("pose_topic_template"),
                "--pose-frame-id",
                LaunchConfiguration("pose_frame_id"),
                "--marker-map",
                LaunchConfiguration("marker_map"),
                "--approx-camera-info",
                LaunchConfiguration("approx_camera_info"),
                "--approx-horizontal-fov-deg",
                LaunchConfiguration("approx_horizontal_fov_deg"),
                "--map-frame",
                LaunchConfiguration("map_frame"),
                "--odom-frame",
                LaunchConfiguration("odom_frame"),
                "--base-frame",
                LaunchConfiguration("base_frame"),
                "--initialpose-topic",
                LaunchConfiguration("initialpose_topic"),
                "--initialpose-xy-std",
                LaunchConfiguration("initialpose_xy_std"),
                "--initialpose-yaw-std-deg",
                LaunchConfiguration("initialpose_yaw_std_deg"),
                "--allow-tf-publish",
                LaunchConfiguration("allow_tf_publish"),
                "--force-tf-publish",
                LaunchConfiguration("force_tf_publish"),
                "--max-xy-std",
                LaunchConfiguration("max_xy_std"),
                "--max-yaw-std-deg",
                LaunchConfiguration("max_yaw_std_deg"),
                "--camera-x",
                LaunchConfiguration("camera_x"),
                "--camera-y",
                LaunchConfiguration("camera_y"),
                "--camera-z",
                LaunchConfiguration("camera_z"),
                "--camera-pitch",
                LaunchConfiguration("camera_pitch"),
                "--camera-yaw",
                LaunchConfiguration("camera_yaw"),
                "--camera-roll",
                LaunchConfiguration("camera_roll"),
            ],
        ),
    ])
