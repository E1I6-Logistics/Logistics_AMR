#!/usr/bin/env python3
"""Launch ArUco map->odom TF correction."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _corrector_node(publish_tf):
    args = [
        "--pose-topic",
        LaunchConfiguration("pose_topic"),
        "--marker-map",
        LaunchConfiguration("marker_map"),
        "--marker-id",
        LaunchConfiguration("marker_id"),
        "--map-frame",
        LaunchConfiguration("map_frame"),
        "--odom-frame",
        LaunchConfiguration("odom_frame"),
        "--base-frame",
        LaunchConfiguration("base_frame"),
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
        "--outlier-z-max",
        LaunchConfiguration("outlier_z_max"),
        "--outlier-tilt-deg",
        LaunchConfiguration("outlier_tilt_deg"),
        "--expected-base-yaw-deg",
        LaunchConfiguration("expected_base_yaw_deg"),
        "--yaw-tolerance-deg",
        LaunchConfiguration("yaw_tolerance_deg"),
        "--median-window",
        LaunchConfiguration("median_window"),
        "--stale-timeout",
        LaunchConfiguration("stale_timeout"),
    ]
    if publish_tf:
        args.insert(0, "--publish-tf")

    return Node(
        package="logitle_aruco_tools",
        executable="logitle_aruco_tf_corrector",
        name="aruco_tf_corrector",
        output="screen",
        arguments=args,
        condition=IfCondition(LaunchConfiguration("publish_tf"))
        if publish_tf
        else UnlessCondition(LaunchConfiguration("publish_tf")),
    )


def generate_launch_description():
    default_marker_map = PathJoinSubstitution([
        FindPackageShare("logitle_aruco_tools"),
        "config",
        "logitle_marker_map.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("pose_topic", default_value="/aruco/id24/pose_camera"),
        DeclareLaunchArgument("marker_map", default_value=default_marker_map),
        DeclareLaunchArgument("marker_id", default_value="24"),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("camera_x", default_value="0.045"),
        DeclareLaunchArgument("camera_y", default_value="0.0"),
        DeclareLaunchArgument("camera_z", default_value="0.115"),
        DeclareLaunchArgument("camera_pitch", default_value="-5.0"),
        DeclareLaunchArgument("camera_yaw", default_value="0.0"),
        DeclareLaunchArgument("camera_roll", default_value="0.0"),
        DeclareLaunchArgument("outlier_z_max", default_value="0.15"),
        DeclareLaunchArgument("outlier_tilt_deg", default_value="25"),
        DeclareLaunchArgument("expected_base_yaw_deg", default_value="-90"),
        DeclareLaunchArgument("yaw_tolerance_deg", default_value="4"),
        DeclareLaunchArgument("median_window", default_value="15"),
        DeclareLaunchArgument("stale_timeout", default_value="1.0"),
        DeclareLaunchArgument("publish_tf", default_value="false"),
        _corrector_node(publish_tf=False),
        _corrector_node(publish_tf=True),
    ])
