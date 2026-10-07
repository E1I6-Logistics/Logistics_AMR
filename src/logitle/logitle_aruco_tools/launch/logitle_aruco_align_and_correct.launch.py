#!/usr/bin/env python3
"""Launch ArUco pickup alignment followed by pose correction."""

import os
import socket

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# Camera yaw [deg, left positive] per robot hostname (2026-10-07), fitted on
# the N5 floor mark with both wheel edges (robot1 also on N6, after its camera
# screw was tightened). An explicit camera_yaw:= launch argument still wins. robot3's 1.5 cm offset at N6 is not a camera yaw:
# 2.1 deg fixed N6 but moved N5 off its mark by the same amount.
CAMERA_YAW_BY_HOST = {
    "turtlebot1": "4.6",
    "turtlebot3": "4.1",
}

# Lidar yaw [deg, left positive] per robot hostname. scan_yaw:= still wins.
# Changing it shifts the lateral reading by about 0.7 cm per degree, so refit
# camera_yaw after it. The heading errors seen at N5 were not a lidar mount
# yaw (robot1 read N6 correctly), so they live in HEADING_TRIMS_BY_HOST.
SCAN_YAW_BY_HOST = {}
# Wall alignment heading trims per robot hostname: the lidar heading [deg]
# the robot reads at each node while it is square to the wall, from the
# wheel-to-wall distances (2026-10-07). pair_heading_trims:= still wins.
HEADING_TRIMS_BY_HOST = {
    "turtlebot1": "N5:-1.6,N6:-0.2",
    "turtlebot3": "N5:-0.7",
}
# Wall alignment lateral trims per robot hostname: the lateral [m, robot's
# right +] the robot reads while it sits on the node's floor mark, measured
# from both wheel edges. pair_lateral_trims:= still wins.
LATERAL_TRIMS_BY_HOST = {
    "turtlebot1": "N3:0.031",
}


def default_robot_namespace():
    return ""


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
        DeclareLaunchArgument("camera_auto_start", default_value="true"),
        DeclareLaunchArgument("camera_info_url", default_value=""),
        DeclareLaunchArgument("detection_rate_hz", default_value="10.0"),
        DeclareLaunchArgument("dictionary", default_value="5X5_1000"),
        DeclareLaunchArgument("marker_id", default_value="25"),
        DeclareLaunchArgument("marker_size", default_value="0.04"),
        DeclareLaunchArgument("marker_map", default_value=default_marker_map),
        DeclareLaunchArgument(
            "pose_topic_template",
            default_value=scoped_topic(robot_namespace, "aruco/id{marker_id}/pose_camera"),
        ),
        DeclareLaunchArgument("pose_frame_id", default_value="camera_optical_frame"),
        DeclareLaunchArgument("align_action_name", default_value=scoped_topic(robot_namespace, "aruco_align_and_correct")),
        DeclareLaunchArgument("correct_action_name", default_value=scoped_topic(robot_namespace, "aruco_correct_pose")),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("cmd_vel_stamped", default_value="true"),
        DeclareLaunchArgument("cmd_frame_id", default_value="base_footprint"),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument("base_frame", default_value="base_footprint"),
        DeclareLaunchArgument("initialpose_topic", default_value="/initialpose"),
        DeclareLaunchArgument("target_x", default_value="0.0"),
        DeclareLaunchArgument("target_z", default_value="0.0"),
        DeclareLaunchArgument("x_tolerance", default_value="0.005"),
        DeclareLaunchArgument("z_tolerance", default_value="0.007"),
        DeclareLaunchArgument("z_min_stop", default_value="0.25"),
        DeclareLaunchArgument("stable_sec", default_value="0.4"),
        DeclareLaunchArgument("max_linear", default_value="0.030"),
        DeclareLaunchArgument("max_angular", default_value="0.04"),
        DeclareLaunchArgument("min_angular", default_value="0.020"),
        DeclareLaunchArgument("kx", default_value="0.8"),
        DeclareLaunchArgument("wall_yaw_tolerance_deg", default_value="2.0"),
        DeclareLaunchArgument("kz", default_value="1.2"),
        DeclareLaunchArgument("kyaw", default_value="0.6"),
        DeclareLaunchArgument("align_timeout_sec", default_value="25.0"),
        DeclareLaunchArgument("correct_timeout_sec", default_value="10.0"),
        DeclareLaunchArgument("required_samples", default_value="15"),
        DeclareLaunchArgument("yaw_tolerance_deg", default_value="3.0"),
        DeclareLaunchArgument("camera_x", default_value="0.045"),
        DeclareLaunchArgument("camera_y", default_value="0.0"),
        DeclareLaunchArgument("camera_z", default_value="0.115"),
        DeclareLaunchArgument("camera_pitch", default_value="-5.0"),
        DeclareLaunchArgument(
            "camera_yaw", default_value=CAMERA_YAW_BY_HOST.get(socket.gethostname(), "0.0")
        ),
        DeclareLaunchArgument("camera_roll", default_value="0.0"),
        DeclareLaunchArgument(
            "scan_yaw", default_value=SCAN_YAW_BY_HOST.get(socket.gethostname(), "0.0")
        ),
        DeclareLaunchArgument(
            "pair_lateral_trims", default_value=LATERAL_TRIMS_BY_HOST.get(socket.gethostname(), "none")
        ),
        DeclareLaunchArgument(
            "pair_heading_trims", default_value=HEADING_TRIMS_BY_HOST.get(socket.gethostname(), "none")
        ),
        DeclareLaunchArgument("pose_source", default_value="camera"),
        DeclareLaunchArgument("approx_camera_info", default_value="false"),
        DeclareLaunchArgument("approx_horizontal_fov_deg", default_value="62.2"),
        Node(
            package="logitle_aruco_tools",
            executable="logitle_aruco_pose_corrector_action_server",
            name="aruco_pose_corrector_action_server",
            namespace=robot_namespace,
            output="screen",
            arguments=[
                "--action-name",
                LaunchConfiguration("correct_action_name"),
                "--pose-source",
                LaunchConfiguration("pose_source"),
                "--image-topic",
                LaunchConfiguration("image_topic"),
                "--camera-info-topic",
                LaunchConfiguration("camera_info_topic"),
                "--detection-rate-hz",
                LaunchConfiguration("detection_rate_hz"),
                "--dictionary",
                LaunchConfiguration("dictionary"),
                "--pose-topic-template",
                LaunchConfiguration("pose_topic_template"),
                "--pose-frame-id",
                LaunchConfiguration("pose_frame_id"),
                "--marker-id",
                LaunchConfiguration("marker_id"),
                "--marker-size",
                LaunchConfiguration("marker_size"),
                "--marker-map",
                LaunchConfiguration("marker_map"),
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
            ],
        ),
        Node(
            package="logitle_aruco_tools",
            executable="logitle_align_and_correct_action_server",
            name="align_and_correct_with_aruco_action_server",
            namespace=robot_namespace,
            output="screen",
            arguments=[
                "--action-name",
                LaunchConfiguration("align_action_name"),
                "--correct-action-name",
                LaunchConfiguration("correct_action_name"),
                "--pose-source",
                LaunchConfiguration("pose_source"),
                "--image-topic",
                LaunchConfiguration("image_topic"),
                "--camera-info-topic",
                LaunchConfiguration("camera_info_topic"),
                "--camera-auto-start",
                LaunchConfiguration("camera_auto_start"),
                "--camera-info-url",
                LaunchConfiguration("camera_info_url"),
                "--detection-rate-hz",
                LaunchConfiguration("detection_rate_hz"),
                "--dictionary",
                LaunchConfiguration("dictionary"),
                "--pose-topic-template",
                LaunchConfiguration("pose_topic_template"),
                "--pose-frame-id",
                LaunchConfiguration("pose_frame_id"),
                "--marker-map",
                LaunchConfiguration("marker_map"),
                "--marker-id",
                LaunchConfiguration("marker_id"),
                "--marker-size",
                LaunchConfiguration("marker_size"),
                "--cmd-vel-topic",
                LaunchConfiguration("cmd_vel_topic"),
                "--cmd-vel-stamped",
                LaunchConfiguration("cmd_vel_stamped"),
                "--cmd-frame-id",
                LaunchConfiguration("cmd_frame_id"),
                "--target-z",
                LaunchConfiguration("target_z"),
                "--target-x",
                LaunchConfiguration("target_x"),
                "--x-tolerance",
                LaunchConfiguration("x_tolerance"),
                "--z-tolerance",
                LaunchConfiguration("z_tolerance"),
                "--z-min-stop",
                LaunchConfiguration("z_min_stop"),
                "--stable-sec",
                LaunchConfiguration("stable_sec"),
                "--max-linear",
                LaunchConfiguration("max_linear"),
                "--max-angular",
                LaunchConfiguration("max_angular"),
                "--min-angular",
                LaunchConfiguration("min_angular"),
                "--kx",
                LaunchConfiguration("kx"),
                "--kz",
                LaunchConfiguration("kz"),
                "--wall-yaw-tolerance-deg",
                LaunchConfiguration("wall_yaw_tolerance_deg"),
                "--kyaw",
                LaunchConfiguration("kyaw"),
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
                "--scan-yaw",
                LaunchConfiguration("scan_yaw"),
                "--pair-lateral-trims",
                LaunchConfiguration("pair_lateral_trims"),
                "--pair-heading-trims",
                LaunchConfiguration("pair_heading_trims"),
                "--map-frame",
                LaunchConfiguration("map_frame"),
                "--initialpose-topic",
                LaunchConfiguration("initialpose_topic"),
                "--align-timeout-sec",
                LaunchConfiguration("align_timeout_sec"),
                "--correct-timeout-sec",
                LaunchConfiguration("correct_timeout_sec"),
                "--required-samples",
                LaunchConfiguration("required_samples"),
                "--yaw-tolerance-deg",
                LaunchConfiguration("yaw_tolerance_deg"),
                "--approx-camera-info",
                LaunchConfiguration("approx_camera_info"),
                "--approx-horizontal-fov-deg",
                LaunchConfiguration("approx_horizontal_fov_deg"),
            ],
        ),
    ])
