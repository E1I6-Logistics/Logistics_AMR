#!/usr/bin/env python3
"""Launch a TF continuity monitor for map->base_link."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def default_robot_namespace():
    return {
        "1": "tb3_0",
        "2": "tb3_1",
        "3": "tb3_2",
    }.get(os.environ.get("USER", "")[-1:], "")


def scoped_frame(namespace, name):
    return f"{namespace}/{name}" if namespace else name


def generate_launch_description():
    robot_namespace = default_robot_namespace()
    return LaunchDescription([
        DeclareLaunchArgument("parent_frame", default_value="map"),
        DeclareLaunchArgument("child_frame", default_value=scoped_frame(robot_namespace, "base_link")),
        DeclareLaunchArgument("sample_period", default_value="0.1"),
        DeclareLaunchArgument("duration_sec", default_value="0.0"),
        DeclareLaunchArgument("jump_xy_threshold", default_value="0.08"),
        DeclareLaunchArgument("jump_yaw_threshold_deg", default_value="10.0"),
        DeclareLaunchArgument("stale_threshold", default_value="0.5"),
        DeclareLaunchArgument("csv", default_value=""),
        Node(
            package="logitle_aruco_tools",
            executable="logitle_tf_continuity_monitor",
            name="tf_continuity_monitor",
            namespace=robot_namespace,
            output="screen",
            arguments=[
                "--parent-frame",
                LaunchConfiguration("parent_frame"),
                "--child-frame",
                LaunchConfiguration("child_frame"),
                "--sample-period",
                LaunchConfiguration("sample_period"),
                "--duration-sec",
                LaunchConfiguration("duration_sec"),
                "--jump-xy-threshold",
                LaunchConfiguration("jump_xy_threshold"),
                "--jump-yaw-threshold-deg",
                LaunchConfiguration("jump_yaw_threshold_deg"),
                "--stale-threshold",
                LaunchConfiguration("stale_threshold"),
                "--csv",
                LaunchConfiguration("csv"),
            ],
        ),
    ])
