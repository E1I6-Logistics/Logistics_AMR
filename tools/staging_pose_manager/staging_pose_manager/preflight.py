"""ROS graph and TF preflight checks for staging registration."""

from dataclasses import dataclass
import time

import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import TransformException


@dataclass
class CheckResult:
    """One preflight check result."""

    group: str
    label: str
    passed: bool
    required: bool
    detail: str = ""


def _normalized_names(node_names):
    return {name.rsplit("/", 1)[-1] for name in node_names}


def run_checks(node, tf_buffer, discovery_time=1.5):
    """Inspect topics, nodes, action endpoints and important transforms."""
    deadline = time.monotonic() + discovery_time
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    topics = dict(node.get_topic_names_and_types())
    node_names = _normalized_names(name for name, _ in node.get_node_names_and_namespaces())
    results = []

    def topic(group, name, required=True):
        present = name in topics and node.count_publishers(name) > 0
        results.append(CheckResult(group, name, present, required))

    def ros_node(group, name, required=True):
        results.append(CheckResult(group, name, name in node_names, required))

    topic("Robot Bringup", "/scan")
    topic("Robot Bringup", "/odom")
    topic("Localization", "/map")
    ros_node("Localization", "amcl")
    ros_node("Nav2", "planner_server")
    ros_node("Nav2", "controller_server")
    ros_node("Nav2", "bt_navigator")

    action_topics = {
        "/navigate_to_pose/_action/status",
        "/navigate_to_pose/_action/feedback",
    }
    action_ready = bool(action_topics.intersection(topics))
    results.append(CheckResult("Nav2", "/navigate_to_pose", action_ready, True))

    cmd_vel_types = topics.get("/cmd_vel", [])
    expected_cmd_type = "geometry_msgs/msg/TwistStamped"
    cmd_subscribers = node.count_subscribers("/cmd_vel")
    cmd_publishers = node.count_publishers("/cmd_vel")
    results.append(CheckResult(
        "Teleop",
        "/cmd_vel TwistStamped input",
        cmd_subscribers > 0 and expected_cmd_type in cmd_vel_types,
        True,
        (
            "expected geometry_msgs/msg/TwistStamped; "
            f"discovered: {', '.join(cmd_vel_types) or 'none'}"
        ),
    ))
    results.append(CheckResult(
        "Teleop",
        "teleop publisher",
        cmd_publishers > 0,
        False,
        "teleop가 idle 상태면 publisher만 존재해도 감지됩니다.",
    ))

    results.append(_tf_check(tf_buffer, "Robot Bringup", "odom", "base_footprint"))
    results.append(_tf_check(tf_buffer, "Localization", "map", "base_footprint"))
    return results


def _tf_check(tf_buffer, group, target, source):
    label = f"{target} -> {source} TF"
    try:
        tf_buffer.lookup_transform(
            target,
            source,
            Time(),
            timeout=Duration(seconds=0.5),
        )
        return CheckResult(group, label, True, True)
    except TransformException as exc:
        return CheckResult(group, label, False, True, str(exc))


def print_report(results):
    """Print a grouped, operator-friendly report."""
    current_group = None
    for result in results:
        if result.group != current_group:
            current_group = result.group
            print(f"\n[{current_group}]")
        status = "PASS" if result.passed else ("WARN" if not result.required else "FAIL")
        print(f"  {result.label:<28} [{status}]")
        if result.detail and not result.passed:
            print(f"    {result.detail}")
    return all(result.passed for result in results if result.required)


def check_localization(tf_buffer):
    """Return whether map to base_footprint is currently available."""
    return _tf_check(tf_buffer, "Localization", "map", "base_footprint").passed
