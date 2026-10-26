"""Unit-explicit navigation metrics."""

import math


def normalize_angle(angle_rad):
    """Normalize radians to [-pi, pi]."""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def quaternion_to_yaw(quaternion):
    """Convert a geometry quaternion object to yaw radians."""
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw_rad):
    """Return quaternion components for a planar yaw angle."""
    half = yaw_rad / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


def planar_error_mm(goal_x_m, goal_y_m, actual_x_m, actual_y_m):
    """Return Euclidean position error in millimetres."""
    return 1000.0 * math.hypot(actual_x_m - goal_x_m, actual_y_m - goal_y_m)


def signed_delta_mm(goal_m, actual_m):
    """Return actual minus goal in millimetres."""
    return 1000.0 * (actual_m - goal_m)


def yaw_error_deg(goal_yaw_rad, actual_yaw_rad):
    """Return signed shortest yaw error in degrees."""
    return math.degrees(normalize_angle(actual_yaw_rad - goal_yaw_rad))
