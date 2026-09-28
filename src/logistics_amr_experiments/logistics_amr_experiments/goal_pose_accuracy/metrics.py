import math


def normalize_angle(angle):
    return math.atan2(
        math.sin(angle),
        math.cos(angle),
    )


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (
        q.w * q.z
        + q.x * q.y
    )

    cosy_cosp = 1.0 - 2.0 * (
        q.y * q.y
        + q.z * q.z
    )

    return math.atan2(
        siny_cosp,
        cosy_cosp,
    )


def yaw_to_quaternion(yaw):
    half = yaw / 2.0

    return {
        'x': 0.0,
        'y': 0.0,
        'z': math.sin(half),
        'w': math.cos(half),
    }


def position_error(
    goal_x,
    goal_y,
    actual_x,
    actual_y,
):
    dx = actual_x - goal_x
    dy = actual_y - goal_y

    return math.hypot(dx, dy)


def heading_error_rad(
    goal_yaw,
    actual_yaw,
):
    return normalize_angle(
        actual_yaw - goal_yaw
    )


def physical_position_error_cm(
    dx_cm,
    dy_cm,
):
    return math.hypot(
        dx_cm,
        dy_cm,
    )