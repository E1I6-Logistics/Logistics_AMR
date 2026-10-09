"""차동구동 로봇의 경유지 전진 및 횡오차 기반 S자 후진 제어 기하 함수."""
import math


def wrap_angle(a): return math.atan2(math.sin(a), math.cos(a))
def clamp(v, lo, hi): return max(lo, min(hi, v))


# -------------------------------------------------------------------------
# 횡오차 -> 작은 목표 Yaw 변환. 도크 접근 시 목표 Yaw만 감쇠한다.
# 실제 Yaw를 도크 축에 정렬시키는 PD 각속도 제어는 유지한다.
# -------------------------------------------------------------------------
def yaw_target_from_lateral(lateral_error_m, dock_distance_m, gain, limit_deg, taper_start_m, taper_end_m):
    if taper_start_m <= taper_end_m: raise ValueError('invalid taper distances')
    scale = clamp((dock_distance_m-taper_end_m)/(taper_start_m-taper_end_m), 0.0, 1.0)
    target = clamp(gain*lateral_error_m, -math.radians(limit_deg), math.radians(limit_deg))
    return target*scale, scale


def reverse_angular_error(relative_yaw_rad, target_yaw_rad):
    # 기존 precision_docking_server.py에서 사용하던 부호 규칙 유지
    return wrap_angle(relative_yaw_rad + target_yaw_rad)


def final_entry_ok(lateral_error_m, yaw_error_rad, lat_tol_m, yaw_tol_deg):
    return abs(lateral_error_m) <= lat_tol_m and abs(wrap_angle(yaw_error_rad)) <= math.radians(yaw_tol_deg)


def map_target_yaw_to_odom(current_map_yaw, current_odom_yaw, desired_map_yaw):
    """Freeze desired MAP heading in Odom frame at final approach entry."""
    return wrap_angle(current_odom_yaw + wrap_angle(desired_map_yaw-current_map_yaw))


def reverse_axis_progress(start_x, start_y, start_yaw, x, y):
    dx, dy = x-start_x, y-start_y
    c, s = math.cos(start_yaw), math.sin(start_yaw)
    return max(0.0, -(dx*c + dy*s)), abs(-dx*s + dy*c)


def forward_heading_command(x, y, yaw, tx, ty, kp, max_w):
    return clamp(kp*wrap_angle(math.atan2(ty-y, tx-x)-yaw), -max_w, max_w)
