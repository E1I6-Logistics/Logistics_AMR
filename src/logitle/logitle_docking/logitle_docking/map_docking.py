"""Map 기준 도킹 완료 로봇 Pose와 V 도크 모델 Pose를 계산하는 순수 2D 기하 함수."""
import math
import numpy as np


def wrap(a): return math.atan2(math.sin(a), math.cos(a))


def rot(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]], dtype=np.float64)


# -------------------------------------------------------------------------
# final_pose는 로봇이 완전히 도킹됐을 때의 base_footprint Map Pose다.
# 도킹할 때는 로봇 전방이 도크의 바깥을 향하고, 후진하면서 진입한다고 가정.
# -------------------------------------------------------------------------
def staging_pose(final_pose, outward_distance):
    x, y, yaw = [float(v) for v in final_pose]
    d = float(outward_distance)
    if not math.isfinite(d) or d <= 0: raise ValueError('invalid staging offset')
    return x + d*math.cos(yaw), y + d*math.sin(yaw), wrap(yaw)


def dock_model_pose(final_pose, rear_offset):
    x, y, yaw = [float(v) for v in final_pose]
    d = float(rear_offset)
    if not math.isfinite(d) or not 0 < d < 0.5: raise ValueError('invalid model rear offset')
    return x - d*math.cos(yaw), y - d*math.sin(yaw), wrap(yaw)


# -------------------------------------------------------------------------
# 기존 ICP와 동일한 변환 정의: source(base) -> target(V 모델)
# p_model = R @ p_base + t
# -------------------------------------------------------------------------
def base_to_model_transform(base_map_pose, model_map_pose):
    bx, by, byaw = [float(v) for v in base_map_pose]
    mx, my, myaw = [float(v) for v in model_map_pose]
    Rm = rot(myaw)
    T = np.eye(3, dtype=np.float32)
    T[:2, :2] = Rm.T @ rot(byaw)
    T[:2, 2] = Rm.T @ (np.array([bx, by]) - np.array([mx, my]))
    return T


def observed_model_map_pose(base_map_pose, source_to_model_T):
    bx, by, byaw = [float(v) for v in base_map_pose]
    T = np.asarray(source_to_model_T)
    R, t = T[:2, :2], T[:2, 2]
    origin_map = np.array([bx, by]) + rot(byaw) @ (-R.T @ t)
    model_yaw_in_base = math.atan2(float(R[0, 1]), float(R[0, 0]))
    return float(origin_map[0]), float(origin_map[1]), wrap(byaw + model_yaw_in_base)


def accept_map_prior(measured_model_map_pose, expected_model_map_pose, max_xy_m=0.040, max_yaw_deg=7.0):
    ox, oy, oyaw = measured_model_map_pose
    ex, ey, eyaw = expected_model_map_pose
    pos_err = math.hypot(ox-ex, oy-ey)
    yaw_err = abs(wrap(oyaw-eyaw))
    ok = pos_err <= max_xy_m and yaw_err <= math.radians(max_yaw_deg)
    return bool(ok), float(pos_err), float(math.degrees(yaw_err))
