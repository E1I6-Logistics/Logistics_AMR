"""Map 기반 도킹의 ICP 검증 도구. 초기 광역 ICP 탐색은 사용하지 않는다."""
import math
import numpy as np
from scipy.spatial import cKDTree


class StableDockTracker:
    """연속 스캔의 도크 Map Pose가 위치·방향 허용오차 안에서 일치해야 통과."""
    def __init__(self, required=3, xy_tolerance_m=0.025, yaw_tolerance_deg=3.0):
        self.required = max(1, int(required))
        self.xy_tol, self.yaw_tol = float(xy_tolerance_m), math.radians(yaw_tolerance_deg)
        self.poses = []

    def reset(self): self.poses.clear()

    def observe(self, x, y, yaw):
        pose = (float(x), float(y), float(yaw))
        if self.poses:
            px, py, pyaw = self.poses[0]
            delta_yaw = math.atan2(math.sin(yaw-pyaw), math.cos(yaw-pyaw))
            if math.hypot(x-px, y-py) > self.xy_tol or abs(delta_yaw) > self.yaw_tol: self.reset()
        self.poses.append(pose)
        if len(self.poses) > self.required: self.poses.pop(0)
        return len(self.poses) >= self.required

    @property
    def count(self): return len(self.poses)


# -------------------------------------------------------------------------
# Map 기준 예상 V자 도크 위치 주변의 LiDAR 점만 사용한다.
# -------------------------------------------------------------------------
def select_scan_near_expected_dock(scan_points, model_points, source_to_model_transform, gate_radius_m=0.055):
    scan = np.asarray(scan_points, dtype=np.float32)
    if len(scan) == 0: return scan
    T = source_to_model_transform    # base -> 도크 모델의 2D 변환 행렬
    predicted_model_in_base = (np.asarray(model_points) - T[:2, 2]) @ T[:2, :2]
    distances, _ = cKDTree(predicted_model_in_base).query(scan, workers=1)
    return scan[distances <= gate_radius_m]


# -------------------------------------------------------------------------
# 단순 대응점 개수가 아닌 중앙부 + 양쪽 날개 + RMSE/인라이어 비율 검사.
# 도크 한쪽이 가려지면 무리하게 움직이지 않고 검증 실패를 반환한다.
# -------------------------------------------------------------------------
def validate_micro_fit(scan_points, model_points, model_tree, transform, match_radius_m=0.027):
    scan, model = np.asarray(scan_points), np.asarray(model_points)
    if len(scan) < 12: return False
    transformed = scan @ transform[:2, :2].T + transform[:2, 2]
    distances, model_indices = model_tree.query(transformed, workers=1)
    inlier = distances < match_radius_m
    if np.count_nonzero(inlier) < 12 or np.mean(inlier) < 0.45: return False

    # 많은 스캔 포인트가 하나의 모델 점에 몰리는 오인식을 방지: 모델 고유 점으로 센다.
    unique_indices = np.unique(model_indices[inlier])
    matched_model = model[unique_indices]
    center = int(np.count_nonzero(matched_model[:, 0] < 0.005))
    left = int(np.count_nonzero((matched_model[:, 0] >= 0.005) & (matched_model[:, 1] > 0)))
    right = int(np.count_nonzero((matched_model[:, 0] >= 0.005) & (matched_model[:, 1] < 0)))
    rmse = float(np.sqrt(np.mean(np.square(distances[inlier]))))
    return center >= 3 and left >= 1 and right >= 1 and rmse <= 0.020
