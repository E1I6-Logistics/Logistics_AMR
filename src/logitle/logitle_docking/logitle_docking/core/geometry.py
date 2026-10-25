import math
from typing import Optional, Tuple

import numpy as np
from scipy.spatial import cKDTree


def normalize_angle(angle: float) -> float:
    """각도를 [-pi, pi] 범위로 정규화."""
    return math.atan2(math.sin(angle), math.cos(angle))


def get_yaw_from_quaternion(q) -> float:
    """ROS Quaternion 메시지에서 2D 평면 Yaw 각도(라디안) 추출."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def generate_v_funnel(
    charger_width: float,
    wing_length: float,
    wing_angle_rad: float
) -> np.ndarray:
    """
    V자형 도킹 스테이션의 2D 점군 모델을 생성.

    충전기 중앙 평면부(charger_width)와 양쪽 날개(wing_length)를
    일정 간격의 2D 포인트 리스트로 합성하여 반환.
    """
    pts = []
    half_w = charger_width / 2.0

    # 도크 중앙 평면부
    for y in np.arange(-half_w, half_w + 0.001, 0.005):
        pts.append([0.0, float(y)])

    # 도크 좌우 경사 날개 (V-Funnel)
    for seg_len in np.arange(0.01, wing_length + 0.005, 0.025):
        px = float(seg_len * math.cos(wing_angle_rad))
        py = float(half_w + seg_len * math.sin(wing_angle_rad))
        pts.extend([[px, py], [px, -py]])

    return np.array(pts, dtype=np.float32)


def icp_2d(
    source_pts: np.ndarray,
    target_tree: cKDTree,
    target_pts: np.ndarray,
    initial_transform: Optional[np.ndarray] = None,
    max_iter: int = 20,
    search_radius: float = 0.5
) -> Tuple[np.ndarray, float, int]:
    """
    2D 점군 간의 SVD 기반 ICP(Iterative Closest Point) 정합 알고리즘.

    :param source_pts: 입력 관측 점군 (Nx2)
    :param target_tree: 기준 타겟 KD-Tree
    :param target_pts: 기준 타겟 점군 (Mx2)
    :param initial_transform: 초기 3x3 변환 행렬
    :param max_iter: 최대 반복 횟수
    :param search_radius: 최근접 이웃 탐색 반경
    :return: (T, fitness, matched_count) 변환행렬, 적합도, 매칭 점 개수
    """
    if initial_transform is None:
        T = np.identity(3, dtype=np.float32)
    else:
        T = np.copy(initial_transform)

    prev_error = float('inf')
    distances = np.array([], dtype=np.float32)
    matched_count = 0

    for _ in range(max_iter):
        R_current, t_current = T[:2, :2], T[:2, 2]
        transformed = (source_pts @ R_current.T) + t_current
        distances, indices = target_tree.query(transformed, workers=1)

        valid = distances < search_radius
        matched_count = int(np.sum(valid))
        if matched_count < 10:
            break

        pts_src = source_pts[valid]
        pts_tgt = target_pts[indices[valid]]
        centroid_src = np.mean(pts_src, axis=0)
        centroid_tgt = np.mean(pts_tgt, axis=0)

        H = (pts_src - centroid_src).T @ (pts_tgt - centroid_tgt)
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[1, :] *= -1
            R = Vt.T @ U.T

        t = centroid_tgt - (R @ centroid_src)
        current_error = np.sum(np.abs(T[:2, :2] - R)) + np.sum(np.abs(T[:2, 2] - t))

        T[:2, :2], T[:2, 2] = R, t
        if abs(prev_error - current_error) < 1e-4:
            break
        prev_error = current_error

    fitness = float(np.mean(distances < 0.08)) if len(distances) > 0 else 0.0
    return T, fitness, matched_count
