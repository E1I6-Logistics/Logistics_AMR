"""Pure geometry and validation helpers for precision docking.

This module intentionally has no ROS imports so the safety-critical math can be
unit tested on development machines that do not have ROS 2 installed.
"""

from dataclasses import dataclass
import math

import numpy as np


def normalize_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def make_v_funnel(
    charger_width: float,
    wing_length: float,
    wing_angle_deg: float,
    spacing: float = 0.005,
) -> np.ndarray:
    """Return the measured charger face and two symmetric wings."""
    if charger_width <= 0.0 or wing_length <= 0.0 or spacing <= 0.0:
        raise ValueError('charger dimensions and spacing must be positive')
    if not 0.0 < wing_angle_deg < 90.0:
        raise ValueError('wing_angle_deg must be between 0 and 90 degrees')

    half_width = charger_width / 2.0
    wing_angle = math.radians(wing_angle_deg)
    points = [[0.0, y] for y in np.arange(
        -half_width, half_width + spacing * 0.5, spacing
    )]
    for length in np.arange(spacing, wing_length + spacing * 0.5, spacing):
        x = length * math.cos(wing_angle)
        y = half_width + length * math.sin(wing_angle)
        points.extend(((x, y), (x, -y)))
    return np.asarray(points, dtype=np.float32)


def staging_pose(
    dock_x: float,
    dock_y: float,
    dock_yaw: float,
    staging_distance: float,
) -> tuple[float, float, float]:
    """Compute the map-frame waypoint in front of the dock."""
    if staging_distance <= 0.0:
        raise ValueError('staging_distance must be positive')
    return (
        dock_x + staging_distance * math.cos(dock_yaw),
        dock_y + staging_distance * math.sin(dock_yaw),
        normalize_angle(dock_yaw),
    )


def relative_pose(
    origin_x: float,
    origin_y: float,
    origin_yaw: float,
    target_x: float,
    target_y: float,
    target_yaw: float,
) -> tuple[float, float, float]:
    """Express a map-frame target pose in the origin/base frame."""
    dx = target_x - origin_x
    dy = target_y - origin_y
    c = math.cos(origin_yaw)
    s = math.sin(origin_yaw)
    return (
        c * dx + s * dy,
        -s * dx + c * dy,
        normalize_angle(target_yaw - origin_yaw),
    )


def model_transform_from_relative_pose(
    relative_x: float,
    relative_y: float,
    relative_yaw: float,
) -> np.ndarray:
    """Create the base/scan-to-dock-model transform used by ICP."""
    c = math.cos(relative_yaw)
    s = math.sin(relative_yaw)
    # Rotate base points by -relative_yaw into the dock model frame.
    rotation = np.asarray(((c, s), (-s, c)), dtype=np.float32)
    translation = -(rotation @ np.asarray(
        (relative_x, relative_y), dtype=np.float32
    ))
    transform = np.identity(3, dtype=np.float32)
    transform[:2, :2] = rotation
    transform[:2, 2] = translation
    return transform


@dataclass(frozen=True)
class IcpResult:
    """Quality metrics and transform produced by a 2-D ICP pass."""

    transform: np.ndarray
    fitness: float
    rmse: float
    matched_count: int


def _nearest_neighbors(source: np.ndarray, target: np.ndarray):
    """Return nearest target distance/index for each source point."""
    squared = np.sum(
        np.square(source[:, np.newaxis, :] - target[np.newaxis, :, :]),
        axis=2,
    )
    indices = np.argmin(squared, axis=1)
    distances = np.sqrt(squared[np.arange(len(source)), indices])
    return distances, indices


def icp_2d(
    source_points: np.ndarray,
    target_points: np.ndarray,
    initial_transform: np.ndarray,
    max_iterations: int,
    search_radius: float,
    inlier_distance: float,
) -> IcpResult:
    """Run point-to-point ICP and return explicit safety metrics."""
    source = np.asarray(source_points, dtype=np.float32)
    target = np.asarray(target_points, dtype=np.float32)
    transform = np.asarray(initial_transform, dtype=np.float32).copy()
    if source.ndim != 2 or source.shape[1] != 2 or len(source) == 0:
        return IcpResult(transform, 0.0, float('inf'), 0)
    if target.ndim != 2 or target.shape[1] != 2 or len(target) == 0:
        return IcpResult(transform, 0.0, float('inf'), 0)

    previous_error = float('inf')
    for _ in range(max_iterations):
        transformed = (
            source @ transform[:2, :2].T
        ) + transform[:2, 2]
        distances, indices = _nearest_neighbors(transformed, target)
        valid = distances < search_radius
        if int(np.sum(valid)) < 3:
            break

        source_matches = source[valid]
        target_matches = target[indices[valid]]
        source_centroid = np.mean(source_matches, axis=0)
        target_centroid = np.mean(target_matches, axis=0)
        covariance = (
            source_matches - source_centroid
        ).T @ (target_matches - target_centroid)
        u, _, vt = np.linalg.svd(covariance)
        rotation = vt.T @ u.T
        if np.linalg.det(rotation) < 0:
            vt[1, :] *= -1
            rotation = vt.T @ u.T
        translation = target_centroid - rotation @ source_centroid

        error = float(
            np.sum(np.abs(transform[:2, :2] - rotation))
            + np.sum(np.abs(transform[:2, 2] - translation))
        )
        transform[:2, :2] = rotation
        transform[:2, 2] = translation
        if abs(previous_error - error) < 1e-5:
            break
        previous_error = error

    transformed = (source @ transform[:2, :2].T) + transform[:2, 2]
    distances, _ = _nearest_neighbors(transformed, target)
    inliers = distances < inlier_distance
    matched_count = int(np.sum(inliers))
    fitness = float(matched_count / len(source))
    rmse = (
        float(np.sqrt(np.mean(np.square(distances[inliers]))))
        if matched_count else float('inf')
    )
    return IcpResult(transform, fitness, rmse, matched_count)


def icp_is_safe(
    result: IcpResult,
    expected_transform: np.ndarray,
    min_matches: int,
    min_fitness: float,
    max_rmse: float,
    max_distance_jump: float,
    max_lateral_jump: float,
    max_yaw_jump_deg: float,
) -> bool:
    """Fail closed unless ICP quality and map/previous-pose agreement pass."""
    result_yaw = math.atan2(
        -result.transform[1, 0], result.transform[0, 0]
    )
    expected_yaw = math.atan2(
        -expected_transform[1, 0], expected_transform[0, 0]
    )
    return (
        result.matched_count >= min_matches
        and result.fitness >= min_fitness
        and math.isfinite(result.rmse)
        and result.rmse <= max_rmse
        and abs(
            float(result.transform[0, 2] - expected_transform[0, 2])
        ) <= max_distance_jump
        and abs(
            float(result.transform[1, 2] - expected_transform[1, 2])
        ) <= max_lateral_jump
        and abs(normalize_angle(result_yaw - expected_yaw)) <= math.radians(
            max_yaw_jump_deg
        )
    )
