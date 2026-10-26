"""Unit tests for wrapped pose statistics."""

import math

import pytest

from staging_pose_manager.pose_statistics import calculate_statistics


def test_yaw_mean_wraps_across_pi():
    samples = [
        {"x": 1.0, "y": 2.0, "yaw": math.radians(179.0)},
        {"x": 3.0, "y": 4.0, "yaw": math.radians(-179.0)},
    ]

    result = calculate_statistics(samples)

    assert result["mean"]["x"] == 2.0
    assert abs(abs(result["mean"]["yaw"]) - math.pi) < 1e-9
    assert math.degrees(result["std"]["yaw"]) == pytest.approx(math.sqrt(2.0))


def test_empty_samples_are_rejected():
    with pytest.raises(ValueError, match="at least one"):
        calculate_statistics([])


def test_non_finite_samples_are_rejected():
    with pytest.raises(ValueError, match="finite"):
        calculate_statistics([{"x": math.nan, "y": 0.0, "yaw": 0.0}])
