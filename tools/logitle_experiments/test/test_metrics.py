"""Metric calculation tests."""

import math

from logitle_experiments.common.metrics import (
    normalize_angle,
    planar_error_mm,
    signed_delta_mm,
    yaw_error_deg,
)


def test_planar_error_uses_millimetres():
    """Convert planar error from metres to millimetres."""
    assert planar_error_mm(0.0, 0.0, 0.3, 0.4) == 500.0


def test_signed_delta_preserves_direction():
    """Keep the sign when converting a coordinate delta."""
    assert math.isclose(signed_delta_mm(1.0, 0.9), -100.0)


def test_angle_normalization_and_error():
    """Normalize wrapped yaw differences before conversion."""
    assert math.isclose(normalize_angle(3.0 * math.pi), math.pi)
    assert math.isclose(yaw_error_deg(0.0, math.pi / 2.0), 90.0)
