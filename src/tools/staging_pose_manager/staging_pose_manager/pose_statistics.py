"""Pose statistics with correct handling of wrapped yaw angles."""

import math
import statistics


def angle_diff(a, b):
    """Return ``a - b`` normalized to [-pi, pi]."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


def circular_mean(angles):
    """Return the circular mean of a non-empty sequence of angles."""
    angles = list(angles)
    if not angles:
        raise ValueError("at least one angle is required")

    sin_mean = statistics.fmean(math.sin(angle) for angle in angles)
    cos_mean = statistics.fmean(math.cos(angle) for angle in angles)
    if math.hypot(sin_mean, cos_mean) < 1e-12:
        raise ValueError("circular mean is undefined for these angles")
    return math.atan2(sin_mean, cos_mean)


def calculate_statistics(samples):
    """Calculate x/y arithmetic statistics and circular yaw statistics."""
    samples = list(samples)
    if not samples:
        raise ValueError("at least one pose sample is required")

    try:
        xs = [float(sample["x"]) for sample in samples]
        ys = [float(sample["y"]) for sample in samples]
        yaws = [float(sample["yaw"]) for sample in samples]
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("each sample must contain numeric x, y and yaw") from exc

    values = xs + ys + yaws
    if not all(math.isfinite(value) for value in values):
        raise ValueError("pose samples must be finite")

    mean_yaw = circular_mean(yaws)
    yaw_errors = [angle_diff(yaw, mean_yaw) for yaw in yaws]
    stdev = statistics.stdev if len(samples) > 1 else lambda values: 0.0

    return {
        "mean": {
            "x": statistics.fmean(xs),
            "y": statistics.fmean(ys),
            "yaw": mean_yaw,
        },
        "std": {
            "x": stdev(xs),
            "y": stdev(ys),
            "yaw": stdev(yaw_errors),
        },
    }
