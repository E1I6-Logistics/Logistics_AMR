"""Unit tests for TF sample collection behavior."""

from staging_pose_manager import register_pose


class FakePoseNode:
    """Return a predefined sequence of TF poses."""

    def __init__(self, samples):
        self.samples = iter(samples)

    def get_pose(self):
        return next(self.samples)


def pose(stamp, x):
    """Build a minimal pose sample."""
    return {"stamp": stamp, "x": x, "y": 0.0, "yaw": 0.0}


def test_collect_samples_ignores_duplicate_tf_stamps(monkeypatch):
    node = FakePoseNode([
        pose("1.000000000", 1.0),
        pose("1.000000000", 99.0),
        pose("1.100000000", 2.0),
        pose("1.200000000", 3.0),
    ])
    monkeypatch.setattr(register_pose.rclpy, "ok", lambda: True)
    monkeypatch.setattr(register_pose.rclpy, "spin_once", lambda *args, **kwargs: None)

    samples, statistics = register_pose.collect_samples(node, 3, 0.000001)

    assert [sample["stamp"] for sample in samples] == [
        "1.000000000",
        "1.100000000",
        "1.200000000",
    ]
    assert statistics["mean"]["x"] == 2.0
