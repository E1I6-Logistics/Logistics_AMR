"""Unit tests for automatic perturbation sequencing."""

import math

import pytest

from staging_pose_manager.motion import OdomMotionController


def controller_without_ros(settle_time=1.0):
    """Construct a controller shell for testing high-level sequencing."""
    controller = object.__new__(OdomMotionController)
    controller.settle_time = settle_time
    return controller


def test_translation_waits_between_both_motion_legs():
    """Translation must settle after outward and return movement."""
    controller = controller_without_ros()
    events = []
    controller._translate = lambda distance, speed: events.append(
        ("move", distance, speed)
    )
    controller.hold_stopped = lambda: events.append(("settle",))

    controller.perturb_translation(0.5, outward_sign=1.0, speed=0.12)

    assert events == [
        ("move", 0.5, 0.12),
        ("settle",),
        ("move", 0.5, -0.12),
        ("settle",),
    ]


def test_rotation_waits_between_both_motion_legs():
    """Rotation must settle after outward and return movement."""
    controller = controller_without_ros()
    events = []
    controller._rotate = lambda angle, speed: events.append(
        ("rotate", angle, speed)
    )
    controller.hold_stopped = lambda: events.append(("settle",))

    controller.perturb_rotation(math.pi / 3.0, outward_sign=-1.0, speed=0.35)

    assert events[0][0] == "rotate"
    assert events[0][1:] == pytest.approx((math.pi / 3.0, -0.35))
    assert events[1] == ("settle",)
    assert events[2][0] == "rotate"
    assert events[2][1:] == pytest.approx((math.pi / 3.0, 0.35))
    assert events[3] == ("settle",)
