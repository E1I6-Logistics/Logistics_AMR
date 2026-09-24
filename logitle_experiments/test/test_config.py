"""Configuration validation tests."""

from pathlib import Path

from logitle_experiments.common.config import load_bundle


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_navigation_configs_are_valid():
    """Load the turtle2 navigation sweep configuration."""
    bundle = load_bundle(
        PACKAGE_ROOT / "config/robots/turtle2.yaml",
        PACKAGE_ROOT / "config/scenarios/straight_goal.yaml",
        PACKAGE_ROOT / "config/experiments/xy_tolerance_sweep.yaml",
    )
    assert bundle["robot"]["robot"]["ros_domain_id"] == 31
    assert len(bundle["experiment"]["conditions"]) == 3


def test_amcl_config_is_valid():
    """Load the turtle2 AMCL validation configuration."""
    bundle = load_bundle(
        PACKAGE_ROOT / "config/robots/turtle2.yaml",
        PACKAGE_ROOT / "config/scenarios/goal_alignment.yaml",
        PACKAGE_ROOT / "config/experiments/amcl_validation.yaml",
    )
    assert bundle["experiment"]["experiment"]["type"] == "amcl_validation"
    assert list(bundle["scenario"]["scenario"]["reference_points"]) == ["goal"]
