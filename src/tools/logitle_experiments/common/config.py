"""Configuration loading and validation."""

from copy import deepcopy
from pathlib import Path

import yaml


class ConfigError(ValueError):
    """Raised when an experiment configuration is incomplete or inconsistent."""


def load_yaml(path):
    """Load one YAML mapping."""
    config_path = Path(path).expanduser().resolve()
    with config_path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ConfigError(f"Expected a YAML mapping: {config_path}")
    return data


def load_bundle(robot_path, scenario_path, experiment_path):
    """Load and validate robot, scenario, and experiment configurations."""
    bundle = {
        "robot": load_yaml(robot_path),
        "scenario": load_yaml(scenario_path),
        "experiment": load_yaml(experiment_path),
    }
    validate_bundle(bundle)
    return bundle


def validate_bundle(bundle):
    """Validate fields shared by every experiment runner."""
    robot = bundle["robot"].get("robot", {})
    scenario = bundle["scenario"].get("scenario", {})
    experiment = bundle["experiment"].get("experiment", {})

    for key in ("name", "ros_domain_id"):
        if key not in robot:
            raise ConfigError(f"robot.{key} is required")

    if not scenario.get("id"):
        raise ConfigError("scenario.id is required")
    if not experiment.get("name") or not experiment.get("type"):
        raise ConfigError("experiment.name and experiment.type are required")

    expected_type = experiment["type"]
    if expected_type == "navigation_sweep":
        _validate_navigation(bundle)
    elif expected_type == "amcl_validation":
        _validate_amcl(bundle)
    else:
        raise ConfigError(f"Unsupported experiment.type: {expected_type}")


def _validate_pose(pose, name):
    for key in ("x", "y", "yaw"):
        if key not in pose:
            raise ConfigError(f"{name}.{key} is required")
        float(pose[key])


def _validate_navigation(bundle):
    scenario = bundle["scenario"]["scenario"]
    _validate_pose(scenario.get("start", {}), "scenario.start")
    _validate_pose(scenario.get("goal", {}), "scenario.goal")

    experiment = bundle["experiment"]
    conditions = experiment.get("conditions", [])
    if not conditions:
        raise ConfigError("At least one experiment condition is required")

    condition_ids = set()
    for condition in conditions:
        condition_id = condition.get("id")
        if not condition_id or condition_id in condition_ids:
            raise ConfigError("Condition IDs must be non-empty and unique")
        condition_ids.add(condition_id)
        if not condition.get("parameters"):
            raise ConfigError(f"Condition {condition_id} has no parameters")

    order = experiment.get("execution", {}).get("condition_order", [])
    unknown = [item for item in order if item not in condition_ids]
    if unknown:
        raise ConfigError(f"Unknown condition IDs in condition_order: {unknown}")


def _validate_amcl(bundle):
    references = bundle["scenario"]["scenario"].get("reference_points", {})
    if not references:
        raise ConfigError("AMCL validation requires scenario.reference_points")
    for name, pose in references.items():
        _validate_pose(pose, f"scenario.reference_points.{name}")


def merged_snapshot(bundle):
    """Return a detached configuration snapshot safe to serialize."""
    return deepcopy(bundle)
