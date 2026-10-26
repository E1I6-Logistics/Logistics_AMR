"""Supervised Nav2 parameter sweep with physical measurement input."""

import argparse
import os
import time
from collections import Counter
from pathlib import Path

import rclpy

from ..common.config import load_bundle, merged_snapshot
from ..common.metrics import (
    planar_error_mm,
    signed_delta_mm,
    yaw_error_deg,
)
from ..common.ros_runtime import ExperimentNode
from ..common.storage import (
    RunStore,
    file_sha256,
    git_commit,
    make_run_id,
    now_iso,
    update_runs_index,
)
from ..common.ui import (
    OperatorAbort,
    choice,
    confirm,
    optional_nonnegative_float,
    optional_percentage,
    optional_text,
    wait_for_enter,
)


SCHEMA_VERSION = 1


def ensure_domain(expected_domain):
    """Set an absent ROS domain or reject an accidental mismatch."""
    expected = str(expected_domain)
    existing = os.environ.get("ROS_DOMAIN_ID")
    if existing is None:
        os.environ["ROS_DOMAIN_ID"] = expected
    elif existing != expected:
        raise RuntimeError(
            f"ROS_DOMAIN_ID mismatch: environment={existing}, config={expected}"
        )


def parameter_plan(conditions, fixed_parameters=None):
    """Return all parameters that must be backed up before a sweep."""
    plan = {}
    for node_name, values in (fixed_parameters or {}).items():
        plan.setdefault(node_name, set()).update(values.keys())
    for condition in conditions:
        for node_name, values in condition["parameters"].items():
            plan.setdefault(node_name, set()).update(values.keys())
    return plan


def combined_parameters(fixed_parameters, condition_parameters):
    """Merge fixed and condition parameters without changing either config."""
    combined = {
        node_name: dict(values) for node_name, values in fixed_parameters.items()
    }
    for node_name, values in condition_parameters.items():
        target = combined.setdefault(node_name, {})
        overlap = set(target).intersection(values)
        if overlap:
            raise RuntimeError(
                f"Parameters cannot be both fixed and swept on {node_name}: "
                f"{sorted(overlap)}"
            )
        target.update(values)
    return combined


def condition_sequence(experiment_config):
    """Create a deterministic sequence with per-condition trial numbers."""
    conditions = {
        condition["id"]: condition for condition in experiment_config["conditions"]
    }
    execution = experiment_config.get("execution", {})
    order = list(execution.get("condition_order", []))
    if not order:
        trials = int(experiment_config["experiment"].get("trials_per_condition", 1))
        order = [
            condition_id
            for condition_id in conditions
            for _ in range(trials)
        ]

    counts = Counter()
    sequence = []
    for index, condition_id in enumerate(order, start=1):
        counts[condition_id] += 1
        sequence.append(
            {
                "sequence_index": index,
                "trial": counts[condition_id],
                "condition": conditions[condition_id],
            }
        )
    return sequence


def prefix_pose(row, prefix, amcl=None, transform=None):
    """Add one AMCL and TF sample to a trial row."""
    if amcl is not None:
        row.update(
            {
                f"{prefix}_amcl_x_m": amcl["x_m"],
                f"{prefix}_amcl_y_m": amcl["y_m"],
                f"{prefix}_amcl_yaw_rad": amcl["yaw_rad"],
                f"{prefix}_amcl_cov_x": amcl["cov_x"],
                f"{prefix}_amcl_cov_y": amcl["cov_y"],
                f"{prefix}_amcl_cov_yaw": amcl["cov_yaw"],
                f"{prefix}_amcl_stamp_sec": amcl["stamp_sec"],
            }
        )
    if transform is not None:
        row.update(
            {
                f"{prefix}_tf_x_m": transform["x_m"],
                f"{prefix}_tf_y_m": transform["y_m"],
                f"{prefix}_tf_yaw_rad": transform["yaw_rad"],
            }
        )


def flatten_observed_parameters(row, observed):
    """Copy known experimental parameters into unit-explicit CSV columns."""
    controller = observed.get("/controller_server", {})
    mapping = {
        "goal_checker.xy_goal_tolerance": "xy_goal_tolerance_m",
        "goal_checker.yaw_goal_tolerance": "yaw_goal_tolerance_rad",
        "FollowPath.max_vel_x": "max_vel_x_mps",
        "FollowPath.max_speed_xy": "max_speed_xy_mps",
        "FollowPath.xy_goal_tolerance": "follow_path_xy_goal_tolerance_m",
        "FollowPath.trans_stopped_velocity": "trans_stopped_velocity_mps",
    }
    for parameter_name, field_name in mapping.items():
        if parameter_name in controller:
            row[field_name] = controller[parameter_name]


def runtime_parameter_snapshot(node, experiment_config, modified_plan):
    """Capture modified and explicitly requested parameters."""
    requested = {
        name: set(values) for name, values in modified_plan.items()
    }
    for node_name, names in experiment_config.get("record_parameters", {}).items():
        requested.setdefault(node_name, set()).update(names)
    snapshot = {}
    for node_name, names in requested.items():
        normalized = node_name if node_name.startswith("/") else f"/{node_name}"
        snapshot[normalized] = node.parameters.get(normalized, sorted(names))
    return snapshot


def base_trial_row(context, item, start_pose, goal_pose):
    """Create the immutable portion of one trial record."""
    condition = item["condition"]
    return {
        "schema_version": SCHEMA_VERSION,
        "run_id": context["run_id"],
        "sequence_index": item["sequence_index"],
        "timestamp_start": now_iso(),
        "robot_id": context["robot_name"],
        "ros_domain_id": context["ros_domain_id"],
        "experiment_name": context["experiment_name"],
        "scenario_id": context["scenario_id"],
        "condition_id": condition["id"],
        "condition_label": condition.get("label", condition["id"]),
        "trial": item["trial"],
        "status": "RUNNING",
        "start_expected_x_m": start_pose["x"],
        "start_expected_y_m": start_pose["y"],
        "start_expected_yaw_rad": start_pose["yaw"],
        "goal_x_m": goal_pose["x"],
        "goal_y_m": goal_pose["y"],
        "goal_yaw_rad": goal_pose["yaw"],
    }


def collect_manual_result(row, measurement):
    """Collect only observations that the ROS graph cannot know."""
    row["physical_position_error_mm"] = optional_nonnegative_float(
        "물리 GOAL 중심과 base_footprint 기준점 사이 직선거리 [mm]"
    )
    row["stop_direction"] = choice(
        "목표 yaw 방향을 바라봤을 때 정지 방향",
        ("left", "right", "front", "back", "center", "unknown"),
        default="unknown",
    )
    row["overshoot_observed"] = confirm("Overshoot가 관찰됐습니까?", default=False)
    row["controller_oscillation"] = confirm(
        "제어 진동 또는 반복 재회전이 있었습니까?", default=False
    )
    row["localization_jump_observed"] = confirm(
        "RViz에서 localization jump가 관찰됐습니까?", default=False
    )
    row["external_intervention"] = confirm(
        "주행 중 사람·장애물 등 외부 개입이 있었습니까?", default=False
    )
    row["battery_percent"] = optional_percentage("현재 배터리 잔량 [%]")
    row["measurement_method"] = measurement.get("method", "steel_ruler")
    row["measurement_resolution_mm"] = measurement.get("resolution_mm", 1)
    row["photo_refs"] = optional_text("관련 사진 파일명")
    row["notes"] = optional_text("특이사항")


def run_experiment(args):
    """Run one configured navigation sweep."""
    bundle = load_bundle(args.robot_config, args.scenario_config, args.experiment_config)
    robot_config = bundle["robot"]
    scenario_config = bundle["scenario"]
    experiment_config = bundle["experiment"]
    robot = robot_config["robot"]
    scenario = scenario_config["scenario"]
    experiment = experiment_config["experiment"]
    ensure_domain(robot["ros_domain_id"])
    if args.resume and not args.run_id:
        raise ValueError("--resume requires --run-id")

    rclpy.init()
    node = ExperimentNode(robot_config)
    store = None
    metadata = None
    restore_errors = []
    run_status = "FAILED"

    try:
        node.wait_for_amcl_publisher(timeout_sec=10.0)
        node.wait_for_initialpose_subscriber(timeout_sec=10.0)
        node.wait_for_navigation()
        fixed_parameters = experiment_config.get("fixed_parameters", {})
        plan = parameter_plan(experiment_config["conditions"], fixed_parameters)
        original = node.parameters.backup(plan)
        runtime_before = runtime_parameter_snapshot(node, experiment_config, plan)

        if args.check_only:
            print("Preflight OK")
            print(f"Robot: {robot['name']} / ROS_DOMAIN_ID={robot['ros_domain_id']}")
            print(f"Experiment: {experiment['name']}")
            print(f"Conditions: {len(experiment_config['conditions'])}")
            return None

        run_id = args.run_id or make_run_id(robot["name"], experiment["name"])
        store = RunStore(args.output_root, run_id, resume=args.resume)
        store.open_trials()
        context = {
            "run_id": run_id,
            "robot_name": robot["name"],
            "ros_domain_id": robot["ros_domain_id"],
            "experiment_name": experiment["name"],
            "scenario_id": scenario["id"],
        }
        repo_path = Path(args.repo_path).expanduser().resolve()
        metadata = {
            "schema_version": SCHEMA_VERSION,
            "run_id": run_id,
            "status": "RUNNING",
            "started_at": now_iso(),
            "finished_at": None,
            "git_commit": git_commit(repo_path),
            "repository": str(repo_path),
            "robot": robot,
            "scenario_id": scenario["id"],
            "experiment": experiment,
            "runtime_parameters_before": runtime_before,
            "original_modified_parameters": original,
            "restore_errors": [],
            "nav_launch_command": args.nav_launch_command,
            "params_file": args.params_file,
            "params_file_sha256": file_sha256(args.params_file)
            if args.params_file
            else None,
            "map_file": args.map_file,
            "map_file_sha256": file_sha256(args.map_file)
            if args.map_file
            else None,
        }
        store.write_yaml("metadata.yaml", metadata)
        store.write_yaml("config_snapshot.yaml", merged_snapshot(bundle))
        store.append_event("run_started")

        start_pose = scenario["start"]
        goal_pose = scenario["goal"]
        initialization = experiment_config.get("initial_localization", {})
        measurement = experiment_config.get("measurement", {})
        execution = experiment_config.get("execution", {})
        settle_sec = float(execution.get("success_wait_sec", 3.0))
        capture_trace = bool(execution.get("capture_trace", False))
        completed = store.completed_keys()

        for item in condition_sequence(experiment_config):
            key = (
                item["sequence_index"],
                item["condition"]["id"],
                item["trial"],
            )
            if key in completed:
                print(f"Skipping completed trial: {key}")
                continue

            row = base_trial_row(context, item, start_pose, goal_pose)
            condition = item["condition"]
            print("\n" + "=" * 64)
            print(
                f"Sequence {item['sequence_index']} / "
                f"{condition.get('label', condition['id'])} / trial {item['trial']}"
            )
            print("=" * 64)

            try:
                parameters = combined_parameters(
                    fixed_parameters, condition["parameters"]
                )
                observed = node.parameters.apply_condition(parameters)
                fixed = runtime_parameter_snapshot(node, experiment_config, plan)
                for node_name, values in fixed.items():
                    observed.setdefault(node_name, {}).update(values)
                flatten_observed_parameters(row, observed)

                wait_for_enter(
                    "로봇을 물리 START 중심과 방향선에 맞추고 완전히 정지시키세요."
                )
                initial_message = node.publish_initial_pose(
                    start_pose,
                    initialization.get("position_stddev_m", 0.05),
                    initialization.get("yaw_stddev_deg", 5.0),
                )
                node.spin_for(float(initialization.get("settle_sec", 1.0)))
                if not confirm("RViz scan-map 정합이 정상입니까?", default=True):
                    row.update(
                        {
                            "status": "INVALID_START",
                            "scan_alignment_ok": False,
                            "failure_reason": "operator_rejected_scan_alignment",
                            "timestamp_end": now_iso(),
                        }
                    )
                    store.append_event(
                        "trial_invalid_start", sequence_index=item["sequence_index"]
                    )
                    continue
                row["scan_alignment_ok"] = True

                start_amcl = node.amcl_sample(node.latest_amcl or initial_message)
                start_tf = node.tf_sample()
                prefix_pose(row, "start", start_amcl, start_tf)

                wait_for_enter("GOAL 주행을 시작할 준비가 됐으면 확인하세요.")
                node.reset_motion_metrics()
                if capture_trace:
                    node.begin_trace(store, run_id, item["sequence_index"])
                nav_result = node.navigate(goal_pose)
                node.spin_for(settle_sec)
                node.end_trace()

                end_message = node.wait_for_amcl(timeout_sec=5.0)
                end_amcl = node.amcl_sample(end_message)
                end_tf = node.tf_sample()
                prefix_pose(row, "end", end_amcl, end_tf)

                row.update(
                    {
                        "nav_result": nav_result["status"],
                        "navigation_time_sec": nav_result["time_sec"],
                        "observed_cmd_vel_max_x_mps": node.cmd_vel_max_x,
                        "estimated_position_error_mm": planar_error_mm(
                            goal_pose["x"],
                            goal_pose["y"],
                            end_amcl["x_m"],
                            end_amcl["y_m"],
                        ),
                        "estimated_dx_mm": signed_delta_mm(
                            goal_pose["x"], end_amcl["x_m"]
                        ),
                        "estimated_dy_mm": signed_delta_mm(
                            goal_pose["y"], end_amcl["y_m"]
                        ),
                        "estimated_yaw_error_deg": yaw_error_deg(
                            goal_pose["yaw"], end_amcl["yaw_rad"]
                        ),
                    }
                )
                collect_manual_result(row, measurement)
                row["status"] = (
                    "COMPLETED" if nav_result["status"] == "SUCCEEDED" else "FAILED"
                )
            except OperatorAbort:
                row["status"] = "INTERRUPTED"
                row["failure_reason"] = "operator_abort"
                raise
            except KeyboardInterrupt:
                row["status"] = "INTERRUPTED"
                row["failure_reason"] = "keyboard_interrupt"
                raise
            except Exception as error:
                row["status"] = "ERROR"
                row["failure_reason"] = str(error)
                store.append_event(
                    "trial_error",
                    sequence_index=item["sequence_index"],
                    error=str(error),
                )
                if not confirm(f"Trial 오류: {error}\n다음 trial로 계속할까요?", default=False):
                    raise
            finally:
                node.end_trace()
                if row["status"] != "RUNNING":
                    row["timestamp_end"] = now_iso()
                    store.append_trial(row)

        run_status = "COMPLETED"
        store.append_event("run_completed")
        return store.run_dir
    except (OperatorAbort, KeyboardInterrupt):
        run_status = "INTERRUPTED"
        if store is not None:
            store.append_event("run_interrupted")
        return None
    finally:
        if node.parameters.original:
            restore_errors = node.parameters.restore()
        if store is not None:
            if metadata is not None:
                metadata["status"] = run_status
                metadata["finished_at"] = now_iso()
                metadata["restore_errors"] = restore_errors
                store.write_yaml("metadata.yaml", metadata)
                update_runs_index(
                    args.output_root, metadata, store.trials_path, store.run_dir
                )
            for error in restore_errors:
                store.append_event("parameter_restore_error", error=error)
            store.close()
        node.destroy_node()
        rclpy.shutdown()


def parse_args():
    """Parse command-line paths and safety options."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-config", required=True)
    parser.add_argument("--scenario-config", required=True)
    parser.add_argument("--experiment-config", required=True)
    parser.add_argument(
        "--output-root",
        default=str(Path.cwd() / "experiment_data"),
    )
    parser.add_argument("--repo-path", default=str(Path.cwd()))
    parser.add_argument("--run-id")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--check-only", action="store_true")
    parser.add_argument("--nav-launch-command", default="")
    parser.add_argument("--params-file", default="")
    parser.add_argument("--map-file", default="")
    return parser.parse_args()


def main():
    """Console entry point."""
    args = parse_args()
    run_dir = run_experiment(args)
    if run_dir is not None:
        print(f"Run data: {run_dir}")
        try:
            from ..analysis.report import build_report

            summary_path, figures = build_report(run_dir)
            print(f"Summary: {summary_path}")
            print(f"Figures: {len(figures)}")
        except Exception as error:
            print(f"Analysis failed; raw data is preserved: {error}")


if __name__ == "__main__":
    main()
