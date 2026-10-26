"""Supervised AMCL reference alignment and stationary stability test."""

import argparse
import os
import time
from pathlib import Path

import rclpy

from ..common.config import load_bundle, merged_snapshot
from ..common.metrics import planar_error_mm, signed_delta_mm, yaw_error_deg
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


def add_pose(row, prefix, amcl, transform):
    """Add a flattened AMCL and TF sample."""
    row.update(
        {
            f"{prefix}_amcl_x_m": amcl["x_m"],
            f"{prefix}_amcl_y_m": amcl["y_m"],
            f"{prefix}_amcl_yaw_rad": amcl["yaw_rad"],
            f"{prefix}_amcl_cov_x": amcl["cov_x"],
            f"{prefix}_amcl_cov_y": amcl["cov_y"],
            f"{prefix}_amcl_cov_yaw": amcl["cov_yaw"],
            f"{prefix}_amcl_stamp_sec": amcl["stamp_sec"],
            f"{prefix}_tf_x_m": transform["x_m"],
            f"{prefix}_tf_y_m": transform["y_m"],
            f"{prefix}_tf_yaw_rad": transform["yaw_rad"],
        }
    )


def stationary_samples(node, store, run_id, sequence_index, duration_sec, interval_sec):
    """Record periodic snapshots, including repeated source timestamps."""
    started = time.monotonic()
    count = max(1, int(duration_sec / interval_sec))
    for _ in range(count):
        node.spin_for(interval_sec)
        if node.latest_amcl is None:
            continue
        sample = node.amcl_sample(node.latest_amcl)
        store.append_trace(
            {
                "schema_version": SCHEMA_VERSION,
                "run_id": run_id,
                "sequence_index": sequence_index,
                "trial_elapsed_sec": time.monotonic() - started,
                "source": "amcl_snapshot",
                "source_stamp_sec": sample["stamp_sec"],
                "amcl_x_m": sample["x_m"],
                "amcl_y_m": sample["y_m"],
                "amcl_yaw_rad": sample["yaw_rad"],
                "amcl_cov_x": sample["cov_x"],
                "amcl_cov_y": sample["cov_y"],
                "amcl_cov_yaw": sample["cov_yaw"],
            }
        )


def run_validation(args):
    """Run reference-point alignment and stationary stability checks."""
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
    run_status = "FAILED"

    try:
        node.wait_for_amcl_publisher(timeout_sec=10.0)
        node.wait_for_initialpose_subscriber(timeout_sec=10.0)
        if args.check_only:
            print("AMCL validation preflight OK")
            return None

        run_id = args.run_id or make_run_id(robot["name"], experiment["name"])
        store = RunStore(args.output_root, run_id, resume=args.resume)
        store.open_trials()
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

        initialization = experiment_config.get("initial_localization", {})
        execution = experiment_config.get("execution", {})
        references = scenario["reference_points"]
        repeats = int(experiment.get("trials_per_reference", 3))
        stationary_sec = float(execution.get("stationary_duration_sec", 30.0))
        interval_sec = float(execution.get("sample_interval_sec", 1.0))
        completed = store.completed_keys()
        sequence_index = 0

        for reference_name, reference_pose in references.items():
            for trial in range(1, repeats + 1):
                sequence_index += 1
                key = (
                    sequence_index,
                    f"{reference_name}_alignment",
                    trial,
                )
                if key in completed:
                    print(f"Skipping completed trial: {key}")
                    continue
                row = {
                    "schema_version": SCHEMA_VERSION,
                    "run_id": run_id,
                    "sequence_index": sequence_index,
                    "timestamp_start": now_iso(),
                    "robot_id": robot["name"],
                    "ros_domain_id": robot["ros_domain_id"],
                    "experiment_name": experiment["name"],
                    "scenario_id": scenario["id"],
                    "condition_id": f"{reference_name}_alignment",
                    "condition_label": f"{reference_name} alignment",
                    "trial": trial,
                    "status": "RUNNING",
                    "start_expected_x_m": reference_pose["x"],
                    "start_expected_y_m": reference_pose["y"],
                    "start_expected_yaw_rad": reference_pose["yaw"],
                    "goal_x_m": reference_pose["x"],
                    "goal_y_m": reference_pose["y"],
                    "goal_yaw_rad": reference_pose["yaw"],
                    "nav_result": "NOT_APPLICABLE",
                }

                print("\n" + "=" * 64)
                print(f"{reference_name.upper()} reference / trial {trial}")
                print("=" * 64)
                try:
                    wait_for_enter(
                        f"로봇 기준점을 물리 {reference_name.upper()} 중심에 놓고 "
                        "목표 방향선에 맞추세요."
                    )
                    initial_message = node.publish_initial_pose(
                        reference_pose,
                        initialization.get("position_stddev_m", 0.05),
                        initialization.get("yaw_stddev_deg", 5.0),
                    )
                    node.spin_for(float(initialization.get("settle_sec", 1.0)))
                    aligned = confirm("RViz scan-map 정합이 정상입니까?", default=True)
                    row["scan_alignment_ok"] = aligned
                    if not aligned:
                        row["status"] = "INVALID_ALIGNMENT"
                        row["failure_reason"] = "operator_rejected_scan_alignment"
                        row["stop_direction"] = choice(
                            "scan 또는 footprint가 어긋난 방향",
                            ("left", "right", "front", "back", "unknown"),
                            default="unknown",
                        )
                        row["notes"] = optional_text("정합 특이사항")
                        continue

                    initial_amcl = node.amcl_sample(initial_message)
                    initial_tf = node.tf_sample()
                    add_pose(row, "start", initial_amcl, initial_tf)

                    before_motion_count = node.amcl_count
                    wait_for_enter(
                        "로봇을 제자리에서 0.2 rad보다 크게 천천히 회전한 뒤 "
                        "원래 방향선으로 돌아오세요."
                    )
                    motion_message = node.wait_for_amcl(
                        timeout_sec=10.0, after_count=before_motion_count
                    )
                    node.spin_for(float(execution.get("post_motion_settle_sec", 3.0)))
                    end_amcl = node.amcl_sample(node.latest_amcl or motion_message)
                    end_tf = node.tf_sample()
                    add_pose(row, "end", end_amcl, end_tf)

                    stationary_samples(
                        node,
                        store,
                        run_id,
                        sequence_index,
                        stationary_sec,
                        interval_sec,
                    )
                    row.update(
                        {
                            "estimated_position_error_mm": planar_error_mm(
                                reference_pose["x"],
                                reference_pose["y"],
                                end_amcl["x_m"],
                                end_amcl["y_m"],
                            ),
                            "estimated_dx_mm": signed_delta_mm(
                                reference_pose["x"], end_amcl["x_m"]
                            ),
                            "estimated_dy_mm": signed_delta_mm(
                                reference_pose["y"], end_amcl["y_m"]
                            ),
                            "estimated_yaw_error_deg": yaw_error_deg(
                                reference_pose["yaw"], end_amcl["yaw_rad"]
                            ),
                            "status": "COMPLETED",
                        }
                    )
                    row["stop_direction"] = choice(
                        "반복 정합에서 관찰된 편향 방향",
                        ("left", "right", "front", "back", "center", "unknown"),
                        default="unknown",
                    )
                    row["localization_jump_observed"] = confirm(
                        "관찰 중 localization jump가 있었습니까?", default=False
                    )
                    row["battery_percent"] = optional_percentage(
                        "현재 배터리 잔량 [%]"
                    )
                    row["photo_refs"] = optional_text("관련 사진 파일명")
                    row["notes"] = optional_text("특이사항")
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
                        "trial_error", sequence_index=sequence_index, error=str(error)
                    )
                    if not confirm(f"Trial 오류: {error}\n계속할까요?", default=False):
                        raise
                finally:
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
        if store is not None:
            if metadata is not None:
                metadata["status"] = run_status
                metadata["finished_at"] = now_iso()
                store.write_yaml("metadata.yaml", metadata)
                update_runs_index(
                    args.output_root, metadata, store.trials_path, store.run_dir
                )
            store.close()
        node.destroy_node()
        rclpy.shutdown()


def parse_args():
    """Parse configuration and output paths."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-config", required=True)
    parser.add_argument("--scenario-config", required=True)
    parser.add_argument("--experiment-config", required=True)
    parser.add_argument("--output-root", default=str(Path.cwd() / "experiment_data"))
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
    run_dir = run_validation(parse_args())
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
