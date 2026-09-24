"""Crash-resistant run directory and CSV storage."""

import csv
import hashlib
import json
import subprocess
from datetime import datetime
from pathlib import Path

import yaml


TRIAL_FIELDS = [
    "schema_version",
    "run_id",
    "sequence_index",
    "timestamp_start",
    "timestamp_end",
    "robot_id",
    "ros_domain_id",
    "experiment_name",
    "scenario_id",
    "condition_id",
    "condition_label",
    "trial",
    "status",
    "failure_reason",
    "xy_goal_tolerance_m",
    "yaw_goal_tolerance_rad",
    "max_vel_x_mps",
    "max_speed_xy_mps",
    "follow_path_xy_goal_tolerance_m",
    "trans_stopped_velocity_mps",
    "start_expected_x_m",
    "start_expected_y_m",
    "start_expected_yaw_rad",
    "start_amcl_x_m",
    "start_amcl_y_m",
    "start_amcl_yaw_rad",
    "start_amcl_cov_x",
    "start_amcl_cov_y",
    "start_amcl_cov_yaw",
    "start_amcl_stamp_sec",
    "start_tf_x_m",
    "start_tf_y_m",
    "start_tf_yaw_rad",
    "goal_x_m",
    "goal_y_m",
    "goal_yaw_rad",
    "nav_result",
    "navigation_time_sec",
    "observed_cmd_vel_max_x_mps",
    "end_amcl_x_m",
    "end_amcl_y_m",
    "end_amcl_yaw_rad",
    "end_amcl_cov_x",
    "end_amcl_cov_y",
    "end_amcl_cov_yaw",
    "end_amcl_stamp_sec",
    "end_tf_x_m",
    "end_tf_y_m",
    "end_tf_yaw_rad",
    "estimated_position_error_mm",
    "estimated_dx_mm",
    "estimated_dy_mm",
    "estimated_yaw_error_deg",
    "physical_position_error_mm",
    "stop_direction",
    "scan_alignment_ok",
    "overshoot_observed",
    "controller_oscillation",
    "localization_jump_observed",
    "external_intervention",
    "battery_percent",
    "measurement_method",
    "measurement_resolution_mm",
    "photo_refs",
    "notes",
]


TRACE_FIELDS = [
    "schema_version",
    "run_id",
    "sequence_index",
    "trial_elapsed_sec",
    "source",
    "source_stamp_sec",
    "amcl_x_m",
    "amcl_y_m",
    "amcl_yaw_rad",
    "amcl_cov_x",
    "amcl_cov_y",
    "amcl_cov_yaw",
    "cmd_vel_x_mps",
    "cmd_vel_yaw_rps",
]


RUN_INDEX_FIELDS = [
    "run_id",
    "started_at",
    "finished_at",
    "robot_id",
    "ros_domain_id",
    "experiment_name",
    "scenario_id",
    "status",
    "trial_rows",
    "completed_rows",
    "nav_success_count",
    "git_commit",
    "result_directory",
]


def now_iso():
    """Return a local ISO timestamp with seconds."""
    return datetime.now().astimezone().isoformat(timespec="seconds")


def file_sha256(path):
    """Return a SHA-256 hash for one file, or None when unavailable."""
    file_path = Path(path).expanduser()
    if not file_path.is_file():
        return None
    digest = hashlib.sha256()
    with file_path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def git_commit(repo_path):
    """Return the current commit and dirty marker without mutating Git."""
    try:
        commit = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=repo_path,
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
        dirty = subprocess.run(
            ["git", "status", "--porcelain"],
            cwd=repo_path,
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
        return f"{commit}{'-dirty' if dirty else ''}"
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


class RunStore:
    """Own one immutable run directory and append-only trial files."""

    def __init__(self, output_root, run_id, resume=False):
        """Create a new run directory or open one explicitly for resume."""
        self.run_id = run_id
        self.run_dir = Path(output_root).expanduser().resolve() / run_id
        if self.run_dir.exists() and not resume:
            raise FileExistsError(f"Run directory already exists: {self.run_dir}")
        self.run_dir.mkdir(parents=True, exist_ok=resume)
        (self.run_dir / "figures").mkdir(exist_ok=True)
        (self.run_dir / "photos").mkdir(exist_ok=True)
        self.trials_path = self.run_dir / "trials.csv"
        self.trace_path = self.run_dir / "trace.csv"
        self.events_path = self.run_dir / "events.jsonl"
        self._trial_stream = None
        self._trial_writer = None
        self._trace_stream = None
        self._trace_writer = None

    def write_yaml(self, filename, data):
        """Write a human-readable YAML document."""
        path = self.run_dir / filename
        with path.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(data, stream, allow_unicode=True, sort_keys=False)
        return path

    def append_event(self, event, **details):
        """Append a diagnostic event independently of trial completion."""
        record = {"timestamp": now_iso(), "event": event, **details}
        with self.events_path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, ensure_ascii=False) + "\n")
            stream.flush()

    def open_trials(self):
        """Open the trial CSV and create its header when needed."""
        existed = self.trials_path.exists() and self.trials_path.stat().st_size > 0
        self._trial_stream = self.trials_path.open(
            "a", newline="", encoding="utf-8"
        )
        self._trial_writer = csv.DictWriter(
            self._trial_stream, fieldnames=TRIAL_FIELDS, extrasaction="ignore"
        )
        if not existed:
            self._trial_writer.writeheader()
            self._trial_stream.flush()

    def append_trial(self, row):
        """Append and flush one completed, failed, or interrupted trial."""
        if self._trial_writer is None:
            self.open_trials()
        normalized = {field: row.get(field, "") for field in TRIAL_FIELDS}
        self._trial_writer.writerow(normalized)
        self._trial_stream.flush()

    def append_trace(self, row):
        """Append and flush one optional high-frequency trace row."""
        if self._trace_writer is None:
            existed = self.trace_path.exists() and self.trace_path.stat().st_size > 0
            self._trace_stream = self.trace_path.open(
                "a", newline="", encoding="utf-8"
            )
            self._trace_writer = csv.DictWriter(
                self._trace_stream, fieldnames=TRACE_FIELDS, extrasaction="ignore"
            )
            if not existed:
                self._trace_writer.writeheader()
                self._trace_stream.flush()
        normalized = {field: row.get(field, "") for field in TRACE_FIELDS}
        self._trace_writer.writerow(normalized)
        self._trace_stream.flush()

    def completed_keys(self):
        """Return (sequence_index, condition_id, trial) keys already recorded."""
        if not self.trials_path.exists():
            return set()
        with self.trials_path.open("r", encoding="utf-8", newline="") as stream:
            return {
                (
                    int(row["sequence_index"]),
                    row["condition_id"],
                    int(row["trial"]),
                )
                for row in csv.DictReader(stream)
                if row.get("sequence_index") and row.get("trial")
            }

    def close(self):
        """Close any open CSV streams."""
        for stream in (self._trial_stream, self._trace_stream):
            if stream is not None:
                stream.flush()
                stream.close()


def make_run_id(robot_name, experiment_name):
    """Create a sortable unique run identifier."""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"{timestamp}_{robot_name}_{experiment_name}"


def update_runs_index(output_root, metadata, trials_path, result_directory):
    """Insert or replace one run in the lightweight top-level index."""
    index_path = Path(output_root).expanduser().resolve() / "runs_index.csv"
    index_path.parent.mkdir(parents=True, exist_ok=True)
    existing = []
    if index_path.exists() and index_path.stat().st_size > 0:
        with index_path.open("r", encoding="utf-8", newline="") as stream:
            existing = list(csv.DictReader(stream))

    trial_rows = []
    trial_path = Path(trials_path)
    if trial_path.exists():
        with trial_path.open("r", encoding="utf-8", newline="") as stream:
            trial_rows = list(csv.DictReader(stream))

    robot = metadata.get("robot", {})
    experiment = metadata.get("experiment", {})
    row = {
        "run_id": metadata["run_id"],
        "started_at": metadata.get("started_at", ""),
        "finished_at": metadata.get("finished_at", ""),
        "robot_id": robot.get("name", ""),
        "ros_domain_id": robot.get("ros_domain_id", ""),
        "experiment_name": experiment.get("name", ""),
        "scenario_id": metadata.get("scenario_id", ""),
        "status": metadata.get("status", ""),
        "trial_rows": len(trial_rows),
        "completed_rows": sum(item.get("status") == "COMPLETED" for item in trial_rows),
        "nav_success_count": sum(
            item.get("nav_result") == "SUCCEEDED" for item in trial_rows
        ),
        "git_commit": metadata.get("git_commit", ""),
        "result_directory": str(Path(result_directory).resolve()),
    }
    rows = [item for item in existing if item.get("run_id") != row["run_id"]]
    rows.append(row)
    temporary = index_path.with_suffix(".csv.tmp")
    with temporary.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=RUN_INDEX_FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    temporary.replace(index_path)
    return index_path
