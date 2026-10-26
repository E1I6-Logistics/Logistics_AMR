"""Crash-safe session state and CSV persistence."""

import csv
from datetime import datetime
import json
import math
import os
from pathlib import Path
import re

from staging_pose_manager.pose_statistics import calculate_statistics


SAMPLE_FIELDS = [
    "station_id", "session_id", "measurement", "trial", "sample",
    "stamp", "x", "y", "yaw",
]
TRIAL_FIELDS = [
    "station_id", "session_id", "trial", "perturbation", "description",
    "target_value", "target_unit", "mean_x", "mean_y", "mean_yaw",
    "sample_std_x", "sample_std_y", "sample_std_yaw", "sample_count",
    "completed_at",
]
SUMMARY_FIELDS = [
    "station_id", "session_id", "trial_count", "samples_per_trial",
    "sample_interval", "mean_x", "mean_y", "mean_yaw", "std_x",
    "std_y", "std_yaw", "completed_at",
]


def validate_station_id(value):
    """Validate a station ID before using it as a directory name."""
    value = value.strip()
    if not value or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,63}", value):
        raise ValueError(
            "Station ID는 영문/숫자로 시작하고 . _ - 만 포함해야 합니다."
        )
    return value


def _atomic_json(path, data):
    temp_path = path.with_suffix(path.suffix + ".tmp")
    with temp_path.open("w", encoding="utf-8") as stream:
        json.dump(data, stream, ensure_ascii=False, indent=2)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temp_path, path)


def _atomic_csv(path, fieldnames, rows):
    temp_path = path.with_suffix(path.suffix + ".tmp")
    with temp_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temp_path, path)


class SessionManager:
    """Own one station registration session and persist it atomically."""

    STATUS_FILE = "status.json"

    def __init__(self, session_dir, state):
        self.session_dir = Path(session_dir)
        self.state = state

    @classmethod
    def create(cls, records_dir, station_id, trial_count, sample_count, interval):
        station_id = validate_station_id(station_id)
        trial_count = int(trial_count)
        sample_count = int(sample_count)
        interval = float(interval)
        if trial_count <= 0 or sample_count <= 0:
            raise ValueError("trial_count and sample_count must be positive")
        if not math.isfinite(interval) or interval <= 0.0:
            raise ValueError("sample interval must be positive and finite")
        station_dir = Path(records_dir).expanduser() / station_id
        station_dir.mkdir(parents=True, exist_ok=True)
        session_id = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        session_dir = station_dir / session_id
        session_dir.mkdir()
        now = datetime.now().astimezone().isoformat(timespec="seconds")
        state = {
            "schema_version": 1,
            "station_id": station_id,
            "session_id": session_id,
            "status": "incomplete",
            "trial_count": trial_count,
            "sample_count": sample_count,
            "sample_interval": interval,
            "created_at": now,
            "updated_at": now,
            "baseline": None,
            "trials": [],
            "current_trial": None,
            "deferred_trials": [],
        }
        manager = cls(session_dir, state)
        manager._persist()
        return manager

    @classmethod
    def load(cls, session_dir):
        session_dir = Path(session_dir).expanduser()
        with (session_dir / cls.STATUS_FILE).open(encoding="utf-8") as stream:
            state = json.load(stream)
        manager = cls(session_dir, state)
        manager._validate_state()
        manager._write_csvs()
        return manager

    @classmethod
    def find_incomplete(cls, records_dir):
        sessions = []
        for status_path in Path(records_dir).expanduser().glob("*/*/status.json"):
            try:
                manager = cls.load(status_path.parent)
            except (OSError, ValueError, json.JSONDecodeError):
                continue
            if manager.state["status"] == "incomplete":
                sessions.append(manager)
        return sorted(
            sessions,
            key=lambda item: item.state.get("updated_at", ""),
            reverse=True,
        )

    def _validate_state(self):
        required = {
            "station_id", "session_id", "status", "trial_count",
            "sample_count", "sample_interval", "trials",
        }
        if not required.issubset(self.state):
            raise ValueError("invalid session status file")
        validate_station_id(self.state["station_id"])
        trial_count = int(self.state["trial_count"])
        sample_count = int(self.state["sample_count"])
        interval = float(self.state["sample_interval"])
        if trial_count <= 0 or sample_count <= 0:
            raise ValueError("invalid session counts")
        if not math.isfinite(interval) or interval <= 0.0:
            raise ValueError("invalid sample interval")

        trial_numbers = [int(item["trial"]) for item in self.state["trials"]]
        if len(trial_numbers) != len(set(trial_numbers)):
            raise ValueError("duplicate completed trial")
        if any(number < 1 or number > trial_count for number in trial_numbers):
            raise ValueError("completed trial number is out of range")
        measurements = list(self.state["trials"])
        if self.state.get("baseline"):
            measurements.append(self.state["baseline"])
        if any(len(item.get("samples", [])) != sample_count for item in measurements):
            raise ValueError("stored measurement contains partial samples")

    @property
    def completed_trials(self):
        return sorted(self.state["trials"], key=lambda row: int(row["trial"]))

    @property
    def is_complete(self):
        return len(self.completed_trials) == int(self.state["trial_count"])

    def next_trial_number(self):
        completed = {int(row["trial"]) for row in self.completed_trials}
        current = self.state.get("current_trial")
        if current and int(current["trial"]) not in completed:
            return int(current["trial"])

        deferred = [
            int(number) for number in self.state.get("deferred_trials", [])
            if int(number) not in completed
        ]
        for number in range(1, int(self.state["trial_count"]) + 1):
            if number not in completed and number not in deferred:
                return number
        return deferred[0] if deferred else None

    def save_baseline(self, samples, statistics):
        self._require_full_samples(samples)
        self.state["baseline"] = {
            "samples": list(samples),
            "statistics": statistics,
            "completed_at": self._now(),
        }
        self._persist()

    def start_trial(self, trial, perturbation):
        self.state["current_trial"] = {
            "trial": int(trial),
            "perturbation": dict(perturbation),
            "started_at": self._now(),
        }
        self._persist()

    def defer_trial(self, trial):
        trial = int(trial)
        deferred = self.state.setdefault("deferred_trials", [])
        deferred[:] = [number for number in deferred if int(number) != trial]
        deferred.append(trial)
        self.state["current_trial"] = None
        self._persist()

    def complete_trial(self, trial, perturbation, samples, result):
        self._require_full_samples(samples)
        trial = int(trial)
        row = {
            "trial": trial,
            "perturbation": dict(perturbation),
            "samples": list(samples),
            "statistics": result,
            "completed_at": self._now(),
        }
        self.state["trials"] = [
            item for item in self.state["trials"] if int(item["trial"]) != trial
        ]
        self.state["trials"].append(row)
        self.state["current_trial"] = None
        self.state["deferred_trials"] = [
            number for number in self.state.get("deferred_trials", [])
            if int(number) != trial
        ]
        if self.is_complete:
            self.state["status"] = "complete"
            self.state["completed_at"] = self._now()
        self._persist()

    def finalize(self):
        if not self.is_complete:
            raise ValueError("all configured trials must be complete")
        self.state["status"] = "complete"
        self.state["completed_at"] = self.state.get("completed_at", self._now())
        self._persist()
        return calculate_statistics(
            item["statistics"]["mean"] for item in self.completed_trials
        )

    def _require_full_samples(self, samples):
        if len(samples) != int(self.state["sample_count"]):
            raise ValueError("partial sample sets must not be saved")

    def _persist(self):
        self.state["updated_at"] = self._now()
        _atomic_json(self.session_dir / self.STATUS_FILE, self.state)
        self._write_csvs()

    def _write_csvs(self):
        """Regenerate derived CSVs from the authoritative status data."""
        _atomic_csv(self.session_dir / "samples.csv", SAMPLE_FIELDS, self._sample_rows())
        _atomic_csv(self.session_dir / "trials.csv", TRIAL_FIELDS, self._trial_rows())
        if self.is_complete:
            _atomic_csv(
                self.session_dir / "summary.csv",
                SUMMARY_FIELDS,
                [self._summary_row()],
            )

    def _sample_rows(self):
        rows = []
        measurements = []
        if self.state.get("baseline"):
            measurements.append(("baseline", "", self.state["baseline"]))
        measurements.extend(
            ("trial", item["trial"], item) for item in self.completed_trials
        )
        for measurement, trial, item in measurements:
            for index, sample in enumerate(item["samples"], start=1):
                rows.append({
                    "station_id": self.state["station_id"],
                    "session_id": self.state["session_id"],
                    "measurement": measurement,
                    "trial": trial,
                    "sample": index,
                    "stamp": sample.get("stamp", ""),
                    "x": sample["x"],
                    "y": sample["y"],
                    "yaw": sample["yaw"],
                })
        return rows

    def _trial_rows(self):
        rows = []
        for item in self.completed_trials:
            perturbation = item["perturbation"]
            result = item["statistics"]
            rows.append({
                "station_id": self.state["station_id"],
                "session_id": self.state["session_id"],
                "trial": item["trial"],
                "perturbation": perturbation["type"],
                "description": perturbation.get("description", ""),
                "target_value": perturbation.get("target_value", ""),
                "target_unit": perturbation.get("target_unit", ""),
                "mean_x": result["mean"]["x"],
                "mean_y": result["mean"]["y"],
                "mean_yaw": result["mean"]["yaw"],
                "sample_std_x": result["std"]["x"],
                "sample_std_y": result["std"]["y"],
                "sample_std_yaw": result["std"]["yaw"],
                "sample_count": len(item["samples"]),
                "completed_at": item["completed_at"],
            })
        return rows

    def _summary_row(self):
        result = calculate_statistics(
            item["statistics"]["mean"] for item in self.completed_trials
        )
        return {
            "station_id": self.state["station_id"],
            "session_id": self.state["session_id"],
            "trial_count": self.state["trial_count"],
            "samples_per_trial": self.state["sample_count"],
            "sample_interval": self.state["sample_interval"],
            "mean_x": result["mean"]["x"],
            "mean_y": result["mean"]["y"],
            "mean_yaw": result["mean"]["yaw"],
            "std_x": result["std"]["x"],
            "std_y": result["std"]["y"],
            "std_yaw": result["std"]["yaw"],
            "completed_at": self.state.get("completed_at", self._now()),
        }

    @staticmethod
    def _now():
        return datetime.now().astimezone().isoformat(timespec="milliseconds")
