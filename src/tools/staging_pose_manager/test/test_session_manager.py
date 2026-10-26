"""Unit tests for crash-safe registration session persistence."""

import csv

import pytest

from staging_pose_manager.pose_statistics import calculate_statistics
from staging_pose_manager.session_manager import SessionManager, validate_station_id


def make_samples(offset, count=3):
    return [
        {
            "stamp": f"{index}.0",
            "x": offset + index * 0.01,
            "y": 2.0,
            "yaw": 0.1,
        }
        for index in range(count)
    ]


def test_partial_samples_are_never_saved(tmp_path):
    session = SessionManager.create(tmp_path, "staging_A", 1, 3, 0.05)
    partial = make_samples(1.0, count=2)

    with pytest.raises(ValueError, match="partial"):
        session.save_baseline(partial, calculate_statistics(partial))

    assert session.state["baseline"] is None
    assert list(csv.DictReader((session.session_dir / "samples.csv").open())) == []


def test_resume_preserves_condition_and_completed_trials(tmp_path):
    session = SessionManager.create(tmp_path, "staging_A", 2, 3, 0.05)
    first = make_samples(1.0)
    condition = {
        "type": "clockwise",
        "target_value": 60.0,
        "target_unit": "deg",
        "description": "",
    }
    session.complete_trial(1, condition, first, calculate_statistics(first))
    session.start_trial(2, condition)

    resumed = SessionManager.load(session.session_dir)

    assert len(resumed.completed_trials) == 1
    assert resumed.next_trial_number() == 2
    assert resumed.state["current_trial"]["perturbation"] == condition


def test_load_repairs_derived_csv_from_status(tmp_path):
    """A crash between status and CSV writes is repaired when resuming."""
    session = SessionManager.create(tmp_path, "staging_A", 1, 3, 0.05)
    samples = make_samples(1.0)
    condition = {"type": "incoming", "description": "north"}
    session.complete_trial(1, condition, samples, calculate_statistics(samples))
    (session.session_dir / "trials.csv").write_text("stale\n", encoding="utf-8")

    SessionManager.load(session.session_dir)

    with (session.session_dir / "trials.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 1
    assert rows[0]["trial"] == "1"


def test_deferred_trials_rotate_and_are_not_counted_complete(tmp_path):
    """Deferring revisits other pending slots and never lowers trial count."""
    session = SessionManager.create(tmp_path, "staging_A", 2, 3, 0.05)
    session.defer_trial(1)
    assert session.next_trial_number() == 2
    session.defer_trial(2)
    assert session.next_trial_number() == 1
    assert not session.is_complete


def test_complete_session_writes_all_csv_files_and_excludes_baseline(tmp_path):
    session = SessionManager.create(tmp_path, "staging_A", 2, 3, 0.05)
    baseline = make_samples(100.0)
    trial_one = make_samples(1.0)
    trial_two = make_samples(3.0)
    manual = {"type": "incoming", "description": "north"}

    session.save_baseline(baseline, calculate_statistics(baseline))
    session.complete_trial(1, manual, trial_one, calculate_statistics(trial_one))
    session.complete_trial(2, manual, trial_two, calculate_statistics(trial_two))
    result = session.finalize()

    assert result["mean"]["x"] == pytest.approx(2.01)
    assert (session.session_dir / "summary.csv").is_file()
    with (session.session_dir / "samples.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 9
    assert sum(row["measurement"] == "baseline" for row in rows) == 3


@pytest.mark.parametrize("value", ["", "../escape", "bad/name", "bad space"])
def test_invalid_station_ids_are_rejected(value):
    with pytest.raises(ValueError):
        validate_station_id(value)
