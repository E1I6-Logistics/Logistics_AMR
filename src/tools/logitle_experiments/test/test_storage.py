"""Crash-resistant CSV storage tests."""

import csv

from logitle_experiments.common.storage import RunStore


def test_trial_rows_are_flushed_with_blank_missing_values(tmp_path):
    """Write known values while leaving unavailable fields blank."""
    store = RunStore(tmp_path, "test_run")
    store.append_trial(
        {
            "schema_version": 1,
            "run_id": "test_run",
            "sequence_index": 1,
            "condition_id": "xy_100",
            "trial": 1,
            "physical_position_error_mm": 128,
        }
    )
    store.close()

    with (tmp_path / "test_run" / "trials.csv").open(
        "r", encoding="utf-8", newline=""
    ) as stream:
        rows = list(csv.DictReader(stream))
    assert rows[0]["physical_position_error_mm"] == "128"
    assert rows[0]["estimated_position_error_mm"] == ""
