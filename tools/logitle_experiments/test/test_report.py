"""Summary and graph generation tests."""

import csv

from logitle_experiments.analysis.report import build_report
from logitle_experiments.common.storage import TRACE_FIELDS, TRIAL_FIELDS


def test_report_creates_summary_and_figures(tmp_path):
    """Generate condition and AMCL stability summaries from sample data."""
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    rows = []
    for sequence, value in enumerate((100, 120, 140), start=1):
        rows.append(
            {
                "schema_version": 1,
                "run_id": "run",
                "sequence_index": sequence,
                "condition_id": "xy_100",
                "condition_label": "XY 100 mm",
                "trial": sequence,
                "status": "COMPLETED",
                "nav_result": "SUCCEEDED",
                "physical_position_error_mm": value,
                "estimated_position_error_mm": value / 2,
                "navigation_time_sec": 10 + sequence,
                "stop_direction": "left",
                "overshoot_observed": sequence == 1,
                "controller_oscillation": False,
                "localization_jump_observed": False,
                "external_intervention": False,
            }
        )
    with (run_dir / "trials.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=TRIAL_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in TRIAL_FIELDS})
    with (run_dir / "trace.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=TRACE_FIELDS)
        writer.writeheader()
        for index, x_m in enumerate((1.000, 1.002, 0.999), start=1):
            row = {
                "schema_version": 1,
                "run_id": "run",
                "sequence_index": 1,
                "trial_elapsed_sec": index,
                "source": "amcl_snapshot",
                "source_stamp_sec": 100 + index,
                "amcl_x_m": x_m,
                "amcl_y_m": 2.0,
                "amcl_yaw_rad": 0.1,
                "amcl_cov_x": 0.01,
                "amcl_cov_y": 0.01,
                "amcl_cov_yaw": 0.02,
            }
            writer.writerow({field: row.get(field, "") for field in TRACE_FIELDS})

    summary_path, figures = build_report(run_dir)
    assert summary_path.exists()
    assert len(figures) >= 4
    with summary_path.open("r", encoding="utf-8", newline="") as stream:
        summary_rows = list(csv.DictReader(stream))
    assert summary_rows[0]["physical_error_mm_mean"] == "120.000000"
    assert summary_rows[0]["overshoot_count"] == "1"
    assert summary_rows[0]["stop_left_count"] == "3"
    stability_path = run_dir / "amcl_stability_summary.csv"
    assert stability_path.exists()
    with stability_path.open("r", encoding="utf-8", newline="") as stream:
        stability_rows = list(csv.DictReader(stream))
    assert stability_rows[0]["unique_source_stamps"] == "3"
    assert stability_rows[0]["x_peak_to_peak_mm"] == "3.000000000"
