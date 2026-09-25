"""Create deterministic summaries and PNG figures from one run directory."""

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


NUMERIC_COLUMNS = [
    "sequence_index",
    "trial",
    "physical_position_error_mm",
    "estimated_position_error_mm",
    "estimated_dx_mm",
    "estimated_dy_mm",
    "estimated_yaw_error_deg",
    "navigation_time_sec",
    "observed_cmd_vel_max_x_mps",
]


def load_trials(run_dir):
    """Load one run's trial table with explicit numeric coercion."""
    path = Path(run_dir).expanduser().resolve() / "trials.csv"
    frame = pd.read_csv(path, keep_default_na=True)
    for column in NUMERIC_COLUMNS:
        if column in frame.columns:
            frame[column] = pd.to_numeric(frame[column], errors="coerce")
    return frame


def _metric_stats(series, prefix):
    values = pd.to_numeric(series, errors="coerce").dropna()
    if values.empty:
        return {
            f"{prefix}_count": 0,
            f"{prefix}_mean": np.nan,
            f"{prefix}_median": np.nan,
            f"{prefix}_sample_std": np.nan,
            f"{prefix}_p95": np.nan,
            f"{prefix}_min": np.nan,
            f"{prefix}_max": np.nan,
        }
    return {
        f"{prefix}_count": int(values.count()),
        f"{prefix}_mean": float(values.mean()),
        f"{prefix}_median": float(values.median()),
        f"{prefix}_sample_std": float(values.std(ddof=1)) if len(values) > 1 else np.nan,
        f"{prefix}_p95": float(values.quantile(0.95, interpolation="linear")),
        f"{prefix}_min": float(values.min()),
        f"{prefix}_max": float(values.max()),
    }


def _truthy_count(series):
    """Count normalized boolean values from a CSV column."""
    return int(
        series.astype(str)
        .str.strip()
        .str.lower()
        .isin(("true", "1", "yes"))
        .sum()
    )


def _column(frame, name, default=""):
    """Return a column or an aligned default series for older schemas."""
    if name in frame.columns:
        return frame[name]
    return pd.Series(default, index=frame.index)


def summarize_trials(frame):
    """Return one condition-level summary row per condition."""
    rows = []
    for (condition_id, condition_label), group in frame.groupby(
        ["condition_id", "condition_label"], dropna=False, sort=False
    ):
        success = group["nav_result"].eq("SUCCEEDED")
        not_applicable = group["nav_result"].eq("NOT_APPLICABLE")
        eligible = ~(group["nav_result"].isna() | not_applicable)
        success_rate = (
            float(success[eligible].mean()) if int(eligible.sum()) > 0 else np.nan
        )
        row = {
            "condition_id": condition_id,
            "condition_label": condition_label,
            "trial_rows": int(len(group)),
            "completed_rows": int(group["status"].eq("COMPLETED").sum()),
            "nav_success_count": int(success.sum()),
            "nav_success_rate": success_rate,
            "overshoot_count": _truthy_count(
                _column(group, "overshoot_observed")
            ),
            "controller_oscillation_count": _truthy_count(
                _column(group, "controller_oscillation")
            ),
            "localization_jump_count": _truthy_count(
                _column(group, "localization_jump_observed")
            ),
            "external_intervention_count": _truthy_count(
                _column(group, "external_intervention")
            ),
        }
        for direction in ("left", "right", "front", "back", "center", "unknown"):
            row[f"stop_{direction}_count"] = int(
                _column(group, "stop_direction")
                .astype(str)
                .str.lower()
                .eq(direction)
                .sum()
            )
        row.update(
            _metric_stats(group["physical_position_error_mm"], "physical_error_mm")
        )
        row.update(
            _metric_stats(group["estimated_position_error_mm"], "estimated_error_mm")
        )
        row.update(_metric_stats(group["navigation_time_sec"], "navigation_time_sec"))
        rows.append(row)
    return pd.DataFrame(rows)


def summarize_amcl_stability(run_dir):
    """Write per-trial AMCL stationary stability statistics when available."""
    run_path = Path(run_dir)
    trace_path = run_path / "trace.csv"
    if not trace_path.exists():
        return None
    trace = pd.read_csv(trace_path)
    required = {
        "source",
        "sequence_index",
        "trial_elapsed_sec",
        "source_stamp_sec",
        "amcl_x_m",
        "amcl_y_m",
        "amcl_yaw_rad",
        "amcl_cov_x",
        "amcl_cov_y",
        "amcl_cov_yaw",
    }
    if not required.issubset(trace.columns):
        return None
    trace = trace[trace["source"].eq("amcl_snapshot")].copy()
    if trace.empty:
        return None
    numeric = required - {"source"}
    for column in numeric:
        trace[column] = pd.to_numeric(trace[column], errors="coerce")

    rows = []
    for sequence_index, group in trace.groupby("sequence_index", sort=True):
        group = group.sort_values("trial_elapsed_sec")
        x = group["amcl_x_m"].dropna()
        y = group["amcl_y_m"].dropna()
        paired = group.dropna(subset=["amcl_x_m", "amcl_y_m"])
        if paired.empty:
            max_radius_mm = np.nan
        else:
            dx = paired["amcl_x_m"] - paired["amcl_x_m"].mean()
            dy = paired["amcl_y_m"] - paired["amcl_y_m"].mean()
            max_radius_mm = float(np.hypot(dx, dy).max() * 1000.0)
        yaw = group["amcl_yaw_rad"].dropna().to_numpy()
        yaw_p2p_deg = (
            float(np.degrees(np.ptp(np.unwrap(yaw)))) if yaw.size else np.nan
        )
        row = {
            "sequence_index": int(sequence_index),
            "sample_rows": int(len(group)),
            "unique_source_stamps": int(group["source_stamp_sec"].nunique()),
            "x_peak_to_peak_mm": float((x.max() - x.min()) * 1000.0)
            if not x.empty
            else np.nan,
            "y_peak_to_peak_mm": float((y.max() - y.min()) * 1000.0)
            if not y.empty
            else np.nan,
            "yaw_peak_to_peak_deg": yaw_p2p_deg,
            "max_radius_from_mean_mm": max_radius_mm,
        }
        for column, prefix in (
            ("amcl_cov_x", "cov_x"),
            ("amcl_cov_y", "cov_y"),
            ("amcl_cov_yaw", "cov_yaw"),
        ):
            values = group[column].dropna()
            row[f"{prefix}_start"] = float(values.iloc[0]) if not values.empty else np.nan
            row[f"{prefix}_end"] = float(values.iloc[-1]) if not values.empty else np.nan
            row[f"{prefix}_max"] = float(values.max()) if not values.empty else np.nan
        rows.append(row)

    output_path = run_path / "amcl_stability_summary.csv"
    pd.DataFrame(rows).to_csv(output_path, index=False, float_format="%.9f")
    return output_path


def _save_figure(figure, path):
    figure.tight_layout()
    figure.savefig(path, dpi=160, bbox_inches="tight")
    plt.close(figure)


def plot_physical_error(frame, figures_dir):
    """Plot physical-error distributions with every observed point."""
    valid = frame.dropna(subset=["physical_position_error_mm"])
    if valid.empty:
        return None
    labels = list(dict.fromkeys(valid["condition_label"].astype(str)))
    values = [
        valid.loc[
            valid["condition_label"].astype(str).eq(label),
            "physical_position_error_mm",
        ].to_numpy()
        for label in labels
    ]
    figure, axis = plt.subplots(figsize=(8, 5))
    axis.boxplot(values, labels=labels, showmeans=True)
    for index, series in enumerate(values, start=1):
        offsets = np.linspace(-0.08, 0.08, len(series)) if len(series) > 1 else [0.0]
        axis.scatter(index + offsets, series, color="#1f77b4", zorder=3)
    axis.set_title("Physical position error by condition")
    axis.set_ylabel("Physical position error (mm)")
    axis.grid(axis="y", alpha=0.3)
    path = figures_dir / "physical_error_by_condition.png"
    _save_figure(figure, path)
    return path


def plot_trial_order(frame, figures_dir):
    """Plot physical error in execution order to expose time drift."""
    valid = frame.dropna(subset=["sequence_index", "physical_position_error_mm"])
    if valid.empty:
        return None
    figure, axis = plt.subplots(figsize=(9, 4.5))
    for label, group in valid.groupby("condition_label", sort=False):
        axis.plot(
            group["sequence_index"],
            group["physical_position_error_mm"],
            marker="o",
            linestyle="-",
            label=str(label),
        )
    axis.set_title("Physical error in execution order")
    axis.set_xlabel("Sequence index")
    axis.set_ylabel("Physical position error (mm)")
    axis.legend()
    axis.grid(alpha=0.3)
    path = figures_dir / "physical_error_by_sequence.png"
    _save_figure(figure, path)
    return path


def plot_estimated_vs_physical(frame, figures_dir):
    """Compare localization-derived and physically measured errors."""
    valid = frame.dropna(
        subset=["estimated_position_error_mm", "physical_position_error_mm"]
    )
    if valid.empty:
        return None
    figure, axis = plt.subplots(figsize=(6, 6))
    for label, group in valid.groupby("condition_label", sort=False):
        axis.scatter(
            group["estimated_position_error_mm"],
            group["physical_position_error_mm"],
            label=str(label),
        )
    maximum = float(
        max(
            valid["estimated_position_error_mm"].max(),
            valid["physical_position_error_mm"].max(),
        )
    )
    axis.plot([0, maximum], [0, maximum], linestyle="--", color="gray")
    axis.set_title("AMCL estimated error vs physical error")
    axis.set_xlabel("AMCL estimated error (mm)")
    axis.set_ylabel("Physical error (mm)")
    axis.legend()
    axis.grid(alpha=0.3)
    path = figures_dir / "estimated_vs_physical.png"
    _save_figure(figure, path)
    return path


def plot_navigation_time(frame, figures_dir):
    """Plot navigation-time distributions by condition."""
    valid = frame.dropna(subset=["navigation_time_sec"])
    if valid.empty:
        return None
    labels = list(dict.fromkeys(valid["condition_label"].astype(str)))
    values = [
        valid.loc[
            valid["condition_label"].astype(str).eq(label), "navigation_time_sec"
        ].to_numpy()
        for label in labels
    ]
    figure, axis = plt.subplots(figsize=(8, 5))
    axis.boxplot(values, labels=labels, showmeans=True)
    axis.set_title("Navigation time by condition")
    axis.set_ylabel("Navigation time (s)")
    axis.grid(axis="y", alpha=0.3)
    path = figures_dir / "navigation_time_by_condition.png"
    _save_figure(figure, path)
    return path


def plot_stop_direction(frame, figures_dir):
    """Plot categorical stop directions by condition."""
    valid = frame[frame["stop_direction"].notna()]
    if valid.empty:
        return None
    table = pd.crosstab(valid["condition_label"], valid["stop_direction"])
    axis = table.plot(kind="bar", stacked=True, figsize=(9, 5))
    axis.set_title("Stop direction counts")
    axis.set_xlabel("Condition")
    axis.set_ylabel("Trial count")
    axis.legend(title="Direction", bbox_to_anchor=(1.02, 1), loc="upper left")
    path = figures_dir / "stop_direction_counts.png"
    _save_figure(axis.figure, path)
    return path


def plot_amcl_stability(run_dir, figures_dir):
    """Plot periodic AMCL snapshots from stationary validation."""
    trace_path = Path(run_dir) / "trace.csv"
    if not trace_path.exists():
        return None
    trace = pd.read_csv(trace_path)
    trace = trace[trace["source"].eq("amcl_snapshot")].copy()
    if trace.empty:
        return None
    for column in ("trial_elapsed_sec", "amcl_x_m", "amcl_y_m"):
        trace[column] = pd.to_numeric(trace[column], errors="coerce")
    figure, axes = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    for sequence_index, group in trace.groupby("sequence_index"):
        axes[0].plot(
            group["trial_elapsed_sec"], group["amcl_x_m"], label=str(sequence_index)
        )
        axes[1].plot(group["trial_elapsed_sec"], group["amcl_y_m"])
    axes[0].set_ylabel("AMCL x (m)")
    axes[1].set_ylabel("AMCL y (m)")
    axes[1].set_xlabel("Elapsed time (s)")
    axes[0].set_title("AMCL stationary samples")
    axes[0].legend(title="Sequence", ncol=3)
    for axis in axes:
        axis.grid(alpha=0.3)
    path = figures_dir / "amcl_stationary_trace.png"
    _save_figure(figure, path)
    return path


def plot_cmd_velocity(run_dir, figures_dir):
    """Plot final command velocity traces when speed capture is enabled."""
    trace_path = Path(run_dir) / "trace.csv"
    if not trace_path.exists():
        return None
    trace = pd.read_csv(trace_path)
    trace = trace[trace["source"].eq("cmd_vel")].copy()
    if trace.empty:
        return None
    for column in ("trial_elapsed_sec", "cmd_vel_x_mps"):
        trace[column] = pd.to_numeric(trace[column], errors="coerce")
    figure, axis = plt.subplots(figsize=(10, 5))
    for sequence_index, group in trace.groupby("sequence_index"):
        axis.plot(
            group["trial_elapsed_sec"],
            group["cmd_vel_x_mps"],
            alpha=0.75,
            label=str(sequence_index),
        )
    axis.set_title("Final command velocity by trial")
    axis.set_xlabel("Elapsed time (s)")
    axis.set_ylabel("cmd_vel linear x (m/s)")
    axis.grid(alpha=0.3)
    axis.legend(title="Sequence", ncol=3, fontsize="small")
    path = figures_dir / "cmd_vel_trace.png"
    _save_figure(figure, path)
    return path


def build_report(run_dir):
    """Generate summary.csv and all applicable figures."""
    run_path = Path(run_dir).expanduser().resolve()
    frame = load_trials(run_path)
    summary = summarize_trials(frame)
    summary_path = run_path / "summary.csv"
    summary.to_csv(summary_path, index=False, float_format="%.6f")
    summarize_amcl_stability(run_path)
    figures_dir = run_path / "figures"
    figures_dir.mkdir(exist_ok=True)
    figures = [
        plot_physical_error(frame, figures_dir),
        plot_trial_order(frame, figures_dir),
        plot_estimated_vs_physical(frame, figures_dir),
        plot_navigation_time(frame, figures_dir),
        plot_stop_direction(frame, figures_dir),
        plot_amcl_stability(run_path, figures_dir),
        plot_cmd_velocity(run_path, figures_dir),
    ]
    return summary_path, [path for path in figures if path is not None]


def parse_args():
    """Parse one run directory."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir")
    return parser.parse_args()


def main():
    """Console entry point."""
    summary_path, figures = build_report(parse_args().run_dir)
    print(f"Summary: {summary_path}")
    for figure in figures:
        print(f"Figure: {figure}")


if __name__ == "__main__":
    main()
