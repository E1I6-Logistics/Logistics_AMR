#!/usr/bin/env python3

import argparse
import math
import re
from pathlib import Path

import matplotlib
matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


# =========================================================
# File name parser
#
# Example:
# 20260917_145436_baseline_trial1_rectangle_summary.csv
# =========================================================

FILE_PATTERN = re.compile(
    r'_(baseline|ekf)_trial(\d+)_.*_(summary|trace)\.csv$'
)


# =========================================================
# Utility
# =========================================================

def parse_file_info(path: Path):
    match = FILE_PATTERN.search(path.name)

    if match is None:
        return None

    condition = match.group(1)
    trial = int(match.group(2))
    kind = match.group(3)

    return condition, trial, kind


def clean_dataframe(df):
    """
    Remove unnecessary index columns that may appear when CSV is saved.
    """

    remove_columns = [
        col for col in df.columns
        if col.startswith("Unnamed:")
    ]

    if "index" in df.columns:
        remove_columns.append("index")

    if remove_columns:
        df = df.drop(columns=remove_columns)

    return df


def angle_difference_deg(measured_deg, expected_deg):
    """
    Shortest angular difference in degrees.
    Result range: [-180, 180]
    """

    diff_rad = math.radians(
        measured_deg - expected_deg
    )

    return math.degrees(
        math.atan2(
            math.sin(diff_rad),
            math.cos(diff_rad),
        )
    )


def reduction_percent(baseline, ekf):
    if abs(baseline) < 1e-12:
        return np.nan

    return (
        (baseline - ekf)
        / baseline
        * 100.0
    )


# =========================================================
# Load summary files
# =========================================================

def load_summaries(input_dir: Path):

    rows = []

    for path in sorted(
        input_dir.rglob("*_summary.csv")
    ):

        # 분석 결과 파일은 다시 읽지 않음
        if path.name in {
            "summary_all.csv",
            "comparison_summary.csv",
        }:
            continue

        info = parse_file_info(path)

        if info is None:
            continue

        condition, trial, kind = info

        if kind != "summary":
            continue

        df = pd.read_csv(path)
        df = clean_dataframe(df)

        if len(df) != 1:
            raise ValueError(
                f"Summary file must contain exactly one row:\n"
                f"{path}"
            )

        row = df.iloc[0].copy()

        # ---------------------------------------------
        # File name과 CSV 내부 condition/trial 검증
        # ---------------------------------------------

        if "condition" in df.columns:

            csv_condition = str(
                row["condition"]
            ).strip().lower()

            if csv_condition != condition:

                raise ValueError(
                    f"Condition mismatch:\n"
                    f"File: {path.name}\n"
                    f"Filename condition: {condition}\n"
                    f"CSV condition: {csv_condition}"
                )

        if "trial" in df.columns:

            csv_trial = int(
                row["trial"]
            )

            if csv_trial != trial:

                raise ValueError(
                    f"Trial mismatch:\n"
                    f"File: {path.name}\n"
                    f"Filename trial: {trial}\n"
                    f"CSV trial: {csv_trial}"
                )

        row["condition"] = condition
        row["trial"] = trial
        row["source_file"] = path.name

        rows.append(row)

    if not rows:
        raise RuntimeError(
            f"No summary CSV files found in:\n"
            f"{input_dir}"
        )

    summary = pd.DataFrame(rows)

    summary["trial"] = (
        summary["trial"]
        .astype(int)
    )

    summary = summary.sort_values(
        ["condition", "trial"]
    ).reset_index(drop=True)

    # ---------------------------------------------
    # 중복 Trial 검증
    # ---------------------------------------------

    duplicated = summary.duplicated(
        subset=[
            "condition",
            "trial",
        ],
        keep=False,
    )

    if duplicated.any():

        print(
            summary.loc[
                duplicated,
                [
                    "condition",
                    "trial",
                    "source_file",
                ],
            ]
        )

        raise ValueError(
            "Duplicate condition/trial summary files detected."
        )

    return summary


# =========================================================
# Load trace files
# =========================================================

def load_traces(input_dir: Path):

    traces = []

    seen_pairs = set()

    for path in sorted(
        input_dir.rglob("*_trace.csv")
    ):

        if path.name == "trace_all.csv":
            continue

        info = parse_file_info(path)

        if info is None:
            continue

        condition, trial, kind = info

        if kind != "trace":
            continue

        pair = (
            condition,
            trial,
        )

        if pair in seen_pairs:

            raise ValueError(
                "Duplicate trace detected for "
                f"{condition} trial {trial}"
            )

        seen_pairs.add(pair)

        df = pd.read_csv(path)
        df = clean_dataframe(df)

        required = {
            "elapsed_sec",
            "phase",
            "relative_x_m",
            "relative_y_m",
            "relative_yaw_deg",
        }

        missing = (
            required
            - set(df.columns)
        )

        if missing:

            raise ValueError(
                f"Missing trace columns in {path.name}:\n"
                f"{sorted(missing)}"
            )

        df.insert(
            0,
            "trial",
            trial,
        )

        df.insert(
            0,
            "condition",
            condition,
        )

        df["source_file"] = path.name

        traces.append(df)

    if not traces:

        raise RuntimeError(
            f"No trace CSV files found in:\n"
            f"{input_dir}"
        )

    trace = pd.concat(
        traces,
        ignore_index=True,
    )

    trace["trial"] = (
        trace["trial"]
        .astype(int)
    )

    trace = trace.sort_values(
        [
            "condition",
            "trial",
            "elapsed_sec",
        ]
    ).reset_index(drop=True)

    return trace


# =========================================================
# Validation
# =========================================================

def validate_dataset(
    summary,
    trace,
):

    # ---------------------------------------------
    # Summary와 Trace의 trial pair 비교
    # ---------------------------------------------

    summary_pairs = set(
        zip(
            summary["condition"],
            summary["trial"],
        )
    )

    trace_pairs = set(
        zip(
            trace["condition"],
            trace["trial"],
        )
    )

    if summary_pairs != trace_pairs:

        only_summary = (
            summary_pairs
            - trace_pairs
        )

        only_trace = (
            trace_pairs
            - summary_pairs
        )

        raise ValueError(
            "Summary/trace trial mismatch.\n"
            f"Summary only: {only_summary}\n"
            f"Trace only: {only_trace}"
        )

    # ---------------------------------------------
    # Baseline / EKF 존재 확인
    # ---------------------------------------------

    conditions = set(
        summary["condition"]
    )

    required_conditions = {
        "baseline",
        "ekf",
    }

    if not required_conditions.issubset(
        conditions
    ):

        raise ValueError(
            "Both baseline and ekf data are required."
        )

    # ---------------------------------------------
    # 동일 실험 조건인지 확인
    # ---------------------------------------------

    numeric_conditions = [
        "long_distance_m",
        "short_distance_m",
        "linear_speed_mps",
        "angular_speed_radps",
        "estimated_travel_distance_m",
    ]

    for column in numeric_conditions:

        if column not in summary.columns:
            continue

        values = pd.to_numeric(
            summary[column],
            errors="coerce",
        ).dropna().to_numpy()

        if len(values) <= 1:
            continue

        if not np.allclose(
            values,
            values[0],
            rtol=0.0,
            atol=1e-9,
        ):

            raise ValueError(
                f"Experiment condition mismatch: {column}\n"
                f"{summary[['condition', 'trial', column]]}"
            )

    string_conditions = [
        "experiment",
        "turn_direction",
    ]

    for column in string_conditions:

        if column not in summary.columns:
            continue

        values = (
            summary[column]
            .astype(str)
            .str.strip()
            .unique()
        )

        if len(values) > 1:

            raise ValueError(
                f"Experiment condition mismatch: {column}\n"
                f"{values}"
            )

    print()
    print("Dataset validation: OK")

    for condition in [
        "baseline",
        "ekf",
    ]:

        count = len(
            summary[
                summary["condition"]
                == condition
            ]
        )

        print(
            f"  {condition}: {count} trials"
        )


# =========================================================
# Statistics
# =========================================================

def build_condition_stats(summary):

    rows = []

    for condition in [
        "baseline",
        "ekf",
    ]:

        df = summary[
            summary["condition"]
            == condition
        ]

        if df.empty:
            continue

        position_cm = (
            pd.to_numeric(
                df["position_error_m"]
            )
            * 100.0
        )

        yaw_deg = pd.to_numeric(
            df["yaw_error_abs_deg"]
        )

        drift = pd.to_numeric(
            df["drift_percent"]
        )

        rows.append({
            "condition":
                condition,

            "n":
                len(df),

            "position_error_mean_cm":
                position_cm.mean(),

            "position_error_std_cm":
                position_cm.std(ddof=1),

            "yaw_error_mean_deg":
                yaw_deg.mean(),

            "yaw_error_std_deg":
                yaw_deg.std(ddof=1),

            "drift_mean_percent":
                drift.mean(),

            "drift_std_percent":
                drift.std(ddof=1),
        })

    return pd.DataFrame(rows)


def build_comparison_summary(
    condition_stats,
):

    baseline = (
        condition_stats[
            condition_stats["condition"]
            == "baseline"
        ].iloc[0]
    )

    ekf = (
        condition_stats[
            condition_stats["condition"]
            == "ekf"
        ].iloc[0]
    )

    rows = [
        {
            "metric":
                "position_error_cm",

            "baseline_mean":
                baseline[
                    "position_error_mean_cm"
                ],

            "ekf_mean":
                ekf[
                    "position_error_mean_cm"
                ],

            "reduction_percent":
                reduction_percent(
                    baseline[
                        "position_error_mean_cm"
                    ],
                    ekf[
                        "position_error_mean_cm"
                    ],
                ),
        },

        {
            "metric":
                "yaw_error_deg",

            "baseline_mean":
                baseline[
                    "yaw_error_mean_deg"
                ],

            "ekf_mean":
                ekf[
                    "yaw_error_mean_deg"
                ],

            "reduction_percent":
                reduction_percent(
                    baseline[
                        "yaw_error_mean_deg"
                    ],
                    ekf[
                        "yaw_error_mean_deg"
                    ],
                ),
        },

        {
            "metric":
                "drift_percent",

            "baseline_mean":
                baseline[
                    "drift_mean_percent"
                ],

            "ekf_mean":
                ekf[
                    "drift_mean_percent"
                ],

            "reduction_percent":
                reduction_percent(
                    baseline[
                        "drift_mean_percent"
                    ],
                    ekf[
                        "drift_mean_percent"
                    ],
                ),
        },
    ]

    return pd.DataFrame(rows)


# =========================================================
# Turn-by-turn yaw error
# =========================================================

def extract_turn_yaw_errors(
    summary,
    trace,
):

    results = []

    summary_lookup = (
        summary
        .set_index(
            [
                "condition",
                "trial",
            ]
        )
    )

    for (
        condition,
        trial,
    ), df in trace.groupby(
        [
            "condition",
            "trial",
        ],
        sort=True,
    ):

        df = df.sort_values(
            "elapsed_sec"
        )

        # 실제 회전 명령 phase만 추출
        turn_rows = df[
            df["phase"]
            .astype(str)
            .str.contains(
                "turn_90",
                regex=False,
            )
        ]

        if turn_rows.empty:
            continue

        # phase 등장 순서 유지
        phase_order = (
            turn_rows["phase"]
            .drop_duplicates()
            .tolist()
        )

        row_summary = summary_lookup.loc[
            (
                condition,
                trial,
            )
        ]

        direction = str(
            row_summary["turn_direction"]
        ).lower()

        direction_sign = (
            1.0
            if direction == "ccw"
            else -1.0
        )

        for turn_number, phase in enumerate(
            phase_order,
            start=1,
        ):

            phase_data = turn_rows[
                turn_rows["phase"]
                == phase
            ]

            # 해당 회전 phase의 마지막 yaw 사용
            measured_yaw = float(
                phase_data.iloc[-1][
                    "relative_yaw_deg"
                ]
            )

            expected_yaw = (
                direction_sign
                * 90.0
                * turn_number
            )

            error = (
                angle_difference_deg(
                    measured_yaw,
                    expected_yaw,
                )
            )

            results.append({
                "condition":
                    condition,

                "trial":
                    trial,

                "turn":
                    turn_number,

                "expected_yaw_deg":
                    expected_yaw,

                "measured_yaw_deg":
                    measured_yaw,

                "yaw_error_deg":
                    error,

                "yaw_error_abs_deg":
                    abs(error),
            })

    return pd.DataFrame(results)


# =========================================================
# Plot 1
# XY trajectory
# =========================================================

def plot_trajectory(
    summary,
    trace,
    output_dir,
):

    fig, ax = plt.subplots(
        figsize=(8, 7)
    )

    styles = {
        "baseline": "-",
        "ekf": "-.",
    }

    for condition in [
        "baseline",
        "ekf",
    ]:

        condition_df = trace[
            trace["condition"]
            == condition
        ]

        for trial, df in condition_df.groupby(
            "trial"
        ):

            df = df.sort_values(
                "elapsed_sec"
            )

            ax.plot(
                df["relative_x_m"],
                df["relative_y_m"],
                linestyle=styles[
                    condition
                ],
                linewidth=1.4,
                alpha=0.7,
                label=(
                    f"{condition.title()} "
                    f"T{trial}"
                ),
            )

            # 종료점 표시
            end = df.iloc[-1]

            ax.scatter(
                end["relative_x_m"],
                end["relative_y_m"],
                s=30,
            )

    # ---------------------------------------------
    # Commanded ideal rectangle
    #
    # 주의:
    # 실제 ground truth가 아니라 명령상 목표 경로
    # ---------------------------------------------

    first = summary.iloc[0]

    length = float(
        first["long_distance_m"]
    )

    width = float(
        first["short_distance_m"]
    )

    direction = str(
        first["turn_direction"]
    ).lower()

    y_sign = (
        1.0
        if direction == "ccw"
        else -1.0
    )

    ideal_x = [
        0.0,
        length,
        length,
        0.0,
        0.0,
    ]

    ideal_y = [
        0.0,
        0.0,
        y_sign * width,
        y_sign * width,
        0.0,
    ]

    ax.plot(
        ideal_x,
        ideal_y,
        linestyle="--",
        linewidth=2.0,
        label="Commanded rectangle",
    )

    ax.scatter(
        [0.0],
        [0.0],
        marker="o",
        s=70,
        label="Start",
    )

    ax.set_title(
        "Odometry trajectory: Baseline vs EKF"
    )

    ax.set_xlabel(
        "Relative X [m]"
    )

    ax.set_ylabel(
        "Relative Y [m]"
    )

    # 매우 중요:
    # X/Y 축 실제 비율을 동일하게 유지
    ax.set_aspect(
        "equal",
        adjustable="box",
    )

    ax.grid(
        True,
        alpha=0.3,
    )

    ax.legend()

    fig.tight_layout()

    fig.savefig(
        output_dir
        / "trajectory_comparison.png",
        dpi=300,
        bbox_inches="tight",
    )

    fig.savefig(
        output_dir
        / "trajectory_comparison.pdf",
        bbox_inches="tight",
    )

    plt.close(fig)


# =========================================================
# Plot 2
# Final position error
#
# Individual trials + mean ± SD
# =========================================================

def plot_position_error(
    summary,
    output_dir,
):

    fig, ax = plt.subplots(
        figsize=(7, 6)
    )

    conditions = [
        "baseline",
        "ekf",
    ]

    for x, condition in enumerate(
        conditions
    ):

        values = (
            pd.to_numeric(
                summary[
                    summary["condition"]
                    == condition
                ]["position_error_m"]
            )
            .to_numpy()
            * 100.0
        )

        offsets = np.linspace(
            -0.07,
            0.07,
            len(values),
        )

        ax.scatter(
            np.full(
                len(values),
                x,
            )
            + offsets,
            values,
            s=65,
            label=(
                "Individual trials"
                if x == 0
                else None
            ),
        )

        mean = np.mean(
            values
        )

        std = (
            np.std(
                values,
                ddof=1,
            )
            if len(values) > 1
            else 0.0
        )

        ax.errorbar(
            x,
            mean,
            yerr=std,
            marker="D",
            markersize=8,
            capsize=7,
            linewidth=2,
            label=(
                "Mean ± SD"
                if x == 0
                else None
            ),
        )

        ax.text(
            x,
            mean,
            f"  {mean:.2f} cm",
            va="center",
        )

    ax.set_xticks(
        [0, 1]
    )

    ax.set_xticklabels(
        [
            "Baseline",
            "EKF",
        ]
    )

    ax.set_ylabel(
        "Final position error [cm]"
    )

    ax.set_title(
        "Closed-loop odometry position error"
    )

    ax.grid(
        True,
        axis="y",
        alpha=0.3,
    )

    ax.legend()

    fig.tight_layout()

    fig.savefig(
        output_dir
        / "position_error.png",
        dpi=300,
        bbox_inches="tight",
    )

    fig.savefig(
        output_dir
        / "position_error.pdf",
        bbox_inches="tight",
    )

    plt.close(fig)


# =========================================================
# Plot 3
# Turn-by-turn yaw accumulation
# =========================================================

def plot_yaw_accumulation(
    yaw_df,
    output_dir,
):

    if yaw_df.empty:
        print(
            "WARNING: No turn yaw data found."
        )
        return

    fig, ax = plt.subplots(
        figsize=(8, 6)
    )

    for condition in [
        "baseline",
        "ekf",
    ]:

        condition_df = yaw_df[
            yaw_df["condition"]
            == condition
        ]

        if condition_df.empty:
            continue

        stats = (
            condition_df
            .groupby("turn")[
                "yaw_error_abs_deg"
            ]
            .agg(
                [
                    "mean",
                    "std",
                ]
            )
            .reset_index()
        )

        stats["std"] = (
            stats["std"]
            .fillna(0.0)
        )

        ax.errorbar(
            stats["turn"],
            stats["mean"],
            yerr=stats["std"],
            marker="o",
            capsize=5,
            linewidth=2,
            label=condition.title(),
        )

    ax.set_xticks(
        [
            1,
            2,
            3,
            4,
        ]
    )

    ax.set_xticklabels(
        [
            "90°",
            "180°",
            "270°",
            "360°",
        ]
    )

    ax.set_xlabel(
        "Completed rotation"
    )

    ax.set_ylabel(
        "Absolute yaw error [deg]"
    )

    ax.set_title(
        "Yaw error accumulation during rectangle turns"
    )

    ax.grid(
        True,
        alpha=0.3,
    )

    ax.legend()

    fig.tight_layout()

    fig.savefig(
        output_dir
        / "yaw_accumulation.png",
        dpi=300,
        bbox_inches="tight",
    )

    fig.savefig(
        output_dir
        / "yaw_accumulation.pdf",
        bbox_inches="tight",
    )

    plt.close(fig)


# =========================================================
# Console report
# =========================================================

def print_report(
    stats,
    comparison,
):

    print()
    print(
        "=" * 65
    )

    print(
        "ODOMETRY DRIFT ANALYSIS"
    )

    print(
        "=" * 65
    )

    for _, row in stats.iterrows():

        print()

        print(
            row["condition"].upper()
        )

        print(
            f"  Trials         : "
            f"{int(row['n'])}"
        )

        print(
            f"  Position Error : "
            f"{row['position_error_mean_cm']:.2f}"
            f" ± "
            f"{row['position_error_std_cm']:.2f}"
            f" cm"
        )

        print(
            f"  Yaw Error      : "
            f"{row['yaw_error_mean_deg']:.2f}"
            f" ± "
            f"{row['yaw_error_std_deg']:.2f}"
            f" deg"
        )

        print(
            f"  Drift          : "
            f"{row['drift_mean_percent']:.2f}"
            f" ± "
            f"{row['drift_std_percent']:.2f}"
            f" %"
        )

    print()
    print(
        "-" * 65
    )

    print(
        "BASELINE -> EKF REDUCTION"
    )

    print(
        "-" * 65
    )

    for _, row in comparison.iterrows():

        print(
            f"  {row['metric']:<22}"
            f"{row['reduction_percent']:.2f} %"
        )

    print(
        "=" * 65
    )

    print()


# =========================================================
# Main
# =========================================================

def main():

    parser = argparse.ArgumentParser(
        description=(
            "Analyze TurtleBot3 baseline vs EKF "
            "odometry drift experiments."
        )
    )

    parser.add_argument(
    "--input-dir",
    type=Path,
    default=Path(__file__).resolve().parent,
    help=(
        "Directory containing original "
        "summary/trace CSV files."
    ),
)

    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help=(
            "Analysis output directory. "
            "Default: <input-dir>/analysis"
        ),
    )

    args = parser.parse_args()

    input_dir = (
        args.input_dir
        .expanduser()
        .resolve()
    )

    if args.output_dir is None:

        output_dir = (
            input_dir
            / "analysis"
        )

    else:

        output_dir = (
            args.output_dir
            .expanduser()
            .resolve()
        )

    figures_dir = (
        output_dir
        / "figures"
    )

    output_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    figures_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    # ---------------------------------------------
    # Load
    # ---------------------------------------------

    summary = load_summaries(
        input_dir
    )

    trace = load_traces(
        input_dir
    )

    # ---------------------------------------------
    # Validate
    # ---------------------------------------------

    validate_dataset(
        summary,
        trace,
    )

    # ---------------------------------------------
    # Save combined raw data
    # ---------------------------------------------

    summary.to_csv(
        output_dir
        / "summary_all.csv",
        index=False,
    )

    trace.to_csv(
        output_dir
        / "trace_all.csv",
        index=False,
    )

    # ---------------------------------------------
    # Statistics
    # ---------------------------------------------

    stats = build_condition_stats(
        summary
    )

    stats.to_csv(
        output_dir
        / "condition_stats.csv",
        index=False,
    )

    comparison = (
        build_comparison_summary(
            stats
        )
    )

    comparison.to_csv(
        output_dir
        / "comparison_summary.csv",
        index=False,
    )

    # ---------------------------------------------
    # Turn yaw analysis
    # ---------------------------------------------

    yaw_df = extract_turn_yaw_errors(
        summary,
        trace,
    )

    yaw_df.to_csv(
        output_dir
        / "turn_yaw_errors.csv",
        index=False,
    )

    # ---------------------------------------------
    # Graphs
    # ---------------------------------------------

    plot_trajectory(
        summary,
        trace,
        figures_dir,
    )

    plot_position_error(
        summary,
        figures_dir,
    )

    plot_yaw_accumulation(
        yaw_df,
        figures_dir,
    )

    # ---------------------------------------------
    # Console
    # ---------------------------------------------

    print_report(
        stats,
        comparison,
    )

    print(
        f"Analysis directory:\n"
        f"{output_dir}"
    )

    print()

    print(
        "Generated figures:"
    )

    print(
        f"  {figures_dir / 'trajectory_comparison.png'}"
    )

    print(
        f"  {figures_dir / 'position_error.png'}"
    )

    print(
        f"  {figures_dir / 'yaw_accumulation.png'}"
    )


if __name__ == "__main__":
    main()