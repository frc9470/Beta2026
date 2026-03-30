#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


REQUIRED_COLUMNS = {
    "timestamp_sec",
    "run_id",
    "mode",
    "segment_index",
    "commanded_rpm",
    "commanded_volts",
    "measured_rpm",
    "battery_volts",
    "flywheel_avg_motor_volts",
    "flywheel_total_supply_current_amps",
    "flywheel_avg_stator_current_amps",
    "electrical_power_watts",
    "velocity_error_rpm",
    "at_setpoint",
    "settled",
    "aborted",
}


def load_characterization_data(input_dir: Path) -> tuple[pd.DataFrame, list[str]]:
    frames: list[pd.DataFrame] = []
    excluded: list[str] = []
    for csv_path in sorted(input_dir.glob("*.csv")):
        frame = pd.read_csv(csv_path)
        missing = REQUIRED_COLUMNS - set(frame.columns)
        if missing:
            excluded.append(f"{csv_path.name}: missing columns {sorted(missing)}")
            continue
        frame["source_file"] = csv_path.name
        frames.append(frame)
    if not frames:
        return pd.DataFrame(columns=sorted(REQUIRED_COLUMNS | {"source_file"})), excluded
    return pd.concat(frames, ignore_index=True), excluded


def load_demand_config(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {"ready_cap_sec": 0.5, "inactive_rpm": 0.0, "active_windows": []}
    text = path.read_text(encoding="utf-8").strip()
    if not text:
        return {"ready_cap_sec": 0.5, "inactive_rpm": 0.0, "active_windows": []}
    try:
        import yaml  # type: ignore

        data = yaml.safe_load(text)
    except ModuleNotFoundError:
        data = json.loads(text)
    return {
        "ready_cap_sec": float(data.get("ready_cap_sec", 0.5)),
        "inactive_rpm": float(data.get("inactive_rpm", 0.0)),
        "active_windows": list(data.get("active_windows", [])),
    }


def compute_hold_power_summary(data: pd.DataFrame) -> tuple[pd.DataFrame, pd.DataFrame]:
    filtered = data[
        (data["mode"] == "VELOCITY_HOLD_SWEEP")
        & data["settled"].astype(bool)
        & ~data["aborted"].astype(bool)
    ].copy()
    if filtered.empty:
        return filtered, pd.DataFrame(columns=["commanded_rpm", "measured_rpm", "electrical_power_watts", "samples"])

    per_segment = (
        filtered.groupby(["run_id", "segment_index", "commanded_rpm"], as_index=False)
        .agg(
            measured_rpm=("measured_rpm", "mean"),
            electrical_power_watts=("electrical_power_watts", "mean"),
            flywheel_total_supply_current_amps=("flywheel_total_supply_current_amps", "mean"),
            flywheel_avg_motor_volts=("flywheel_avg_motor_volts", "mean"),
            samples=("timestamp_sec", "count"),
        )
        .sort_values(["commanded_rpm", "run_id", "segment_index"])
    )

    summary = (
        per_segment.groupby("commanded_rpm", as_index=False)
        .agg(
            measured_rpm=("measured_rpm", "mean"),
            electrical_power_watts=("electrical_power_watts", "mean"),
            flywheel_total_supply_current_amps=("flywheel_total_supply_current_amps", "mean"),
            flywheel_avg_motor_volts=("flywheel_avg_motor_volts", "mean"),
            samples=("samples", "sum"),
        )
        .sort_values("commanded_rpm")
    )
    return per_segment, summary


def compute_voltage_step_summary(data: pd.DataFrame) -> pd.DataFrame:
    filtered = data[
        (data["mode"] == "OPEN_LOOP_VOLTAGE_SWEEP")
        & ~data["aborted"].astype(bool)
    ].copy()
    if filtered.empty:
        return filtered
    filtered["relative_time_sec"] = filtered.groupby(["run_id", "segment_index"])["timestamp_sec"].transform(
        lambda column: column - column.iloc[0]
    )
    return filtered


def integrate_energy(segment: pd.DataFrame) -> float:
    if segment.empty:
        return float("nan")
    if len(segment) == 1:
        return float(segment["electrical_power_watts"].iloc[0] * 0.02)
    return float(np.trapz(segment["electrical_power_watts"], segment["timestamp_sec"]))


def first_crossing_time(segment: pd.DataFrame, threshold_rpm: float, column: str) -> float:
    matches = segment[segment[column] >= threshold_rpm]
    if matches.empty:
        return float("nan")
    return float(matches["timestamp_sec"].iloc[0] - segment["timestamp_sec"].iloc[0])


def first_true_time(segment: pd.DataFrame, column: str) -> float:
    matches = segment[segment[column].astype(bool)]
    if matches.empty:
        return float("nan")
    return float(matches["timestamp_sec"].iloc[0] - segment["timestamp_sec"].iloc[0])


def compute_step_response_metrics(data: pd.DataFrame) -> pd.DataFrame:
    filtered = data[
        (data["mode"] == "CLOSED_LOOP_STEP_SWEEP")
        & ~data["aborted"].astype(bool)
    ].copy()
    if filtered.empty:
        return pd.DataFrame(
            columns=[
                "run_id",
                "segment_index",
                "idle_rpm",
                "target_rpm",
                "time_to_90_sec",
                "time_to_setpoint_sec",
                "overshoot_rpm",
                "step_energy_j",
            ]
        )

    metrics: list[dict[str, float]] = []
    filtered = filtered.sort_values(["run_id", "segment_index", "timestamp_sec"])
    for run_id, run_frame in filtered.groupby("run_id"):
        segments = {int(idx): segment.copy() for idx, segment in run_frame.groupby("segment_index")}
        for segment_index, segment in segments.items():
            if segment_index % 2 == 0:
                continue
            previous = segments.get(segment_index - 1)
            if previous is None or segment.empty:
                continue
            idle_rpm = float(previous["commanded_rpm"].iloc[0])
            target_rpm = float(segment["commanded_rpm"].iloc[0])
            metrics.append(
                {
                    "run_id": float(run_id),
                    "segment_index": float(segment_index),
                    "idle_rpm": idle_rpm,
                    "target_rpm": target_rpm,
                    "time_to_90_sec": first_crossing_time(segment, 0.9 * target_rpm, "measured_rpm"),
                    "time_to_setpoint_sec": first_true_time(segment, "at_setpoint"),
                    "overshoot_rpm": float(segment["measured_rpm"].max() - target_rpm),
                    "step_energy_j": integrate_energy(segment),
                }
            )
    if not metrics:
        return pd.DataFrame(columns=[
            "run_id",
            "segment_index",
            "idle_rpm",
            "target_rpm",
            "time_to_90_sec",
            "time_to_setpoint_sec",
            "overshoot_rpm",
            "step_energy_j",
        ])
    return pd.DataFrame(metrics).sort_values(["target_rpm", "idle_rpm"])


def hold_power_at_rpm(hold_summary: pd.DataFrame, rpm: float) -> float:
    if hold_summary.empty:
        return float("nan")
    return float(
        np.interp(
            rpm,
            hold_summary["commanded_rpm"].to_numpy(),
            hold_summary["electrical_power_watts"].to_numpy(),
        )
    )


def nearest_step_metric(step_metrics: pd.DataFrame, idle_rpm: float, target_rpm: float) -> pd.Series | None:
    if step_metrics.empty:
        return None
    candidates = step_metrics.copy()
    candidates["distance"] = (
        (candidates["idle_rpm"] - idle_rpm).abs() + (candidates["target_rpm"] - target_rpm).abs()
    )
    if candidates.empty:
        return None
    return candidates.sort_values("distance").iloc[0]


def compute_break_even_seconds(step_metrics: pd.DataFrame, hold_summary: pd.DataFrame) -> dict[float, float]:
    results: dict[float, float] = {}
    if step_metrics.empty or hold_summary.empty:
        return results
    baseline = step_metrics[np.isclose(step_metrics["idle_rpm"], 0.0)]
    for target_rpm, baseline_group in baseline.groupby("target_rpm"):
        baseline_energy = float(baseline_group["step_energy_j"].mean())
        best_value = float("inf")
        for _, row in step_metrics[step_metrics["target_rpm"] == target_rpm].iterrows():
            idle_rpm = float(row["idle_rpm"])
            if idle_rpm <= 0.0:
                continue
            hold_power = hold_power_at_rpm(hold_summary, idle_rpm)
            if not np.isfinite(hold_power) or hold_power <= 0.0:
                continue
            saved_energy = baseline_energy - float(row["step_energy_j"])
            if saved_energy <= 0.0:
                continue
            best_value = min(best_value, saved_energy / hold_power)
        if np.isfinite(best_value):
            results[float(target_rpm)] = float(best_value)
    return results


def recommend_active_rpm(
    step_metrics: pd.DataFrame,
    hold_summary: pd.DataFrame,
    demand_config: dict[str, Any],
) -> dict[str, Any]:
    ready_cap_sec = float(demand_config["ready_cap_sec"])
    active_windows = demand_config.get("active_windows", [])
    break_even_by_target = compute_break_even_seconds(step_metrics, hold_summary)
    if step_metrics.empty or hold_summary.empty or not active_windows:
        return {
            "inactive_rpm": float(demand_config.get("inactive_rpm", 0.0)),
            "recommended_active_rpm": None,
            "ready_cap_sec": ready_cap_sec,
            "worst_case_ready_sec": None,
            "break_even_hold_sec": min(break_even_by_target.values()) if break_even_by_target else None,
            "analyzed_target_rpms": sorted(step_metrics["target_rpm"].unique().tolist()) if not step_metrics.empty else [],
        }

    candidates = sorted(step_metrics["idle_rpm"].unique().tolist())
    best: dict[str, Any] | None = None
    for candidate_rpm in candidates:
        total_energy = 0.0
        worst_ready = 0.0
        valid = True
        for window in active_windows:
            shot_rpm = float(window["shot_rpm"])
            occurrences = int(window.get("occurrences_per_match", 1))
            hold_sec = float(window.get("pre_shot_hold_sec", 0.0))
            metric = nearest_step_metric(step_metrics, candidate_rpm, shot_rpm)
            if metric is None:
                valid = False
                break
            ready_time = float(metric["time_to_setpoint_sec"])
            if not np.isfinite(ready_time) or ready_time > ready_cap_sec:
                valid = False
                break
            hold_power = hold_power_at_rpm(hold_summary, candidate_rpm)
            if not np.isfinite(hold_power):
                valid = False
                break
            total_energy += occurrences * (float(metric["step_energy_j"]) + (hold_power * hold_sec))
            worst_ready = max(worst_ready, ready_time)
        if not valid:
            continue
        candidate = {
            "inactive_rpm": float(demand_config.get("inactive_rpm", 0.0)),
            "recommended_active_rpm": float(candidate_rpm),
            "ready_cap_sec": ready_cap_sec,
            "worst_case_ready_sec": worst_ready,
            "break_even_hold_sec": min(break_even_by_target.values()) if break_even_by_target else None,
            "analyzed_target_rpms": sorted(step_metrics["target_rpm"].unique().tolist()),
            "estimated_match_energy_j": total_energy,
        }
        if best is None or total_energy < best["estimated_match_energy_j"]:
            best = candidate
    if best is None:
        return {
            "inactive_rpm": float(demand_config.get("inactive_rpm", 0.0)),
            "recommended_active_rpm": None,
            "ready_cap_sec": ready_cap_sec,
            "worst_case_ready_sec": None,
            "break_even_hold_sec": min(break_even_by_target.values()) if break_even_by_target else None,
            "analyzed_target_rpms": sorted(step_metrics["target_rpm"].unique().tolist()),
        }
    return best


def plot_hold_power(summary: pd.DataFrame, output_path: Path) -> None:
    fig, ax = plt.subplots(figsize=(8, 5))
    if summary.empty:
        ax.text(0.5, 0.5, "No hold-sweep data", ha="center", va="center")
    else:
        ax.plot(summary["commanded_rpm"], summary["electrical_power_watts"], marker="o")
        ax.set_xlabel("Commanded RPM")
        ax.set_ylabel("Electrical Power (W)")
        ax.set_title("Flywheel Hold Power vs RPM")
        ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(output_path)
    plt.close(fig)


def plot_voltage_step_response(voltage_data: pd.DataFrame, output_path: Path) -> None:
    fig, ax = plt.subplots(figsize=(8, 5))
    if voltage_data.empty:
        ax.text(0.5, 0.5, "No open-loop voltage sweep data", ha="center", va="center")
    else:
        plotted = False
        for (run_id, segment_index), segment in voltage_data.groupby(["run_id", "segment_index"]):
            volts = float(segment["commanded_volts"].iloc[0])
            if volts <= 0.0:
                continue
            ax.plot(
                segment["relative_time_sec"],
                segment["measured_rpm"],
                label=f"Run {int(run_id)} seg {int(segment_index)} @ {volts:.1f}V",
            )
            plotted = True
        if plotted:
            ax.legend(fontsize=7)
        ax.set_xlabel("Relative Time (s)")
        ax.set_ylabel("Measured RPM")
        ax.set_title("Open-Loop Voltage Sweep Response")
        ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(output_path)
    plt.close(fig)


def plot_step_heatmap(
    step_metrics: pd.DataFrame,
    value_column: str,
    title: str,
    output_path: Path,
    colorbar_label: str,
) -> None:
    fig, ax = plt.subplots(figsize=(8, 5))
    if step_metrics.empty:
        ax.text(0.5, 0.5, "No closed-loop step sweep data", ha="center", va="center")
        fig.tight_layout()
        fig.savefig(output_path)
        plt.close(fig)
        return

    pivot = step_metrics.pivot_table(
        index="target_rpm",
        columns="idle_rpm",
        values=value_column,
        aggfunc="mean",
    ).sort_index().sort_index(axis=1)
    image = ax.imshow(pivot.to_numpy(), aspect="auto", origin="lower")
    ax.set_xticks(range(len(pivot.columns)))
    ax.set_xticklabels([f"{value:.0f}" for value in pivot.columns], rotation=45, ha="right")
    ax.set_yticks(range(len(pivot.index)))
    ax.set_yticklabels([f"{value:.0f}" for value in pivot.index])
    ax.set_xlabel("Idle RPM")
    ax.set_ylabel("Target RPM")
    ax.set_title(title)
    colorbar = fig.colorbar(image, ax=ax)
    colorbar.set_label(colorbar_label)
    fig.tight_layout()
    fig.savefig(output_path)
    plt.close(fig)


def write_summary(
    output_path: Path,
    total_files: int,
    excluded: list[str],
    hold_summary: pd.DataFrame,
    step_metrics: pd.DataFrame,
    recommendation: dict[str, Any],
) -> None:
    lines = [
        "# Shooter Power Analysis",
        "",
        f"- Runs used: {int(step_metrics['run_id'].nunique()) if not step_metrics.empty else 0}",
        f"- Source CSV files loaded: {total_files}",
        f"- Excluded inputs: {len(excluded)}",
        "",
        "## Excluded Inputs",
    ]
    if excluded:
        lines.extend([f"- {item}" for item in excluded])
    else:
        lines.append("- None")

    lines.extend([
        "",
        "## Hold-Power Curve",
    ])
    if hold_summary.empty:
        lines.append("- No settled velocity-hold data available.")
    else:
        min_row = hold_summary.iloc[0]
        max_row = hold_summary.iloc[-1]
        lines.append(
            f"- Points: {len(hold_summary)} from {min_row['commanded_rpm']:.0f} RPM to {max_row['commanded_rpm']:.0f} RPM"
        )
        lines.append(
            f"- Power span: {min_row['electrical_power_watts']:.1f} W to {max_row['electrical_power_watts']:.1f} W"
        )

    lines.extend([
        "",
        "## Recommendation",
        f"- Recommended active RPM: {recommendation['recommended_active_rpm']}",
        f"- Worst-case ready time: {recommendation['worst_case_ready_sec']}",
        f"- Ready cap: {recommendation['ready_cap_sec']}",
        f"- Break-even hold duration: {recommendation['break_even_hold_sec']}",
    ])
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_recommendation(output_path: Path, recommendation: dict[str, Any]) -> None:
    output_path.write_text(json.dumps(recommendation, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def run_analysis(input_dir: Path, output_dir: Path, demand_path: Path) -> dict[str, Any]:
    output_dir.mkdir(parents=True, exist_ok=True)
    data, excluded = load_characterization_data(input_dir)
    demand_config = load_demand_config(demand_path)
    _, hold_summary = compute_hold_power_summary(data)
    voltage_summary = compute_voltage_step_summary(data)
    step_metrics = compute_step_response_metrics(data)
    recommendation = recommend_active_rpm(step_metrics, hold_summary, demand_config)

    plot_hold_power(hold_summary, output_dir / "hold_power_vs_rpm.png")
    plot_voltage_step_response(voltage_summary, output_dir / "voltage_step_response.png")
    plot_step_heatmap(
        step_metrics,
        "time_to_setpoint_sec",
        "Ready Time Heatmap",
        output_dir / "ready_time_heatmap.png",
        "Time to Setpoint (s)",
    )
    plot_step_heatmap(
        step_metrics,
        "step_energy_j",
        "Step Energy Heatmap",
        output_dir / "step_energy_heatmap.png",
        "Integrated Energy (J)",
    )
    write_summary(
        output_dir / "summary.md",
        len(list(input_dir.glob("*.csv"))),
        excluded,
        hold_summary,
        step_metrics,
        recommendation,
    )
    write_recommendation(output_dir / "recommendation.json", recommendation)
    return recommendation


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analyze shooter characterization CSV logs.")
    parser.add_argument("--input", type=Path, required=True, help="Directory containing pulled CSV logs")
    parser.add_argument("--output", type=Path, required=True, help="Directory to write plots and summary files")
    parser.add_argument(
        "--demand",
        type=Path,
        default=Path("analysis/shooter_demand.yaml"),
        help="Demand-config file used for resting RPM recommendations",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    recommendation = run_analysis(args.input, args.output, args.demand)
    print(json.dumps(recommendation, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
