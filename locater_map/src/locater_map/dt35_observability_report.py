from __future__ import annotations

from pathlib import Path
from typing import Any

from .dt35_analysis import (
    ObservabilityRow,
    ObservabilitySummary,
    analyze_dt35_hits,
    analyze_observability,
    generate_grid_poses,
    summarize_observability,
    write_observability_rows_csv,
    write_observability_summary_json,
)


def generate_dt35_observability_report(
    config: dict[str, Any],
    *,
    x_min_cm: float = -580.0,
    x_max_cm: float = 580.0,
    y_min_cm: float = -580.0,
    y_max_cm: float = 580.0,
    step_cm: float = 100.0,
    yaws_deg: list[float] | None = None,
) -> tuple[list[ObservabilityRow], ObservabilitySummary]:
    yaws = yaws_deg if yaws_deg is not None else [-180.0, -135.0, -90.0, -45.0, 0.0, 45.0, 90.0, 135.0]
    hit_rows = analyze_dt35_hits(
        config,
        generate_grid_poses(x_min_cm, x_max_cm, y_min_cm, y_max_cm, step_cm, yaws),
    )
    rows = analyze_observability(hit_rows)
    return rows, summarize_observability(rows, hit_rows)


def write_dt35_observability_report(
    csv_path: str | Path,
    json_path: str | Path,
    md_path: str | Path,
    rows: list[ObservabilityRow],
    summary: ObservabilitySummary,
) -> None:
    write_observability_rows_csv(csv_path, rows)
    write_observability_summary_json(json_path, summary)
    out = Path(md_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(build_observability_markdown(rows, summary), encoding="utf-8")


def build_observability_markdown(rows: list[ObservabilityRow], summary: ObservabilitySummary) -> str:
    lines = [
        "# DT35 Observability Matrix",
        "",
        "Purpose: show how many translation dimensions the two side-facing DT35 sensors can constrain at each lidar XY and H30 yaw pose.",
        "",
        "Interpretation:",
        "- `rank0_no_dt35`: no usable DT35 translation constraint; rely on lidar and encoder/H30 prediction.",
        "- `rank1_x`: DT35 mainly clamps world X; world Y still comes from lidar anchors and encoder interpolation.",
        "- `rank1_y`: DT35 mainly clamps world Y; world X still comes from lidar anchors and encoder interpolation.",
        "- `rank1_xy`: DT35 clamps one diagonal component; this is still one-dimensional, not full 2D localization.",
        "- `rank2_xy`: two independent DT35 constraints are available; this is rare with left/right side-facing sensors.",
        "",
        "Summary:",
        f"- poses: {summary.poses}",
        f"- rank counts: {summary.rank_counts}",
        f"- constraint states: {summary.constraint_state_counts}",
        f"- principal axes: {summary.principal_axis_counts}",
        f"- risk counts: {summary.risk_counts}",
        f"- underconstrained poses: {summary.underconstrained_poses}",
        f"- no-DT35 poses: {summary.no_dt35_poses}",
        f"- one-dimensional poses: {summary.one_dim_poses}",
        f"- two-dimensional poses: {summary.two_dim_poses}",
        "",
        "## Representative Weak Poses",
        "",
    ]
    weak = [row for row in rows if row.translation_rank < 2]
    for row in weak[:24]:
        lines.extend(
            [
                f"### {row.pose_label}",
                "",
                f"- Pose: x={row.pose_x_cm:.1f} cm, y={row.pose_y_cm:.1f} cm, yaw={row.pose_yaw_deg:.1f} deg",
                f"- Constraint state: {row.constraint_state}",
                f"- Principal axis: {row.principal_axis_label}",
                f"- Usable sensor count: {row.usable_sensor_count}",
                f"- Sensor 1: usable={row.sensor_1_usable}, risk={row.sensor_1_risk}, target={row.sensor_1_target}, axis={row.sensor_1_axis}",
                f"- Sensor 2: usable={row.sensor_2_usable}, risk={row.sensor_2_risk}, target={row.sensor_2_target}, axis={row.sensor_2_axis}",
                "",
            ]
        )
    return "\n".join(lines).rstrip() + "\n"
