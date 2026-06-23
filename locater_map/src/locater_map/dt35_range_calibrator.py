from __future__ import annotations

import csv
import json
from dataclasses import asdict, dataclass
from math import isfinite, sqrt
from pathlib import Path
from statistics import median
from typing import Any

from .data_model import RobotFrame
from .dt35_analysis import DT35FrameResidualRow, analyze_dt35_frames


@dataclass(slots=True)
class DT35RangeBiasEstimate:
    sensor_key: str
    sensor_name: str
    frames: int
    usable_frames: int
    expected_targets: str
    mean_measured_cm: float
    mean_expected_cm: float
    mean_residual_cm: float
    median_residual_cm: float
    rms_residual_cm: float
    max_abs_residual_cm: float
    std_residual_cm: float
    suggested_distance_bias_mm: float
    current_distance_bias_mm: float
    suggested_total_distance_bias_mm: float
    recommended: bool
    rejection_reason: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True)
class DT35RangeBiasSummary:
    frames: int
    sensors: int
    total_usable_frames: int
    pose_source: str
    yaw_source: str
    start_side: str
    start_policy: str
    notes: list[str]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def load_robot_frames_csv(path: str | Path) -> list[RobotFrame]:
    with Path(path).open("r", encoding="utf-8", newline="") as f:
        return [RobotFrame.from_row(row) for row in csv.DictReader(f)]


def estimate_dt35_range_bias(
    config: dict[str, Any],
    frames: list[RobotFrame],
    *,
    start_side: str | None = None,
    start_policy: str | None = None,
    pose_source: str = "lidar",
    yaw_source: str = "h30",
    min_frames: int = 5,
) -> tuple[list[DT35RangeBiasEstimate], DT35RangeBiasSummary]:
    rows = analyze_dt35_frames(
        config,
        frames,
        pose_source=pose_source,
        yaw_source=yaw_source,
        start_side=start_side,
        start_policy=start_policy,
    )
    grouped: dict[str, list[DT35FrameResidualRow]] = {}
    for row in rows:
        if not _usable_for_bias(row):
            continue
        grouped.setdefault(row.sensor_key, []).append(row)

    estimates: list[DT35RangeBiasEstimate] = []
    dt35_cfg = config.get("dt35", {})
    calibration_cfg = dt35_cfg.get("calibration", {})
    max_bias_mm = float(calibration_cfg.get("max_plausible_distance_bias_mm", 80.0))
    max_std_cm = float(calibration_cfg.get("max_residual_std_cm", 5.0))
    for sensor_key, sensor_rows in sorted(grouped.items()):
        if len(sensor_rows) < max(1, min_frames):
            continue
        sensor_cfg = dt35_cfg.get(sensor_key, {})
        estimates.append(_estimate_one_sensor(sensor_key, sensor_cfg, sensor_rows, max_bias_mm, max_std_cm))

    robot_cfg = config.get("robot", {})
    side = start_side or str(robot_cfg.get("default_start_side", "none"))
    policy = start_policy or str(robot_cfg.get("start_pose_policy", "off"))
    summary = DT35RangeBiasSummary(
        frames=len(frames),
        sensors=len(estimates),
        total_usable_frames=sum(item.usable_frames for item in estimates),
        pose_source=pose_source,
        yaw_source=yaw_source,
        start_side=side,
        start_policy=policy,
        notes=_summary_notes(estimates, side, policy),
    )
    return estimates, summary


def write_range_bias_json(path: str | Path, estimates: list[DT35RangeBiasEstimate], summary: DT35RangeBiasSummary) -> None:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "summary": _json_safe(summary.to_dict()),
        "estimates": [_json_safe(item.to_dict()) for item in estimates],
    }
    out.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


def write_range_bias_markdown(
    path: str | Path,
    estimates: list[DT35RangeBiasEstimate],
    summary: DT35RangeBiasSummary,
) -> None:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(build_range_bias_markdown(estimates, summary), encoding="utf-8")


def build_range_bias_markdown(estimates: list[DT35RangeBiasEstimate], summary: DT35RangeBiasSummary) -> str:
    lines = [
        "# DT35 Range Bias Estimate",
        "",
        "Coordinate assumption: lidar XY/YAW are local to the startup pose. The startup pose is mapped to the configured field start position before DT35 residuals are computed.",
        "",
        "Correction sign: `suggested_distance_bias_mm` should be added to the measured DT35 distance in the upper-computer model. It is not a firmware scale/offset change.",
        "",
        "Summary:",
        f"- frames: {summary.frames}",
        f"- sensors with enough usable samples: {summary.sensors}",
        f"- total usable sensor frames: {summary.total_usable_frames}",
        f"- pose source: {summary.pose_source}",
        f"- yaw source: {summary.yaw_source}",
        f"- start side/policy: {summary.start_side} / {summary.start_policy}",
        "",
    ]
    if summary.notes:
        lines.append("Notes:")
        lines.extend(f"- {note}" for note in summary.notes)
        lines.append("")
    lines.append("## Sensor Estimates")
    lines.append("")
    for item in estimates:
        lines.extend(
            [
                f"### {item.sensor_key} {item.sensor_name}",
                "",
                f"- usable frames: {item.usable_frames}/{item.frames}",
                f"- targets: {item.expected_targets}",
                f"- measured mean: {item.mean_measured_cm:.2f} cm",
                f"- expected mean: {item.mean_expected_cm:.2f} cm",
                f"- residual mean/median/RMS/max: {item.mean_residual_cm:+.2f} / {item.median_residual_cm:+.2f} / {item.rms_residual_cm:.2f} / {item.max_abs_residual_cm:.2f} cm",
                f"- residual std: {item.std_residual_cm:.2f} cm",
                f"- current distance_bias_mm: {item.current_distance_bias_mm:+.1f}",
                f"- suggested additional distance_bias_mm: {item.suggested_distance_bias_mm:+.1f}",
                f"- suggested total distance_bias_mm: {item.suggested_total_distance_bias_mm:+.1f}",
                f"- recommended to apply: {item.recommended}",
                f"- rejection reason: {item.rejection_reason or '-'}",
                "",
            ]
        )
    return "\n".join(lines).rstrip() + "\n"


def _usable_for_bias(row: DT35FrameResidualRow) -> bool:
    return (
        row.sensor_valid
        and row.usable_for_correction
        and row.usable_for_fusion
        and row.within_range
        and isfinite(row.measured_distance_cm)
        and isfinite(row.expected_distance_cm)
        and isfinite(row.residual_cm)
    )


def _estimate_one_sensor(
    sensor_key: str,
    sensor_cfg: dict[str, Any],
    rows: list[DT35FrameResidualRow],
    max_bias_mm: float,
    max_std_cm: float,
) -> DT35RangeBiasEstimate:
    measured = [row.measured_distance_cm for row in rows]
    expected = [row.expected_distance_cm for row in rows]
    residuals = [row.residual_cm for row in rows]
    mean_residual = _mean(residuals)
    std_residual = _std(residuals)
    current_bias = float(sensor_cfg.get("distance_bias_mm", 0.0))
    suggested_delta = -mean_residual * 10.0
    recommended, rejection_reason = _recommend_bias(suggested_delta, std_residual, max_bias_mm, max_std_cm)
    return DT35RangeBiasEstimate(
        sensor_key=sensor_key,
        sensor_name=str(sensor_cfg.get("name", rows[0].sensor_name)),
        frames=len(rows),
        usable_frames=len(rows),
        expected_targets=",".join(sorted({row.expected_target for row in rows if row.expected_target})),
        mean_measured_cm=_mean(measured),
        mean_expected_cm=_mean(expected),
        mean_residual_cm=mean_residual,
        median_residual_cm=median(residuals),
        rms_residual_cm=sqrt(sum(item * item for item in residuals) / len(residuals)),
        max_abs_residual_cm=max(abs(item) for item in residuals),
        std_residual_cm=std_residual,
        suggested_distance_bias_mm=suggested_delta,
        current_distance_bias_mm=current_bias,
        suggested_total_distance_bias_mm=current_bias + suggested_delta,
        recommended=recommended,
        rejection_reason=rejection_reason,
    )


def _recommend_bias(suggested_delta_mm: float, std_residual_cm: float, max_bias_mm: float, max_std_cm: float) -> tuple[bool, str]:
    if abs(suggested_delta_mm) > max_bias_mm:
        return False, (
            f"suggested bias {suggested_delta_mm:+.1f} mm exceeds plausible mounting/range error "
            f"limit {max_bias_mm:.1f} mm; treat as field-model or hit-target mismatch"
        )
    if std_residual_cm > max_std_cm:
        return False, (
            f"residual std {std_residual_cm:.2f} cm exceeds stable-bias limit {max_std_cm:.2f} cm; "
            "capture more poses or inspect obstacles"
        )
    return True, ""


def _summary_notes(estimates: list[DT35RangeBiasEstimate], side: str, policy: str) -> list[str]:
    notes = [
        "Use this as a software-model bias first; do not change the DT35 firmware scale/offset unless hardware evidence proves the sensor calibration is wrong.",
        "Bias estimates are only valid for rows whose DT35 ray hits a modeled usable wall/obstacle and is not a corner/grazing/ignored target.",
    ]
    if policy != "always_local_display":
        notes.append("Current start policy is not always_local_display; verify the input CSV was transformed into field coordinates before trusting the estimate.")
    if side not in ("red", "blue"):
        notes.append("Start side is not red/blue; field-coordinate mapping may be disabled.")
    if not estimates:
        notes.append("No sensor had enough usable samples. Capture more motion near modeled walls or reduce the minimum-frame threshold for inspection only.")
    return notes


def _mean(values: list[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def _std(values: list[float]) -> float:
    if len(values) <= 1:
        return 0.0
    mean = _mean(values)
    return sqrt(sum((item - mean) * (item - mean) for item in values) / len(values))


def _json_safe(value: Any) -> Any:
    if isinstance(value, float):
        return value if isfinite(value) else None
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_json_safe(item) for item in value]
    return value
