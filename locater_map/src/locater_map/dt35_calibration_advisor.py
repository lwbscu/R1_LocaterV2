from __future__ import annotations

import csv
import json
from collections import defaultdict
from dataclasses import asdict, dataclass
from math import isfinite, sqrt
from pathlib import Path
from typing import Any

from .dt35_analysis import DT35FrameResidualRow


@dataclass(slots=True)
class DT35TargetCalibrationAdvice:
    expected_target: str
    expected_target_type: str
    frames: int
    fusion_usable_frames: int
    sensors: str
    mean_residual_cm: float
    mean_abs_residual_cm: float
    rms_residual_cm: float
    max_abs_residual_cm: float
    mean_hit_dx_cm: float
    mean_hit_dy_cm: float
    suggestion_axis: str
    suggested_shift_cm: float
    suggested_action: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True)
class DT35CalibrationAdviceSummary:
    targets: int
    total_frames: int
    actionable_targets: int
    worst_target: str
    worst_rms_residual_cm: float | None
    ignored_targets: int
    generated_from: str
    notes: list[str]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def build_calibration_advice(
    rows: list[DT35FrameResidualRow],
    *,
    min_frames: int = 3,
    actionable_residual_cm: float = 3.0,
    source: str = "dt35_analyze",
) -> tuple[list[DT35TargetCalibrationAdvice], DT35CalibrationAdviceSummary]:
    grouped: dict[str, list[DT35FrameResidualRow]] = defaultdict(list)
    ignored_targets = 0
    for row in rows:
        if row.expected_target_type == "ignore":
            ignored_targets += 1
            continue
        if not row.sensor_valid or not row.usable_for_correction:
            continue
        if not row.expected_target or not isfinite(row.residual_cm):
            continue
        grouped[row.expected_target].append(row)

    advice = [_target_advice(target, items) for target, items in sorted(grouped.items()) if len(items) >= max(1, min_frames)]
    advice.sort(key=lambda item: (-item.rms_residual_cm, item.expected_target))
    actionable = [
        item for item in advice
        if item.frames >= max(1, min_frames)
        and item.suggestion_axis in ("x", "y")
        and abs(item.suggested_shift_cm) >= actionable_residual_cm
    ]
    worst = advice[0] if advice else None
    summary = DT35CalibrationAdviceSummary(
        targets=len(advice),
        total_frames=sum(item.frames for item in advice),
        actionable_targets=len(actionable),
        worst_target=worst.expected_target if worst else "",
        worst_rms_residual_cm=worst.rms_residual_cm if worst else None,
        ignored_targets=ignored_targets,
        generated_from=source,
        notes=_summary_notes(advice, actionable, ignored_targets, actionable_residual_cm),
    )
    return advice, summary


def write_calibration_advice_csv(path: str | Path, advice: list[DT35TargetCalibrationAdvice]) -> None:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(DT35TargetCalibrationAdvice.__dataclass_fields__.keys())
    with out.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for item in advice:
            writer.writerow(_json_safe(item.to_dict()))


def write_calibration_advice_summary(path: str | Path, summary: DT35CalibrationAdviceSummary) -> None:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(_json_safe(summary.to_dict()), ensure_ascii=False, indent=2), encoding="utf-8")


def write_calibration_advice_markdown(
    path: str | Path,
    advice: list[DT35TargetCalibrationAdvice],
    summary: DT35CalibrationAdviceSummary,
) -> None:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(build_calibration_advice_markdown(advice, summary), encoding="utf-8")


def build_calibration_advice_markdown(
    advice: list[DT35TargetCalibrationAdvice],
    summary: DT35CalibrationAdviceSummary,
) -> str:
    lines = [
        "# DT35 Calibration Advice",
        "",
        "Assumption: lidar XY, H30 yaw, and DT35 distance are trusted. Persistent residual means the field model target is probably misplaced or the ray is hitting unmodeled geometry.",
        "",
        "Summary:",
        f"- targets: {summary.targets}",
        f"- actionable targets: {summary.actionable_targets}",
        f"- worst target: {summary.worst_target}",
        f"- worst RMS residual: {summary.worst_rms_residual_cm}",
        f"- ignored target rows skipped: {summary.ignored_targets}",
        "",
    ]
    if summary.notes:
        lines.append("Notes:")
        lines.extend(f"- {note}" for note in summary.notes)
        lines.append("")
    lines.append("## Target Advice")
    lines.append("")
    for item in advice:
        lines.extend(
            [
                f"### {item.expected_target}",
                "",
                f"- Type: {item.expected_target_type}",
                f"- Frames: {item.frames}, fusion usable: {item.fusion_usable_frames}",
                f"- Sensors: {item.sensors}",
                f"- Residual mean/RMS/max: {item.mean_residual_cm:.2f} / {item.rms_residual_cm:.2f} / {item.max_abs_residual_cm:.2f} cm",
                f"- Mean measured-hit minus expected-hit: dx={item.mean_hit_dx_cm:.2f} cm, dy={item.mean_hit_dy_cm:.2f} cm",
                f"- Suggested shift: {item.suggestion_axis} {item.suggested_shift_cm:.2f} cm",
                f"- Action: {item.suggested_action}",
                "",
            ]
        )
    return "\n".join(lines).rstrip() + "\n"


def _target_advice(target: str, rows: list[DT35FrameResidualRow]) -> DT35TargetCalibrationAdvice:
    residuals = [row.residual_cm for row in rows if isfinite(row.residual_cm)]
    hit_dx = [row.measured_hit_x_cm - row.expected_hit_x_cm for row in rows if isfinite(row.measured_hit_x_cm) and isfinite(row.expected_hit_x_cm)]
    hit_dy = [row.measured_hit_y_cm - row.expected_hit_y_cm for row in rows if isfinite(row.measured_hit_y_cm) and isfinite(row.expected_hit_y_cm)]
    mean_dx = _mean(hit_dx)
    mean_dy = _mean(hit_dy)
    axis = _target_axis(target, mean_dx, mean_dy)
    suggested_shift = mean_dx if axis == "x" else mean_dy if axis == "y" else sqrt(mean_dx * mean_dx + mean_dy * mean_dy)
    target_type = rows[0].expected_target_type
    return DT35TargetCalibrationAdvice(
        expected_target=target,
        expected_target_type=target_type,
        frames=len(rows),
        fusion_usable_frames=sum(1 for row in rows if row.usable_for_fusion),
        sensors=",".join(sorted({row.sensor_key for row in rows})),
        mean_residual_cm=_mean(residuals),
        mean_abs_residual_cm=_mean([abs(item) for item in residuals]),
        rms_residual_cm=sqrt(sum(item * item for item in residuals) / len(residuals)) if residuals else 0.0,
        max_abs_residual_cm=max((abs(item) for item in residuals), default=0.0),
        mean_hit_dx_cm=mean_dx,
        mean_hit_dy_cm=mean_dy,
        suggestion_axis=axis,
        suggested_shift_cm=suggested_shift,
        suggested_action=_suggested_action(target, target_type, axis, suggested_shift),
    )


def _target_axis(target: str, mean_dx: float, mean_dy: float) -> str:
    lower = target.lower()
    if lower.endswith(("_left", "_right")) or lower in ("field_left", "field_right", "center_divider_wall"):
        return "x"
    if lower.endswith(("_top", "_bottom")) or lower in ("field_top", "field_bottom", "upper_red_r1_r2_wall", "upper_blue_r1_r2_wall", "lower_used_weapon_wall"):
        return "y"
    if abs(mean_dx) >= abs(mean_dy) * 1.8:
        return "x"
    if abs(mean_dy) >= abs(mean_dx) * 1.8:
        return "y"
    return "xy"


def _suggested_action(target: str, target_type: str, axis: str, shift: float) -> str:
    if abs(shift) < 0.5:
        return f"No target shift suggested for `{target}`; residual is below practical adjustment resolution."
    if axis == "x":
        return f"Move modeled target `{target}` by {shift:+.2f} cm along world X, then rerun residual validation."
    if axis == "y":
        return f"Move modeled target `{target}` by {shift:+.2f} cm along world Y, then rerun residual validation."
    if target_type == "solid_obstacle":
        return f"Check `{target}` obstacle footprint and hit face; average residual is diagonal/ambiguous ({shift:.2f} cm)."
    return f"Check `{target}` geometry; residual direction is not axis-aligned enough for an automatic single-axis shift."


def _summary_notes(
    advice: list[DT35TargetCalibrationAdvice],
    actionable: list[DT35TargetCalibrationAdvice],
    ignored_targets: int,
    actionable_residual_cm: float,
) -> list[str]:
    notes: list[str] = []
    if ignored_targets:
        notes.append(f"Skipped {ignored_targets} ignored-interference rows; they should not be used for geometry calibration.")
    if not advice:
        notes.append("No usable DT35 target rows with enough samples were available.")
    elif not actionable:
        notes.append(f"No target exceeded the actionable shift threshold of {actionable_residual_cm:.1f} cm.")
    else:
        notes.append("Apply only one or two largest target shifts at a time, then capture a new log and rerun validation.")
    notes.append("Do not change DT35 scale/offset or H30 yaw offset to hide a target-specific residual.")
    return notes


def _mean(values: list[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def _json_safe(value: Any) -> Any:
    if isinstance(value, float):
        return value if isfinite(value) else None
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_json_safe(item) for item in value]
    return value
