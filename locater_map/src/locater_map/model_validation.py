from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from collections import defaultdict
from math import hypot, isfinite
from pathlib import Path
from typing import Any

from .data_model import RobotFrame
from .dt35_analysis import analyze_dt35_frames, apply_display_policy, summarize_residuals
from .fusion_model import FusionConfig, LiveFusionFilter, wrap_deg


@dataclass(slots=True)
class PoseErrorSummary:
    frames: int = 0
    lidar_reference_frames: int = 0
    raw_rms_xy_cm: float | None = None
    fused_rms_xy_cm: float | None = None
    raw_max_xy_cm: float | None = None
    fused_max_xy_cm: float | None = None
    raw_rms_yaw_deg: float | None = None
    fused_rms_yaw_deg: float | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True)
class SensorInputSummary:
    frames: int = 0
    lidar_valid_frames: int = 0
    h30_valid_frames: int = 0
    dt35_1_valid_frames: int = 0
    dt35_2_valid_frames: int = 0
    encoder_1_pulse_seen_frames: int = 0
    encoder_2_pulse_seen_frames: int = 0

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True)
class ValidationGateSummary:
    passed: bool
    checks: dict[str, bool]
    thresholds: dict[str, float]
    notes: list[str]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True)
class ModelValidationReport:
    inputs: SensorInputSummary
    pose_error: PoseErrorSummary
    dt35_residuals: dict[str, Any]
    dt35_breakdown: dict[str, Any]
    dt35_quality: dict[str, Any]
    gates: ValidationGateSummary
    config: dict[str, Any]

    def to_dict(self) -> dict[str, Any]:
        return {
            "inputs": self.inputs.to_dict(),
            "pose_error": self.pose_error.to_dict(),
            "dt35_residuals": self.dt35_residuals,
            "dt35_breakdown": self.dt35_breakdown,
            "dt35_quality": self.dt35_quality,
            "gates": self.gates.to_dict(),
            "config": self.config,
        }


def validate_model_log(
    config: dict[str, Any],
    frames: list[RobotFrame],
    fusion_cfg: FusionConfig,
    *,
    start_side: str | None = None,
    start_policy: str | None = None,
) -> tuple[ModelValidationReport, list[RobotFrame]]:
    display_frames = [apply_display_policy(config, frame, start_side, start_policy) for frame in frames]
    fused_frames: list[RobotFrame] = []
    live_filter = LiveFusionFilter(fusion_cfg, config, use_start_transform=False)

    raw_xy_errors: list[float] = []
    fused_xy_errors: list[float] = []
    raw_yaw_errors: list[float] = []
    fused_yaw_errors: list[float] = []
    for frame in display_frames:
        fused = live_filter.process(frame)
        fused_frames.append(fused)
        if not (frame.lidar_valid or frame.lidar_online):
            continue
        raw_xy_errors.append(hypot(frame.pos_x_cm - frame.lidar_x_cm, frame.pos_y_cm - frame.lidar_y_cm))
        fused_xy_errors.append(hypot(fused.pos_x_cm - fused.lidar_x_cm, fused.pos_y_cm - fused.lidar_y_cm))
        raw_yaw_errors.append(wrap_deg(frame.pos_yaw_deg - frame.lidar_yaw_deg))
        fused_yaw_errors.append(wrap_deg(fused.pos_yaw_deg - fused.lidar_yaw_deg))

    residual_rows = analyze_dt35_frames(
        config,
        frames,
        pose_source="lidar",
        yaw_source="h30",
        start_side=start_side,
        start_policy=start_policy,
    )
    residual_summary = summarize_residuals(residual_rows)
    dt35_breakdown = {
        "by_sensor": _group_residual_rows(residual_rows, lambda row: row.sensor_key),
        "by_target_type": _group_residual_rows(residual_rows, lambda row: row.expected_target_type or "no_hit"),
        "by_target": _group_residual_rows(residual_rows, lambda row: row.expected_target or "no_hit"),
    }
    pose_summary = PoseErrorSummary(
        frames=len(display_frames),
        lidar_reference_frames=len(raw_xy_errors),
        raw_rms_xy_cm=_rms(raw_xy_errors),
        fused_rms_xy_cm=_rms(fused_xy_errors),
        raw_max_xy_cm=max(raw_xy_errors, default=None),
        fused_max_xy_cm=max(fused_xy_errors, default=None),
        raw_rms_yaw_deg=_rms(raw_yaw_errors),
        fused_rms_yaw_deg=_rms(fused_yaw_errors),
    )
    inputs = _input_summary(display_frames)
    gates = _validation_gates(config, pose_summary, residual_summary.to_dict(), inputs)
    report = ModelValidationReport(
        inputs=inputs,
        pose_error=pose_summary,
        dt35_residuals=residual_summary.to_dict(),
        dt35_breakdown=dt35_breakdown,
        dt35_quality=_dt35_quality(config, dt35_breakdown["by_target"]),
        gates=gates,
        config={
            "fusion": asdict(fusion_cfg),
            "start_side": start_side,
            "start_policy": start_policy,
            "pose_reference": "lidar",
            "yaw_reference": "lidar",
            "dt35_pose_source": "lidar",
            "dt35_yaw_source": "h30",
        },
    )
    return report, fused_frames


def write_validation_report(path: str | Path, report: ModelValidationReport) -> None:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report.to_dict(), ensure_ascii=False, indent=2), encoding="utf-8")


def _rms(values: list[float]) -> float | None:
    if not values:
        return None
    return (sum(item * item for item in values) / len(values)) ** 0.5


def _input_summary(frames: list[RobotFrame]) -> SensorInputSummary:
    return SensorInputSummary(
        frames=len(frames),
        lidar_valid_frames=sum(1 for frame in frames if frame.lidar_valid or frame.lidar_online),
        h30_valid_frames=sum(1 for frame in frames if frame.h30_valid or frame.h30_has_attitude),
        dt35_1_valid_frames=sum(1 for frame in frames if frame.dt35_1_valid),
        dt35_2_valid_frames=sum(1 for frame in frames if frame.dt35_2_valid),
        encoder_1_pulse_seen_frames=sum(1 for frame in frames if frame.x_pulse_seen or bool(frame.status & (1 << 10))),
        encoder_2_pulse_seen_frames=sum(1 for frame in frames if frame.y_pulse_seen or bool(frame.status & (1 << 11))),
    )


def _group_residual_rows(rows: list[Any], key_fn: Any) -> dict[str, dict[str, Any]]:
    groups: dict[str, list[Any]] = defaultdict(list)
    for row in rows:
        groups[str(key_fn(row) or "unknown")].append(row)
    return {key: _residual_group_stats(group_rows) for key, group_rows in sorted(groups.items())}


def _residual_group_stats(rows: list[Any]) -> dict[str, Any]:
    geometry_usable = [row for row in rows if row.usable_for_correction and isfinite(row.residual_cm)]
    fusion_usable = [row for row in rows if getattr(row, "usable_for_fusion", row.usable_for_correction) and isfinite(row.residual_cm)]
    residuals = [row.residual_cm for row in fusion_usable]
    incidences = [row.incidence_deg for row in fusion_usable if isfinite(row.incidence_deg)]
    weights = [row.correction_weight for row in fusion_usable if isfinite(row.correction_weight)]
    signed_mean = (sum(residuals) / len(residuals)) if residuals else None
    mean_abs = (sum(abs(item) for item in residuals) / len(residuals)) if residuals else None
    rms = ((sum(item * item for item in residuals) / len(residuals)) ** 0.5) if residuals else None
    return {
        "rays": len(rows),
        "valid_rays": sum(1 for row in rows if row.sensor_valid),
        "usable_rays": len(geometry_usable),
        "fusion_usable_rays": len(fusion_usable),
        "residual_gate_rejected_rays": sum(
            1 for row in rows if row.usable_for_correction and not getattr(row, "residual_within_gate", True)
        ),
        "floor_hit_suspect_rays": sum(1 for row in rows if getattr(row, "floor_hit_suspect", False)),
        "ignored_rays": sum(1 for row in rows if row.expected_target_type == "ignore"),
        "corner_ambiguous_rays": sum(1 for row in rows if getattr(row, "corner_ambiguous", False)),
        "out_of_range_rays": sum(1 for row in rows if row.expected_target and not row.within_range),
        "signed_mean_residual_cm": signed_mean,
        "mean_abs_residual_cm": mean_abs,
        "rms_residual_cm": rms,
        "max_abs_residual_cm": max((abs(item) for item in residuals), default=None),
        "mean_incidence_deg": (sum(incidences) / len(incidences)) if incidences else None,
        "mean_correction_weight": (sum(weights) / len(weights)) if weights else None,
        "valid_rate": _safe_ratio(sum(1 for row in rows if row.sensor_valid), len(rows)),
        "usable_rate": _safe_ratio(len(geometry_usable), len(rows)),
        "fusion_usable_rate": _safe_ratio(len(fusion_usable), len(rows)),
        "floor_hit_suspect_rate": _safe_ratio(sum(1 for row in rows if getattr(row, "floor_hit_suspect", False)), len(rows)),
        "corner_rate": _safe_ratio(sum(1 for row in rows if getattr(row, "corner_ambiguous", False)), len(rows)),
        "out_of_range_rate": _safe_ratio(sum(1 for row in rows if row.expected_target and not row.within_range), len(rows)),
        "ignore_rate": _safe_ratio(sum(1 for row in rows if row.expected_target_type == "ignore"), len(rows)),
    }


def _dt35_quality(config: dict[str, Any], by_target: dict[str, dict[str, Any]]) -> dict[str, Any]:
    field_model = config.get("field_model", {})
    residual_warn = float(field_model.get("residual_warn_cm", 8.0))
    min_target_rays = int(field_model.get("validation_min_target_rays", 3))
    min_valid_usable_rate = float(field_model.get("validation_min_valid_usable_rate", 0.7))
    bad_targets: list[dict[str, Any]] = []
    good_targets: list[str] = []

    for name, stats in by_target.items():
        rays = int(stats.get("rays") or 0)
        if name == "no_hit" or rays < min_target_rays:
            continue
        reasons: list[str] = []
        rms = stats.get("rms_residual_cm")
        valid_rays = int(stats.get("valid_rays") or 0)
        usable_rays = int(stats.get("usable_rays") or 0)
        fusion_usable_rays = int(stats.get("fusion_usable_rays") or 0)
        valid_usable_rate = _safe_ratio(usable_rays, valid_rays)
        valid_fusion_usable_rate = _safe_ratio(fusion_usable_rays, valid_rays)
        corner_rate = float(stats.get("corner_rate") or 0.0)
        floor_hit_rate = float(stats.get("floor_hit_suspect_rate") or 0.0)
        ignore_rate = float(stats.get("ignore_rate") or 0.0)

        if rms is not None and float(rms) > residual_warn:
            reasons.append("high_residual")
        if valid_rays > 0 and valid_usable_rate < min_valid_usable_rate and ignore_rate < 0.9:
            reasons.append("low_valid_usable_rate")
        if valid_rays > 0 and valid_fusion_usable_rate < min_valid_usable_rate and ignore_rate < 0.9:
            reasons.append("low_fusion_usable_rate")
        if corner_rate > 0.25:
            reasons.append("many_corner_hits")
        if floor_hit_rate > 0.25:
            reasons.append("many_floor_or_near_hits")

        if reasons:
            bad_targets.append({
                "target": name,
                "reasons": reasons,
                "valid_usable_rate": valid_usable_rate,
                "valid_fusion_usable_rate": valid_fusion_usable_rate,
                **stats,
            })
        elif int(stats.get("fusion_usable_rays") or 0) > 0:
            good_targets.append(name)

    bad_targets.sort(key=lambda item: (
        -len(item["reasons"]),
        -(float(item.get("rms_residual_cm") or 0.0)),
        -int(item.get("rays") or 0),
        str(item.get("target", "")),
    ))
    return {
        "passed": not bad_targets,
        "bad_targets": bad_targets,
        "good_target_count": len(good_targets),
        "good_targets": good_targets[:30],
        "thresholds": {
            "residual_warn_cm": residual_warn,
            "validation_min_target_rays": min_target_rays,
            "validation_min_valid_usable_rate": min_valid_usable_rate,
        },
    }


def _safe_ratio(num: int, den: int) -> float:
    return float(num) / float(den) if den else 0.0


def _validation_gates(
    config: dict[str, Any],
    pose: PoseErrorSummary,
    residuals: dict[str, Any],
    inputs: SensorInputSummary,
) -> ValidationGateSummary:
    residual_warn = float(config.get("field_model", {}).get("residual_warn_cm", 8.0))
    max_fused_rms = float(config.get("field_model", {}).get("validation_max_fused_rms_xy_cm", 8.0))
    min_improvement_cm = float(config.get("field_model", {}).get("validation_min_improvement_cm", 0.0))

    checks: dict[str, bool] = {}
    notes: list[str] = []

    checks["has_lidar_reference"] = pose.lidar_reference_frames > 0
    checks["has_h30_yaw"] = inputs.h30_valid_frames > 0
    checks["has_dt35_measurements"] = (inputs.dt35_1_valid_frames + inputs.dt35_2_valid_frames) > 0
    checks["has_usable_dt35_geometry"] = int(residuals.get("usable_rays") or 0) > 0
    checks["has_fusion_usable_dt35"] = int(residuals.get("fusion_usable_rays") or 0) > 0

    if pose.raw_rms_xy_cm is None or pose.fused_rms_xy_cm is None:
        checks["fusion_not_worse_than_raw"] = False
        checks["fused_rms_within_limit"] = False
    else:
        checks["fusion_not_worse_than_raw"] = pose.fused_rms_xy_cm <= pose.raw_rms_xy_cm - min_improvement_cm + 1.0e-6
        checks["fused_rms_within_limit"] = pose.fused_rms_xy_cm <= max_fused_rms

    dt35_rms = residuals.get("rms_residual_cm")
    checks["dt35_residual_within_limit"] = bool(dt35_rms is not None and float(dt35_rms) <= residual_warn)

    if not checks["has_dt35_measurements"]:
        notes.append("DT35 has no valid measurement frames; this report cannot validate DT35 correction.")
    if not checks["has_usable_dt35_geometry"]:
        notes.append("DT35 measurements did not hit usable modeled geometry; check field model, range, or pose.")
    if not checks["has_fusion_usable_dt35"]:
        notes.append("DT35 geometry was usable but no ray passed the residual fusion gate; inspect residual_gate_rejected_rays.")
    if int(residuals.get("floor_hit_suspect_rays") or 0) > 0:
        notes.append("Some DT35 rays were rejected as floor/near-hit suspects because measured distance was much shorter than the modeled target.")
    if not checks["fusion_not_worse_than_raw"]:
        notes.append("Live fusion is not better than raw firmware pose against lidar under current settings.")
    if not checks["dt35_residual_within_limit"]:
        notes.append("DT35 residual is above the configured field_model.residual_warn_cm threshold.")

    return ValidationGateSummary(
        passed=all(checks.values()),
        checks=checks,
        thresholds={
            "residual_warn_cm": residual_warn,
            "validation_max_fused_rms_xy_cm": max_fused_rms,
            "validation_min_improvement_cm": min_improvement_cm,
        },
        notes=notes,
    )
