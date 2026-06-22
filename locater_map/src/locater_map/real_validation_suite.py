from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from datetime import datetime
from math import isfinite
from pathlib import Path
from typing import Any

from .data_model import RobotFrame
from .dt35_analysis import analyze_dt35_frames, summarize_residuals, write_residual_rows_csv, write_residual_summary_json
from .dt35_calibration_advisor import (
    build_calibration_advice,
    write_calibration_advice_csv,
    write_calibration_advice_markdown,
    write_calibration_advice_summary,
)
from .field_model_audit import build_field_model_audit, write_field_model_audit
from .fusion_model import FusionConfig, load_frames_csv, write_frames_csv
from .model_validation import validate_model_log, write_validation_report
from .path_diagnostics import generate_path_diagnostic, write_path_diagnostic_csv, write_path_diagnostic_summary


@dataclass(slots=True)
class RealValidationArtifacts:
    suite_report_json: str
    model_validation_json: str
    fused_frames_csv: str
    path_report_csv: str
    path_report_json: str
    dt35_residuals_csv: str
    dt35_residuals_json: str
    dt35_advice_csv: str
    dt35_advice_json: str
    dt35_advice_md: str
    field_model_audit_json: str

    def to_dict(self) -> dict[str, str]:
        return asdict(self)


@dataclass(slots=True)
class RealValidationSuiteResult:
    input_csv: str
    output_dir: str
    is_synthetic: bool
    real_validation_passed: bool
    completion_candidate: bool
    checks: dict[str, bool]
    thresholds: dict[str, float]
    notes: list[str]
    frame_summary: dict[str, Any]
    pose_error: dict[str, Any]
    dt35_residuals: dict[str, Any]
    dt35_advice: dict[str, Any]
    dt35_quality: dict[str, Any]
    path_summary: dict[str, Any]
    field_model_self_check: dict[str, Any]
    artifacts: RealValidationArtifacts

    def to_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["artifacts"] = self.artifacts.to_dict()
        return _json_safe(payload)


def run_real_validation_suite(
    config: dict[str, Any],
    csv_path: str | Path,
    output_dir: str | Path | None,
    fusion_cfg: FusionConfig,
    *,
    start_side: str | None = None,
    start_policy: str | None = None,
) -> RealValidationSuiteResult:
    source = Path(csv_path)
    frames = load_frames_csv(source)
    out_dir = _resolve_output_dir(config, source, output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    field_audit_path = out_dir / "field_model_audit.json"
    model_report_path = out_dir / "model_validation.json"
    fused_frames_path = out_dir / "fused_frames.csv"
    path_report_path = out_dir / "path_report.csv"
    path_summary_path = out_dir / "path_report.json"
    residual_rows_path = out_dir / "dt35_residuals.csv"
    residual_summary_path = out_dir / "dt35_residuals.json"
    advice_csv_path = out_dir / "dt35_advice.csv"
    advice_summary_path = out_dir / "dt35_advice.json"
    advice_md_path = out_dir / "dt35_advice.md"
    suite_report_path = out_dir / "real_validation_suite.json"

    write_field_model_audit(field_audit_path, config)
    field_audit = build_field_model_audit(config)

    model_report, fused_frames = validate_model_log(
        config,
        frames,
        fusion_cfg,
        start_side=start_side,
        start_policy=start_policy,
    )
    write_validation_report(model_report_path, model_report)
    write_frames_csv(fused_frames_path, fused_frames)

    path_rows, path_summary, path_fused = generate_path_diagnostic(
        config,
        frames,
        fusion_cfg,
        start_side=start_side,
        start_policy=start_policy,
    )
    write_path_diagnostic_csv(path_report_path, path_rows)
    write_path_diagnostic_summary(path_summary_path, path_summary)
    # Keep the fused frame artifact from model validation as the canonical file.
    # `path_fused` is intentionally evaluated by generate_path_diagnostic.
    _ = path_fused

    residual_rows = analyze_dt35_frames(
        config,
        frames,
        pose_source="lidar",
        yaw_source="h30",
        start_side=start_side,
        start_policy=start_policy,
    )
    residual_summary = summarize_residuals(residual_rows)
    write_residual_rows_csv(residual_rows_path, residual_rows)
    write_residual_summary_json(residual_summary_path, residual_summary)

    thresholds = _thresholds(config)
    advice, advice_summary = build_calibration_advice(
        residual_rows,
        actionable_residual_cm=float(thresholds["dt35_advice_actionable_residual_cm"]),
        source=str(source),
    )
    write_calibration_advice_csv(advice_csv_path, advice)
    write_calibration_advice_summary(advice_summary_path, advice_summary)
    write_calibration_advice_markdown(advice_md_path, advice, advice_summary)

    checks, notes = _checks(
        config,
        frames,
        field_audit,
        model_report.to_dict(),
        path_summary.to_dict(),
        advice_summary.to_dict(),
    )
    is_synthetic = _is_synthetic(frames)
    if is_synthetic:
        notes.append("Input protocol is synthetic; this validates math regression only, not real hardware completion.")
    real_passed = (not is_synthetic) and all(checks.values())

    artifacts = RealValidationArtifacts(
        suite_report_json=str(suite_report_path),
        model_validation_json=str(model_report_path),
        fused_frames_csv=str(fused_frames_path),
        path_report_csv=str(path_report_path),
        path_report_json=str(path_summary_path),
        dt35_residuals_csv=str(residual_rows_path),
        dt35_residuals_json=str(residual_summary_path),
        dt35_advice_csv=str(advice_csv_path),
        dt35_advice_json=str(advice_summary_path),
        dt35_advice_md=str(advice_md_path),
        field_model_audit_json=str(field_audit_path),
    )
    result = RealValidationSuiteResult(
        input_csv=str(source),
        output_dir=str(out_dir),
        is_synthetic=is_synthetic,
        real_validation_passed=real_passed,
        completion_candidate=real_passed,
        checks=checks,
        thresholds=_thresholds(config),
        notes=notes,
        frame_summary=_frame_summary(frames),
        pose_error=model_report.pose_error.to_dict(),
        dt35_residuals=residual_summary.to_dict(),
        dt35_advice=advice_summary.to_dict(),
        dt35_quality=model_report.dt35_quality,
        path_summary=path_summary.to_dict(),
        field_model_self_check=field_audit.get("model_self_check", {}),
        artifacts=artifacts,
    )
    suite_report_path.write_text(json.dumps(result.to_dict(), ensure_ascii=False, indent=2), encoding="utf-8")
    return result


def _resolve_output_dir(config: dict[str, Any], source: Path, output_dir: str | Path | None) -> Path:
    if output_dir:
        return Path(output_dir)
    root = Path(config.get("_project_root", Path.cwd()))
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return root / "logs" / f"{stamp}_{source.stem}_real_validation"


def _is_synthetic(frames: list[RobotFrame]) -> bool:
    return bool(frames) and all(str(frame.protocol).startswith("synthetic") for frame in frames)


def _frame_summary(frames: list[RobotFrame]) -> dict[str, Any]:
    return {
        "frames": len(frames),
        "protocols": sorted({str(frame.protocol) for frame in frames}),
        "lidar_frames": sum(1 for frame in frames if frame.lidar_valid or frame.lidar_online),
        "h30_frames": sum(1 for frame in frames if frame.h30_valid or frame.h30_has_attitude),
        "dt35_1_frames": sum(1 for frame in frames if frame.dt35_1_valid),
        "dt35_2_frames": sum(1 for frame in frames if frame.dt35_2_valid),
        "encoder_1_seen_frames": sum(1 for frame in frames if frame.x_pulse_seen or bool(frame.status & (1 << 10))),
        "encoder_2_seen_frames": sum(1 for frame in frames if frame.y_pulse_seen or bool(frame.status & (1 << 11))),
    }


def _thresholds(config: dict[str, Any]) -> dict[str, float]:
    field_model = config.get("field_model", {})
    return {
        "residual_warn_cm": float(field_model.get("residual_warn_cm", 8.0)),
        "validation_max_fused_rms_xy_cm": float(field_model.get("validation_max_fused_rms_xy_cm", 8.0)),
        "validation_min_real_frames": float(field_model.get("validation_min_real_frames", 30.0)),
        "validation_min_dt35_sensor_frames": float(field_model.get("validation_min_dt35_sensor_frames", 5.0)),
        "validation_min_lidar_frames": float(field_model.get("validation_min_lidar_frames", 5.0)),
        "validation_min_h30_frames": float(field_model.get("validation_min_h30_frames", 5.0)),
        "dt35_advice_actionable_residual_cm": float(field_model.get("dt35_advice_actionable_residual_cm", 3.0)),
    }


def _checks(
    config: dict[str, Any],
    frames: list[RobotFrame],
    field_audit: dict[str, Any],
    model_report: dict[str, Any],
    path_summary: dict[str, Any],
    dt35_advice: dict[str, Any],
) -> tuple[dict[str, bool], list[str]]:
    frame_summary = _frame_summary(frames)
    thresholds = _thresholds(config)
    gates = model_report.get("gates", {})
    gate_checks = dict(gates.get("checks", {}))
    pose_error = model_report.get("pose_error", {})
    residuals = model_report.get("dt35_residuals", {})
    quality = model_report.get("dt35_quality", {})
    self_check = field_audit.get("model_self_check", {})

    checks = {
        "has_frames": frame_summary["frames"] >= int(thresholds["validation_min_real_frames"]),
        "not_synthetic_input": not _is_synthetic(frames),
        "field_model_self_check": bool(self_check.get("passed", False)),
        "has_lidar_reference": frame_summary["lidar_frames"] >= int(thresholds["validation_min_lidar_frames"]),
        "has_h30_yaw": frame_summary["h30_frames"] >= int(thresholds["validation_min_h30_frames"]),
        "has_dt35_1_measurements": frame_summary["dt35_1_frames"] >= int(thresholds["validation_min_dt35_sensor_frames"]),
        "has_dt35_2_measurements": frame_summary["dt35_2_frames"] >= int(thresholds["validation_min_dt35_sensor_frames"]),
        "has_both_encoder_pulse_flags": frame_summary["encoder_1_seen_frames"] > 0 and frame_summary["encoder_2_seen_frames"] > 0,
        "has_usable_dt35_geometry": bool(gate_checks.get("has_usable_dt35_geometry", False)),
        "has_fusion_usable_dt35": bool(gate_checks.get("has_fusion_usable_dt35", False)),
        "fusion_not_worse_than_raw": bool(gate_checks.get("fusion_not_worse_than_raw", False)),
        "fused_rms_within_limit": bool(gate_checks.get("fused_rms_within_limit", False)),
        "dt35_residual_within_limit": bool(gate_checks.get("dt35_residual_within_limit", False)),
        "dt35_quality_passed": bool(quality.get("passed", False)),
        "path_report_has_dt35": int(path_summary.get("dt35_fusion_allowed_frames") or 0) > 0,
        "path_report_dt35_active": int(path_summary.get("dt35_active_frames") or 0) > 0,
        "path_report_dt35_not_worse_than_no_dt35": _not_worse(
            path_summary.get("fused_rms_xy_cm"),
            path_summary.get("no_dt35_rms_xy_cm"),
        ),
        "dt35_geometry_has_no_actionable_target_shift": int(dt35_advice.get("actionable_targets") or 0) == 0,
    }
    notes = list(gates.get("notes", []))
    if not checks["has_both_encoder_pulse_flags"]:
        notes.append("Encoder pulse flags are missing; rotate both orthogonal encoder wheels during the real validation run.")
    if not checks["has_dt35_1_measurements"] or not checks["has_dt35_2_measurements"]:
        notes.append("Both DT35 sensors need valid frames; check UART5 wiring/address if either side is missing.")
    if not checks["field_model_self_check"]:
        notes.append("Field model self-check failed; inspect field_model_audit.json failed_checks.")
    if not checks["path_report_dt35_active"]:
        notes.append("DT35 did not produce any pose correction in the path report; inspect dt35 valid flags, residual gate, and target geometry.")
    if not checks["path_report_dt35_not_worse_than_no_dt35"]:
        notes.append("DT35 fusion is worse than the no-DT35 baseline; inspect path_report.json dt35_improvement and residual groups.")
    if not checks["dt35_geometry_has_no_actionable_target_shift"]:
        notes.append("DT35/H30 are treated as accurate; inspect dt35_advice.md and adjust field-model targets or mounting geometry instead of changing DT35 scale or H30 yaw offset.")

    raw_rms = pose_error.get("raw_rms_xy_cm")
    fused_rms = pose_error.get("fused_rms_xy_cm")
    residual_rms = residuals.get("rms_residual_cm")
    if _is_number(raw_rms) and _is_number(fused_rms):
        notes.append(f"Pose RMS raw={float(raw_rms):.3f}cm fused={float(fused_rms):.3f}cm.")
    if _is_number(residual_rms):
        notes.append(f"DT35 residual RMS={float(residual_rms):.3f}cm.")
    worst_advice_rms = dt35_advice.get("worst_rms_residual_cm")
    if _is_number(worst_advice_rms):
        notes.append(
            f"DT35 advice actionable_targets={int(dt35_advice.get('actionable_targets') or 0)} "
            f"worst_target={dt35_advice.get('worst_target', '')} "
            f"worst_rms={float(worst_advice_rms):.3f}cm."
        )
    no_dt35_rms = path_summary.get("no_dt35_rms_xy_cm")
    path_fused_rms = path_summary.get("fused_rms_xy_cm")
    if _is_number(no_dt35_rms) and _is_number(path_fused_rms):
        notes.append(f"Path report RMS no_dt35={float(no_dt35_rms):.3f}cm fused={float(path_fused_rms):.3f}cm.")
    return checks, notes


def _not_worse(candidate: Any, baseline: Any, tolerance_cm: float = 1.0e-6) -> bool:
    if not _is_number(candidate) or not _is_number(baseline):
        return False
    return float(candidate) <= float(baseline) + tolerance_cm


def _is_number(value: Any) -> bool:
    try:
        return isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _json_safe(value: Any) -> Any:
    if isinstance(value, float):
        return value if isfinite(value) else None
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_json_safe(item) for item in value]
    return value
