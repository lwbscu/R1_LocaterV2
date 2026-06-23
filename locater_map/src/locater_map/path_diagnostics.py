from __future__ import annotations

import csv
import json
from collections import Counter
from dataclasses import asdict, dataclass, replace
from math import hypot, isfinite, sqrt
from pathlib import Path
from typing import Any

from .data_model import RobotFrame
from .dt35_analysis import DT35FrameResidualRow, analyze_dt35_frames, apply_display_policy
from .fusion_model import FusionConfig, wrap_deg
from .model_validation import validate_model_log
from .synthetic_sim import SyntheticConfig, generate_synthetic_frames
from .utils_transform import heading_vector_from_front_yaw


@dataclass(slots=True)
class PathDiagnosticRow:
    seq: int
    lidar_x_cm: float
    lidar_y_cm: float
    lidar_yaw_deg: float
    raw_x_cm: float
    raw_y_cm: float
    raw_yaw_deg: float
    fused_x_cm: float
    fused_y_cm: float
    fused_yaw_deg: float
    no_dt35_x_cm: float
    no_dt35_y_cm: float
    no_dt35_yaw_deg: float
    encoder_x_cm: float
    encoder_y_cm: float
    h30_yaw_deg: float
    raw_xy_error_cm: float
    fused_xy_error_cm: float
    no_dt35_xy_error_cm: float
    raw_yaw_error_deg: float
    fused_yaw_error_deg: float
    no_dt35_yaw_error_deg: float
    fused_improvement_cm: float
    dt35_improvement_cm: float
    dt35_correction_dx_cm: float
    dt35_correction_dy_cm: float
    dt35_correction_yaw_deg: float
    dt35_correction_mag_cm: float
    dt35_correction_axis: str
    dt35_helped_frame: bool
    dt35_usable_sensor_count: int
    dt35_translation_rank: int
    dt35_constraint_state: str
    dt35_principal_axis_dx: float
    dt35_principal_axis_dy: float
    dt35_principal_axis_label: str
    dt35_condition_ratio: float
    dt35_1_valid: bool
    dt35_1_target: str
    dt35_1_type: str
    dt35_1_expected_cm: float
    dt35_1_measured_cm: float
    dt35_1_residual_cm: float
    dt35_1_ray_yaw_deg: float
    dt35_1_ray_dx: float
    dt35_1_ray_dy: float
    dt35_1_constraint_axis: str
    dt35_1_correction_dx_per_cm: float
    dt35_1_correction_dy_per_cm: float
    dt35_1_allowed: bool
    dt35_1_fusion_allowed: bool
    dt35_1_residual_gate_cm: float
    dt35_1_residual_within_gate: bool
    dt35_1_floor_hit_suspect: bool
    dt35_1_corner: bool
    dt35_2_valid: bool
    dt35_2_target: str
    dt35_2_type: str
    dt35_2_expected_cm: float
    dt35_2_measured_cm: float
    dt35_2_residual_cm: float
    dt35_2_ray_yaw_deg: float
    dt35_2_ray_dx: float
    dt35_2_ray_dy: float
    dt35_2_constraint_axis: str
    dt35_2_correction_dx_per_cm: float
    dt35_2_correction_dy_per_cm: float
    dt35_2_allowed: bool
    dt35_2_fusion_allowed: bool
    dt35_2_residual_gate_cm: float
    dt35_2_residual_within_gate: bool
    dt35_2_floor_hit_suspect: bool
    dt35_2_corner: bool

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True)
class PathDiagnosticSummary:
    frames: int
    raw_rms_xy_cm: float | None
    fused_rms_xy_cm: float | None
    no_dt35_rms_xy_cm: float | None
    raw_max_xy_cm: float | None
    fused_max_xy_cm: float | None
    no_dt35_max_xy_cm: float | None
    improved_frames: int
    worsened_frames: int
    dt35_helped_frames: int
    dt35_worsened_frames: int
    dt35_active_frames: int
    dt35_mean_correction_cm: float | None
    dt35_max_correction_cm: float | None
    dt35_valid_frames: int
    dt35_allowed_frames: int
    dt35_fusion_allowed_frames: int
    dt35_residual_gate_rejected_frames: int
    dt35_floor_hit_suspect_frames: int
    dt35_corner_frames: int
    dt35_rank_counts: dict[str, int]
    dt35_constraint_state_counts: dict[str, int]
    dt35_principal_axis_counts: dict[str, int]
    dt35_rank_error_stats: dict[str, dict[str, Any]]
    dt35_constraint_state_error_stats: dict[str, dict[str, Any]]
    dt35_principal_axis_error_stats: dict[str, dict[str, Any]]
    dt35_target_counts: dict[str, int]
    dt35_type_counts: dict[str, int]
    worst_frames: list[dict[str, Any]]
    model_validation: dict[str, Any]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def generate_synthetic_path_diagnostic(
    config: dict[str, Any],
    synthetic_cfg: SyntheticConfig,
    fusion_cfg: FusionConfig,
) -> tuple[list[PathDiagnosticRow], PathDiagnosticSummary, list[RobotFrame]]:
    truth_frames = generate_synthetic_frames(config, synthetic_cfg)
    firmware_like_frames = [
        replace(frame, pos_x_cm=frame.encoder_x_cm, pos_y_cm=frame.encoder_y_cm, pos_yaw_deg=frame.h30_yaw_deg)
        for frame in truth_frames
    ]
    return generate_path_diagnostic(config, firmware_like_frames, fusion_cfg, start_policy="off")


def generate_path_diagnostic(
    config: dict[str, Any],
    frames: list[RobotFrame],
    fusion_cfg: FusionConfig,
    *,
    start_side: str | None = None,
    start_policy: str | None = None,
) -> tuple[list[PathDiagnosticRow], PathDiagnosticSummary, list[RobotFrame]]:
    report, fused_frames = validate_model_log(config, frames, fusion_cfg, start_side=start_side, start_policy=start_policy)
    no_dt35_cfg = replace(fusion_cfg, use_dt35=False, dt35_gain=0.0)
    _no_dt35_report, no_dt35_frames = validate_model_log(config, frames, no_dt35_cfg, start_side=start_side, start_policy=start_policy)
    display_frames = [apply_display_policy(config, frame, start_side, start_policy) for frame in frames]
    dt35_rows = analyze_dt35_frames(config, frames, pose_source="lidar", yaw_source="h30", start_side=start_side, start_policy=start_policy)
    dt35_by_seq = _index_dt35_rows(dt35_rows)

    rows: list[PathDiagnosticRow] = []
    target_counter: Counter[str] = Counter()
    type_counter: Counter[str] = Counter()
    for raw, fused, no_dt35 in zip(display_frames, fused_frames, no_dt35_frames):
        sensor_1 = dt35_by_seq.get((raw.seq, "sensor_1"))
        sensor_2 = dt35_by_seq.get((raw.seq, "sensor_2"))
        for sensor in (sensor_1, sensor_2):
            if sensor is not None:
                target_counter[sensor.expected_target or "no_hit"] += 1
                type_counter[sensor.expected_target_type or "no_hit"] += 1
        raw_error = hypot(raw.pos_x_cm - raw.lidar_x_cm, raw.pos_y_cm - raw.lidar_y_cm)
        fused_error = hypot(fused.pos_x_cm - fused.lidar_x_cm, fused.pos_y_cm - fused.lidar_y_cm)
        no_dt35_error = hypot(no_dt35.pos_x_cm - no_dt35.lidar_x_cm, no_dt35.pos_y_cm - no_dt35.lidar_y_cm)
        dt35_dx = fused.pos_x_cm - no_dt35.pos_x_cm
        dt35_dy = fused.pos_y_cm - no_dt35.pos_y_cm
        dt35_mag = hypot(dt35_dx, dt35_dy)
        rows.append(
            PathDiagnosticRow(
                seq=int(raw.seq),
                lidar_x_cm=raw.lidar_x_cm,
                lidar_y_cm=raw.lidar_y_cm,
                lidar_yaw_deg=raw.lidar_yaw_deg,
                raw_x_cm=raw.pos_x_cm,
                raw_y_cm=raw.pos_y_cm,
                raw_yaw_deg=raw.pos_yaw_deg,
                fused_x_cm=fused.pos_x_cm,
                fused_y_cm=fused.pos_y_cm,
                fused_yaw_deg=fused.pos_yaw_deg,
                no_dt35_x_cm=no_dt35.pos_x_cm,
                no_dt35_y_cm=no_dt35.pos_y_cm,
                no_dt35_yaw_deg=no_dt35.pos_yaw_deg,
                encoder_x_cm=raw.encoder_x_cm,
                encoder_y_cm=raw.encoder_y_cm,
                h30_yaw_deg=raw.h30_yaw_deg,
                raw_xy_error_cm=raw_error,
                fused_xy_error_cm=fused_error,
                no_dt35_xy_error_cm=no_dt35_error,
                raw_yaw_error_deg=wrap_deg(raw.pos_yaw_deg - raw.lidar_yaw_deg),
                fused_yaw_error_deg=wrap_deg(fused.pos_yaw_deg - fused.lidar_yaw_deg),
                no_dt35_yaw_error_deg=wrap_deg(no_dt35.pos_yaw_deg - no_dt35.lidar_yaw_deg),
                fused_improvement_cm=raw_error - fused_error,
                dt35_improvement_cm=no_dt35_error - fused_error,
                dt35_correction_dx_cm=dt35_dx,
                dt35_correction_dy_cm=dt35_dy,
                dt35_correction_yaw_deg=wrap_deg(fused.pos_yaw_deg - no_dt35.pos_yaw_deg),
                dt35_correction_mag_cm=dt35_mag,
                dt35_correction_axis=_vector_axis(dt35_dx, dt35_dy),
                dt35_helped_frame=fused_error < no_dt35_error,
                **_observability_fields(sensor_1, sensor_2),
                **_sensor_fields("dt35_1", sensor_1),
                **_sensor_fields("dt35_2", sensor_2),
            )
        )

    raw_errors = [row.raw_xy_error_cm for row in rows]
    fused_errors = [row.fused_xy_error_cm for row in rows]
    no_dt35_errors = [row.no_dt35_xy_error_cm for row in rows]
    dt35_corrections = [row.dt35_correction_mag_cm for row in rows]
    summary = PathDiagnosticSummary(
        frames=len(rows),
        raw_rms_xy_cm=_rms(raw_errors),
        fused_rms_xy_cm=_rms(fused_errors),
        no_dt35_rms_xy_cm=_rms(no_dt35_errors),
        raw_max_xy_cm=max(raw_errors, default=None),
        fused_max_xy_cm=max(fused_errors, default=None),
        no_dt35_max_xy_cm=max(no_dt35_errors, default=None),
        improved_frames=sum(1 for row in rows if row.fused_improvement_cm > 0.0),
        worsened_frames=sum(1 for row in rows if row.fused_improvement_cm < 0.0),
        dt35_helped_frames=sum(1 for row in rows if row.dt35_improvement_cm > 0.0),
        dt35_worsened_frames=sum(1 for row in rows if row.dt35_improvement_cm < 0.0),
        dt35_active_frames=sum(1 for row in rows if row.dt35_correction_mag_cm > 1.0e-6 or abs(row.dt35_correction_yaw_deg) > 1.0e-6),
        dt35_mean_correction_cm=(sum(dt35_corrections) / len(dt35_corrections)) if dt35_corrections else None,
        dt35_max_correction_cm=max(dt35_corrections, default=None),
        dt35_valid_frames=sum(1 for row in rows if row.dt35_1_valid or row.dt35_2_valid),
        dt35_allowed_frames=sum(1 for row in rows if row.dt35_1_allowed or row.dt35_2_allowed),
        dt35_fusion_allowed_frames=sum(1 for row in rows if row.dt35_1_fusion_allowed or row.dt35_2_fusion_allowed),
        dt35_residual_gate_rejected_frames=sum(
            1
            for row in rows
            if (row.dt35_1_allowed and not row.dt35_1_residual_within_gate)
            or (row.dt35_2_allowed and not row.dt35_2_residual_within_gate)
        ),
        dt35_floor_hit_suspect_frames=sum(
            1 for row in rows if row.dt35_1_floor_hit_suspect or row.dt35_2_floor_hit_suspect
        ),
        dt35_corner_frames=sum(1 for row in rows if row.dt35_1_corner or row.dt35_2_corner),
        dt35_rank_counts=dict(sorted(Counter(str(row.dt35_translation_rank) for row in rows).items())),
        dt35_constraint_state_counts=dict(sorted(Counter(row.dt35_constraint_state for row in rows).items())),
        dt35_principal_axis_counts=dict(sorted(Counter(row.dt35_principal_axis_label for row in rows).items())),
        dt35_rank_error_stats=_group_error_stats(rows, lambda row: str(row.dt35_translation_rank)),
        dt35_constraint_state_error_stats=_group_error_stats(rows, lambda row: row.dt35_constraint_state),
        dt35_principal_axis_error_stats=_group_error_stats(rows, lambda row: row.dt35_principal_axis_label),
        dt35_target_counts=dict(sorted(target_counter.items())),
        dt35_type_counts=dict(sorted(type_counter.items())),
        worst_frames=[row.to_dict() for row in sorted(rows, key=lambda item: item.fused_xy_error_cm, reverse=True)[:10]],
        model_validation=report.to_dict(),
    )
    return rows, summary, fused_frames


def write_path_diagnostic_csv(path: str | Path, rows: list[PathDiagnosticRow]) -> None:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=_row_fieldnames())
        writer.writeheader()
        for row in rows:
            writer.writerow(row.to_dict())


def write_path_diagnostic_summary(path: str | Path, summary: PathDiagnosticSummary) -> None:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(_json_safe(summary.to_dict()), ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")


def _index_dt35_rows(rows: list[DT35FrameResidualRow]) -> dict[tuple[int, str], DT35FrameResidualRow]:
    return {(int(row.seq), row.sensor_key): row for row in rows}


def _sensor_fields(prefix: str, row: DT35FrameResidualRow | None) -> dict[str, Any]:
    if row is None:
        return {
            f"{prefix}_valid": False,
            f"{prefix}_target": "",
            f"{prefix}_type": "",
            f"{prefix}_expected_cm": float("nan"),
            f"{prefix}_measured_cm": float("nan"),
            f"{prefix}_residual_cm": float("nan"),
            f"{prefix}_ray_yaw_deg": float("nan"),
            f"{prefix}_ray_dx": float("nan"),
            f"{prefix}_ray_dy": float("nan"),
            f"{prefix}_constraint_axis": "",
            f"{prefix}_correction_dx_per_cm": float("nan"),
            f"{prefix}_correction_dy_per_cm": float("nan"),
            f"{prefix}_allowed": False,
            f"{prefix}_fusion_allowed": False,
            f"{prefix}_residual_gate_cm": float("nan"),
            f"{prefix}_residual_within_gate": False,
            f"{prefix}_floor_hit_suspect": False,
            f"{prefix}_corner": False,
        }
    ray_dx, ray_dy = heading_vector_from_front_yaw(row.ray_yaw_deg)
    return {
        f"{prefix}_valid": row.sensor_valid,
        f"{prefix}_target": row.expected_target,
        f"{prefix}_type": row.expected_target_type,
        f"{prefix}_expected_cm": row.expected_distance_cm,
        f"{prefix}_measured_cm": row.measured_distance_cm,
        f"{prefix}_residual_cm": row.residual_cm,
        f"{prefix}_ray_yaw_deg": row.ray_yaw_deg,
        f"{prefix}_ray_dx": ray_dx,
        f"{prefix}_ray_dy": ray_dy,
        f"{prefix}_constraint_axis": _constraint_axis(ray_dx, ray_dy),
        f"{prefix}_correction_dx_per_cm": -ray_dx,
        f"{prefix}_correction_dy_per_cm": -ray_dy,
        f"{prefix}_allowed": row.usable_for_correction,
        f"{prefix}_fusion_allowed": row.usable_for_fusion,
        f"{prefix}_residual_gate_cm": row.residual_gate_cm,
        f"{prefix}_residual_within_gate": row.residual_within_gate,
        f"{prefix}_floor_hit_suspect": row.floor_hit_suspect,
        f"{prefix}_corner": row.corner_ambiguous,
    }


def _observability_fields(sensor_1: DT35FrameResidualRow | None, sensor_2: DT35FrameResidualRow | None) -> dict[str, Any]:
    usable_rows = [row for row in (sensor_1, sensor_2) if row is not None and row.usable_for_correction]
    a00 = 0.0
    a01 = 0.0
    a11 = 0.0
    for row in usable_rows:
        ray_dx, ray_dy = heading_vector_from_front_yaw(row.ray_yaw_deg)
        weight = max(0.0, float(row.correction_weight))
        a00 += weight * ray_dx * ray_dx
        a01 += weight * ray_dx * ray_dy
        a11 += weight * ray_dy * ray_dy
    major, minor, axis_dx, axis_dy = _principal_axis(a00, a01, a11)
    rank = _translation_rank(major, minor)
    axis_label = _constraint_axis(axis_dx, axis_dy) if rank > 0 else "none"
    condition = major / minor if minor > 1.0e-9 else float("inf") if major > 1.0e-9 else float("nan")
    return {
        "dt35_usable_sensor_count": len(usable_rows),
        "dt35_translation_rank": rank,
        "dt35_constraint_state": _constraint_state(rank, axis_label),
        "dt35_principal_axis_dx": axis_dx if rank > 0 else float("nan"),
        "dt35_principal_axis_dy": axis_dy if rank > 0 else float("nan"),
        "dt35_principal_axis_label": axis_label,
        "dt35_condition_ratio": condition,
    }


def _constraint_axis(ray_dx: float, ray_dy: float) -> str:
    abs_x = abs(ray_dx)
    abs_y = abs(ray_dy)
    if abs_x >= 0.85 and abs_y < 0.5:
        return "x"
    if abs_y >= 0.85 and abs_x < 0.5:
        return "y"
    return "xy"


def _vector_axis(dx: float, dy: float) -> str:
    mag = hypot(dx, dy)
    if mag <= 1.0e-6:
        return "none"
    return _constraint_axis(dx / mag, dy / mag)


def _principal_axis(a00: float, a01: float, a11: float) -> tuple[float, float, float, float]:
    trace = a00 + a11
    delta = sqrt(max(0.0, (a00 - a11) * (a00 - a11) + 4.0 * a01 * a01))
    major = 0.5 * (trace + delta)
    minor = 0.5 * (trace - delta)
    if major <= 1.0e-12:
        return 0.0, 0.0, float("nan"), float("nan")
    if abs(a01) > 1.0e-12:
        vx = a01
        vy = major - a00
    elif a00 >= a11:
        vx, vy = 1.0, 0.0
    else:
        vx, vy = 0.0, 1.0
    length = sqrt(vx * vx + vy * vy)
    if length <= 1.0e-12:
        vx, vy = 1.0, 0.0
    else:
        vx /= length
        vy /= length
    if vx < -1.0e-9 or (abs(vx) <= 1.0e-9 and vy < 0.0):
        vx = -vx
        vy = -vy
    return major, max(0.0, minor), vx, vy


def _translation_rank(major: float, minor: float) -> int:
    if major <= 1.0e-6:
        return 0
    if minor <= max(1.0e-6, major * 0.02):
        return 1
    return 2


def _constraint_state(rank: int, axis_label: str) -> str:
    if rank <= 0:
        return "rank0_no_dt35"
    if rank == 1:
        return f"rank1_{axis_label}"
    return "rank2_xy"


def _row_fieldnames() -> list[str]:
    return [field for field in PathDiagnosticRow.__dataclass_fields__]


def _rms(values: list[float]) -> float | None:
    if not values:
        return None
    return (sum(value * value for value in values) / len(values)) ** 0.5


def _group_error_stats(rows: list[PathDiagnosticRow], key_fn) -> dict[str, dict[str, Any]]:
    groups: dict[str, list[PathDiagnosticRow]] = {}
    for row in rows:
        groups.setdefault(str(key_fn(row)), []).append(row)
    return {key: _error_stats(group_rows) for key, group_rows in sorted(groups.items())}


def _error_stats(rows: list[PathDiagnosticRow]) -> dict[str, Any]:
    raw_errors = [row.raw_xy_error_cm for row in rows]
    fused_errors = [row.fused_xy_error_cm for row in rows]
    improvements = [row.fused_improvement_cm for row in rows]
    return {
        "frames": len(rows),
        "raw_rms_xy_cm": _rms(raw_errors),
        "fused_rms_xy_cm": _rms(fused_errors),
        "raw_max_xy_cm": max(raw_errors, default=None),
        "fused_max_xy_cm": max(fused_errors, default=None),
        "mean_improvement_cm": sum(improvements) / len(improvements) if improvements else None,
        "improved_frames": sum(1 for row in rows if row.fused_improvement_cm > 0.0),
        "worsened_frames": sum(1 for row in rows if row.fused_improvement_cm < 0.0),
    }


def _json_safe(value: Any) -> Any:
    if isinstance(value, float):
        return value if isfinite(value) else None
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_json_safe(item) for item in value]
    return value
