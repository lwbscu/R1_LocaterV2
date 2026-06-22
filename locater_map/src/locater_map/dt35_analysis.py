from __future__ import annotations

import csv
import json
from collections import Counter
from dataclasses import asdict, dataclass
from math import isfinite, sqrt
from pathlib import Path
from typing import Any

from dataclasses import replace

from .data_model import RobotFrame
from .utils_transform import dt35_ray, heading_vector_from_front_yaw
from .utils_transform import transform_frame


@dataclass(slots=True)
class PoseSpec:
    x_cm: float
    y_cm: float
    yaw_deg: float
    label: str = ""


@dataclass(slots=True)
class DT35HitRow:
    pose_label: str
    pose_x_cm: float
    pose_y_cm: float
    pose_yaw_deg: float
    sensor_key: str
    sensor_name: str
    sensor_x_cm: float
    sensor_y_cm: float
    ray_yaw_deg: float
    ray_dx: float
    ray_dy: float
    constraint_axis: str
    correction_dx_per_cm: float
    correction_dy_per_cm: float
    measured_distance_cm: float
    expected_distance_cm: float
    residual_cm: float
    expected_target: str
    expected_target_type: str
    correction_allowed: bool
    corner_ambiguous: bool
    within_range: bool
    usable_for_correction: bool
    incidence_deg: float
    incidence_scale: float
    correction_weight: float
    expected_hit_x_cm: float
    expected_hit_y_cm: float


@dataclass(slots=True)
class CoverageSummary:
    poses: int
    rays: int
    usable_rays: int
    out_of_range_rays: int
    ignored_rays: int
    grazing_filtered_rays: int
    corner_ambiguous_rays: int
    no_hit_rays: int
    constraint_axis_counts: dict[str, int]
    risk_counts: dict[str, int]
    sensor_axis_counts: dict[str, int]
    sensor_risk_counts: dict[str, int]
    yaw_axis_counts: dict[str, int]
    target_type_counts: dict[str, int]
    target_counts: dict[str, int]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True)
class ObservabilityRow:
    pose_label: str
    pose_x_cm: float
    pose_y_cm: float
    pose_yaw_deg: float
    usable_sensor_count: int
    translation_rank: int
    constraint_state: str
    principal_axis_dx: float
    principal_axis_dy: float
    principal_axis_label: str
    eigen_major: float
    eigen_minor: float
    condition_ratio: float
    sensor_1_usable: bool
    sensor_1_risk: str
    sensor_1_target: str
    sensor_1_axis: str
    sensor_2_usable: bool
    sensor_2_risk: str
    sensor_2_target: str
    sensor_2_axis: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True)
class ObservabilitySummary:
    poses: int
    rank_counts: dict[str, int]
    constraint_state_counts: dict[str, int]
    usable_sensor_count_counts: dict[str, int]
    principal_axis_counts: dict[str, int]
    risk_counts: dict[str, int]
    target_type_counts: dict[str, int]
    underconstrained_poses: int
    no_dt35_poses: int
    one_dim_poses: int
    two_dim_poses: int

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True)
class DT35FrameResidualRow:
    seq: int
    source_time_ms: int
    pose_source: str
    yaw_source: str
    pose_x_cm: float
    pose_y_cm: float
    pose_yaw_deg: float
    sensor_key: str
    sensor_name: str
    sensor_valid: bool
    measured_distance_cm: float
    expected_distance_cm: float
    residual_cm: float
    residual_gate_cm: float
    residual_within_gate: bool
    abs_residual_cm: float
    expected_target: str
    expected_target_type: str
    correction_allowed: bool
    corner_ambiguous: bool
    within_range: bool
    usable_for_correction: bool
    usable_for_fusion: bool
    incidence_deg: float
    incidence_scale: float
    correction_weight: float
    sensor_x_cm: float
    sensor_y_cm: float
    ray_yaw_deg: float
    measured_hit_x_cm: float
    measured_hit_y_cm: float
    expected_hit_x_cm: float
    expected_hit_y_cm: float


@dataclass(slots=True)
class ResidualSummary:
    frames: int
    rays: int
    valid_rays: int
    usable_rays: int
    fusion_usable_rays: int
    residual_gate_rejected_rays: int
    ignored_rays: int
    out_of_range_rays: int
    grazing_filtered_rays: int
    corner_ambiguous_rays: int
    mean_abs_residual_cm: float | None
    rms_residual_cm: float | None
    max_abs_residual_cm: float | None
    target_type_counts: dict[str, int]
    target_counts: dict[str, int]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


DEFAULT_POSES = (
    PoseSpec(-3.8, 11.3, 0.6, "red_start_near_origin"),
    PoseSpec(-3.797, 11.322, 0.574, "lidar_origin_sample"),
    PoseSpec(236.403, 1.580, 0.516, "lidar_forest_side_sample"),
    PoseSpec(1.480, -162.786, -1.203, "lidar_bottom_corridor_sample"),
    PoseSpec(154.289, -92.150, -92.188, "lidar_rotated_sample"),
    PoseSpec(-360.0, 520.0, 90.0, "top_red_long_pole_ignore_check"),
    PoseSpec(154.2, -92.2, -92.1, "top_corridor_rotated_check"),
    PoseSpec(-550.0, 50.0, 0.0, "red_forest_side_check"),
    PoseSpec(-400.0, -420.0, 0.0, "red_ramp_side_check"),
    PoseSpec(40.0, 0.0, 0.0, "center_divider_check"),
)


DEFAULT_YAW_MATRIX_BASE_POSES = (
    PoseSpec(-3.797, 11.322, 0.0, "lidar_origin_sample"),
    PoseSpec(236.403, 1.580, 0.0, "lidar_forest_side_sample"),
    PoseSpec(1.480, -162.786, 0.0, "lidar_bottom_corridor_sample"),
    PoseSpec(154.289, -92.150, 0.0, "lidar_rotated_sample_xy"),
    PoseSpec(-400.0, -420.0, 0.0, "red_ramp_side_check"),
    PoseSpec(-360.0, 520.0, 0.0, "top_red_long_pole_ignore_check"),
)


def parse_pose_specs(text: str | None) -> list[PoseSpec]:
    if not text:
        return list(DEFAULT_POSES)
    poses: list[PoseSpec] = []
    for index, item in enumerate(text.split(";"), start=1):
        item = item.strip()
        if not item:
            continue
        parts = [part.strip() for part in item.split(",")]
        if len(parts) not in (3, 4):
            raise ValueError(f"pose #{index} must be x,y,yaw or x,y,yaw,label")
        label = parts[3] if len(parts) == 4 else f"pose_{index}"
        poses.append(PoseSpec(float(parts[0]), float(parts[1]), float(parts[2]), label))
    if not poses:
        raise ValueError("no valid DT35 poses were provided")
    return poses


def parse_xy_pose_specs(text: str | None) -> list[PoseSpec]:
    if not text:
        return list(DEFAULT_YAW_MATRIX_BASE_POSES)
    poses: list[PoseSpec] = []
    for index, item in enumerate(text.split(";"), start=1):
        item = item.strip()
        if not item:
            continue
        parts = [part.strip() for part in item.split(",")]
        if len(parts) not in (2, 3):
            raise ValueError(f"xy pose #{index} must be x,y or x,y,label")
        label = parts[2] if len(parts) == 3 else f"xy_pose_{index}"
        poses.append(PoseSpec(float(parts[0]), float(parts[1]), 0.0, label))
    if not poses:
        raise ValueError("no valid DT35 xy poses were provided")
    return poses


def generate_yaw_matrix_poses(base_poses: list[PoseSpec], yaws_deg: list[float]) -> list[PoseSpec]:
    poses: list[PoseSpec] = []
    for base in base_poses:
        for yaw in yaws_deg:
            poses.append(PoseSpec(base.x_cm, base.y_cm, yaw, f"{base.label}_yaw{yaw:g}"))
    return poses


def analyze_dt35_hits(config: dict[str, Any], poses: list[PoseSpec]) -> list[DT35HitRow]:
    field_model = _field_model_from_config(config)
    dt35_cfg = config.get("dt35", {})
    rows: list[DT35HitRow] = []
    for pose in poses:
        for sensor_key in ("sensor_1", "sensor_2"):
            sensor_cfg = dt35_cfg.get(sensor_key, {})
            ray = dt35_ray(
                pose.x_cm,
                pose.y_cm,
                pose.yaw_deg,
                sensor_cfg,
                _nominal_distance_mm(sensor_cfg),
                field_model,
            )
            expected_distance = float(ray["expected_distance_cm"])
            max_range = float(ray["max_range_cm"])
            within_range = isfinite(expected_distance) and 0.0 < expected_distance <= max_range
            correction_allowed = bool(ray["correction_allowed"])
            ray_dx, ray_dy = heading_vector_from_front_yaw(float(ray["ray_yaw_deg"]))
            rows.append(
                DT35HitRow(
                    pose_label=pose.label,
                    pose_x_cm=pose.x_cm,
                    pose_y_cm=pose.y_cm,
                    pose_yaw_deg=pose.yaw_deg,
                    sensor_key=sensor_key,
                    sensor_name=str(ray["name"]),
                    sensor_x_cm=float(ray["sensor_x_cm"]),
                    sensor_y_cm=float(ray["sensor_y_cm"]),
                    ray_yaw_deg=float(ray["ray_yaw_deg"]),
                    ray_dx=ray_dx,
                    ray_dy=ray_dy,
                    constraint_axis=_constraint_axis(ray_dx, ray_dy),
                    correction_dx_per_cm=-ray_dx,
                    correction_dy_per_cm=-ray_dy,
                    measured_distance_cm=float("nan"),
                    expected_distance_cm=expected_distance,
                    residual_cm=float("nan"),
                    expected_target=str(ray["expected_target"]),
                    expected_target_type=str(ray["expected_target_type"]),
                    correction_allowed=correction_allowed,
                    corner_ambiguous=bool(ray.get("corner_ambiguous", False)),
                    within_range=within_range,
                    usable_for_correction=correction_allowed and within_range,
                    incidence_deg=float(ray["incidence_deg"]),
                    incidence_scale=float(ray["incidence_scale"]),
                    correction_weight=float(ray["correction_weight"]),
                    expected_hit_x_cm=float(ray["expected_hit_x_cm"]),
                    expected_hit_y_cm=float(ray["expected_hit_y_cm"]),
                )
            )
    return rows


def analyze_dt35_frames(
    config: dict[str, Any],
    frames: list[RobotFrame],
    pose_source: str = "lidar",
    yaw_source: str = "h30",
    start_side: str | None = None,
    start_policy: str | None = None,
) -> list[DT35FrameResidualRow]:
    field_model = _field_model_from_config(config)
    dt35_cfg = config.get("dt35", {})
    residual_gate_cm = _dt35_residual_gate_cm(config)
    rows: list[DT35FrameResidualRow] = []
    for frame in frames:
        display = _display_frame(config, frame, start_side, start_policy)
        pose_x, pose_y = _pose_xy(display, pose_source)
        pose_yaw = _pose_yaw(display, yaw_source)
        for sensor_key, distance_mm, sensor_valid in (
            ("sensor_1", display.dt35_1_mm, display.dt35_1_valid),
            ("sensor_2", display.dt35_2_mm, display.dt35_2_valid),
        ):
            sensor_cfg = dt35_cfg.get(sensor_key, {})
            ray = dt35_ray(pose_x, pose_y, pose_yaw, sensor_cfg, distance_mm, field_model)
            expected_distance = float(ray["expected_distance_cm"])
            residual = float(ray["residual_cm"])
            max_range = float(ray["max_range_cm"])
            within_range = isfinite(expected_distance) and 0.0 < expected_distance <= max_range
            correction_allowed = bool(ray["correction_allowed"])
            residual_ok = isfinite(residual)
            geometry_usable = bool(sensor_valid) and correction_allowed and within_range and residual_ok
            residual_within_gate = residual_ok and abs(residual) <= residual_gate_cm
            rows.append(
                DT35FrameResidualRow(
                    seq=int(display.seq),
                    source_time_ms=int(display.source_time_ms),
                    pose_source=pose_source,
                    yaw_source=yaw_source,
                    pose_x_cm=pose_x,
                    pose_y_cm=pose_y,
                    pose_yaw_deg=pose_yaw,
                    sensor_key=sensor_key,
                    sensor_name=str(ray["name"]),
                    sensor_valid=bool(sensor_valid),
                    measured_distance_cm=float(ray["distance_cm"]) if sensor_valid else float("nan"),
                    expected_distance_cm=expected_distance,
                    residual_cm=residual,
                    residual_gate_cm=residual_gate_cm,
                    residual_within_gate=residual_within_gate,
                    abs_residual_cm=abs(residual) if residual_ok else float("nan"),
                    expected_target=str(ray["expected_target"]),
                    expected_target_type=str(ray["expected_target_type"]),
                    correction_allowed=correction_allowed,
                    corner_ambiguous=bool(ray.get("corner_ambiguous", False)),
                    within_range=within_range,
                    usable_for_correction=geometry_usable,
                    usable_for_fusion=geometry_usable and residual_within_gate,
                    incidence_deg=float(ray["incidence_deg"]),
                    incidence_scale=float(ray["incidence_scale"]),
                    correction_weight=float(ray["correction_weight"]),
                    sensor_x_cm=float(ray["sensor_x_cm"]),
                    sensor_y_cm=float(ray["sensor_y_cm"]),
                    ray_yaw_deg=float(ray["ray_yaw_deg"]),
                    measured_hit_x_cm=float(ray["hit_x_cm"]),
                    measured_hit_y_cm=float(ray["hit_y_cm"]),
                    expected_hit_x_cm=float(ray["expected_hit_x_cm"]),
                    expected_hit_y_cm=float(ray["expected_hit_y_cm"]),
                )
            )
    return rows


def apply_display_policy(
    config: dict[str, Any],
    frame: RobotFrame,
    start_side: str | None = None,
    start_policy: str | None = None,
) -> RobotFrame:
    return _display_frame(config, frame, start_side, start_policy)


def generate_grid_poses(
    x_min_cm: float,
    x_max_cm: float,
    y_min_cm: float,
    y_max_cm: float,
    step_cm: float,
    yaws_deg: list[float],
) -> list[PoseSpec]:
    if step_cm <= 0.0:
        raise ValueError("step_cm must be positive")
    poses: list[PoseSpec] = []
    y = y_min_cm
    while y <= y_max_cm + 1.0e-6:
        x = x_min_cm
        while x <= x_max_cm + 1.0e-6:
            for yaw in yaws_deg:
                poses.append(PoseSpec(x, y, yaw, f"x{x:.0f}_y{y:.0f}_yaw{yaw:.0f}"))
            x += step_cm
        y += step_cm
    return poses


def summarize_coverage(rows: list[DT35HitRow]) -> CoverageSummary:
    target_types = Counter(row.expected_target_type or "no_hit" for row in rows)
    targets = Counter(row.expected_target or "no_hit" for row in rows)
    axis = Counter(row.constraint_axis or "unknown" for row in rows)
    risk = Counter(_risk_label(row) for row in rows)
    sensor_axis = Counter(f"{row.sensor_key}:{row.constraint_axis or 'unknown'}" for row in rows)
    sensor_risk = Counter(f"{row.sensor_key}:{_risk_label(row)}" for row in rows)
    yaw_axis = Counter(f"yaw_{row.pose_yaw_deg:.0f}:{row.constraint_axis or 'unknown'}" for row in rows)
    return CoverageSummary(
        poses=len({(row.pose_x_cm, row.pose_y_cm, row.pose_yaw_deg, row.pose_label) for row in rows}),
        rays=len(rows),
        usable_rays=sum(1 for row in rows if row.usable_for_correction),
        out_of_range_rays=sum(1 for row in rows if row.expected_target and not row.within_range),
        ignored_rays=sum(1 for row in rows if row.expected_target_type == "ignore"),
        grazing_filtered_rays=sum(1 for row in rows if _is_grazing_filtered(row)),
        corner_ambiguous_rays=sum(1 for row in rows if row.corner_ambiguous),
        no_hit_rays=sum(1 for row in rows if not row.expected_target),
        constraint_axis_counts=dict(sorted(axis.items())),
        risk_counts=dict(sorted(risk.items())),
        sensor_axis_counts=dict(sorted(sensor_axis.items())),
        sensor_risk_counts=dict(sorted(sensor_risk.items())),
        yaw_axis_counts=dict(sorted(yaw_axis.items())),
        target_type_counts=dict(sorted(target_types.items())),
        target_counts=dict(sorted(targets.items())),
    )


def analyze_observability(rows: list[DT35HitRow]) -> list[ObservabilityRow]:
    grouped: dict[tuple[float, float, float, str], list[DT35HitRow]] = {}
    for row in rows:
        key = (row.pose_x_cm, row.pose_y_cm, row.pose_yaw_deg, row.pose_label)
        grouped.setdefault(key, []).append(row)

    out: list[ObservabilityRow] = []
    for (x_cm, y_cm, yaw_deg, label), group in grouped.items():
        sensors = {row.sensor_key: row for row in group}
        usable = [row for row in group if row.usable_for_correction]
        a00 = sum(row.correction_weight * row.ray_dx * row.ray_dx for row in usable)
        a01 = sum(row.correction_weight * row.ray_dx * row.ray_dy for row in usable)
        a11 = sum(row.correction_weight * row.ray_dy * row.ray_dy for row in usable)
        major, minor, axis_dx, axis_dy = _principal_axis(a00, a01, a11)
        rank = _translation_rank(major, minor)
        axis_label = _constraint_axis(axis_dx, axis_dy) if rank > 0 else "none"
        state = _constraint_state(rank, axis_label)
        condition = major / minor if minor > 1.0e-9 else float("inf") if major > 1.0e-9 else float("nan")
        sensor_1 = sensors.get("sensor_1")
        sensor_2 = sensors.get("sensor_2")
        out.append(
            ObservabilityRow(
                pose_label=label,
                pose_x_cm=x_cm,
                pose_y_cm=y_cm,
                pose_yaw_deg=yaw_deg,
                usable_sensor_count=len(usable),
                translation_rank=rank,
                constraint_state=state,
                principal_axis_dx=axis_dx if rank > 0 else float("nan"),
                principal_axis_dy=axis_dy if rank > 0 else float("nan"),
                principal_axis_label=axis_label,
                eigen_major=major,
                eigen_minor=minor,
                condition_ratio=condition,
                sensor_1_usable=bool(sensor_1 and sensor_1.usable_for_correction),
                sensor_1_risk=_risk_label(sensor_1) if sensor_1 else "missing",
                sensor_1_target=sensor_1.expected_target if sensor_1 else "",
                sensor_1_axis=sensor_1.constraint_axis if sensor_1 else "",
                sensor_2_usable=bool(sensor_2 and sensor_2.usable_for_correction),
                sensor_2_risk=_risk_label(sensor_2) if sensor_2 else "missing",
                sensor_2_target=sensor_2.expected_target if sensor_2 else "",
                sensor_2_axis=sensor_2.constraint_axis if sensor_2 else "",
            )
        )
    return sorted(out, key=lambda row: (row.pose_y_cm, row.pose_x_cm, row.pose_yaw_deg, row.pose_label))


def summarize_observability(rows: list[ObservabilityRow], hit_rows: list[DT35HitRow] | None = None) -> ObservabilitySummary:
    rank_counts = Counter(str(row.translation_rank) for row in rows)
    state_counts = Counter(row.constraint_state for row in rows)
    usable_counts = Counter(str(row.usable_sensor_count) for row in rows)
    axis_counts = Counter(row.principal_axis_label for row in rows)
    risks: Counter[str] = Counter()
    targets: Counter[str] = Counter()
    if hit_rows is not None:
        risks.update(_risk_label(row) for row in hit_rows)
        targets.update(row.expected_target_type or "no_hit" for row in hit_rows)
    else:
        for row in rows:
            risks.update((row.sensor_1_risk, row.sensor_2_risk))
    return ObservabilitySummary(
        poses=len(rows),
        rank_counts=dict(sorted(rank_counts.items())),
        constraint_state_counts=dict(sorted(state_counts.items())),
        usable_sensor_count_counts=dict(sorted(usable_counts.items())),
        principal_axis_counts=dict(sorted(axis_counts.items())),
        risk_counts=dict(sorted(risks.items())),
        target_type_counts=dict(sorted(targets.items())),
        underconstrained_poses=sum(1 for row in rows if row.translation_rank < 2),
        no_dt35_poses=sum(1 for row in rows if row.translation_rank == 0),
        one_dim_poses=sum(1 for row in rows if row.translation_rank == 1),
        two_dim_poses=sum(1 for row in rows if row.translation_rank >= 2),
    )


def summarize_residuals(rows: list[DT35FrameResidualRow]) -> ResidualSummary:
    residuals = [row.residual_cm for row in rows if row.usable_for_fusion and isfinite(row.residual_cm)]
    target_types = Counter(row.expected_target_type or "no_hit" for row in rows)
    targets = Counter(row.expected_target or "no_hit" for row in rows)
    return ResidualSummary(
        frames=len({row.seq for row in rows}),
        rays=len(rows),
        valid_rays=sum(1 for row in rows if row.sensor_valid),
        usable_rays=sum(1 for row in rows if row.usable_for_correction),
        fusion_usable_rays=sum(1 for row in rows if row.usable_for_fusion),
        residual_gate_rejected_rays=sum(
            1 for row in rows if row.usable_for_correction and not row.residual_within_gate
        ),
        ignored_rays=sum(1 for row in rows if row.expected_target_type == "ignore"),
        out_of_range_rays=sum(1 for row in rows if row.expected_target and not row.within_range),
        grazing_filtered_rays=sum(1 for row in rows if _is_grazing_filtered(row)),
        corner_ambiguous_rays=sum(1 for row in rows if row.corner_ambiguous),
        mean_abs_residual_cm=(sum(abs(item) for item in residuals) / len(residuals)) if residuals else None,
        rms_residual_cm=((sum(item * item for item in residuals) / len(residuals)) ** 0.5) if residuals else None,
        max_abs_residual_cm=max((abs(item) for item in residuals), default=None),
        target_type_counts=dict(sorted(target_types.items())),
        target_counts=dict(sorted(targets.items())),
    )


def write_coverage_summary_json(path: str | Path, summary: CoverageSummary) -> None:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(summary.to_dict(), ensure_ascii=False, indent=2), encoding="utf-8")


def write_observability_rows_csv(path: str | Path, rows: list[ObservabilityRow]) -> None:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(asdict(rows[0]).keys()) if rows else _observability_fieldnames())
        writer.writeheader()
        for row in rows:
            writer.writerow(asdict(row))


def write_observability_summary_json(path: str | Path, summary: ObservabilitySummary) -> None:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(summary.to_dict(), ensure_ascii=False, indent=2), encoding="utf-8")


def write_residual_rows_csv(path: str | Path, rows: list[DT35FrameResidualRow]) -> None:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(asdict(rows[0]).keys()) if rows else _residual_fieldnames())
        writer.writeheader()
        for row in rows:
            writer.writerow(asdict(row))


def write_residual_summary_json(path: str | Path, summary: ResidualSummary) -> None:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(summary.to_dict(), ensure_ascii=False, indent=2), encoding="utf-8")


def write_hit_rows_csv(path: str | Path, rows: list[DT35HitRow]) -> None:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(asdict(rows[0]).keys()) if rows else _fieldnames())
        writer.writeheader()
        for row in rows:
            writer.writerow(asdict(row))


def print_hit_rows(rows: list[DT35HitRow]) -> None:
    print("pose,sensor,target,type,risk,axis,geom_allowed,corner_ambiguous,within_range,usable,incidence_deg,weight,expected_cm,ray_yaw,ray_dx,ray_dy,corr_dx_per_cm,corr_dy_per_cm,hit_x,hit_y")
    for row in rows:
        allowed = "1" if row.correction_allowed else "0"
        within = "1" if row.within_range else "0"
        usable = "1" if row.usable_for_correction else "0"
        print(
            f"{row.pose_label},{row.sensor_key},{row.expected_target},{row.expected_target_type},"
            f"{_risk_label(row)},{row.constraint_axis},"
            f"{allowed},{int(row.corner_ambiguous)},{within},{usable},{row.incidence_deg:.2f},{row.correction_weight:.3f},"
            f"{row.expected_distance_cm:.2f},{row.ray_yaw_deg:.2f},{row.ray_dx:.3f},{row.ray_dy:.3f},"
            f"{row.correction_dx_per_cm:.3f},{row.correction_dy_per_cm:.3f},"
            f"{row.expected_hit_x_cm:.2f},{row.expected_hit_y_cm:.2f}"
        )


def print_coverage_summary(summary: CoverageSummary) -> None:
    print(
        f"poses={summary.poses} rays={summary.rays} usable={summary.usable_rays} "
        f"out_of_range={summary.out_of_range_rays} ignore={summary.ignored_rays} "
        f"grazing_filtered={summary.grazing_filtered_rays} corner_ambiguous={summary.corner_ambiguous_rays} "
        f"no_hit={summary.no_hit_rays}"
    )
    print("target_type_counts=" + json.dumps(summary.target_type_counts, ensure_ascii=False, sort_keys=True))
    print("constraint_axis_counts=" + json.dumps(summary.constraint_axis_counts, ensure_ascii=False, sort_keys=True))
    print("risk_counts=" + json.dumps(summary.risk_counts, ensure_ascii=False, sort_keys=True))
    print("sensor_axis_counts=" + json.dumps(summary.sensor_axis_counts, ensure_ascii=False, sort_keys=True))


def print_observability_summary(summary: ObservabilitySummary) -> None:
    print(
        f"poses={summary.poses} underconstrained={summary.underconstrained_poses} "
        f"rank0={summary.no_dt35_poses} rank1={summary.one_dim_poses} rank2={summary.two_dim_poses}"
    )
    print("rank_counts=" + json.dumps(summary.rank_counts, ensure_ascii=False, sort_keys=True))
    print("constraint_state_counts=" + json.dumps(summary.constraint_state_counts, ensure_ascii=False, sort_keys=True))
    print("principal_axis_counts=" + json.dumps(summary.principal_axis_counts, ensure_ascii=False, sort_keys=True))
    print("usable_sensor_count_counts=" + json.dumps(summary.usable_sensor_count_counts, ensure_ascii=False, sort_keys=True))
    print("risk_counts=" + json.dumps(summary.risk_counts, ensure_ascii=False, sort_keys=True))


def print_residual_summary(summary: ResidualSummary) -> None:
    print(
        f"frames={summary.frames} rays={summary.rays} valid={summary.valid_rays} usable={summary.usable_rays} "
        f"fusion_usable={summary.fusion_usable_rays} gate_rejected={summary.residual_gate_rejected_rays} "
        f"out_of_range={summary.out_of_range_rays} ignore={summary.ignored_rays} "
        f"grazing_filtered={summary.grazing_filtered_rays} corner_ambiguous={summary.corner_ambiguous_rays} "
        f"mean_abs_residual_cm={summary.mean_abs_residual_cm} rms_residual_cm={summary.rms_residual_cm} "
        f"max_abs_residual_cm={summary.max_abs_residual_cm}"
    )
    print("target_type_counts=" + json.dumps(summary.target_type_counts, ensure_ascii=False, sort_keys=True))


def _field_model_from_config(config: dict[str, Any]) -> dict[str, Any]:
    model = dict(config.get("field_model", {}))
    map_cfg = config.get("map", {})
    model.setdefault("enabled", True)
    model.setdefault("use_field_boundary", True)
    model.setdefault("field_width_cm", map_cfg.get("field_width_cm", 1215.0))
    model.setdefault("field_height_cm", map_cfg.get("field_height_cm", 1210.0))
    return model


def _nominal_distance_mm(sensor_cfg: dict[str, Any]) -> float:
    max_range_cm = float(sensor_cfg.get("max_range_cm", 250.0))
    return max_range_cm * 10.0


def _fieldnames() -> list[str]:
    return [field for field in DT35HitRow.__dataclass_fields__]


def _residual_fieldnames() -> list[str]:
    return [field for field in DT35FrameResidualRow.__dataclass_fields__]


def _observability_fieldnames() -> list[str]:
    return [field for field in ObservabilityRow.__dataclass_fields__]


def _is_grazing_filtered(row: DT35HitRow | DT35FrameResidualRow) -> bool:
    return (
        row.expected_target_type in ("usable_wall", "solid_obstacle")
        and row.within_range
        and not row.correction_allowed
    )


def _constraint_axis(ray_dx: float, ray_dy: float) -> str:
    abs_x = abs(ray_dx)
    abs_y = abs(ray_dy)
    if abs_x >= 0.85 and abs_y < 0.5:
        return "x"
    if abs_y >= 0.85 and abs_x < 0.5:
        return "y"
    return "xy"


def _risk_label(row: DT35HitRow | DT35FrameResidualRow) -> str:
    if not row.expected_target:
        return "no_hit"
    if row.expected_target_type == "ignore":
        return "ignored_geometry"
    if not row.within_range:
        return "out_of_range"
    if row.corner_ambiguous:
        return "corner_ambiguous"
    if _is_grazing_filtered(row):
        return "grazing_filtered"
    if hasattr(row, "usable_for_fusion") and row.usable_for_correction and not row.usable_for_fusion:
        return "high_residual_rejected"
    if row.usable_for_correction:
        return "usable"
    return "skipped"


def _dt35_residual_gate_cm(config: dict[str, Any]) -> float:
    display = config.get("display", {})
    field_model = config.get("field_model", {})
    if "dt35_residual_gate_cm" in field_model:
        return float(field_model["dt35_residual_gate_cm"])
    return float(display.get("live_fusion_dt35_residual_gate_cm", 40.0))


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


def _display_frame(
    config: dict[str, Any],
    frame: RobotFrame,
    start_side: str | None,
    start_policy: str | None,
) -> RobotFrame:
    transformed = transform_frame(frame, config.get("transform", {}))
    robot_cfg = config.get("robot", {})
    side = start_side or str(robot_cfg.get("default_start_side", "none"))
    policy = start_policy or str(robot_cfg.get("start_pose_policy", "off"))
    if side not in ("red", "blue") or policy == "off":
        return transformed
    if policy == "auto_lidar_offline" and transformed.lidar_online:
        return transformed
    pose = robot_cfg.get(f"start_pose_{side}", {})
    ox = float(pose.get("x_cm", 0.0))
    oy = float(pose.get("y_cm", 0.0))
    oyaw = float(pose.get("yaw_deg", 0.0))
    from math import cos, radians, sin

    start_yaw = radians(oyaw)
    start_sin = sin(start_yaw)
    start_cos = cos(start_yaw)

    def apply_pose(x_cm: float, y_cm: float, yaw_deg: float) -> tuple[float, float, float]:
        return (
            ox + x_cm * start_cos - y_cm * start_sin,
            oy + x_cm * start_sin + y_cm * start_cos,
            yaw_deg + oyaw,
        )

    pos_x, pos_y, pos_yaw = apply_pose(transformed.pos_x_cm, transformed.pos_y_cm, transformed.pos_yaw_deg)
    calib_x, calib_y, calib_yaw = apply_pose(transformed.calib_x_cm, transformed.calib_y_cm, transformed.calib_yaw_deg)
    h30_x, h30_y, h30_yaw = apply_pose(transformed.h30_x_cm, transformed.h30_y_cm, transformed.h30_yaw_deg)
    lidar_x, lidar_y, lidar_yaw = apply_pose(transformed.lidar_x_cm, transformed.lidar_y_cm, transformed.lidar_yaw_deg)
    return replace(
        transformed,
        pos_x_cm=pos_x,
        pos_y_cm=pos_y,
        pos_yaw_deg=pos_yaw,
        calib_x_cm=calib_x,
        calib_y_cm=calib_y,
        calib_yaw_deg=calib_yaw,
        h30_x_cm=h30_x,
        h30_y_cm=h30_y,
        h30_yaw_deg=h30_yaw,
        lidar_x_cm=lidar_x,
        lidar_y_cm=lidar_y,
        lidar_yaw_deg=lidar_yaw,
        encoder_x_cm=calib_x,
        encoder_y_cm=calib_y,
    )


def _pose_xy(frame: RobotFrame, source: str) -> tuple[float, float]:
    if source == "lidar" and (frame.lidar_valid or frame.lidar_online):
        return frame.lidar_x_cm, frame.lidar_y_cm
    if source == "encoder":
        return frame.encoder_x_cm, frame.encoder_y_cm
    if source == "calib":
        return frame.calib_x_cm, frame.calib_y_cm
    return frame.pos_x_cm, frame.pos_y_cm


def _pose_yaw(frame: RobotFrame, source: str) -> float:
    if source == "h30" and frame.h30_valid:
        return frame.h30_yaw_deg
    if source == "lidar" and (frame.lidar_valid or frame.lidar_online):
        return frame.lidar_yaw_deg
    if source in ("encoder", "calib"):
        return frame.calib_yaw_deg
    return frame.pos_yaw_deg
