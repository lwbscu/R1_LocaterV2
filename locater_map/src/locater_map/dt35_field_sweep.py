from __future__ import annotations

import csv
import json
from collections import Counter, defaultdict
from dataclasses import asdict, dataclass
from math import isfinite
from pathlib import Path
from typing import Any

from .dt35_analysis import PoseSpec, analyze_dt35_hits, analyze_observability, generate_grid_poses, summarize_observability
from .dt35_role_report import DT35RoleRow, build_dt35_role_rows


@dataclass(slots=True)
class DT35FieldSweepRow:
    pose_label: str
    pose_x_cm: float
    pose_y_cm: float
    pose_yaw_deg: float
    translation_rank: int
    constraint_state: str
    usable_sensor_count: int
    principal_axis_label: str
    sensor_1_risk: str
    sensor_1_target: str
    sensor_1_axis: str
    sensor_1_expected_cm: float | None
    sensor_2_risk: str
    sensor_2_target: str
    sensor_2_axis: str
    sensor_2_expected_cm: float | None
    has_usable_wall_x: bool
    has_usable_wall_y: bool
    has_forest_constraint: bool
    has_ramp_constraint: bool
    has_ignored_interference: bool
    primary_note: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True)
class DT35FieldSweepSummary:
    poses: int
    rays: int
    rank_counts: dict[str, int]
    constraint_state_counts: dict[str, int]
    target_type_counts: dict[str, int]
    risk_counts: dict[str, int]
    yaw_state_counts: dict[str, int]
    required_category_counts: dict[str, int]
    missing_categories: list[str]
    no_dt35_poses: int
    one_dim_poses: int
    two_dim_poses: int
    ignored_interference_poses: int
    forest_constraint_poses: int
    ramp_constraint_poses: int
    usable_wall_x_poses: int
    usable_wall_y_poses: int
    ignored_targets_never_corrected: bool
    model_passed: bool
    weak_pose_examples: list[dict[str, Any]]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


REQUIRED_SWEEP_CATEGORIES = [
    "usable_wall_x",
    "usable_wall_y",
    "forest_constraint",
    "ramp_constraint",
    "ignored_interference",
]


def run_dt35_field_sweep(
    config: dict[str, Any],
    *,
    x_min_cm: float = -580.0,
    x_max_cm: float = 580.0,
    y_min_cm: float = -580.0,
    y_max_cm: float = 580.0,
    step_cm: float = 100.0,
    yaws_deg: list[float] | None = None,
) -> tuple[list[DT35FieldSweepRow], DT35FieldSweepSummary]:
    yaws = yaws_deg if yaws_deg is not None else [-180.0, -135.0, -90.0, -45.0, 0.0, 45.0, 90.0, 135.0]
    poses = generate_grid_poses(x_min_cm, x_max_cm, y_min_cm, y_max_cm, step_cm, yaws)
    hit_rows = analyze_dt35_hits(config, poses)
    observability = analyze_observability(hit_rows)
    role_rows = build_dt35_role_rows(hit_rows)

    roles_by_pose: dict[tuple[str, float, float, float], list[DT35RoleRow]] = defaultdict(list)
    for role in role_rows:
        roles_by_pose[(role.pose_label, role.pose_x_cm, role.pose_y_cm, role.pose_yaw_deg)].append(role)

    rows = [_sweep_row(obs, roles_by_pose[(obs.pose_label, obs.pose_x_cm, obs.pose_y_cm, obs.pose_yaw_deg)]) for obs in observability]
    summary = _summary(rows, role_rows)
    return rows, summary


def write_field_sweep_csv(path: str | Path, rows: list[DT35FieldSweepRow]) -> None:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(DT35FieldSweepRow.__dataclass_fields__.keys())
    with out.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(_json_safe(row.to_dict()))


def write_field_sweep_summary(path: str | Path, summary: DT35FieldSweepSummary) -> None:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(_json_safe(summary.to_dict()), ensure_ascii=False, indent=2), encoding="utf-8")


def print_field_sweep_summary(summary: DT35FieldSweepSummary) -> None:
    print(
        f"poses={summary.poses} rays={summary.rays} model_passed={summary.model_passed} "
        f"missing={summary.missing_categories}"
    )
    print("rank_counts=" + json.dumps(summary.rank_counts, ensure_ascii=False, sort_keys=True))
    print("constraint_state_counts=" + json.dumps(summary.constraint_state_counts, ensure_ascii=False, sort_keys=True))
    print("required_category_counts=" + json.dumps(summary.required_category_counts, ensure_ascii=False, sort_keys=True))
    print("risk_counts=" + json.dumps(summary.risk_counts, ensure_ascii=False, sort_keys=True))


def _sweep_row(obs: Any, roles: list[DT35RoleRow]) -> DT35FieldSweepRow:
    by_sensor = {role.sensor_key: role for role in roles}
    sensor_1 = by_sensor.get("sensor_1")
    sensor_2 = by_sensor.get("sensor_2")
    return DT35FieldSweepRow(
        pose_label=obs.pose_label,
        pose_x_cm=obs.pose_x_cm,
        pose_y_cm=obs.pose_y_cm,
        pose_yaw_deg=obs.pose_yaw_deg,
        translation_rank=obs.translation_rank,
        constraint_state=obs.constraint_state,
        usable_sensor_count=obs.usable_sensor_count,
        principal_axis_label=obs.principal_axis_label,
        sensor_1_risk=sensor_1.risk if sensor_1 else "missing",
        sensor_1_target=sensor_1.expected_target if sensor_1 else "",
        sensor_1_axis=sensor_1.world_constraint_axis if sensor_1 else "",
        sensor_1_expected_cm=_finite_or_none(sensor_1.expected_distance_cm if sensor_1 else None),
        sensor_2_risk=sensor_2.risk if sensor_2 else "missing",
        sensor_2_target=sensor_2.expected_target if sensor_2 else "",
        sensor_2_axis=sensor_2.world_constraint_axis if sensor_2 else "",
        sensor_2_expected_cm=_finite_or_none(sensor_2.expected_distance_cm if sensor_2 else None),
        has_usable_wall_x=_has_usable_wall_axis(roles, "x"),
        has_usable_wall_y=_has_usable_wall_axis(roles, "y"),
        has_forest_constraint=any(role.usable_for_fusion and "forest" in role.expected_target for role in roles),
        has_ramp_constraint=any(role.usable_for_fusion and "ramp" in role.expected_target for role in roles),
        has_ignored_interference=any(role.risk == "ignored_interference" for role in roles),
        primary_note=_primary_note(obs, roles),
    )


def _summary(rows: list[DT35FieldSweepRow], roles: list[DT35RoleRow]) -> DT35FieldSweepSummary:
    category_counts = {
        "usable_wall_x": sum(1 for row in rows if row.has_usable_wall_x),
        "usable_wall_y": sum(1 for row in rows if row.has_usable_wall_y),
        "forest_constraint": sum(1 for row in rows if row.has_forest_constraint),
        "ramp_constraint": sum(1 for row in rows if row.has_ramp_constraint),
        "ignored_interference": sum(1 for row in rows if row.has_ignored_interference),
    }
    missing = [category for category in REQUIRED_SWEEP_CATEGORIES if category_counts.get(category, 0) <= 0]
    ignored_never_corrected = all(not role.usable_for_fusion for role in roles if role.risk == "ignored_interference")
    rank_counts = Counter(str(row.translation_rank) for row in rows)
    state_counts = Counter(row.constraint_state for row in rows)
    target_type_counts = Counter(role.expected_target_type or "no_hit" for role in roles)
    risk_counts = Counter(role.risk for role in roles)
    yaw_state_counts = Counter(f"yaw_{row.pose_yaw_deg:.0f}:{row.constraint_state}" for row in rows)
    return DT35FieldSweepSummary(
        poses=len(rows),
        rays=len(roles),
        rank_counts=dict(sorted(rank_counts.items())),
        constraint_state_counts=dict(sorted(state_counts.items())),
        target_type_counts=dict(sorted(target_type_counts.items())),
        risk_counts=dict(sorted(risk_counts.items())),
        yaw_state_counts=dict(sorted(yaw_state_counts.items())),
        required_category_counts=category_counts,
        missing_categories=missing,
        no_dt35_poses=sum(1 for row in rows if row.translation_rank == 0),
        one_dim_poses=sum(1 for row in rows if row.translation_rank == 1),
        two_dim_poses=sum(1 for row in rows if row.translation_rank >= 2),
        ignored_interference_poses=category_counts["ignored_interference"],
        forest_constraint_poses=category_counts["forest_constraint"],
        ramp_constraint_poses=category_counts["ramp_constraint"],
        usable_wall_x_poses=category_counts["usable_wall_x"],
        usable_wall_y_poses=category_counts["usable_wall_y"],
        ignored_targets_never_corrected=ignored_never_corrected,
        model_passed=(not missing and ignored_never_corrected and len(rows) > 0),
        weak_pose_examples=[row.to_dict() for row in _weak_examples(rows)],
    )


def _has_usable_wall_axis(roles: list[DT35RoleRow], axis: str) -> bool:
    return any(
        role.usable_for_fusion
        and role.expected_target_type == "usable_wall"
        and role.world_constraint_axis == axis
        for role in roles
    )


def _primary_note(obs: Any, roles: list[DT35RoleRow]) -> str:
    if obs.translation_rank <= 0:
        risks = ",".join(sorted({role.risk for role in roles})) or "none"
        return f"no usable DT35 translation constraint; risks={risks}"
    if any(role.risk == "ignored_interference" for role in roles):
        return "ignored interference present; do not use that ray for correction"
    if any(role.usable_for_fusion and "forest" in role.expected_target for role in roles):
        return "forest obstacle blocks at least one ray and is usable for translation correction"
    if any(role.usable_for_fusion and "ramp" in role.expected_target for role in roles):
        return "ramp footprint blocks at least one ray and is usable with reduced weight"
    if obs.translation_rank == 1:
        return f"one-dimensional DT35 constraint along {obs.principal_axis_label}; rely on lidar/encoder for the other axis"
    return "two-dimensional DT35 translation constraint"


def _weak_examples(rows: list[DT35FieldSweepRow], limit: int = 16) -> list[DT35FieldSweepRow]:
    def score(row: DT35FieldSweepRow) -> tuple[int, int, float, float, float]:
        if row.translation_rank == 0:
            level = 0
        elif row.has_ignored_interference:
            level = 1
        elif row.translation_rank == 1:
            level = 2
        else:
            level = 3
        return (level, int(abs(row.pose_yaw_deg)), row.pose_y_cm, row.pose_x_cm, row.pose_yaw_deg)

    return sorted(rows, key=score)[:limit]


def _finite_or_none(value: float | None) -> float | None:
    if value is None:
        return None
    return float(value) if isfinite(float(value)) else None


def _json_safe(value: Any) -> Any:
    if isinstance(value, float):
        return value if isfinite(value) else None
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_json_safe(item) for item in value]
    return value
