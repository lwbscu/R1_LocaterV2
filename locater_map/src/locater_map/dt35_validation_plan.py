from __future__ import annotations

import csv
import json
from collections import Counter
from dataclasses import asdict, dataclass
from math import isfinite
from pathlib import Path
from typing import Any, Callable

from .dt35_analysis import PoseSpec, analyze_dt35_hits, generate_grid_poses
from .dt35_role_report import DT35RoleRow, build_dt35_role_rows


@dataclass(slots=True)
class DT35ValidationCase:
    category: str
    priority: int
    pose_label: str
    pose_x_cm: float
    pose_y_cm: float
    pose_yaw_deg: float
    sensor_key: str
    sensor_name: str
    world_constraint_axis: str
    expected_target: str
    expected_target_type: str
    expected_distance_cm: float
    expected_hit_x_cm: float
    expected_hit_y_cm: float
    incidence_deg: float
    correction_weight: float
    risk: str
    usable_for_fusion: bool
    operator_note: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True)
class DT35ValidationPlanSummary:
    candidates: int
    selected_cases: int
    category_counts: dict[str, int]
    missing_categories: list[str]
    sensor_counts: dict[str, int]
    axis_counts: dict[str, int]
    target_type_counts: dict[str, int]
    required_categories: list[str]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


REQUIRED_CATEGORIES = [
    "usable_wall_x",
    "usable_wall_y",
    "forest_obstacle",
    "ramp_obstacle",
    "ignored_interference",
]


def generate_dt35_validation_plan(
    config: dict[str, Any],
    *,
    x_min_cm: float = -580.0,
    x_max_cm: float = 580.0,
    y_min_cm: float = -580.0,
    y_max_cm: float = 580.0,
    step_cm: float = 80.0,
    yaws_deg: list[float] | None = None,
    per_category: int = 3,
) -> tuple[list[DT35ValidationCase], DT35ValidationPlanSummary]:
    yaws = yaws_deg if yaws_deg is not None else [-180.0, -135.0, -90.0, -45.0, 0.0, 45.0, 90.0, 135.0]
    poses = generate_grid_poses(x_min_cm, x_max_cm, y_min_cm, y_max_cm, step_cm, yaws)
    role_rows = build_dt35_role_rows(analyze_dt35_hits(config, poses))
    cases: list[DT35ValidationCase] = []
    for category, predicate in _category_predicates():
        selected = _select_representatives([row for row in role_rows if predicate(row)], per_category)
        cases.extend(_case_from_role(row, category, index + 1) for index, row in enumerate(selected))
    summary = _summary(role_rows, cases)
    return cases, summary


def write_validation_plan_csv(path: str | Path, cases: list[DT35ValidationCase]) -> None:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(DT35ValidationCase.__dataclass_fields__.keys())
    with out.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for case in cases:
            writer.writerow(_json_safe(case.to_dict()))


def write_validation_plan_summary(path: str | Path, summary: DT35ValidationPlanSummary) -> None:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(_json_safe(summary.to_dict()), ensure_ascii=False, indent=2), encoding="utf-8")


def write_validation_plan_markdown(path: str | Path, cases: list[DT35ValidationCase], summary: DT35ValidationPlanSummary) -> None:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(build_validation_plan_markdown(cases, summary), encoding="utf-8")


def build_validation_plan_markdown(cases: list[DT35ValidationCase], summary: DT35ValidationPlanSummary) -> str:
    lines = [
        "# DT35 Real-Car Validation Checklist",
        "",
        "Purpose: validate the field model before using DT35 for live pose correction.",
        "",
        "Sensor assumptions:",
        "- Lidar XY is the absolute world-frame reference.",
        "- H30 yaw is the heading reference for ray casting.",
        "- DT35 distances are trusted; large residuals usually mean field geometry, ignored interference, corner hits, or unmodeled objects.",
        "- Ignored interference cases should be detected but must not correct pose.",
        "",
        "Recommended capture:",
        "1. Open the upper app or run `python main.py --record --baudrate 115200 --duration-s 20`.",
        "2. Place the robot center at each listed world pose and align H30 yaw as closely as possible.",
        "3. Keep the robot still for 2-3 seconds at each pose.",
        "4. After capture, run `python main.py --real-validation-csv logs/YYYYMMDD_HHMMSS_calib/parsed_frames.csv --lidar-stride 25 --real-validation-output-dir logs/real_validation_latest`.",
        "",
        "Summary:",
        f"- candidates: {summary.candidates}",
        f"- selected cases: {summary.selected_cases}",
        f"- missing categories: {summary.missing_categories}",
        f"- category counts: {summary.category_counts}",
        f"- sensor counts: {summary.sensor_counts}",
        f"- axis counts: {summary.axis_counts}",
        "",
        "## Cases",
        "",
    ]
    for index, case in enumerate(cases, start=1):
        lines.extend(
            [
                f"### {index}. {case.category} / {case.pose_label}",
                "",
                f"- Pose: x={case.pose_x_cm:.1f} cm, y={case.pose_y_cm:.1f} cm, yaw={case.pose_yaw_deg:.1f} deg",
                f"- Sensor: {case.sensor_key} ({case.sensor_name})",
                f"- Expected target: {case.expected_target} ({case.expected_target_type})",
                f"- Expected distance: {case.expected_distance_cm:.1f} cm",
                f"- Expected hit: x={case.expected_hit_x_cm:.1f} cm, y={case.expected_hit_y_cm:.1f} cm",
                f"- Constraint axis: {case.world_constraint_axis}",
                f"- Incidence: {case.incidence_deg:.1f} deg",
                f"- Correction weight: {case.correction_weight:.3f}",
                f"- Risk: {case.risk}",
                f"- Usable for fusion: {case.usable_for_fusion}",
                f"- Operator note: {case.operator_note}",
                "",
            ]
        )
    return "\n".join(lines).rstrip() + "\n"


def print_validation_plan_summary(summary: DT35ValidationPlanSummary) -> None:
    print(
        f"candidates={summary.candidates} selected={summary.selected_cases} "
        f"missing={summary.missing_categories}"
    )
    print("category_counts=" + json.dumps(summary.category_counts, ensure_ascii=False, sort_keys=True))
    print("axis_counts=" + json.dumps(summary.axis_counts, ensure_ascii=False, sort_keys=True))
    print("target_type_counts=" + json.dumps(summary.target_type_counts, ensure_ascii=False, sort_keys=True))


def _category_predicates() -> list[tuple[str, Callable[[DT35RoleRow], bool]]]:
    return [
        ("usable_wall_x", lambda row: row.usable_for_fusion and row.expected_target_type == "usable_wall" and row.world_constraint_axis == "x"),
        ("usable_wall_y", lambda row: row.usable_for_fusion and row.expected_target_type == "usable_wall" and row.world_constraint_axis == "y"),
        ("forest_obstacle", lambda row: row.usable_for_fusion and "forest" in row.expected_target),
        ("ramp_obstacle", lambda row: row.usable_for_fusion and "ramp" in row.expected_target),
        ("ignored_interference", lambda row: row.risk == "ignored_interference"),
        ("diagonal_xy_constraint", lambda row: row.usable_for_fusion and row.world_constraint_axis == "xy"),
    ]


def _select_representatives(rows: list[DT35RoleRow], limit: int) -> list[DT35RoleRow]:
    selected: list[DT35RoleRow] = []
    used_pose_sensor: set[tuple[str, str]] = set()
    used_targets: set[str] = set()
    for row in sorted(rows, key=_score_role):
        key = (row.pose_label, row.sensor_key)
        if key in used_pose_sensor:
            continue
        if len(selected) >= max(1, limit):
            break
        if row.expected_target in used_targets and len(selected) + 1 < max(1, limit):
            continue
        selected.append(row)
        used_pose_sensor.add(key)
        used_targets.add(row.expected_target)
    if len(selected) < max(1, limit):
        for row in sorted(rows, key=_score_role):
            key = (row.pose_label, row.sensor_key)
            if key in used_pose_sensor:
                continue
            selected.append(row)
            used_pose_sensor.add(key)
            if len(selected) >= max(1, limit):
                break
    return selected


def _score_role(row: DT35RoleRow) -> tuple[float, float, float, str]:
    distance = row.expected_distance_cm
    distance_score = abs(distance - 120.0) if isfinite(distance) else 9999.0
    incidence = row.incidence_deg if isfinite(row.incidence_deg) else 90.0
    target_bonus = 0.0 if row.expected_target_type in ("usable_wall", "solid_obstacle", "ignore") else 100.0
    return (target_bonus, distance_score, incidence, row.pose_label)


def _case_from_role(row: DT35RoleRow, category: str, priority: int) -> DT35ValidationCase:
    return DT35ValidationCase(
        category=category,
        priority=priority,
        pose_label=row.pose_label,
        pose_x_cm=row.pose_x_cm,
        pose_y_cm=row.pose_y_cm,
        pose_yaw_deg=row.pose_yaw_deg,
        sensor_key=row.sensor_key,
        sensor_name=row.sensor_name,
        world_constraint_axis=row.world_constraint_axis,
        expected_target=row.expected_target,
        expected_target_type=row.expected_target_type,
        expected_distance_cm=row.expected_distance_cm,
        expected_hit_x_cm=row.expected_hit_x_cm,
        expected_hit_y_cm=row.expected_hit_y_cm,
        incidence_deg=row.incidence_deg,
        correction_weight=row.correction_weight,
        risk=row.risk,
        usable_for_fusion=row.usable_for_fusion,
        operator_note=_operator_note(row, category),
    )


def _operator_note(row: DT35RoleRow, category: str) -> str:
    if category == "ignored_interference":
        return "Place robot here to confirm the ray is ignored; DT35 data should not correct pose in this geometry."
    if category == "forest_obstacle":
        return "Forest block should stop the DT35 ray; compare measured distance with expected target and residual."
    if category == "ramp_obstacle":
        return "Ramp footprint should stop the DT35 ray; residual may be less stable than a flat wall."
    return (
        f"Use lidar XY and H30 yaw at this pose; {row.sensor_key} should measure {row.expected_target} "
        f"and constrain world {row.world_constraint_axis}."
    )


def _summary(role_rows: list[DT35RoleRow], cases: list[DT35ValidationCase]) -> DT35ValidationPlanSummary:
    category_counts = Counter(case.category for case in cases)
    required = list(REQUIRED_CATEGORIES)
    return DT35ValidationPlanSummary(
        candidates=len(role_rows),
        selected_cases=len(cases),
        category_counts=dict(sorted(category_counts.items())),
        missing_categories=[category for category in required if category_counts.get(category, 0) <= 0],
        sensor_counts=dict(sorted(Counter(case.sensor_key for case in cases).items())),
        axis_counts=dict(sorted(Counter(case.world_constraint_axis for case in cases).items())),
        target_type_counts=dict(sorted(Counter(case.expected_target_type or "no_hit" for case in cases).items())),
        required_categories=required,
    )


def _json_safe(value: Any) -> Any:
    if isinstance(value, float):
        return value if isfinite(value) else None
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_json_safe(item) for item in value]
    return value
