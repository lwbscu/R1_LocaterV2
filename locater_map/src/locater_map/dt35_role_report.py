from __future__ import annotations

import csv
import json
from collections import Counter
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

from .dt35_analysis import DT35HitRow


@dataclass(slots=True)
class DT35RoleRow:
    pose_label: str
    pose_x_cm: float
    pose_y_cm: float
    pose_yaw_deg: float
    sensor_key: str
    sensor_name: str
    robot_mount: str
    local_ray_direction: str
    world_ray_yaw_deg: float
    world_ray_dx: float
    world_ray_dy: float
    world_constraint_axis: str
    pose_correction_direction: str
    expected_target: str
    expected_target_type: str
    expected_distance_cm: float
    expected_hit_x_cm: float
    expected_hit_y_cm: float
    incidence_deg: float
    correction_weight: float
    risk: str
    usable_for_fusion: bool
    explanation: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True)
class DT35RoleSummary:
    poses: int
    rows: int
    usable_rows: int
    sensor_risk_counts: dict[str, int]
    sensor_axis_counts: dict[str, int]
    yaw_axis_counts: dict[str, int]
    target_type_counts: dict[str, int]
    target_counts: dict[str, int]
    usable_forest_rows: int
    usable_ramp_rows: int
    ignored_rows: int
    out_of_range_rows: int
    corner_rows: int
    grazing_rows: int

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def build_dt35_role_rows(hit_rows: list[DT35HitRow]) -> list[DT35RoleRow]:
    return [_role_row(row) for row in hit_rows]


def summarize_dt35_roles(rows: list[DT35RoleRow]) -> DT35RoleSummary:
    return DT35RoleSummary(
        poses=len({(row.pose_label, row.pose_x_cm, row.pose_y_cm, row.pose_yaw_deg) for row in rows}),
        rows=len(rows),
        usable_rows=sum(1 for row in rows if row.usable_for_fusion),
        sensor_risk_counts=dict(sorted(Counter(f"{row.sensor_key}:{row.risk}" for row in rows).items())),
        sensor_axis_counts=dict(sorted(Counter(f"{row.sensor_key}:{row.world_constraint_axis}" for row in rows).items())),
        yaw_axis_counts=dict(sorted(Counter(f"yaw_{row.pose_yaw_deg:.0f}:{row.world_constraint_axis}" for row in rows).items())),
        target_type_counts=dict(sorted(Counter(row.expected_target_type or "no_hit" for row in rows).items())),
        target_counts=dict(sorted(Counter(row.expected_target or "no_hit" for row in rows).items())),
        usable_forest_rows=sum(1 for row in rows if row.usable_for_fusion and "forest" in row.expected_target),
        usable_ramp_rows=sum(1 for row in rows if row.usable_for_fusion and "ramp" in row.expected_target),
        ignored_rows=sum(1 for row in rows if row.risk == "ignored_interference"),
        out_of_range_rows=sum(1 for row in rows if row.risk == "out_of_range"),
        corner_rows=sum(1 for row in rows if row.risk == "corner_ambiguous"),
        grazing_rows=sum(1 for row in rows if row.risk == "grazing_filtered"),
    )


def write_dt35_role_csv(path: str | Path, rows: list[DT35RoleRow]) -> None:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(DT35RoleRow.__dataclass_fields__.keys())
    with out.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row.to_dict())


def write_dt35_role_summary(path: str | Path, summary: DT35RoleSummary) -> None:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(summary.to_dict(), ensure_ascii=False, indent=2), encoding="utf-8")


def write_dt35_role_markdown(path: str | Path, rows: list[DT35RoleRow], summary: DT35RoleSummary) -> None:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(build_dt35_role_markdown(rows, summary), encoding="utf-8")


def build_dt35_role_markdown(rows: list[DT35RoleRow], summary: DT35RoleSummary) -> str:
    lines = [
        "# DT35 Role Matrix",
        "",
        "Purpose: explain what each side-facing DT35 measures at selected lidar/world poses and H30 yaw angles.",
        "",
        "Coordinate assumptions:",
        "- Lidar pose is the world-frame pose.",
        "- H30 yaw rotates each DT35 ray from robot frame into world frame.",
        "- DT35-1 is on robot -X and shoots local -X.",
        "- DT35-2 is on robot +X and shoots local +X.",
        "- Usable wall, forest, and ramp hits may correct translation; ignored-interference hits must not correct pose.",
        "",
        "Summary:",
        f"- poses: {summary.poses}",
        f"- rows: {summary.rows}",
        f"- usable rows: {summary.usable_rows}",
        f"- usable forest rows: {summary.usable_forest_rows}",
        f"- usable ramp rows: {summary.usable_ramp_rows}",
        f"- ignored rows: {summary.ignored_rows}",
        f"- target types: {summary.target_type_counts}",
        f"- sensor axes: {summary.sensor_axis_counts}",
        "",
        "## Pose/Yaw Matrix",
        "",
    ]
    current_key: tuple[str, float, float, float] | None = None
    for row in sorted(rows, key=lambda item: (item.pose_label, item.pose_yaw_deg, item.sensor_key)):
        key = (row.pose_label, row.pose_x_cm, row.pose_y_cm, row.pose_yaw_deg)
        if key != current_key:
            current_key = key
            lines.extend(
                [
                    f"### {row.pose_label}",
                    "",
                    f"- Pose: x={row.pose_x_cm:.1f} cm, y={row.pose_y_cm:.1f} cm, yaw={row.pose_yaw_deg:.1f} deg",
                    "",
                ]
            )
        usable = "yes" if row.usable_for_fusion else "no"
        lines.extend(
            [
                f"- {row.sensor_key} ({row.sensor_name})",
                f"  - mount/local ray: {row.robot_mount}, {row.local_ray_direction}",
                f"  - world ray yaw/axis: {row.world_ray_yaw_deg:.1f} deg, {row.world_constraint_axis}",
                f"  - target: {row.expected_target or 'none'} ({row.expected_target_type or 'no_hit'})",
                f"  - expected distance: {row.expected_distance_cm:.1f} cm",
                f"  - risk/usable: {row.risk}, {usable}",
                f"  - correction direction: {row.pose_correction_direction}",
                f"  - explanation: {row.explanation}",
                "",
            ]
        )
    return "\n".join(lines).rstrip() + "\n"


def print_dt35_role_rows(rows: list[DT35RoleRow]) -> None:
    print("pose,sensor,mount,local_ray,world_axis,target,type,risk,usable,expected_cm,hit_x,hit_y,explanation")
    for row in rows:
        print(
            f"{row.pose_label},{row.sensor_key},{row.robot_mount},{row.local_ray_direction},"
            f"{row.world_constraint_axis},{row.expected_target},{row.expected_target_type},{row.risk},"
            f"{int(row.usable_for_fusion)},{row.expected_distance_cm:.2f},"
            f"{row.expected_hit_x_cm:.2f},{row.expected_hit_y_cm:.2f},{row.explanation}"
        )


def print_dt35_role_summary(summary: DT35RoleSummary) -> None:
    print(
        f"poses={summary.poses} rows={summary.rows} usable={summary.usable_rows} "
        f"forest={summary.usable_forest_rows} ramp={summary.usable_ramp_rows} "
        f"ignored={summary.ignored_rows} out_of_range={summary.out_of_range_rows} "
        f"corner={summary.corner_rows} grazing={summary.grazing_rows}"
    )
    print("sensor_axis_counts=" + json.dumps(summary.sensor_axis_counts, ensure_ascii=False, sort_keys=True))
    print("sensor_risk_counts=" + json.dumps(summary.sensor_risk_counts, ensure_ascii=False, sort_keys=True))
    print("target_type_counts=" + json.dumps(summary.target_type_counts, ensure_ascii=False, sort_keys=True))


def _role_row(row: DT35HitRow) -> DT35RoleRow:
    robot_mount, local_ray = _mount_and_ray(row)
    risk = _risk(row)
    direction = _correction_direction(row)
    explanation = _explanation(row, robot_mount, local_ray, direction, risk)
    return DT35RoleRow(
        pose_label=row.pose_label,
        pose_x_cm=row.pose_x_cm,
        pose_y_cm=row.pose_y_cm,
        pose_yaw_deg=row.pose_yaw_deg,
        sensor_key=row.sensor_key,
        sensor_name=row.sensor_name,
        robot_mount=robot_mount,
        local_ray_direction=local_ray,
        world_ray_yaw_deg=row.ray_yaw_deg,
        world_ray_dx=row.ray_dx,
        world_ray_dy=row.ray_dy,
        world_constraint_axis=row.constraint_axis,
        pose_correction_direction=direction,
        expected_target=row.expected_target,
        expected_target_type=row.expected_target_type,
        expected_distance_cm=row.expected_distance_cm,
        expected_hit_x_cm=row.expected_hit_x_cm,
        expected_hit_y_cm=row.expected_hit_y_cm,
        incidence_deg=row.incidence_deg,
        correction_weight=row.correction_weight,
        risk=risk,
        usable_for_fusion=row.usable_for_correction,
        explanation=explanation,
    )


def _mount_and_ray(row: DT35HitRow) -> tuple[str, str]:
    if row.sensor_key == "sensor_1":
        return "left_side(-X)", "leftward(local -X)"
    if row.sensor_key == "sensor_2":
        return "right_side(+X)", "rightward(local +X)"
    return "unknown", "unknown"


def _risk(row: DT35HitRow) -> str:
    if not row.expected_target:
        return "no_hit"
    if row.expected_target_type == "ignore":
        return "ignored_interference"
    if not row.within_range:
        return "out_of_range"
    if row.corner_ambiguous:
        return "corner_ambiguous"
    if row.expected_target_type in ("usable_wall", "solid_obstacle") and not row.correction_allowed:
        return "grazing_filtered"
    if row.usable_for_correction:
        return "usable"
    return "skipped"


def _correction_direction(row: DT35HitRow) -> str:
    if not row.usable_for_correction:
        return "none"
    dx = row.correction_dx_per_cm
    dy = row.correction_dy_per_cm
    parts: list[str] = []
    if abs(dx) >= 0.35:
        parts.append("+X" if dx > 0 else "-X")
    if abs(dy) >= 0.35:
        parts.append("+Y" if dy > 0 else "-Y")
    return "/".join(parts) if parts else "none"


def _explanation(
    row: DT35HitRow,
    mount: str,
    local_ray: str,
    correction_direction: str,
    risk: str,
) -> str:
    axis = row.constraint_axis
    target = row.expected_target or "no modeled target"
    if risk == "usable":
        return (
            f"{row.sensor_key} on {mount} shoots {local_ray}; at yaw {row.pose_yaw_deg:.1f} deg it constrains "
            f"world {axis} by measuring {target}. Positive residual moves pose toward {correction_direction}."
        )
    if risk == "ignored_interference":
        return f"Ray hits ignored interference target {target}; do not use it for correction."
    if risk == "out_of_range":
        return f"Ray would hit {target}, but the expected distance is outside DT35 max range."
    if risk == "corner_ambiguous":
        return f"Ray is near a modeled corner at {target}; skip correction because the hit surface is ambiguous."
    if risk == "grazing_filtered":
        return f"Ray hits {target} at a grazing angle; skip correction because the geometry is weak."
    return f"Ray has no trusted correction target in the modeled field."
