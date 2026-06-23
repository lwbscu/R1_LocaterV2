from __future__ import annotations

import json
from collections import Counter
from dataclasses import asdict
from pathlib import Path
from typing import Any

from .config_loader import resolve_resource
from .dt35_analysis import (
    DEFAULT_POSES,
    PoseSpec,
    analyze_dt35_hits,
    analyze_observability,
    summarize_coverage,
    summarize_observability,
)
from .dt35_field_sweep import run_dt35_field_sweep


def write_field_model_audit(
    path: str | Path,
    config: dict[str, Any],
    poses: list[PoseSpec] | tuple[PoseSpec, ...] | None = None,
) -> None:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(build_field_model_audit(config, poses), ensure_ascii=False, indent=2), encoding="utf-8")


def build_field_model_audit(
    config: dict[str, Any],
    poses: list[PoseSpec] | tuple[PoseSpec, ...] | None = None,
) -> dict[str, Any]:
    selected_poses = list(poses if poses is not None else DEFAULT_POSES)
    hit_rows = analyze_dt35_hits(config, selected_poses)
    observability_rows = analyze_observability(hit_rows)
    map_section = _map_section(config)
    dt35_section = _dt35_section(config)
    field_model_section = _field_model_section(config)
    manual_dimension_checks = _manual_dimension_checks(map_section, dt35_section, field_model_section)
    coverage = asdict(summarize_coverage(hit_rows))
    observability = asdict(summarize_observability(observability_rows, hit_rows))
    behavior = _behavior_section(hit_rows)
    sweep_rows, sweep_summary = run_dt35_field_sweep(config, step_cm=100.0)
    missing_evidence = [
        "real parsed_frames.csv with valid dt35_1_mm and dt35_2_mm",
        "real DT35 residual report showing usable_wall/solid_obstacle residuals within gate",
        "real path report proving fused pose improves or matches lidar reference over actual robot motion",
    ]
    return {
        "map": map_section,
        "dt35": dt35_section,
        "field_model": field_model_section,
        "manual_dimension_interpretation": {
            "field_outer_size_cm": [1215.0, 1210.0],
            "background_scale": "2 px/cm",
            "coordinate_frame": "origin at field outer center, +X right, +Y up",
            "forest_top_view_model": "each forest is 3 x 4 cells, 120 cm per cell, total 360 cm x 480 cm",
            "ramp_top_view_model": "150 cm x 150 cm solid obstacle footprints",
            "ramp_side_view_note": "the 270 cm side-view ramp/platform dimension is not used as a larger top-view DT35 blocker",
            "forest_note": "forest grid edges are modeled as red usable walls; DT35 cannot pass through them",
            "ignored_area_note": "top long-pole racks are modeled as ignored interference, not trusted correction surfaces",
        },
        "manual_dimension_checks": manual_dimension_checks,
        "model_self_check": _model_self_check(
            map_section,
            dt35_section,
            field_model_section,
            coverage,
            observability,
            behavior,
            manual_dimension_checks,
            sweep_summary.to_dict(),
        ),
        "default_pose_behavior": behavior,
        "default_pose_coverage": coverage,
        "default_pose_observability": observability,
        "default_pose_hits": [asdict(row) for row in hit_rows],
        "default_pose_observability_rows": [asdict(row) for row in observability_rows],
        "field_sweep": {
            "grid_step_cm": 100.0,
            "summary": sweep_summary.to_dict(),
            "weak_pose_examples": sweep_summary.weak_pose_examples,
            "sample_rows": [row.to_dict() for row in sweep_rows[:32]],
        },
        "fusion_assumptions": {
            "lidar": "absolute world-frame pose anchor",
            "h30_yaw": "trusted yaw source for DT35 ray projection and fused yaw",
            "encoder": "high-rate interpolation between lidar anchors, with display-side scale learning",
            "dt35": "trusted distance measurement, gated by field geometry, incidence, range, corner ambiguity, and ignored zones",
        },
        "completion_evidence_missing": missing_evidence,
        "completion_verified": False,
    }


def _map_section(config: dict[str, Any]) -> dict[str, Any]:
    map_cfg = config.get("map", {})
    field_w = float(map_cfg.get("field_width_cm", 1215.0))
    field_h = float(map_cfg.get("field_height_cm", 1210.0))
    prior = _read_json_resource(config, map_cfg.get("prior_map_config"))
    image_w = float(prior.get("image_width_px", field_w * 2.0))
    image_h = float(prior.get("image_height_px", field_h * 2.0))
    return {
        "field_width_cm": field_w,
        "field_height_cm": field_h,
        "world_x_range_cm": [-field_w * 0.5, field_w * 0.5],
        "world_y_range_cm": [-field_h * 0.5, field_h * 0.5],
        "background_image": map_cfg.get("background_image", ""),
        "labeled_background_image": map_cfg.get("labeled_background_image", ""),
        "prior_map_config": map_cfg.get("prior_map_config", ""),
        "prior_asset": prior.get("asset", ""),
        "image_width_px": image_w,
        "image_height_px": image_h,
        "pixels_per_cm_x": image_w / field_w if field_w else None,
        "pixels_per_cm_y": image_h / field_h if field_h else None,
        "pixel_world_transform": prior.get("image_pixel_to_world_cm", {}),
    }


def _dt35_section(config: dict[str, Any]) -> dict[str, Any]:
    dt35_cfg = config.get("dt35", {})
    sensors: dict[str, Any] = {}
    for key in ("sensor_1", "sensor_2"):
        item = dict(dt35_cfg.get(key, {}))
        sensors[key] = {
            "name": item.get("name", key),
            "enabled": bool(item.get("enabled", True)),
            "offset_x_cm": float(item.get("offset_x_cm", 0.0)),
            "offset_y_cm": float(item.get("offset_y_cm", 0.0)),
            "yaw_offset_deg": float(item.get("yaw_offset_deg", 0.0)),
            "max_range_cm": float(item.get("max_range_cm", 0.0)),
            "mounting_meaning": _sensor_meaning(key, item),
        }
    return sensors


def _field_model_section(config: dict[str, Any]) -> dict[str, Any]:
    model = config.get("field_model", {})
    segments = [dict(item) for item in model.get("segments", []) if bool(item.get("enabled", True))]
    rectangles = [dict(item) for item in model.get("rectangles", []) if bool(item.get("enabled", True))]
    target_counts = Counter(str(item.get("target_type", "usable_wall")) for item in segments)
    target_counts.update(str(item.get("target_type", "blocker")) for item in rectangles)
    return {
        "enabled": bool(model.get("enabled", True)),
        "use_field_boundary": bool(model.get("use_field_boundary", True)),
        "residual_warn_cm": float(model.get("residual_warn_cm", 8.0)),
        "max_correction_incidence_deg": float(model.get("max_correction_incidence_deg", 75.0)),
        "corner_ambiguity_cm": float(model.get("corner_ambiguity_cm", 3.0)),
        "target_type_counts": dict(sorted(target_counts.items())),
        "segments": segments,
        "rectangles": rectangles,
        "raycast_note": "rectangles are expanded to four blocking/correction edges during raycasting",
    }


def _behavior_section(hit_rows) -> dict[str, Any]:
    usable = [row for row in hit_rows if row.usable_for_correction]
    usable_targets = Counter(row.expected_target for row in usable if row.expected_target)
    usable_types = Counter(row.expected_target_type for row in usable if row.expected_target_type)
    ignored_targets = Counter(row.expected_target for row in hit_rows if row.expected_target_type == "ignore")
    forest_targets = [row.expected_target for row in usable if "forest" in row.expected_target]
    solid_targets = [row.expected_target for row in usable if row.expected_target_type == "solid_obstacle"]
    ramp_targets = [name for name in solid_targets if "ramp" in name]
    return {
        "usable_target_type_counts": dict(sorted(usable_types.items())),
        "usable_target_counts": dict(sorted(usable_targets.items())),
        "ignored_target_counts": dict(sorted(ignored_targets.items())),
        "usable_solid_obstacle_targets": sorted(set(solid_targets)),
        "usable_forest_targets": sorted(set(forest_targets)),
        "usable_ramp_targets": sorted(set(ramp_targets)),
        "usable_solid_obstacle_ray_count": len(solid_targets),
        "usable_forest_ray_count": len(forest_targets),
        "usable_ramp_ray_count": len(ramp_targets),
    }


def _model_self_check(
    map_section: dict[str, Any],
    dt35_section: dict[str, Any],
    field_model_section: dict[str, Any],
    coverage: dict[str, Any],
    observability: dict[str, Any],
    behavior: dict[str, Any],
    manual_dimension_checks: list[dict[str, Any]],
    field_sweep_summary: dict[str, Any],
) -> dict[str, Any]:
    segments = field_model_section.get("segments", [])
    rectangles = field_model_section.get("rectangles", [])
    names = [str(item.get("name", "")) for item in [*segments, *rectangles]]
    target_counts = field_model_section.get("target_type_counts", {})
    checks = [
        _check("field_width_1215cm", abs(float(map_section.get("field_width_cm", 0.0)) - 1215.0) < 1.0e-6),
        _check("field_height_1210cm", abs(float(map_section.get("field_height_cm", 0.0)) - 1210.0) < 1.0e-6),
        _check("background_scale_2px_per_cm_x", abs(float(map_section.get("pixels_per_cm_x", 0.0)) - 2.0) < 1.0e-6),
        _check("background_scale_2px_per_cm_y", abs(float(map_section.get("pixels_per_cm_y", 0.0)) - 2.0) < 1.0e-6),
        _check("field_model_enabled", bool(field_model_section.get("enabled", False))),
        _check("has_usable_wall_targets", int(target_counts.get("usable_wall", 0)) > 0),
        _check("has_ignore_targets", int(target_counts.get("ignore", 0)) > 0),
        _check("has_solid_obstacle_targets", int(target_counts.get("solid_obstacle", 0)) > 0),
        _check("has_forest_blockers", any("forest" in name for name in names)),
        _check("has_ramp_blockers", any("ramp" in name for name in names)),
        _check(
            "dt35_sensor_1_left_side_left_ray",
            float(dt35_section.get("sensor_1", {}).get("offset_x_cm", 0.0)) < 0.0
            and float(dt35_section.get("sensor_1", {}).get("yaw_offset_deg", 0.0)) < 0.0,
        ),
        _check(
            "dt35_sensor_2_right_side_right_ray",
            float(dt35_section.get("sensor_2", {}).get("offset_x_cm", 0.0)) > 0.0
            and float(dt35_section.get("sensor_2", {}).get("yaw_offset_deg", 0.0)) > 0.0,
        ),
        _check("default_poses_have_usable_dt35_rays", int(coverage.get("usable_rays", 0)) > 0),
        _check("default_poses_include_observable_constraints", int(observability.get("one_dim_poses", 0)) + int(observability.get("two_dim_poses", 0)) > 0),
        _check("default_poses_have_usable_solid_obstacle_rays", int(behavior.get("usable_solid_obstacle_ray_count", 0)) > 0),
        _check("default_poses_have_usable_forest_rays", int(behavior.get("usable_forest_ray_count", 0)) > 0),
        _check("default_poses_have_usable_ramp_rays", int(behavior.get("usable_ramp_ray_count", 0)) > 0),
        _check("manual_dimension_checks_passed", all(bool(item.get("passed", False)) for item in manual_dimension_checks)),
        _check("field_sweep_passed", bool(field_sweep_summary.get("model_passed", False))),
        _check("field_sweep_has_forest_constraints", int(field_sweep_summary.get("forest_constraint_poses", 0)) > 0),
        _check("field_sweep_has_ramp_constraints", int(field_sweep_summary.get("ramp_constraint_poses", 0)) > 0),
        _check("field_sweep_has_ignored_interference", int(field_sweep_summary.get("ignored_interference_poses", 0)) > 0),
        _check("field_sweep_ignored_targets_not_corrected", bool(field_sweep_summary.get("ignored_targets_never_corrected", False))),
    ]
    return {
        "passed": all(item["passed"] for item in checks),
        "checks": checks,
        "failed_checks": [item["name"] for item in checks if not item["passed"]],
        "scope": "configuration and synthetic geometry only; real sensor residual validation is tracked separately",
    }


def _check(name: str, passed: bool) -> dict[str, Any]:
    return {"name": name, "passed": bool(passed)}


def _manual_dimension_checks(
    map_section: dict[str, Any],
    dt35_section: dict[str, Any],
    field_model_section: dict[str, Any],
) -> list[dict[str, Any]]:
    rectangles = {str(item.get("name", "")): item for item in field_model_section.get("rectangles", [])}
    segments = {str(item.get("name", "")): item for item in field_model_section.get("segments", [])}
    checks: list[dict[str, Any]] = [
        _dimension_check("field_width_cm", float(map_section.get("field_width_cm", 0.0)), 1215.0, 0.01, "manual outer field width"),
        _dimension_check("field_height_cm", float(map_section.get("field_height_cm", 0.0)), 1210.0, 0.01, "manual outer field height"),
        _dimension_check("map_pixels_per_cm_x", float(map_section.get("pixels_per_cm_x", 0.0)), 2.0, 0.001, "prior-map horizontal scale"),
        _dimension_check("map_pixels_per_cm_y", float(map_section.get("pixels_per_cm_y", 0.0)), 2.0, 0.001, "prior-map vertical scale"),
        _dimension_check("dt35_sensor_1_offset_x_cm", float(dt35_section.get("sensor_1", {}).get("offset_x_cm", 0.0)), -40.4, 0.05, "DT35-1 left mount offset"),
        _dimension_check("dt35_sensor_1_offset_y_cm", float(dt35_section.get("sensor_1", {}).get("offset_y_cm", 0.0)), -3.3, 0.05, "DT35-1 rear offset"),
        _dimension_check("dt35_sensor_1_yaw_offset_deg", float(dt35_section.get("sensor_1", {}).get("yaw_offset_deg", 0.0)), -90.0, 0.01, "DT35-1 left-facing ray"),
        _dimension_check("dt35_sensor_2_offset_x_cm", float(dt35_section.get("sensor_2", {}).get("offset_x_cm", 0.0)), 40.4, 0.05, "DT35-2 right mount offset"),
        _dimension_check("dt35_sensor_2_offset_y_cm", float(dt35_section.get("sensor_2", {}).get("offset_y_cm", 0.0)), -3.3, 0.05, "DT35-2 rear offset"),
        _dimension_check("dt35_sensor_2_yaw_offset_deg", float(dt35_section.get("sensor_2", {}).get("yaw_offset_deg", 0.0)), 90.0, 0.01, "DT35-2 right-facing ray"),
    ]
    for name in ("red_forest_obstacle", "blue_forest_obstacle"):
        item = rectangles.get(name, {})
        checks.append(_dimension_check(f"{name}_width_cm", float(item.get("width_cm", 0.0)), 360.0, 0.5, "forest 3 cells x 120 cm"))
        checks.append(_dimension_check(f"{name}_height_cm", float(item.get("height_cm", 0.0)), 480.0, 0.5, "forest 4 cells x 120 cm"))
        checks.append(_target_type_check(f"{name}_usable_wall", item, "usable_wall"))
        checks.append(_bool_check(f"{name}_not_missing_target_skippable", item.get("missing_target_skippable") is False))
    ramp_expected = {
        "red_left_ramp_zone_450h": (155.0, 148.0, "asset-aligned left ramp visible footprint"),
        "blue_right_ramp_zone_450h": (148.5, 148.5, "asset-aligned right ramp visible footprint"),
    }
    for name, (width_cm, height_cm, note) in ramp_expected.items():
        item = rectangles.get(name, {})
        checks.append(_dimension_check(f"{name}_width_cm", float(item.get("width_cm", 0.0)), width_cm, 0.5, note))
        checks.append(_dimension_check(f"{name}_height_cm", float(item.get("height_cm", 0.0)), height_cm, 0.5, note))
        checks.append(_target_type_check(f"{name}_solid_obstacle", item, "solid_obstacle"))
    bottom_barrier = rectangles.get("bottom_center_barrier_wall", {})
    checks.append(_dimension_check("bottom_center_barrier_wall_width_cm", float(bottom_barrier.get("width_cm", 0.0)), 28.0, 0.5, "asset-aligned bottom center barrier width"))
    checks.append(_dimension_check("bottom_center_barrier_wall_height_cm", float(bottom_barrier.get("height_cm", 0.0)), 161.0, 0.5, "asset-aligned bottom center barrier height"))
    checks.append(_dimension_check("bottom_center_barrier_wall_center_y_cm", float(bottom_barrier.get("center_y_cm", 0.0)), -473.75, 0.5, "asset-aligned bottom center barrier vertical center"))
    divider = segments.get("center_divider_wall", {})
    checks.append(_dimension_check("center_divider_x1_cm", float(divider.get("x1_cm", 999.0)), 0.0, 0.01, "center divider on field centerline"))
    checks.append(_dimension_check("center_divider_x2_cm", float(divider.get("x2_cm", 999.0)), 0.0, 0.01, "center divider on field centerline"))
    for name in ("top_red_long_pole_rack_ignore", "top_blue_long_pole_rack_ignore"):
        item = rectangles.get(name, {})
        checks.append(_target_type_check(f"{name}_ignore", item, "ignore"))
    return checks


def _dimension_check(name: str, actual: float, expected: float, tolerance: float, note: str) -> dict[str, Any]:
    return {
        "name": name,
        "passed": abs(actual - expected) <= tolerance,
        "actual": actual,
        "expected": expected,
        "tolerance": tolerance,
        "note": note,
    }


def _target_type_check(name: str, item: dict[str, Any], expected: str) -> dict[str, Any]:
    actual = str(item.get("target_type", ""))
    return {
        "name": name,
        "passed": actual == expected,
        "actual": actual,
        "expected": expected,
        "note": "field-model semantic class used by DT35 raycasting",
    }


def _bool_check(name: str, passed: bool) -> dict[str, Any]:
    return {
        "name": name,
        "passed": bool(passed),
        "actual": bool(passed),
        "expected": True,
        "note": "boolean field-model invariant",
    }


def _sensor_meaning(key: str, item: dict[str, Any]) -> str:
    x = float(item.get("offset_x_cm", 0.0))
    yaw = float(item.get("yaw_offset_deg", 0.0))
    side = "right" if x > 0 else "left" if x < 0 else "center"
    direction = "local -X" if yaw < 0 else "local +X" if yaw > 0 else "local +Y"
    return f"{key} is mounted on robot {side} side and rays toward {direction}"


def _read_json_resource(config: dict[str, Any], path_text: str | None) -> dict[str, Any]:
    path = resolve_resource(config, path_text)
    if not path or not path.exists():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))
