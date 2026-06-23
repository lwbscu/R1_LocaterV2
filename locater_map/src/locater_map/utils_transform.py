from __future__ import annotations

from dataclasses import replace
from math import acos, cos, degrees, hypot, isfinite, radians, sin
from typing import Any

from .data_model import RobotFrame


def transform_xy_yaw(x: float, y: float, yaw: float, cfg: dict[str, Any]) -> tuple[float, float, float]:
    sx = float(cfg.get("data_x_sign", 1.0))
    sy = float(cfg.get("data_y_sign", 1.0))
    syaw = float(cfg.get("data_yaw_sign", 1.0))
    ox = float(cfg.get("data_x_offset_cm", 0.0))
    oy = float(cfg.get("data_y_offset_cm", 0.0))
    oyaw = float(cfg.get("data_yaw_offset_deg", 0.0))
    return x * sx + ox, y * sy + oy, yaw * syaw + oyaw


def transform_frame(frame: RobotFrame, cfg: dict[str, Any]) -> RobotFrame:
    px, py, pyaw = transform_xy_yaw(frame.pos_x_cm, frame.pos_y_cm, frame.pos_yaw_deg, cfg)
    cx, cy, cyaw = transform_xy_yaw(frame.calib_x_cm, frame.calib_y_cm, frame.calib_yaw_deg, cfg)
    hx, hy, hyaw = transform_xy_yaw(frame.h30_x_cm, frame.h30_y_cm, frame.h30_yaw_deg, cfg)
    lx, ly, lyaw = transform_xy_yaw(frame.lidar_x_cm, frame.lidar_y_cm, frame.lidar_yaw_deg, cfg)
    return replace(
        frame,
        pos_x_cm=px,
        pos_y_cm=py,
        pos_yaw_deg=pyaw,
        calib_x_cm=cx,
        calib_y_cm=cy,
        calib_yaw_deg=cyaw,
        encoder_x_cm=cx,
        encoder_y_cm=cy,
        h30_x_cm=hx,
        h30_y_cm=hy,
        h30_yaw_deg=hyaw,
        lidar_x_cm=lx,
        lidar_y_cm=ly,
        lidar_yaw_deg=lyaw,
    )


def _right_front_vectors(yaw_deg: float) -> tuple[tuple[float, float], tuple[float, float]]:
    """Return robot local +X(right) and +Y(front) unit vectors in world cm axes."""
    yaw = radians(yaw_deg)
    right = (cos(yaw), -sin(yaw))
    front = (sin(yaw), cos(yaw))
    return right, front


def robot_local_to_world(robot_x_cm: float, robot_y_cm: float, robot_yaw_deg: float,
                         local_x_cm: float, local_y_cm: float) -> tuple[float, float]:
    right, front = _right_front_vectors(robot_yaw_deg)
    return (
        robot_x_cm + local_x_cm * right[0] + local_y_cm * front[0],
        robot_y_cm + local_x_cm * right[1] + local_y_cm * front[1],
    )


def heading_vector_from_front_yaw(yaw_deg: float) -> tuple[float, float]:
    """Yaw is measured from world +Y/front, positive clockwise toward +X."""
    yaw = radians(yaw_deg)
    return sin(yaw), cos(yaw)


def dt35_yaw_from_frame(frame: RobotFrame, source: str = "h30") -> float:
    """Return the yaw used to project DT35 rays.

    `h30` is useful for offline sensor consistency analysis. The map display
    should normally use `pos`, so DT35 rays stay visually attached to the
    displayed robot body.
    """
    source = str(source or "h30").lower()
    if source == "pos":
        return frame.pos_yaw_deg
    if source == "lidar":
        return frame.lidar_yaw_deg if (frame.lidar_valid or frame.lidar_online) else frame.pos_yaw_deg
    if source in ("encoder", "calib"):
        return frame.calib_yaw_deg
    return frame.h30_yaw_deg if (frame.h30_valid or frame.h30_has_attitude) else frame.pos_yaw_deg


def _segment_intersection_t(
    ox: float,
    oy: float,
    dx: float,
    dy: float,
    ax: float,
    ay: float,
    bx: float,
    by: float,
) -> float | None:
    def cross(ax: float, ay: float, bx: float, by: float) -> float:
        return ax * by - ay * bx

    sx = bx - ax
    sy = by - ay
    det = cross(dx, dy, sx, sy)
    if abs(det) < 1e-6:
        return None
    qx = ax - ox
    qy = ay - oy
    t = cross(qx, qy, sx, sy) / det
    u = cross(qx, qy, dx, dy) / det
    if t >= 0.0 and 0.0 <= u <= 1.0:
        return t
    return None


FieldSegment = tuple[str, float, float, float, float, str, float, bool]
HitCandidate = tuple[float, str, str, float, float, float, float, float, float, float, bool]
CORRECTION_TARGET_TYPES = {"usable_wall", "solid_obstacle"}
DEFAULT_MISSING_TARGET_SKIPPABLE_TYPES: set[str] = set()
DEFAULT_DISPLAY_CLIP_TARGET_TYPES = {"usable_wall", "solid_obstacle", "blocker"}


def _target_type(item: dict[str, Any], default: str = "usable_wall") -> str:
    return str(item.get("target_type", item.get("type", default)))


def _correction_weight(item: dict[str, Any], target_type: str) -> float:
    if "correction_weight" in item:
        return float(item["correction_weight"])
    if target_type == "usable_wall":
        return 1.0
    if target_type == "solid_obstacle":
        return 0.65
    return 0.0


def _missing_target_skippable(item: dict[str, Any], target_type: str, field_model: dict[str, Any] | None) -> bool:
    if "missing_target_skippable" in item:
        return bool(item["missing_target_skippable"])
    skippable_types = set(
        str(value)
        for value in (field_model or {}).get(
            "missing_target_skippable_types",
            sorted(DEFAULT_MISSING_TARGET_SKIPPABLE_TYPES),
        )
    )
    return target_type in skippable_types


def _field_segments(field_model: dict[str, Any] | None) -> list[FieldSegment]:
    if not field_model or not bool(field_model.get("enabled", False)):
        return []

    segments: list[FieldSegment] = []
    if bool(field_model.get("use_field_boundary", True)):
        width = float(field_model.get("field_width_cm", 1215.0))
        height = float(field_model.get("field_height_cm", 1210.0))
        x0 = -width * 0.5
        x1 = width * 0.5
        y0 = -height * 0.5
        y1 = height * 0.5
        boundary_skippable = bool(field_model.get("field_boundary_missing_target_skippable", True))
        segments.extend([
            ("field_left", x0, y0, x0, y1, "usable_wall", 1.0, boundary_skippable),
            ("field_right", x1, y0, x1, y1, "usable_wall", 1.0, boundary_skippable),
            ("field_bottom", x0, y0, x1, y0, "usable_wall", 1.0, boundary_skippable),
            ("field_top", x0, y1, x1, y1, "usable_wall", 1.0, boundary_skippable),
        ])

    for item in field_model.get("segments", []):
        if not bool(item.get("enabled", True)):
            continue
        target_type = _target_type(item)
        segments.append((
            str(item.get("name", "segment")),
            float(item["x1_cm"]),
            float(item["y1_cm"]),
            float(item["x2_cm"]),
            float(item["y2_cm"]),
            target_type,
            _correction_weight(item, target_type),
            _missing_target_skippable(item, target_type, field_model),
        ))

    for item in field_model.get("rectangles", []):
        if not bool(item.get("enabled", True)):
            continue
        name = str(item.get("name", "rect"))
        target_type = _target_type(item, "blocker")
        weight = _correction_weight(item, target_type)
        cx = float(item.get("center_x_cm", 0.0))
        cy = float(item.get("center_y_cm", 0.0))
        w = float(item.get("width_cm", 0.0))
        h = float(item.get("height_cm", 0.0))
        x0 = cx - w * 0.5
        x1 = cx + w * 0.5
        y0 = cy - h * 0.5
        y1 = cy + h * 0.5
        skippable = _missing_target_skippable(item, target_type, field_model)
        segments.extend([
            (f"{name}_left", x0, y0, x0, y1, target_type, weight, skippable),
            (f"{name}_right", x1, y0, x1, y1, target_type, weight, skippable),
            (f"{name}_bottom", x0, y0, x1, y0, target_type, weight, skippable),
            (f"{name}_top", x0, y1, x1, y1, target_type, weight, skippable),
        ])
    return segments


def expected_dt35_hit(
    sensor_x_cm: float,
    sensor_y_cm: float,
    ray_yaw_deg: float,
    field_model: dict[str, Any] | None,
) -> dict[str, float | str] | None:
    candidates, dx, dy = _dt35_hit_candidates(sensor_x_cm, sensor_y_cm, ray_yaw_deg, field_model)
    if not candidates:
        return None
    return _dt35_hit_from_candidate(candidates[0], candidates, sensor_x_cm, sensor_y_cm, dx, dy, field_model)


def _dt35_hit_candidates(
    sensor_x_cm: float,
    sensor_y_cm: float,
    ray_yaw_deg: float,
    field_model: dict[str, Any] | None,
) -> tuple[list[HitCandidate], float, float]:
    dx, dy = heading_vector_from_front_yaw(ray_yaw_deg)
    incidence_power = max(0.0, float((field_model or {}).get("incidence_weight_power", 1.0)))
    candidates: list[HitCandidate] = []
    for name, ax, ay, bx, by, target_type, weight, skippable in _field_segments(field_model):
        t = _segment_intersection_t(sensor_x_cm, sensor_y_cm, dx, dy, ax, ay, bx, by)
        if t is None:
            continue
        incidence_deg, incidence_scale = _segment_incidence(dx, dy, ax, ay, bx, by, incidence_power)
        candidates.append((t, name, target_type, weight, incidence_deg, incidence_scale, ax, ay, bx, by, skippable))
    candidates.sort(key=lambda item: item[0])
    return candidates, dx, dy


def _dt35_hit_from_candidate(
    best: HitCandidate,
    candidates: list[HitCandidate],
    sensor_x_cm: float,
    sensor_y_cm: float,
    dx: float,
    dy: float,
    field_model: dict[str, Any] | None,
) -> dict[str, float | str]:
    max_incidence_deg = float((field_model or {}).get("max_correction_incidence_deg", 75.0))
    corner_tolerance_cm = max(0.0, float((field_model or {}).get("corner_ambiguity_cm", 3.0)))
    corner_ambiguous = any(
        candidate is not best
        and abs(candidate[0] - best[0]) <= corner_tolerance_cm
        and _segments_are_nonparallel(best, candidate)
        for candidate in candidates
    )
    distance_cm, name, target_type, base_weight, incidence_deg, incidence_scale = best[:6]
    weight = base_weight * incidence_scale
    correction_allowed = (
        target_type in CORRECTION_TARGET_TYPES
        and base_weight > 0.0
        and weight > 0.0
        and incidence_deg <= max_incidence_deg
        and not corner_ambiguous
    )
    return {
        "name": name,
        "target_type": target_type,
        "base_correction_weight": base_weight,
        "corner_ambiguous": "1" if corner_ambiguous else "0",
        "incidence_deg": incidence_deg,
        "incidence_scale": incidence_scale,
        "correction_weight": weight,
        "correction_allowed": "1" if correction_allowed else "0",
        "distance_cm": distance_cm,
        "hit_x_cm": sensor_x_cm + distance_cm * dx,
        "hit_y_cm": sensor_y_cm + distance_cm * dy,
    }


def _expected_dt35_hit_for_measurement(
    sensor_x_cm: float,
    sensor_y_cm: float,
    ray_yaw_deg: float,
    measured_distance_cm: float | None,
    field_model: dict[str, Any] | None,
) -> dict[str, float | str] | None:
    candidates, dx, dy = _dt35_hit_candidates(sensor_x_cm, sensor_y_cm, ray_yaw_deg, field_model)
    if not candidates:
        return None

    nearest = _dt35_hit_from_candidate(candidates[0], candidates, sensor_x_cm, sensor_y_cm, dx, dy, field_model)
    selected_index = 0
    if (
        bool((field_model or {}).get("infer_missing_targets", False))
        and measured_distance_cm is not None
        and isfinite(measured_distance_cm)
        and measured_distance_cm > 0.0
    ):
        gate_cm = max(0.0, float((field_model or {}).get("missing_target_residual_gate_cm", 12.0)))
        max_skip_count = max(0, int((field_model or {}).get("missing_target_max_skip_count", 4)))
        for index, candidate in enumerate(candidates[: max_skip_count + 1]):
            distance_cm, _name, target_type = candidate[:3]
            skipped = candidates[:index]
            if any(not bool(item[10]) for item in skipped):
                break
            if target_type not in CORRECTION_TARGET_TYPES:
                continue
            if abs(measured_distance_cm - distance_cm) <= gate_cm:
                selected_index = index
                break

    selected = candidates[selected_index]
    hit = _dt35_hit_from_candidate(selected, candidates, sensor_x_cm, sensor_y_cm, dx, dy, field_model)
    hit["nearest_hit_x_cm"] = float(nearest["hit_x_cm"])
    hit["nearest_hit_y_cm"] = float(nearest["hit_y_cm"])
    skipped = candidates[:selected_index]
    if skipped:
        hit["inferred_missing_target"] = "1"
        hit["skipped_target_count"] = float(len(skipped))
        hit["skipped_targets"] = ",".join(item[1] for item in skipped)
        hit["nearest_target"] = candidates[0][1]
        hit["nearest_target_type"] = candidates[0][2]
        hit["nearest_distance_cm"] = candidates[0][0]
    else:
        hit["inferred_missing_target"] = "0"
        hit["skipped_target_count"] = 0.0
        hit["skipped_targets"] = ""
        hit["nearest_target"] = candidates[0][1]
        hit["nearest_target_type"] = candidates[0][2]
        hit["nearest_distance_cm"] = candidates[0][0]
    return hit


def _display_clip_target_types(field_model: dict[str, Any] | None) -> set[str]:
    return set(
        str(item)
        for item in (field_model or {}).get(
            "display_clip_target_types",
            sorted(DEFAULT_DISPLAY_CLIP_TARGET_TYPES),
        )
    )


def _segments_are_nonparallel(
    first: HitCandidate,
    second: HitCandidate,
) -> bool:
    _, _, _, _, _, _, ax1, ay1, bx1, by1, _ = first
    _, _, _, _, _, _, ax2, ay2, bx2, by2, _ = second
    vx1 = bx1 - ax1
    vy1 = by1 - ay1
    vx2 = bx2 - ax2
    vy2 = by2 - ay2
    len1 = hypot(vx1, vy1)
    len2 = hypot(vx2, vy2)
    if len1 <= 1.0e-9 or len2 <= 1.0e-9:
        return False
    cross = abs(vx1 * vy2 - vy1 * vx2) / (len1 * len2)
    return cross > 0.1736481777


def _segment_incidence(
    dx: float,
    dy: float,
    ax: float,
    ay: float,
    bx: float,
    by: float,
    power: float,
) -> tuple[float, float]:
    sx = bx - ax
    sy = by - ay
    length = hypot(sx, sy)
    if length <= 1.0e-9:
        return 90.0, 0.0
    nx = -sy / length
    ny = sx / length
    normal_dot = max(0.0, min(1.0, abs(dx * nx + dy * ny)))
    incidence_deg = degrees(acos(normal_dot))
    return incidence_deg, normal_dot ** power


def dt35_ray(
    robot_x_cm: float,
    robot_y_cm: float,
    robot_yaw_deg: float,
    sensor_cfg: dict[str, Any],
    distance_mm: float,
    field_model: dict[str, Any] | None = None,
) -> dict[str, float | bool | str]:
    ox = float(sensor_cfg.get("offset_x_cm", 0.0))
    oy = float(sensor_cfg.get("offset_y_cm", 0.0))
    yaw_offset = float(sensor_cfg.get("yaw_offset_deg", 0.0))
    distance_bias_mm = float(sensor_cfg.get("distance_bias_mm", 0.0))
    max_range_cm = float(sensor_cfg.get("max_range_cm", 999999.0))
    enabled = bool(sensor_cfg.get("enabled", True))
    sensor_x, sensor_y = robot_local_to_world(robot_x_cm, robot_y_cm, robot_yaw_deg, ox, oy)
    d_cm = max(0.0, (float(distance_mm) + distance_bias_mm) / 10.0)
    valid = enabled and 0.0 < d_cm <= max_range_cm
    draw_d = d_cm if valid else max_range_cm
    ray_yaw_deg = robot_yaw_deg + yaw_offset
    dx, dy = heading_vector_from_front_yaw(ray_yaw_deg)
    hit_x = sensor_x + draw_d * dx
    hit_y = sensor_y + draw_d * dy
    expected = _expected_dt35_hit_for_measurement(sensor_x, sensor_y, ray_yaw_deg, d_cm if valid else None, field_model)
    expected_distance = float(expected["distance_cm"]) if expected is not None else float("nan")
    nearest_distance = float(expected["nearest_distance_cm"]) if expected is not None else float("nan")
    nearest_hit_x = float(expected["nearest_hit_x_cm"]) if expected is not None else float("nan")
    nearest_hit_y = float(expected["nearest_hit_y_cm"]) if expected is not None else float("nan")
    nearest_target = str(expected["nearest_target"]) if expected is not None else ""
    nearest_target_type = str(expected["nearest_target_type"]) if expected is not None else ""
    residual = d_cm - expected_distance if valid and isfinite(expected_distance) else float("nan")
    floor_gate_cm = max(0.0, float((field_model or {}).get("floor_hit_negative_residual_gate_cm", 12.0)))
    floor_hit_suspect = valid and isfinite(residual) and residual < -floor_gate_cm
    display_hit_x = hit_x
    display_hit_y = hit_y
    display_distance_cm = draw_d
    display_clipped = False
    clip_types = _display_clip_target_types(field_model)
    if (
        expected is not None
        and isfinite(nearest_distance)
        and isfinite(nearest_hit_x)
        and isfinite(nearest_hit_y)
        and nearest_target_type in clip_types
        and draw_d > nearest_distance
    ):
        display_hit_x = nearest_hit_x
        display_hit_y = nearest_hit_y
        display_distance_cm = nearest_distance
        display_clipped = True
    return {
        "name": str(sensor_cfg.get("name", "DT35")),
        "enabled": enabled,
        "valid": valid,
        "sensor_x_cm": sensor_x,
        "sensor_y_cm": sensor_y,
        "hit_x_cm": hit_x,
        "hit_y_cm": hit_y,
        "measured_hit_x_cm": hit_x,
        "measured_hit_y_cm": hit_y,
        "display_hit_x_cm": display_hit_x,
        "display_hit_y_cm": display_hit_y,
        "display_distance_cm": display_distance_cm,
        "display_clipped_by_model": display_clipped,
        "display_clip_target": nearest_target if display_clipped else "",
        "display_clip_target_type": nearest_target_type if display_clipped else "",
        "distance_cm": d_cm,
        "distance_bias_mm": distance_bias_mm,
        "max_range_cm": max_range_cm,
        "ray_yaw_deg": ray_yaw_deg,
        "expected_hit_x_cm": float(expected["hit_x_cm"]) if expected is not None else float("nan"),
        "expected_hit_y_cm": float(expected["hit_y_cm"]) if expected is not None else float("nan"),
        "expected_distance_cm": expected_distance,
        "residual_cm": residual,
        "floor_hit_suspect": floor_hit_suspect,
        "floor_hit_negative_residual_gate_cm": floor_gate_cm,
        "expected_target": str(expected["name"]) if expected is not None else "",
        "expected_target_type": str(expected["target_type"]) if expected is not None else "",
        "base_correction_weight": float(expected["base_correction_weight"]) if expected is not None else 0.0,
        "corner_ambiguous": bool(int(str(expected["corner_ambiguous"]))) if expected is not None else False,
        "incidence_deg": float(expected["incidence_deg"]) if expected is not None else float("nan"),
        "incidence_scale": float(expected["incidence_scale"]) if expected is not None else 0.0,
        "correction_weight": float(expected["correction_weight"]) if expected is not None else 0.0,
        "correction_allowed": bool(int(str(expected["correction_allowed"]))) if expected is not None else False,
        "inferred_missing_target": bool(int(str(expected["inferred_missing_target"]))) if expected is not None else False,
        "skipped_target_count": int(float(expected["skipped_target_count"])) if expected is not None else 0,
        "skipped_targets": str(expected["skipped_targets"]) if expected is not None else "",
        "nearest_target": nearest_target,
        "nearest_target_type": nearest_target_type,
        "nearest_distance_cm": nearest_distance,
        "nearest_hit_x_cm": nearest_hit_x,
        "nearest_hit_y_cm": nearest_hit_y,
    }
