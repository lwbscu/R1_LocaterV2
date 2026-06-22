from __future__ import annotations

import json
from html import escape
from pathlib import Path
from typing import Any, Callable

from .config_loader import resolve_resource
from .dt35_analysis import DEFAULT_POSES, PoseSpec
from .utils_transform import expected_dt35_hit, heading_vector_from_front_yaw, robot_local_to_world


TARGET_STYLES = {
    "usable_wall": ("#ff3344", "#ff3344", 0.10),
    "ignore": ("#2d82ff", "#2d82ff", 0.18),
    "solid_obstacle": ("#52e35f", "#52e35f", 0.18),
    "blocker": ("#ff9f43", "#ff9f43", 0.16),
}


def write_field_model_svg(path: str | Path, config: dict[str, Any], poses: list[PoseSpec] | tuple[PoseSpec, ...] | None = None) -> None:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(build_field_model_svg(config, poses), encoding="utf-8")


def build_field_model_svg(config: dict[str, Any], poses: list[PoseSpec] | tuple[PoseSpec, ...] | None = None) -> str:
    map_cfg = config.get("map", {})
    field_w_cm = float(map_cfg.get("field_width_cm", 1215.0))
    field_h_cm = float(map_cfg.get("field_height_cm", 1210.0))
    prior = _prior_map_config(config)
    img_w = int(prior.get("image_width_px", round(field_w_cm * 2.0)))
    img_h = int(prior.get("image_height_px", round(field_h_cm * 2.0)))
    px_per_cm_x = img_w / field_w_cm
    px_per_cm_y = img_h / field_h_cm

    def world_to_pixel(x_cm: float, y_cm: float) -> tuple[float, float]:
        return (x_cm + field_w_cm * 0.5) * px_per_cm_x, (field_h_cm * 0.5 - y_cm) * px_per_cm_y

    background = resolve_resource(config, map_cfg.get("labeled_background_image") or map_cfg.get("background_image"))
    elements: list[str] = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{img_w}" height="{img_h}" viewBox="0 0 {img_w} {img_h}">',
        "<title>R1 Locater DT35 field model overlay</title>",
        '<rect x="0" y="0" width="100%" height="100%" fill="#0b1118"/>',
    ]
    background_href = _relative_href(background, config)
    if background_href:
        elements.append(f'<image href="{escape(background_href)}" x="0" y="0" width="{img_w}" height="{img_h}" opacity="0.82"/>')
    elements.append(_legend())
    elements.append(f'<rect x="0" y="0" width="{img_w}" height="{img_h}" fill="none" stroke="#ffffff" stroke-width="4" opacity="0.85"/>')
    elements.extend(_field_segments_svg(config, world_to_pixel, px_per_cm_x, px_per_cm_y))
    for pose in (poses if poses is not None else DEFAULT_POSES):
        elements.extend(_pose_rays(pose, config, world_to_pixel))
    elements.append("</svg>")
    return "\n".join(elements) + "\n"


def _field_segments_svg(
    config: dict[str, Any],
    world_to_pixel: Callable[[float, float], tuple[float, float]],
    px_per_cm_x: float,
    px_per_cm_y: float,
) -> list[str]:
    elements: list[str] = []
    model = config.get("field_model", {})
    for item in model.get("segments", []):
        if not bool(item.get("enabled", True)):
            continue
        target_type = str(item.get("target_type", "usable_wall"))
        stroke, _fill, _opacity = _style(target_type)
        x1, y1 = world_to_pixel(float(item["x1_cm"]), float(item["y1_cm"]))
        x2, y2 = world_to_pixel(float(item["x2_cm"]), float(item["y2_cm"]))
        dash = ' stroke-dasharray="16 10"' if target_type == "ignore" else ""
        title = escape(f"{item.get('name', 'segment')} [{target_type}]")
        elements.append(
            f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" '
            f'stroke="{stroke}" stroke-width="8" stroke-linecap="round"{dash}><title>{title}</title></line>'
        )

    for item in model.get("rectangles", []):
        if not bool(item.get("enabled", True)):
            continue
        target_type = str(item.get("target_type", "blocker"))
        stroke, fill, opacity = _style(target_type)
        cx = float(item.get("center_x_cm", 0.0))
        cy = float(item.get("center_y_cm", 0.0))
        width = float(item.get("width_cm", 0.0))
        height = float(item.get("height_cm", 0.0))
        x, y = world_to_pixel(cx - width * 0.5, cy + height * 0.5)
        dash = ' stroke-dasharray="16 10"' if target_type == "ignore" else ""
        title = escape(f"{item.get('name', 'rectangle')} [{target_type}]")
        elements.append(
            f'<rect x="{x:.2f}" y="{y:.2f}" width="{width * px_per_cm_x:.2f}" height="{height * px_per_cm_y:.2f}" '
            f'fill="{fill}" fill-opacity="{opacity}" stroke="{stroke}" stroke-width="8"{dash}><title>{title}</title></rect>'
        )
    return elements


def _pose_rays(pose: PoseSpec, config: dict[str, Any], world_to_pixel: Callable[[float, float], tuple[float, float]]) -> list[str]:
    items: list[str] = []
    cx, cy = world_to_pixel(pose.x_cm, pose.y_cm)
    label = escape(pose.label or f"{pose.x_cm:g},{pose.y_cm:g},{pose.yaw_deg:g}")
    items.append(f'<circle cx="{cx:.2f}" cy="{cy:.2f}" r="8" fill="#ffffff" stroke="#111827" stroke-width="3"><title>{label}</title></circle>')
    dx, dy = heading_vector_from_front_yaw(pose.yaw_deg)
    hx, hy = world_to_pixel(pose.x_cm + dx * 50.0, pose.y_cm + dy * 50.0)
    items.append(f'<line x1="{cx:.2f}" y1="{cy:.2f}" x2="{hx:.2f}" y2="{hy:.2f}" stroke="#ffffff" stroke-width="4" marker-end="url(#arrow)"/>')

    field_model = dict(config.get("field_model", {}))
    field_model.setdefault("field_width_cm", config.get("map", {}).get("field_width_cm", 1215.0))
    field_model.setdefault("field_height_cm", config.get("map", {}).get("field_height_cm", 1210.0))
    for key in ("sensor_1", "sensor_2"):
        sensor_cfg = config.get("dt35", {}).get(key, {})
        if not bool(sensor_cfg.get("enabled", True)):
            continue
        sx, sy = robot_local_to_world(
            pose.x_cm,
            pose.y_cm,
            pose.yaw_deg,
            float(sensor_cfg.get("offset_x_cm", 0.0)),
            float(sensor_cfg.get("offset_y_cm", 0.0)),
        )
        ray_yaw = pose.yaw_deg + float(sensor_cfg.get("yaw_offset_deg", 0.0))
        hit = expected_dt35_hit(sx, sy, ray_yaw, field_model)
        if hit is None:
            rdx, rdy = heading_vector_from_front_yaw(ray_yaw)
            hit_x = sx + rdx * float(sensor_cfg.get("max_range_cm", 250.0))
            hit_y = sy + rdy * float(sensor_cfg.get("max_range_cm", 250.0))
            target_type = "none"
            target_name = "no_hit"
            correction_allowed = False
        else:
            hit_x = float(hit["hit_x_cm"])
            hit_y = float(hit["hit_y_cm"])
            target_type = str(hit["target_type"])
            target_name = str(hit["name"])
            correction_allowed = bool(int(str(hit["correction_allowed"])))
        stroke, _fill, _opacity = _style(target_type)
        dash = "" if correction_allowed else ' stroke-dasharray="10 8"'
        x1, y1 = world_to_pixel(sx, sy)
        x2, y2 = world_to_pixel(hit_x, hit_y)
        distance = ((hit_x - sx) ** 2 + (hit_y - sy) ** 2) ** 0.5
        title = escape(f"{label} {key} -> {target_name} [{target_type}], {distance:.1f}cm")
        items.append(
            f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" '
            f'stroke="{stroke}" stroke-width="3" opacity="0.85"{dash}><title>{title}</title></line>'
        )
        items.append(f'<circle cx="{x1:.2f}" cy="{y1:.2f}" r="5" fill="#ffffff" stroke="{stroke}" stroke-width="2"><title>{title}</title></circle>')
        items.append(f'<circle cx="{x2:.2f}" cy="{y2:.2f}" r="5" fill="{stroke}" opacity="0.9"><title>{title}</title></circle>')
    return items


def _prior_map_config(config: dict[str, Any]) -> dict[str, Any]:
    path = resolve_resource(config, config.get("map", {}).get("prior_map_config"))
    if not path or not path.exists():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def _relative_href(path: Path | None, config: dict[str, Any]) -> str:
    if not path:
        return ""
    root = Path(config.get("_project_root", path.parent))
    try:
        return Path(path).resolve().relative_to(root.resolve()).as_posix()
    except ValueError:
        return Path(path).resolve().as_posix()


def _style(target_type: str) -> tuple[str, str, float]:
    return TARGET_STYLES.get(target_type, ("#9ca3af", "#9ca3af", 0.12))


def _legend() -> str:
    return """
<defs>
  <marker id="arrow" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto" markerUnits="strokeWidth">
    <path d="M0,0 L8,4 L0,8 z" fill="#ffffff"/>
  </marker>
</defs>
<g transform="translate(24,24)" font-family="Segoe UI, Arial, sans-serif" font-size="26">
  <rect x="0" y="0" width="600" height="178" rx="14" fill="#111827" opacity="0.78"/>
  <text x="22" y="40" fill="#ffffff">DT35 field model overlay</text>
  <line x1="26" y1="72" x2="112" y2="72" stroke="#ff3344" stroke-width="8"/><text x="130" y="82" fill="#ffffff">usable wall</text>
  <line x1="26" y1="110" x2="112" y2="110" stroke="#2d82ff" stroke-width="8" stroke-dasharray="16 10"/><text x="130" y="120" fill="#ffffff">ignored interference</text>
  <rect x="26" y="138" width="86" height="24" fill="#52e35f" fill-opacity="0.24" stroke="#52e35f" stroke-width="6"/><text x="130" y="162" fill="#ffffff">solid obstacle blocks laser</text>
</g>
""".strip()
