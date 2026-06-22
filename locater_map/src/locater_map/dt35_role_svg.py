from __future__ import annotations

from collections import defaultdict
from html import escape
from pathlib import Path
from typing import Any

from .dt35_role_report import DT35RoleRow, summarize_dt35_roles


RISK_COLORS = {
    "usable": "#28d17c",
    "ignored_interference": "#4dabf7",
    "out_of_range": "#8b949e",
    "corner_ambiguous": "#ff9f1c",
    "grazing_filtered": "#ffd166",
    "no_hit": "#6e7681",
    "skipped": "#d29922",
}

TARGET_MARKERS = {
    "usable_wall": "#ff4d5a",
    "solid_obstacle": "#2ecc71",
    "ignore": "#4dabf7",
}


def write_dt35_role_svg(
    path: str | Path,
    rows: list[DT35RoleRow],
    config: dict[str, Any],
    *,
    title: str = "DT35 role map",
) -> None:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(build_dt35_role_svg(rows, config, title=title), encoding="utf-8")


def build_dt35_role_svg(rows: list[DT35RoleRow], config: dict[str, Any], *, title: str = "DT35 role map") -> str:
    map_cfg = config.get("map", {})
    field_w = float(map_cfg.get("field_width_cm", 1215.0))
    field_h = float(map_cfg.get("field_height_cm", 1210.0))
    yaws = sorted({round(row.pose_yaw_deg, 6) for row in rows})
    if not yaws:
        yaws = [0.0]
    cols = min(4, max(1, len(yaws)))
    rows_count = (len(yaws) + cols - 1) // cols
    panel_w = 300.0
    panel_h = 300.0
    margin = 26.0
    gap = 28.0
    top_h = 104.0
    legend_h = 82.0
    width = cols * panel_w + (cols + 1) * gap
    height = top_h + rows_count * (panel_h + margin) + legend_h
    by_yaw: dict[float, list[DT35RoleRow]] = defaultdict(list)
    for row in rows:
        by_yaw[round(row.pose_yaw_deg, 6)].append(row)
    summary = summarize_dt35_roles(rows)

    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width:.0f}" height="{height:.0f}" viewBox="0 0 {width:.0f} {height:.0f}">',
        "<style>",
        "text{font-family:Segoe UI,Arial,sans-serif;fill:#d6deeb;font-size:12px}",
        ".small{font-size:10px;fill:#9fb0c3}",
        ".title{font-size:22px;font-weight:700;fill:#ffffff}",
        ".panel-title{font-size:13px;font-weight:600;fill:#ffffff}",
        ".field{fill:#101820;stroke:#718096;stroke-width:1.2}",
        ".grid{stroke:#304052;stroke-width:.45}",
        ".axis-x{stroke:#ff6b6b;stroke-width:.8}",
        ".axis-y{stroke:#4dabf7;stroke-width:.8}",
        "</style>",
        '<rect x="0" y="0" width="100%" height="100%" fill="#0b1118"/>',
        f'<text class="title" x="{gap:.0f}" y="34">{escape(title)}</text>',
        f'<text x="{gap:.0f}" y="58">poses={summary.poses} rays={summary.rows} usable={summary.usable_rows} forest={summary.usable_forest_rows} ramp={summary.usable_ramp_rows} ignored={summary.ignored_rows} out_of_range={summary.out_of_range_rows} corner={summary.corner_rows}</text>',
        f'<text class="small" x="{gap:.0f}" y="78">Color shows risk/state; dot=sensor_1 right mount local -X, square=sensor_2 left mount local +X. Coordinates are field center cm, +X right, +Y up.</text>',
    ]
    for index, yaw in enumerate(yaws):
        col = index % cols
        row_index = index // cols
        x0 = gap + col * (panel_w + gap)
        y0 = top_h + row_index * (panel_h + margin)
        parts.extend(_panel_svg(x0, y0, panel_w, panel_h, field_w, field_h, yaw, by_yaw.get(yaw, [])))
    parts.extend(_legend_svg(gap, height - legend_h + 18.0))
    parts.append("</svg>")
    return "\n".join(parts)


def _panel_svg(
    x0: float,
    y0: float,
    panel_w: float,
    panel_h: float,
    field_w: float,
    field_h: float,
    yaw: float,
    rows: list[DT35RoleRow],
) -> list[str]:
    scale = min(panel_w / field_w, panel_h / field_h)
    draw_w = field_w * scale
    draw_h = field_h * scale
    ox = x0 + (panel_w - draw_w) * 0.5
    oy = y0 + 18.0 + (panel_h - draw_h) * 0.5

    def sx(x_cm: float) -> float:
        return ox + (x_cm + field_w * 0.5) * scale

    def sy(y_cm: float) -> float:
        return oy + (field_h * 0.5 - y_cm) * scale

    parts = [
        f'<text class="panel-title" x="{x0:.1f}" y="{y0 + 12:.1f}">H30 yaw {yaw:g} deg</text>',
        f'<rect class="field" x="{ox:.1f}" y="{oy:.1f}" width="{draw_w:.1f}" height="{draw_h:.1f}" rx="3"/>',
    ]
    grid_step = 200.0
    gx = -field_w * 0.5
    while gx <= field_w * 0.5 + 1.0e-6:
        parts.append(f'<line class="grid" x1="{sx(gx):.1f}" y1="{oy:.1f}" x2="{sx(gx):.1f}" y2="{oy + draw_h:.1f}"/>')
        gx += grid_step
    gy = -field_h * 0.5
    while gy <= field_h * 0.5 + 1.0e-6:
        parts.append(f'<line class="grid" x1="{ox:.1f}" y1="{sy(gy):.1f}" x2="{ox + draw_w:.1f}" y2="{sy(gy):.1f}"/>')
        gy += grid_step
    parts.append(f'<line class="axis-y" x1="{sx(0):.1f}" y1="{oy:.1f}" x2="{sx(0):.1f}" y2="{oy + draw_h:.1f}"/>')
    parts.append(f'<line class="axis-x" x1="{ox:.1f}" y1="{sy(0):.1f}" x2="{ox + draw_w:.1f}" y2="{sy(0):.1f}"/>')

    for item in rows:
        px = sx(item.pose_x_cm)
        py = sy(item.pose_y_cm)
        color = _row_color(item)
        stroke = TARGET_MARKERS.get(item.expected_target_type, "#c9d1d9")
        tooltip = escape(
            f"{item.pose_label} {item.sensor_key} yaw={item.pose_yaw_deg:g} "
            f"axis={item.world_constraint_axis} target={item.expected_target} "
            f"type={item.expected_target_type} risk={item.risk} expected={item.expected_distance_cm:.1f}cm"
        )
        if item.sensor_key == "sensor_1":
            parts.append(
                f'<circle cx="{px - 2.2:.1f}" cy="{py:.1f}" r="2.8" fill="{color}" stroke="{stroke}" stroke-width="1"><title>{tooltip}</title></circle>'
            )
        else:
            parts.append(
                f'<rect x="{px - 0.5:.1f}" y="{py - 2.8:.1f}" width="5.6" height="5.6" fill="{color}" stroke="{stroke}" stroke-width="1"><title>{tooltip}</title></rect>'
            )
    return parts


def _legend_svg(x: float, y: float) -> list[str]:
    parts = [f'<text class="panel-title" x="{x:.1f}" y="{y:.1f}">Legend</text>']
    lx = x
    ly = y + 20.0
    for label, color in (
        ("usable", RISK_COLORS["usable"]),
        ("ignore/noisy", RISK_COLORS["ignored_interference"]),
        ("out_of_range", RISK_COLORS["out_of_range"]),
        ("corner", RISK_COLORS["corner_ambiguous"]),
        ("grazing", RISK_COLORS["grazing_filtered"]),
        ("no_hit", RISK_COLORS["no_hit"]),
    ):
        parts.append(f'<circle cx="{lx:.1f}" cy="{ly:.1f}" r="5" fill="{color}"/>')
        parts.append(f'<text class="small" x="{lx + 10:.1f}" y="{ly + 4:.1f}">{label}</text>')
        lx += 108.0
    lx = x
    ly += 24.0
    for label, color in (
        ("red stroke=usable wall", TARGET_MARKERS["usable_wall"]),
        ("green stroke=forest/ramp", TARGET_MARKERS["solid_obstacle"]),
        ("blue stroke=ignored area", TARGET_MARKERS["ignore"]),
    ):
        parts.append(f'<rect x="{lx - 5:.1f}" y="{ly - 5:.1f}" width="10" height="10" fill="none" stroke="{color}" stroke-width="2"/>')
        parts.append(f'<text class="small" x="{lx + 10:.1f}" y="{ly + 4:.1f}">{label}</text>')
        lx += 185.0
    return parts


def _row_color(row: DT35RoleRow) -> str:
    return RISK_COLORS.get(row.risk, "#c9d1d9")
