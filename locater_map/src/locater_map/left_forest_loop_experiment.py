from __future__ import annotations

import csv
import json
from dataclasses import asdict, dataclass, replace
from datetime import datetime
from html import escape
from math import cos, hypot, isfinite, radians, sin
from pathlib import Path
from typing import Any

from .config_loader import resolve_resource
from .data_model import RobotFrame
from .dt35_analysis import analyze_dt35_frames, summarize_residuals, write_residual_rows_csv, write_residual_summary_json
from .field_model_export import _field_segments_svg, _legend, _relative_href, _style  # internal SVG helpers
from .fusion_model import FusionConfig, wrap_deg, write_frames_csv, write_metrics_json
from .path_diagnostics import (
    generate_synthetic_path_diagnostic,
    write_path_diagnostic_csv,
    write_path_diagnostic_summary,
)
from .synthetic_sim import SyntheticConfig, generate_synthetic_frames
from .utils_transform import dt35_ray, heading_vector_from_front_yaw, robot_local_to_world


LEFT_FOREST_STAGES = ("ideal", "pid", "async_occlusion", "lidar_noise")


@dataclass(slots=True)
class LeftForestLoopExperimentResult:
    output_dir: str
    stage: str
    frames: int
    hz: float
    duration_s: float
    screenshot_count: int
    ground_truth_csv: str
    truth_csv: str
    firmware_like_csv: str
    fused_csv: str
    raw_serial_log: str
    path_diagnostic_csv: str
    path_diagnostic_summary_json: str
    dt35_residual_csv: str
    dt35_residual_summary_json: str
    overview_svg: str
    overview_png: str
    report_md: str
    report_json: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def run_left_forest_loop_experiment(
    config: dict[str, Any],
    *,
    output_dir: str | Path | None = None,
    duration_s: float = 60.0,
    hz: float = 10.0,
    screenshot_hz: float = 1.0,
    stage: str = "ideal",
    encoder_x_scale: float = 1.0,
    encoder_y_scale: float = 1.0,
    encoder_yaw_scale: float = 1.0,
    h30_yaw_bias_deg: float = 0.0,
    dt35_noise_mm: float = 0.0,
    render_png: bool = True,
) -> LeftForestLoopExperimentResult:
    if stage not in LEFT_FOREST_STAGES:
        raise ValueError(f"unknown left forest stage: {stage}")
    hz = max(1.0, float(hz))
    duration_s = max(1.0, float(duration_s))
    samples = int(round(duration_s * hz)) + 1
    sample_period_s = 1.0 / hz
    root = Path(config.get("_project_root", Path.cwd()))
    out_dir = Path(output_dir) if output_dir else root / "logs" / "RL_data" / f"{_stamp()}_sim_left_forest_{stage}_log"
    sensor_dir = out_dir / "sensor_data"
    png_dir = out_dir / "png"
    svg_dir = out_dir / "svg"
    sensor_dir.mkdir(parents=True, exist_ok=True)
    png_dir.mkdir(parents=True, exist_ok=True)
    svg_dir.mkdir(parents=True, exist_ok=True)

    synthetic_cfg = _stage_synthetic_config(
        stage,
        samples=samples,
        path_name="left_forest_loop",
        sample_period_s=sample_period_s,
        encoder_x_scale=encoder_x_scale,
        encoder_y_scale=encoder_y_scale,
        encoder_yaw_scale=encoder_yaw_scale,
        h30_yaw_bias_deg=h30_yaw_bias_deg,
    )
    synthetic_cfg = replace(synthetic_cfg, dt35_noise_mm=max(float(dt35_noise_mm), synthetic_cfg.dt35_noise_mm))
    fusion_cfg = _stage_fusion_config(
        stage,
        config,
        hz,
    )
    ground_truth_cfg = _ground_truth_config(synthetic_cfg)

    ground_truth_frames = generate_synthetic_frames(config, ground_truth_cfg)
    truth_frames = generate_synthetic_frames(config, synthetic_cfg)
    diagnostic_rows, diagnostic_summary, fused_frames = generate_synthetic_path_diagnostic(config, synthetic_cfg, fusion_cfg)
    replay_tf = _replay_start_transform(config)
    firmware_like_frames = [
        _firmware_like_frame(truth, fused, replay_tf)
        for truth, fused in zip(truth_frames, fused_frames)
    ]
    residual_rows = analyze_dt35_frames(config, truth_frames, pose_source="lidar", yaw_source="h30", start_policy="off")
    residual_summary = summarize_residuals(residual_rows)

    ground_truth_csv = sensor_dir / "ground_truth_frames.csv"
    truth_csv = sensor_dir / "truth_frames.csv"
    firmware_csv = sensor_dir / "firmware_like_frames.csv"
    fused_csv = sensor_dir / "fused_frames.csv"
    raw_log = sensor_dir / "raw_serial.log"
    path_csv = sensor_dir / "path_diagnostic.csv"
    path_json = sensor_dir / "path_diagnostic_summary.json"
    residual_csv = sensor_dir / "dt35_residual_rows.csv"
    residual_json = sensor_dir / "dt35_residual_summary.json"
    report_json = out_dir / "report.json"
    report_md = out_dir / "report.md"
    overview_svg = svg_dir / "overview.svg"
    overview_png = png_dir / "overview.png"

    write_frames_csv(ground_truth_csv, ground_truth_frames)
    write_frames_csv(truth_csv, truth_frames)
    write_frames_csv(firmware_csv, firmware_like_frames)
    write_frames_csv(fused_csv, fused_frames)
    _write_raw_serial_log(raw_log, firmware_like_frames)
    write_path_diagnostic_csv(path_csv, diagnostic_rows)
    write_path_diagnostic_summary(path_json, diagnostic_summary)
    write_residual_rows_csv(residual_csv, residual_rows)
    write_residual_summary_json(residual_json, residual_summary)

    _write_loop_svg(overview_svg, config, ground_truth_frames, fused_frames, frame_index=len(ground_truth_frames) - 1, title=f"left forest loop {stage}")
    if render_png:
        _render_svg_to_png(overview_svg, overview_png)

    screenshot_count = _write_timed_snapshots(
        config,
        ground_truth_frames,
        fused_frames,
        svg_dir,
        png_dir,
        screenshot_hz=max(0.1, float(screenshot_hz)),
        render_png=render_png,
    )

    report_payload = _build_report_payload(
        config=config,
        stage=stage,
        synthetic_cfg=synthetic_cfg,
        fusion_cfg=fusion_cfg,
        ground_truth_frames=ground_truth_frames,
        truth_frames=truth_frames,
        fused_frames=fused_frames,
        diagnostic_summary=diagnostic_summary.to_dict(),
        residual_summary=residual_summary.to_dict(),
        screenshot_count=screenshot_count,
    )
    report_json.write_text(json.dumps(_json_safe(report_payload), ensure_ascii=False, indent=2), encoding="utf-8")
    report_md.write_text(_build_report_markdown(report_payload), encoding="utf-8")

    result = LeftForestLoopExperimentResult(
        output_dir=str(out_dir),
        stage=stage,
        frames=len(truth_frames),
        hz=hz,
        duration_s=duration_s,
        screenshot_count=screenshot_count,
        ground_truth_csv=str(ground_truth_csv),
        truth_csv=str(truth_csv),
        firmware_like_csv=str(firmware_csv),
        fused_csv=str(fused_csv),
        raw_serial_log=str(raw_log),
        path_diagnostic_csv=str(path_csv),
        path_diagnostic_summary_json=str(path_json),
        dt35_residual_csv=str(residual_csv),
        dt35_residual_summary_json=str(residual_json),
        overview_svg=str(overview_svg),
        overview_png=str(overview_png),
        report_md=str(report_md),
        report_json=str(report_json),
    )
    (out_dir / "metadata.json").write_text(json.dumps(_json_safe(result.to_dict()), ensure_ascii=False, indent=2), encoding="utf-8")
    return result


def _stage_synthetic_config(
    stage: str,
    *,
    samples: int,
    path_name: str,
    sample_period_s: float,
    encoder_x_scale: float,
    encoder_y_scale: float,
    encoder_yaw_scale: float,
    h30_yaw_bias_deg: float,
) -> SyntheticConfig:
    base = SyntheticConfig(
        samples=samples,
        path_name=path_name,
        sample_period_s=sample_period_s,
        encoder_x_scale=encoder_x_scale,
        encoder_y_scale=encoder_y_scale,
        encoder_yaw_scale=encoder_yaw_scale,
        h30_yaw_bias_deg=h30_yaw_bias_deg,
    )
    if stage == "ideal":
        return base
    if stage == "pid":
        return replace(
            base,
            encoder_x_scale=0.985,
            encoder_y_scale=1.015,
            path_wiggle_cm=6.0,
            path_wiggle_cycles=9.0,
            path_yaw_wiggle_deg=2.0,
        )
    if stage == "async_occlusion":
        return replace(
            base,
            encoder_x_scale=0.985,
            encoder_y_scale=1.015,
            path_wiggle_cm=6.0,
            path_wiggle_cycles=9.0,
            path_yaw_wiggle_deg=2.0,
            h30_delay_frames=1,
            dt35_delay_frames=2,
            dt35_noise_mm=2.0,
            dt35_dropout_rate=0.012,
            dt35_occlusion_rate=0.040,
        )
    if stage == "lidar_noise":
        return replace(
            base,
            encoder_x_scale=0.985,
            encoder_y_scale=1.015,
            path_wiggle_cm=6.0,
            path_wiggle_cycles=9.0,
            path_yaw_wiggle_deg=2.0,
            lidar_xy_noise_cm=0.3,
            lidar_yaw_noise_deg=0.3,
        )
    raise ValueError(f"unknown left forest stage: {stage}")


def _stage_fusion_config(stage: str, config: dict[str, Any], hz: float) -> FusionConfig:
    base = FusionConfig(
        lidar_stride=max(1, int(round(hz))),
        lidar_gain=1.0,
        dt35_gain=1.0,
        dt35_yaw_gain=0.0,
        dt35_correct_lidar_frames=False,
        dt35_residual_gate_cm=float(config.get("display", {}).get("live_fusion_dt35_residual_gate_cm", 40.0)),
    )
    if stage == "lidar_noise":
        return replace(base, lidar_gain=0.7, dt35_correct_lidar_frames=True, dt35_gain=0.8, dt35_residual_gate_cm=18.0)
    if stage == "async_occlusion":
        return replace(base, dt35_gain=0.02, dt35_residual_gate_cm=18.0, dt35_max_translation_step_cm=2.0)
    return base


def _ground_truth_config(cfg: SyntheticConfig) -> SyntheticConfig:
    return replace(
        cfg,
        encoder_x_scale=1.0,
        encoder_y_scale=1.0,
        encoder_yaw_scale=1.0,
        h30_yaw_bias_deg=0.0,
        lidar_xy_noise_cm=0.0,
        lidar_yaw_noise_deg=0.0,
        dt35_noise_mm=0.0,
        h30_delay_frames=0,
        dt35_delay_frames=0,
        dt35_dropout_rate=0.0,
        dt35_occlusion_rate=0.0,
    )


def _firmware_like_frame(
    truth: RobotFrame,
    fused: RobotFrame,
    replay_tf: tuple[float, float, float, bool] | None = None,
) -> RobotFrame:
    tf = replay_tf or (0.0, 0.0, 0.0, False)
    pos_x, pos_y, pos_yaw = _field_pose_to_local(fused.pos_x_cm, fused.pos_y_cm, fused.pos_yaw_deg, tf)
    enc_x, enc_y, enc_yaw = _field_pose_to_local(truth.encoder_x_cm, truth.encoder_y_cm, truth.h30_yaw_deg, tf)
    lidar_x, lidar_y, lidar_yaw = _field_pose_to_local(truth.lidar_x_cm, truth.lidar_y_cm, truth.lidar_yaw_deg, tf)
    if truth.seq == 0:
        pos_x = pos_y = pos_yaw = 0.0
        enc_x = enc_y = enc_yaw = 0.0
        lidar_x = lidar_y = lidar_yaw = 0.0
    return RobotFrame(
        source_time_ms=truth.source_time_ms,
        pc_time=truth.pc_time,
        seq=truth.seq,
        pos_x_cm=pos_x,
        pos_y_cm=pos_y,
        pos_yaw_deg=pos_yaw,
        calib_x_cm=enc_x,
        calib_y_cm=enc_y,
        calib_yaw_deg=enc_yaw,
        encoder_x_cm=enc_x,
        encoder_y_cm=enc_y,
        h30_yaw_deg=enc_yaw,
        lidar_x_cm=lidar_x,
        lidar_y_cm=lidar_y,
        lidar_yaw_deg=lidar_yaw,
        dt35_1_mm=truth.dt35_1_mm,
        dt35_2_mm=truth.dt35_2_mm,
        dt35_1_valid=truth.dt35_1_valid,
        dt35_2_valid=truth.dt35_2_valid,
        h30_valid=truth.h30_valid,
        h30_has_attitude=truth.h30_has_attitude,
        lidar_valid=truth.lidar_valid,
        lidar_online=truth.lidar_online,
        x_pulse_seen=truth.x_pulse_seen,
        y_pulse_seen=truth.y_pulse_seen,
        x_delta_count=truth.x_delta_count,
        y_delta_count=truth.y_delta_count,
        x_total_count=truth.x_total_count,
        y_total_count=truth.y_total_count,
        x_raw_count=truth.x_raw_count,
        y_raw_count=truth.y_raw_count,
        encoder_dis_p_mm=truth.encoder_dis_p_mm,
        encoder_dis_q_mm=truth.encoder_dis_q_mm,
        status=truth.status,
        raw_line=_csv_line(
            pos_x,
            pos_y,
            pos_yaw,
            lidar_x,
            lidar_y,
            lidar_yaw,
            enc_x,
            enc_y,
            enc_yaw,
            truth.dt35_1_mm,
            truth.dt35_2_mm,
            truth.status,
        ),
        protocol="synthetic_r1_csv_v3",
    )


def _replay_start_transform(config: dict[str, Any]) -> tuple[float, float, float, bool]:
    robot = config.get("robot", {})
    side = str(robot.get("default_start_side", "red"))
    if side not in ("red", "blue"):
        side = "red"
    pose = robot.get(f"start_pose_{side}", {})
    return (
        float(pose.get("x_cm", 0.0)),
        float(pose.get("y_cm", 0.0)),
        float(pose.get("yaw_deg", 0.0)),
        True,
    )


def _field_pose_to_local(
    x_cm: float,
    y_cm: float,
    yaw_deg: float,
    start_tf: tuple[float, float, float, bool],
) -> tuple[float, float, float]:
    ox, oy, oyaw, enabled = start_tf
    if not enabled:
        return x_cm, y_cm, yaw_deg
    yaw = radians(oyaw)
    dx = x_cm - ox
    dy = y_cm - oy
    return (
        dx * cos(yaw) + dy * sin(yaw),
        -dx * sin(yaw) + dy * cos(yaw),
        wrap_deg(yaw_deg - oyaw),
    )


def _write_raw_serial_log(path: Path, frames: list[RobotFrame]) -> None:
    with path.open("w", encoding="utf-8", newline="") as f:
        for frame in frames:
            f.write(frame.raw_line or _csv_line(
                frame.pos_x_cm,
                frame.pos_y_cm,
                frame.pos_yaw_deg,
                frame.lidar_x_cm,
                frame.lidar_y_cm,
                frame.lidar_yaw_deg,
                frame.encoder_x_cm,
                frame.encoder_y_cm,
                frame.h30_yaw_deg,
                frame.dt35_1_mm,
                frame.dt35_2_mm,
                frame.status,
            ))
            f.write("\n")


def _csv_line(*values: Any) -> str:
    out: list[str] = []
    for value in values:
        if isinstance(value, float):
            out.append(f"{value:.3f}")
        else:
            out.append(str(value))
    return ",".join(out)


def _write_timed_snapshots(
    config: dict[str, Any],
    truth_frames: list[RobotFrame],
    fused_frames: list[RobotFrame],
    svg_dir: Path,
    png_dir: Path,
    *,
    screenshot_hz: float,
    render_png: bool,
) -> int:
    if not truth_frames:
        return 0
    interval_s = 1.0 / max(0.1, screenshot_hz)
    sample_period_s = max(0.001, truth_frames[1].pc_time - truth_frames[0].pc_time) if len(truth_frames) > 1 else 0.1
    step = max(1, int(round(interval_s / sample_period_s)))
    indices = list(range(0, len(truth_frames), step))
    if indices[-1] != len(truth_frames) - 1:
        indices.append(len(truth_frames) - 1)
    for index in indices:
        seq = truth_frames[index].seq
        seconds = truth_frames[index].pc_time
        stem = f"t_{seconds:06.1f}s_seq{seq:04d}"
        svg_path = svg_dir / f"{stem}.svg"
        png_path = png_dir / f"{stem}.png"
        _write_loop_svg(svg_path, config, truth_frames, fused_frames, frame_index=index, title=stem)
        if render_png:
            _render_svg_to_png(svg_path, png_path)
    return len(indices)


def _write_loop_svg(
    path: Path,
    config: dict[str, Any],
    truth_frames: list[RobotFrame],
    fused_frames: list[RobotFrame],
    *,
    frame_index: int,
    title: str,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(_build_loop_svg(config, truth_frames, fused_frames, frame_index=frame_index, title=title), encoding="utf-8")


def _build_loop_svg(
    config: dict[str, Any],
    truth_frames: list[RobotFrame],
    fused_frames: list[RobotFrame],
    *,
    frame_index: int,
    title: str,
) -> str:
    map_cfg = config.get("map", {})
    field_w_cm = float(map_cfg.get("field_width_cm", 1215.0))
    field_h_cm = float(map_cfg.get("field_height_cm", 1210.0))
    img_w = int(round(field_w_cm * 2.0))
    img_h = int(round(field_h_cm * 2.0))
    prior_path = resolve_resource(config, map_cfg.get("prior_map_config"))
    if prior_path and prior_path.exists():
        try:
            prior = json.loads(prior_path.read_text(encoding="utf-8"))
            img_w = int(prior.get("image_width_px", img_w))
            img_h = int(prior.get("image_height_px", img_h))
        except (OSError, json.JSONDecodeError, ValueError):
            pass
    legend_w = 660
    canvas_w = img_w + legend_w
    px_per_cm_x = img_w / field_w_cm
    px_per_cm_y = img_h / field_h_cm

    def world_to_pixel(x_cm: float, y_cm: float) -> tuple[float, float]:
        return (x_cm + field_w_cm * 0.5) * px_per_cm_x, (field_h_cm * 0.5 - y_cm) * px_per_cm_y

    background = resolve_resource(config, map_cfg.get("labeled_background_image") or map_cfg.get("background_image"))
    elements = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{canvas_w}" height="{img_h}" viewBox="0 0 {canvas_w} {img_h}">',
        f"<title>{escape(title)}</title>",
        '<rect x="0" y="0" width="100%" height="100%" fill="#0b1118"/>',
    ]
    href = _relative_href(background, config)
    if href:
        elements.append(f'<image href="{escape(href)}" x="0" y="0" width="{img_w}" height="{img_h}" opacity="0.84"/>')
    elements.append(_legend(img_w + 24, 24))
    elements.extend(_field_segments_svg(config, world_to_pixel, px_per_cm_x, px_per_cm_y))
    elements.append(_polyline_svg([frame.lidar_x_cm for frame in truth_frames], [frame.lidar_y_cm for frame in truth_frames], world_to_pixel, "#35d0ff", "truth/lidar full path", 5, 0.65))
    elements.append(_polyline_svg([frame.pos_x_cm for frame in fused_frames[: frame_index + 1]], [frame.pos_y_cm for frame in fused_frames[: frame_index + 1]], world_to_pixel, "#00ffaa", "fused path so far", 6, 0.85))
    elements.append(_polyline_svg([frame.encoder_x_cm for frame in truth_frames[: frame_index + 1]], [frame.encoder_y_cm for frame in truth_frames[: frame_index + 1]], world_to_pixel, "#f59e0b", "encoder/H30 odom path so far", 4, 0.75, dash="12 8"))
    if truth_frames and fused_frames:
        index = max(0, min(frame_index, len(truth_frames) - 1, len(fused_frames) - 1))
        elements.extend(_current_pose_svg(config, truth_frames[index], fused_frames[index], world_to_pixel, px_per_cm_x, px_per_cm_y))
    elements.append("</svg>")
    return "\n".join(item for item in elements if item) + "\n"


def _polyline_svg(
    xs: list[float],
    ys: list[float],
    world_to_pixel,
    color: str,
    label: str,
    width: int,
    opacity: float,
    *,
    dash: str = "",
) -> str:
    if len(xs) < 2 or len(xs) != len(ys):
        return ""
    points = " ".join(f"{x:.2f},{y:.2f}" for x, y in (world_to_pixel(px, py) for px, py in zip(xs, ys)))
    dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
    return (
        f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="{width}" '
        f'opacity="{opacity}" stroke-linejoin="round" stroke-linecap="round"{dash_attr}>'
        f"<title>{escape(label)}</title></polyline>"
    )


def _current_pose_svg(
    config: dict[str, Any],
    truth: RobotFrame,
    fused: RobotFrame,
    world_to_pixel,
    px_per_cm_x: float,
    px_per_cm_y: float,
) -> list[str]:
    elements: list[str] = []
    field_model = _field_model_from_config(config)
    for key, distance_mm, valid in (
        ("sensor_1", truth.dt35_1_mm, truth.dt35_1_valid),
        ("sensor_2", truth.dt35_2_mm, truth.dt35_2_valid),
    ):
        sensor_cfg = config.get("dt35", {}).get(key, {})
        ray = dt35_ray(truth.lidar_x_cm, truth.lidar_y_cm, truth.h30_yaw_deg, sensor_cfg, distance_mm if valid else 0.0, field_model)
        target_stroke, _fill, _opacity = _style(str(ray.get("expected_target_type", "")))
        ray_stroke = "#ffd24a" if key == "sensor_1" else "#29d4ff"
        sx, sy = world_to_pixel(float(ray["sensor_x_cm"]), float(ray["sensor_y_cm"]))
        hx, hy = world_to_pixel(float(ray["display_hit_x_cm"]), float(ray["display_hit_y_cm"]))
        title = (
            f"{key}: target={ray.get('expected_target')} type={ray.get('expected_target_type')} "
            f"meas={float(ray.get('distance_cm', 0.0)):.1f}cm exp={float(ray.get('expected_distance_cm', 0.0)):.1f}cm "
            f"res={float(ray.get('residual_cm', 0.0)):.1f}cm"
        )
        dash = ' stroke-dasharray="10 8"' if not bool(ray.get("correction_allowed", False)) else ""
        elements.append(
            f'<line x1="{sx:.2f}" y1="{sy:.2f}" x2="{hx:.2f}" y2="{hy:.2f}" '
            f'stroke="{ray_stroke}" stroke-width="5" opacity="0.95"{dash}><title>{escape(title)}</title></line>'
        )
        elements.append(f'<circle cx="{hx:.2f}" cy="{hy:.2f}" r="7" fill="{target_stroke}" stroke="#111827" stroke-width="2"><title>{escape(title)}</title></circle>')
    elements.extend(_robot_svg(config, fused, world_to_pixel, px_per_cm_x, px_per_cm_y))
    tx, ty = world_to_pixel(truth.lidar_x_cm, truth.lidar_y_cm)
    elements.append(f'<circle cx="{tx:.2f}" cy="{ty:.2f}" r="7" fill="#35d0ff" stroke="#0b1118" stroke-width="3"><title>true center</title></circle>')
    return elements


def _robot_svg(config: dict[str, Any], frame: RobotFrame, world_to_pixel, px_per_cm_x: float, px_per_cm_y: float) -> list[str]:
    robot_cfg = config.get("robot", {})
    cx, cy = world_to_pixel(frame.pos_x_cm, frame.pos_y_cm)
    size_cm = float(robot_cfg.get("size_cm", 83.0))
    width_px = size_cm * px_per_cm_x
    height_px = size_cm * px_per_cm_y
    texture = resolve_resource(config, robot_cfg.get("texture_path"))
    rotation = frame.pos_yaw_deg + float(robot_cfg.get("yaw_offset_deg", 0.0)) - 90.0 - float(robot_cfg.get("texture_front_dir_deg_in_image", 0.0))
    elements: list[str] = [f'<g transform="translate({cx:.2f},{cy:.2f}) rotate({rotation:.2f})">']
    href = _relative_href(texture, config)
    if href:
        elements.append(
            f'<image href="{escape(href)}" x="{-width_px * 0.5:.2f}" y="{-height_px * 0.5:.2f}" '
            f'width="{width_px:.2f}" height="{height_px:.2f}" opacity="0.96"/>'
        )
    else:
        elements.append(
            f'<rect x="{-width_px * 0.5:.2f}" y="{-height_px * 0.5:.2f}" width="{width_px:.2f}" height="{height_px:.2f}" '
            f'fill="#1f6feb" fill-opacity="0.35" stroke="#ffffff" stroke-width="4"/>'
        )
    elements.append(f'<rect x="{-width_px * 0.5:.2f}" y="{-height_px * 0.5:.2f}" width="{width_px:.2f}" height="{height_px:.2f}" fill="none" stroke="#ffffff" stroke-width="4"/>')
    elements.append(f'<line x1="0" y1="0" x2="{width_px * 0.45:.2f}" y2="0" stroke="#00ffaa" stroke-width="5" marker-end="url(#arrow)"/>')
    elements.append("</g>")
    elements.append(f'<circle cx="{cx:.2f}" cy="{cy:.2f}" r="5" fill="#ffffff" stroke="#111827" stroke-width="2"><title>fused/display center</title></circle>')
    return elements


def _field_model_from_config(config: dict[str, Any]) -> dict[str, Any]:
    model = dict(config.get("field_model", {}))
    model.setdefault("enabled", True)
    model.setdefault("use_field_boundary", True)
    model.setdefault("field_width_cm", config.get("map", {}).get("field_width_cm", 1215.0))
    model.setdefault("field_height_cm", config.get("map", {}).get("field_height_cm", 1210.0))
    return model


def _build_report_payload(
    *,
    config: dict[str, Any],
    stage: str,
    synthetic_cfg: SyntheticConfig,
    fusion_cfg: FusionConfig,
    ground_truth_frames: list[RobotFrame],
    truth_frames: list[RobotFrame],
    fused_frames: list[RobotFrame],
    diagnostic_summary: dict[str, Any],
    residual_summary: dict[str, Any],
    screenshot_count: int,
) -> dict[str, Any]:
    encoder_errors = [
        hypot(frame.encoder_x_cm - frame.lidar_x_cm, frame.encoder_y_cm - frame.lidar_y_cm)
        for frame in truth_frames
    ]
    fused_errors = [
        hypot(fused.pos_x_cm - truth.lidar_x_cm, fused.pos_y_cm - truth.lidar_y_cm)
        for truth, fused in zip(truth_frames, fused_frames)
    ]
    h30_yaw_errors = [wrap_deg(frame.h30_yaw_deg - frame.lidar_yaw_deg) for frame in truth_frames]
    ground_truth_metrics = _ground_truth_metrics(ground_truth_frames, truth_frames, fused_frames)
    return {
        "experiment": "left_forest_loop",
        "stage": stage,
        "notes": [
            "The ideal field model intentionally keeps all official walls/forest/ramp obstacles. Missing practice-field walls, floor hits, people, and R2 occlusions should appear as residual/floor/no-hit anomalies in real logs, not be encoded into the ideal model.",
            "DT35 rays use H30 yaw and the configured side-mounted offsets; displayed rays are clipped at the nearest modeled solid/usable wall to prevent visual wall penetration.",
            "ground_truth_frames.csv is the noiseless field pose. truth_frames.csv is the sensor stream for this stage. firmware_like_frames.csv is the replayable USART1-style fused output.",
        ],
        "synthetic_config": asdict(synthetic_cfg),
        "fusion_config": asdict(fusion_cfg),
        "field_model": {
            "field_width_cm": config.get("map", {}).get("field_width_cm"),
            "field_height_cm": config.get("map", {}).get("field_height_cm"),
            "start_pose_policy": config.get("robot", {}).get("start_pose_policy"),
            "dt35_sensor_1": config.get("dt35", {}).get("sensor_1", {}),
            "dt35_sensor_2": config.get("dt35", {}).get("sensor_2", {}),
        },
        "sample_count": len(truth_frames),
        "duration_s": truth_frames[-1].pc_time if truth_frames else 0.0,
        "screenshot_count": screenshot_count,
        "error_metrics": {
            "encoder_h30_odom_rms_xy_cm": _rms(encoder_errors),
            "encoder_h30_odom_max_xy_cm": max(encoder_errors, default=None),
            "fused_rms_xy_cm": _rms(fused_errors),
            "fused_max_xy_cm": max(fused_errors, default=None),
            "h30_yaw_rms_error_deg": _rms(h30_yaw_errors),
            "h30_yaw_max_abs_error_deg": max((abs(v) for v in h30_yaw_errors), default=None),
        },
        "ground_truth_error_metrics": ground_truth_metrics,
        "path_diagnostic_summary": diagnostic_summary,
        "dt35_residual_summary": residual_summary,
    }


def _build_report_markdown(payload: dict[str, Any]) -> str:
    errors = payload["error_metrics"]
    gt_errors = payload["ground_truth_error_metrics"]
    residual = payload["dt35_residual_summary"]
    diagnostic = payload["path_diagnostic_summary"]
    return "\n".join([
        "# Left Forest Loop Synthetic Validation",
        "",
        "## Setup",
        f"- Stage: `{payload['stage']}`",
        f"- Frames: {payload['sample_count']}",
        f"- Duration: {payload['duration_s']:.2f} s",
        f"- Screenshots: {payload['screenshot_count']}",
        f"- Path: `{payload['synthetic_config']['path_name']}` at {1.0 / payload['synthetic_config']['sample_period_s']:.1f} Hz",
        "",
        "## Pose Error Against Noiseless Field Truth",
        f"- Lidar RMS XY error: {_fmt(gt_errors['lidar_rms_xy_cm'])} cm",
        f"- Lidar max XY error: {_fmt(gt_errors['lidar_max_xy_cm'])} cm",
        f"- Lidar yaw RMS error: {_fmt(gt_errors['lidar_yaw_rms_error_deg'])} deg",
        f"- Encoder/H30 odom RMS XY error: {_fmt(gt_errors['encoder_h30_odom_rms_xy_cm'])} cm",
        f"- Encoder/H30 odom max XY error: {_fmt(gt_errors['encoder_h30_odom_max_xy_cm'])} cm",
        f"- Fused RMS XY error: {_fmt(gt_errors['fused_rms_xy_cm'])} cm",
        f"- Fused max XY error: {_fmt(gt_errors['fused_max_xy_cm'])} cm",
        f"- Fused yaw RMS error: {_fmt(gt_errors['fused_yaw_rms_error_deg'])} deg",
        "",
        "## Pose Error Against Lidar Truth",
        f"- Encoder/H30 odom RMS XY error: {_fmt(errors['encoder_h30_odom_rms_xy_cm'])} cm",
        f"- Encoder/H30 odom max XY error: {_fmt(errors['encoder_h30_odom_max_xy_cm'])} cm",
        f"- Fused RMS XY error: {_fmt(errors['fused_rms_xy_cm'])} cm",
        f"- Fused max XY error: {_fmt(errors['fused_max_xy_cm'])} cm",
        f"- H30 yaw RMS error: {_fmt(errors['h30_yaw_rms_error_deg'])} deg",
        f"- H30 yaw max abs error: {_fmt(errors['h30_yaw_max_abs_error_deg'])} deg",
        "",
        "## DT35 Physical Interaction",
        f"- Valid rays: {residual.get('valid_rays')} / {residual.get('rays')}",
        f"- Usable geometry rays: {residual.get('usable_rays')}",
        f"- Fusion usable rays: {residual.get('fusion_usable_rays')}",
        f"- Floor/near-hit suspects: {residual.get('floor_hit_suspect_rays')}",
        f"- High-residual rejected rays: {residual.get('residual_gate_rejected_rays')}",
        f"- RMS residual: {_fmt(residual.get('rms_residual_cm'))} cm",
        f"- Target type counts: `{json.dumps(residual.get('target_type_counts', {}), ensure_ascii=False, sort_keys=True)}`",
        "",
        "## Fusion Diagnostic",
        f"- Raw odom RMS XY: {_fmt(diagnostic.get('raw_rms_xy_cm'))} cm",
        f"- No-DT35 fused RMS XY: {_fmt(diagnostic.get('no_dt35_rms_xy_cm'))} cm",
        f"- DT35 fused RMS XY: {_fmt(diagnostic.get('fused_rms_xy_cm'))} cm",
        f"- DT35 active frames: {diagnostic.get('dt35_active_frames')}",
        f"- DT35 helped / worsened frames: {diagnostic.get('dt35_helped_frames')} / {diagnostic.get('dt35_worsened_frames')}",
        "",
        "## Interpretation",
        "- In the ideal model, residuals should be near zero because DT35 readings are generated from the same first-hit wall model.",
        "- Any visual ray must stop at the first red/green modeled wall; it must not cross the forest block, ramps, center divider, or field boundary.",
        "- Real logs with missing walls or floor/person/R2 occlusions should show no-hit, floor/near-hit, high-residual, or ignored-target flags rather than being trusted for correction.",
        "",
    ])


def _ground_truth_metrics(
    ground_truth_frames: list[RobotFrame],
    sensor_frames: list[RobotFrame],
    fused_frames: list[RobotFrame],
) -> dict[str, Any]:
    pairs = list(zip(ground_truth_frames, sensor_frames, fused_frames))
    lidar_xy = [hypot(sensor.lidar_x_cm - gt.lidar_x_cm, sensor.lidar_y_cm - gt.lidar_y_cm) for gt, sensor, _ in pairs]
    encoder_xy = [hypot(sensor.encoder_x_cm - gt.lidar_x_cm, sensor.encoder_y_cm - gt.lidar_y_cm) for gt, sensor, _ in pairs]
    fused_xy = [hypot(fused.pos_x_cm - gt.lidar_x_cm, fused.pos_y_cm - gt.lidar_y_cm) for gt, _sensor, fused in pairs]
    lidar_yaw = [wrap_deg(sensor.lidar_yaw_deg - gt.lidar_yaw_deg) for gt, sensor, _ in pairs]
    h30_yaw = [wrap_deg(sensor.h30_yaw_deg - gt.lidar_yaw_deg) for gt, sensor, _ in pairs]
    fused_yaw = [wrap_deg(fused.pos_yaw_deg - gt.lidar_yaw_deg) for gt, _sensor, fused in pairs]
    return {
        "lidar_rms_xy_cm": _rms(lidar_xy),
        "lidar_max_xy_cm": max(lidar_xy, default=None),
        "lidar_yaw_rms_error_deg": _rms(lidar_yaw),
        "lidar_yaw_max_abs_error_deg": max((abs(value) for value in lidar_yaw), default=None),
        "encoder_h30_odom_rms_xy_cm": _rms(encoder_xy),
        "encoder_h30_odom_max_xy_cm": max(encoder_xy, default=None),
        "h30_yaw_rms_error_deg": _rms(h30_yaw),
        "h30_yaw_max_abs_error_deg": max((abs(value) for value in h30_yaw), default=None),
        "fused_rms_xy_cm": _rms(fused_xy),
        "fused_max_xy_cm": max(fused_xy, default=None),
        "fused_yaw_rms_error_deg": _rms(fused_yaw),
        "fused_yaw_max_abs_error_deg": max((abs(value) for value in fused_yaw), default=None),
    }


def _render_svg_to_png(svg_path: Path, png_path: Path) -> None:
    from PySide6.QtCore import QSize
    from PySide6.QtGui import QGuiApplication, QImage, QPainter
    from PySide6.QtSvg import QSvgRenderer
    import sys

    app = QGuiApplication.instance() or QGuiApplication(sys.argv[:1])
    renderer = QSvgRenderer(str(svg_path))
    size = renderer.defaultSize()
    if not size.isValid() or size.width() <= 0 or size.height() <= 0:
        size = QSize(3090, 2420)
    image = QImage(size, QImage.Format.Format_ARGB32)
    image.fill(0x00000000)
    painter = QPainter(image)
    renderer.render(painter)
    painter.end()
    png_path.parent.mkdir(parents=True, exist_ok=True)
    if not image.save(str(png_path)):
        raise RuntimeError(f"failed to save {png_path}")


def _stamp() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def _rms(values: list[float]) -> float | None:
    clean = [value for value in values if isfinite(value)]
    if not clean:
        return None
    return (sum(value * value for value in clean) / len(clean)) ** 0.5


def _fmt(value: Any, digits: int = 3) -> str:
    if value is None:
        return "n/a"
    try:
        number = float(value)
    except (TypeError, ValueError):
        return str(value)
    if not isfinite(number):
        return "n/a"
    return f"{number:.{digits}f}"


def _json_safe(value: Any) -> Any:
    if isinstance(value, float):
        return value if isfinite(value) else None
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_json_safe(item) for item in value]
    return value
