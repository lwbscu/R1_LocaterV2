from __future__ import annotations

import csv
import json
from collections import Counter
from dataclasses import asdict, dataclass
from datetime import datetime
from math import hypot, isfinite, sqrt
from pathlib import Path
from statistics import median
from typing import Any

from .data_model import RobotFrame
from .dt35_analysis import DT35FrameResidualRow, analyze_dt35_frames, apply_display_policy, summarize_residuals
from .dt35_range_calibrator import estimate_dt35_range_bias
from .fusion_model import load_frames_csv, wrap_deg


@dataclass(slots=True)
class SeriesStats:
    count: int
    mean: float | None
    std: float | None
    rms: float | None
    min: float | None
    max: float | None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True)
class AxisErrorStats:
    count: int
    mean_x: float | None
    mean_y: float | None
    std_x: float | None
    std_y: float | None
    rms_xy: float | None
    max_xy: float | None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True)
class DatalogSensorCounts:
    frames: int
    lidar_valid: int
    h30_valid: int
    encoder_1_seen: int
    encoder_2_seen: int
    dt35_1_valid: int
    dt35_2_valid: int

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True)
class DatalogDT35SensorSummary:
    sensor_key: str
    valid_rays: int
    usable_rays: int
    fusion_usable_rays: int
    floor_hit_suspect_rays: int
    residual_gate_rejected_rays: int
    ignored_rays: int
    no_hit_rays: int
    corner_ambiguous_rays: int
    grazing_filtered_rays: int
    mean_residual_cm: float | None
    rms_residual_cm: float | None
    max_abs_residual_cm: float | None
    target_type_counts: dict[str, int]
    target_counts: dict[str, int]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True)
class EncoderWindowScaleStats:
    window_frames: int
    count_x: int
    count_y: int
    count_vector: int
    median_x: float | None
    median_y: float | None
    median_vector: float | None
    std_vector: float | None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True)
class DatalogSummary:
    log_name: str
    input_csv: str
    frames: int
    duration_s: float | None
    protocols: dict[str, int]
    sensor_counts: DatalogSensorCounts
    local_lidar_range: dict[str, dict[str, float | None]]
    map_lidar_range: dict[str, dict[str, float | None]]
    encoder_vs_lidar_delta_cm: AxisErrorStats
    h30_vs_lidar_yaw_delta_deg: SeriesStats
    h30_initial_bias_deg: SeriesStats
    encoder_scale_estimate: dict[str, float | None]
    encoder_window_scale: EncoderWindowScaleStats
    dt35_residuals: dict[str, Any]
    dt35_by_sensor: list[DatalogDT35SensorSummary]
    dt35_range_bias: list[dict[str, Any]]
    notes: list[str]

    def to_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["sensor_counts"] = self.sensor_counts.to_dict()
        payload["encoder_vs_lidar_delta_cm"] = self.encoder_vs_lidar_delta_cm.to_dict()
        payload["h30_vs_lidar_yaw_delta_deg"] = self.h30_vs_lidar_yaw_delta_deg.to_dict()
        payload["h30_initial_bias_deg"] = self.h30_initial_bias_deg.to_dict()
        payload["encoder_window_scale"] = self.encoder_window_scale.to_dict()
        payload["dt35_by_sensor"] = [item.to_dict() for item in self.dt35_by_sensor]
        return _json_safe(payload)


@dataclass(slots=True)
class RLDataAnalysisReport:
    created_at: str
    input_logs: list[str]
    output_dir: str
    start_side: str
    start_policy: str
    summaries: list[DatalogSummary]
    aggregate: dict[str, Any]

    def to_dict(self) -> dict[str, Any]:
        return _json_safe(
            {
                "created_at": self.created_at,
                "input_logs": self.input_logs,
                "output_dir": self.output_dir,
                "start_side": self.start_side,
                "start_policy": self.start_policy,
                "summaries": [item.to_dict() for item in self.summaries],
                "aggregate": self.aggregate,
            }
        )


def analyze_rl_logs(
    config: dict[str, Any],
    log_paths: list[str | Path],
    *,
    output_dir: str | Path | None = None,
    start_side: str = "red",
    start_policy: str = "always_local_display",
    range_bias_min_frames: int = 20,
) -> RLDataAnalysisReport:
    resolved_inputs = [_resolve_log_csv(path) for path in log_paths]
    out_dir = _resolve_output_dir(config, output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    summaries: list[DatalogSummary] = []
    for csv_path in resolved_inputs:
        frames = load_frames_csv(csv_path)
        residual_rows = analyze_dt35_frames(
            config,
            frames,
            pose_source="lidar",
            yaw_source="h30",
            start_side=start_side,
            start_policy=start_policy,
        )
        summaries.append(
            _summarize_log(
                config,
                csv_path,
                frames,
                residual_rows,
                start_side=start_side,
                start_policy=start_policy,
                range_bias_min_frames=range_bias_min_frames,
            )
        )
        _write_per_log_dt35_csv(out_dir / f"{csv_path.parent.parent.name}_dt35_residuals.csv", residual_rows)

    aggregate = _aggregate(summaries)
    report = RLDataAnalysisReport(
        created_at=datetime.now().isoformat(timespec="seconds"),
        input_logs=[str(path) for path in resolved_inputs],
        output_dir=str(out_dir),
        start_side=start_side,
        start_policy=start_policy,
        summaries=summaries,
        aggregate=aggregate,
    )
    write_rl_analysis_json(out_dir / "rl_data_analysis.json", report)
    write_rl_analysis_markdown(out_dir / "rl_data_analysis.md", report)
    write_rl_analysis_csv(out_dir / "rl_data_analysis_summary.csv", summaries)
    return report


def write_rl_analysis_json(path: str | Path, report: RLDataAnalysisReport) -> None:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report.to_dict(), ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")


def write_rl_analysis_markdown(path: str | Path, report: RLDataAnalysisReport) -> None:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(build_rl_analysis_markdown(report), encoding="utf-8")


def write_rl_analysis_csv(path: str | Path, summaries: list[DatalogSummary]) -> None:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "log_name",
        "frames",
        "duration_s",
        "lidar_valid",
        "h30_valid",
        "encoder_1_seen",
        "encoder_2_seen",
        "dt35_1_valid",
        "dt35_2_valid",
        "encoder_lidar_rms_xy_cm",
        "h30_lidar_yaw_delta_std_deg",
        "dt35_fusion_usable_rays",
        "dt35_floor_hit_suspect_rays",
        "dt35_gate_rejected_rays",
    ]
    with out.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for item in summaries:
            sensor = item.sensor_counts
            dt35 = item.dt35_residuals
            writer.writerow(
                {
                    "log_name": item.log_name,
                    "frames": item.frames,
                    "duration_s": item.duration_s,
                    "lidar_valid": sensor.lidar_valid,
                    "h30_valid": sensor.h30_valid,
                    "encoder_1_seen": sensor.encoder_1_seen,
                    "encoder_2_seen": sensor.encoder_2_seen,
                    "dt35_1_valid": sensor.dt35_1_valid,
                    "dt35_2_valid": sensor.dt35_2_valid,
                    "encoder_lidar_rms_xy_cm": item.encoder_vs_lidar_delta_cm.rms_xy,
                    "h30_lidar_yaw_delta_std_deg": item.h30_vs_lidar_yaw_delta_deg.std,
                    "dt35_fusion_usable_rays": dt35.get("fusion_usable_rays"),
                    "dt35_floor_hit_suspect_rays": dt35.get("floor_hit_suspect_rays"),
                    "dt35_gate_rejected_rays": dt35.get("residual_gate_rejected_rays"),
                }
            )


def build_rl_analysis_markdown(report: RLDataAnalysisReport) -> str:
    lines = [
        "# R1 RL Data Analysis",
        "",
        f"- created_at: {report.created_at}",
        f"- start mapping: {report.start_side} / {report.start_policy}",
        f"- output_dir: `{report.output_dir}`",
        "",
        "Assumptions:",
        "- Lidar XY/YAW are local to the startup pose; startup local (0,0,0) is mapped to the configured red/blue start square for field-model checks.",
        "- H30 yaw and DT35 distance are treated as trusted measurements, but DT35 samples are filtered when the ray likely hits floor, missing field walls, ignored racks, corners, or high-residual targets.",
        "- Current practice field is incomplete and uneven; DT35 model residuals are evidence for filtering/modeling, not automatic firmware calibration.",
        "",
        "## Aggregate",
    ]
    aggregate = report.aggregate
    for key in (
        "logs",
        "frames",
        "lidar_valid_frames",
        "h30_valid_frames",
        "dt35_valid_rays",
        "dt35_fusion_usable_rays",
        "dt35_floor_hit_suspect_rays",
        "dt35_residual_gate_rejected_rays",
    ):
        lines.append(f"- {key}: {aggregate.get(key)}")
    lines.extend(
        [
            f"- encoder_lidar_rms_xy_cm_mean: {_fmt(aggregate.get('encoder_lidar_rms_xy_cm_mean'))}",
            f"- h30_lidar_yaw_delta_std_deg_mean: {_fmt(aggregate.get('h30_lidar_yaw_delta_std_deg_mean'))}",
            "",
            "## Calibration Recommendation",
            "",
        ]
    )
    recommendations = aggregate.get("calibration_recommendations", {})
    lines.extend(_recommendation_lines(recommendations))
    lines.extend(
        [
            "",
            "## Per Log",
            "",
        ]
    )
    for item in report.summaries:
        sensor = item.sensor_counts
        dt35 = item.dt35_residuals
        lines.extend(
            [
                f"### {item.log_name}",
                "",
                f"- frames/duration: {item.frames} / {_fmt(item.duration_s)} s",
                f"- lidar/h30 valid frames: {sensor.lidar_valid} / {sensor.h30_valid}",
                f"- encoder wheel pulse frames: {sensor.encoder_1_seen} / {sensor.encoder_2_seen}",
                f"- DT35 valid frames: sensor1={sensor.dt35_1_valid}, sensor2={sensor.dt35_2_valid}",
                f"- encoder vs lidar delta RMS: {_fmt(item.encoder_vs_lidar_delta_cm.rms_xy)} cm; max: {_fmt(item.encoder_vs_lidar_delta_cm.max_xy)} cm",
                f"- H30 vs lidar yaw-delta std: {_fmt(item.h30_vs_lidar_yaw_delta_deg.std)} deg; rms: {_fmt(item.h30_vs_lidar_yaw_delta_deg.rms)} deg",
                f"- H30 initial bias h30-lidar: mean={_fmt(item.h30_initial_bias_deg.mean)} deg; std={_fmt(item.h30_initial_bias_deg.std)} deg",
                f"- DT35 rays valid/usable/fusion usable: {dt35.get('valid_rays')} / {dt35.get('usable_rays')} / {dt35.get('fusion_usable_rays')}",
                f"- DT35 floor-or-near-hit suspect rays: {dt35.get('floor_hit_suspect_rays')}",
                f"- DT35 high-residual rejected rays: {dt35.get('residual_gate_rejected_rays')}",
                f"- DT35 target types: `{json.dumps(dt35.get('target_type_counts', {}), ensure_ascii=False, sort_keys=True)}`",
            ]
        )
        if item.encoder_scale_estimate:
            lines.append(
                f"- encoder scale estimate lidar/encoder: x={_fmt(item.encoder_scale_estimate.get('x'))}, "
                f"y={_fmt(item.encoder_scale_estimate.get('y'))}"
            )
        lines.append(
            f"- encoder window scale lidar/encoder: x={_fmt(item.encoder_window_scale.median_x)} "
            f"(n={item.encoder_window_scale.count_x}), y={_fmt(item.encoder_window_scale.median_y)} "
            f"(n={item.encoder_window_scale.count_y}), vector={_fmt(item.encoder_window_scale.median_vector)} "
            f"(n={item.encoder_window_scale.count_vector}, std={_fmt(item.encoder_window_scale.std_vector)})"
        )
        if item.notes:
            lines.append("- notes:")
            lines.extend(f"  - {note}" for note in item.notes)
        lines.append("")
    lines.extend(
        [
            "## Interpretation Rules",
            "",
            "- `floor_hit_suspect` means measured DT35 distance is much shorter than the modeled first hit. On the current uneven floor this is likely floor/near-object hit and must not be used to tune wall offsets.",
            "- `no_hit` or `ignore` means the field model says this ray is not a reliable correction target, usually because the real practice field lacks that wall or has racks/poles/irregular objects.",
            "- Range-bias suggestions are only trustworthy when enough fusion-usable rows remain and residual standard deviation is small.",
            "",
        ]
    )
    return "\n".join(lines)


def _summarize_log(
    config: dict[str, Any],
    csv_path: Path,
    frames: list[RobotFrame],
    residual_rows: list[DT35FrameResidualRow],
    *,
    start_side: str,
    start_policy: str,
    range_bias_min_frames: int,
) -> DatalogSummary:
    display_frames = [apply_display_policy(config, frame, start_side, start_policy) for frame in frames]
    residual_summary = summarize_residuals(residual_rows)
    estimates, _range_summary = estimate_dt35_range_bias(
        config,
        frames,
        start_side=start_side,
        start_policy=start_policy,
        pose_source="lidar",
        yaw_source="h30",
        min_frames=range_bias_min_frames,
    )
    sensor_counts = DatalogSensorCounts(
        frames=len(frames),
        lidar_valid=sum(1 for frame in frames if frame.lidar_valid or frame.lidar_online),
        h30_valid=sum(1 for frame in frames if frame.h30_valid or frame.h30_has_attitude),
        encoder_1_seen=sum(1 for frame in frames if frame.x_pulse_seen or bool(frame.status & (1 << 10))),
        encoder_2_seen=sum(1 for frame in frames if frame.y_pulse_seen or bool(frame.status & (1 << 11))),
        dt35_1_valid=sum(1 for frame in frames if frame.dt35_1_valid),
        dt35_2_valid=sum(1 for frame in frames if frame.dt35_2_valid),
    )
    lidar_frames = [frame for frame in frames if frame.lidar_valid or frame.lidar_online]
    notes = _notes(frames, residual_rows, sensor_counts)
    return DatalogSummary(
        log_name=csv_path.parent.parent.name,
        input_csv=str(csv_path),
        frames=len(frames),
        duration_s=_duration_s(frames),
        protocols=dict(sorted(Counter(frame.protocol for frame in frames).items())),
        sensor_counts=sensor_counts,
        local_lidar_range={
            "x_cm": _range([frame.lidar_x_cm for frame in lidar_frames]),
            "y_cm": _range([frame.lidar_y_cm for frame in lidar_frames]),
            "yaw_deg": _range([frame.lidar_yaw_deg for frame in lidar_frames]),
        },
        map_lidar_range={
            "x_cm": _range([frame.lidar_x_cm for frame in display_frames if frame.lidar_valid or frame.lidar_online]),
            "y_cm": _range([frame.lidar_y_cm for frame in display_frames if frame.lidar_valid or frame.lidar_online]),
            "yaw_deg": _range([frame.lidar_yaw_deg for frame in display_frames if frame.lidar_valid or frame.lidar_online]),
        },
        encoder_vs_lidar_delta_cm=_encoder_lidar_delta_error(lidar_frames),
        h30_vs_lidar_yaw_delta_deg=_h30_lidar_yaw_delta(lidar_frames),
        h30_initial_bias_deg=_h30_initial_bias(lidar_frames),
        encoder_scale_estimate=_encoder_scale_estimate(lidar_frames),
        encoder_window_scale=_encoder_window_scale(lidar_frames),
        dt35_residuals=residual_summary.to_dict(),
        dt35_by_sensor=_dt35_sensor_summaries(residual_rows),
        dt35_range_bias=[item.to_dict() for item in estimates],
        notes=notes,
    )


def _dt35_sensor_summaries(rows: list[DT35FrameResidualRow]) -> list[DatalogDT35SensorSummary]:
    out: list[DatalogDT35SensorSummary] = []
    for sensor_key in ("sensor_1", "sensor_2"):
        sensor_rows = [row for row in rows if row.sensor_key == sensor_key]
        usable_residuals = [row.residual_cm for row in sensor_rows if row.usable_for_fusion and isfinite(row.residual_cm)]
        target_types = Counter(row.expected_target_type or "no_hit" for row in sensor_rows)
        targets = Counter(row.expected_target or "no_hit" for row in sensor_rows)
        out.append(
            DatalogDT35SensorSummary(
                sensor_key=sensor_key,
                valid_rays=sum(1 for row in sensor_rows if row.sensor_valid),
                usable_rays=sum(1 for row in sensor_rows if row.usable_for_correction),
                fusion_usable_rays=sum(1 for row in sensor_rows if row.usable_for_fusion),
                floor_hit_suspect_rays=sum(1 for row in sensor_rows if row.floor_hit_suspect),
                residual_gate_rejected_rays=sum(
                    1 for row in sensor_rows if row.usable_for_correction and not row.residual_within_gate
                ),
                ignored_rays=sum(1 for row in sensor_rows if row.expected_target_type == "ignore"),
                no_hit_rays=sum(1 for row in sensor_rows if not row.expected_target),
                corner_ambiguous_rays=sum(1 for row in sensor_rows if row.corner_ambiguous),
                grazing_filtered_rays=sum(1 for row in sensor_rows if _is_grazing(row)),
                mean_residual_cm=_mean(usable_residuals),
                rms_residual_cm=_rms(usable_residuals),
                max_abs_residual_cm=max((abs(item) for item in usable_residuals), default=None),
                target_type_counts=dict(sorted(target_types.items())),
                target_counts=dict(sorted(targets.items())),
            )
        )
    return out


def _encoder_lidar_delta_error(frames: list[RobotFrame]) -> AxisErrorStats:
    if len(frames) < 2:
        return AxisErrorStats(0, None, None, None, None, None, None)
    first = frames[0]
    x_errors: list[float] = []
    y_errors: list[float] = []
    xy_errors: list[float] = []
    for frame in frames:
        lidar_dx = frame.lidar_x_cm - first.lidar_x_cm
        lidar_dy = frame.lidar_y_cm - first.lidar_y_cm
        encoder_dx = frame.encoder_x_cm - first.encoder_x_cm
        encoder_dy = frame.encoder_y_cm - first.encoder_y_cm
        ex = encoder_dx - lidar_dx
        ey = encoder_dy - lidar_dy
        x_errors.append(ex)
        y_errors.append(ey)
        xy_errors.append(hypot(ex, ey))
    return AxisErrorStats(
        count=len(xy_errors),
        mean_x=_mean(x_errors),
        mean_y=_mean(y_errors),
        std_x=_std(x_errors),
        std_y=_std(y_errors),
        rms_xy=_rms(xy_errors),
        max_xy=max(xy_errors, default=None),
    )


def _h30_lidar_yaw_delta(frames: list[RobotFrame]) -> SeriesStats:
    usable = [frame for frame in frames if frame.h30_valid or frame.h30_has_attitude]
    if len(usable) < 2:
        return SeriesStats(0, None, None, None, None, None)
    lidar = _unwrap_angles([frame.lidar_yaw_deg for frame in usable])
    h30 = _unwrap_angles([frame.h30_yaw_deg for frame in usable])
    errors = [(h30[i] - h30[0]) - (lidar[i] - lidar[0]) for i in range(len(usable))]
    return _series(errors)


def _h30_initial_bias(frames: list[RobotFrame], window_s: float = 2.0, max_frames: int = 30) -> SeriesStats:
    usable = [frame for frame in frames if (frame.h30_valid or frame.h30_has_attitude) and (frame.lidar_valid or frame.lidar_online)]
    if not usable:
        return SeriesStats(0, None, None, None, None, None)
    first_time = usable[0].pc_time
    selected: list[RobotFrame] = []
    for frame in usable:
        if first_time > 0.0 and frame.pc_time > 0.0:
            if frame.pc_time - first_time > window_s:
                break
        elif len(selected) >= max_frames:
            break
        selected.append(frame)
        if len(selected) >= max_frames:
            break
    errors = [wrap_deg(frame.h30_yaw_deg - frame.lidar_yaw_deg) for frame in selected]
    return _series(errors)


def _encoder_scale_estimate(frames: list[RobotFrame]) -> dict[str, float | None]:
    if len(frames) < 2:
        return {"x": None, "y": None}
    first = frames[0]
    encoder_dx = [frame.encoder_x_cm - first.encoder_x_cm for frame in frames]
    encoder_dy = [frame.encoder_y_cm - first.encoder_y_cm for frame in frames]
    lidar_dx = [frame.lidar_x_cm - first.lidar_x_cm for frame in frames]
    lidar_dy = [frame.lidar_y_cm - first.lidar_y_cm for frame in frames]
    return {
        "x": _least_squares_scale(encoder_dx, lidar_dx),
        "y": _least_squares_scale(encoder_dy, lidar_dy),
    }


def _encoder_window_scale(
    frames: list[RobotFrame],
    *,
    window_frames: int = 5,
    min_axis_delta_cm: float = 5.0,
    min_vector_delta_cm: float = 8.0,
) -> EncoderWindowScaleStats:
    if len(frames) <= window_frames:
        return EncoderWindowScaleStats(window_frames, 0, 0, 0, None, None, None, None)
    x_scales: list[float] = []
    y_scales: list[float] = []
    vector_scales: list[float] = []
    for index in range(window_frames, len(frames)):
        a = frames[index - window_frames]
        b = frames[index]
        edx = b.encoder_x_cm - a.encoder_x_cm
        edy = b.encoder_y_cm - a.encoder_y_cm
        ldx = b.lidar_x_cm - a.lidar_x_cm
        ldy = b.lidar_y_cm - a.lidar_y_cm
        if abs(edx) >= min_axis_delta_cm and abs(ldx) >= min_axis_delta_cm * 0.5:
            _append_plausible_scale(x_scales, ldx / edx)
        if abs(edy) >= min_axis_delta_cm and abs(ldy) >= min_axis_delta_cm * 0.5:
            _append_plausible_scale(y_scales, ldy / edy)
        encoder_dist = hypot(edx, edy)
        lidar_dist = hypot(ldx, ldy)
        if encoder_dist >= min_vector_delta_cm and lidar_dist >= min_vector_delta_cm * 0.5:
            _append_plausible_scale(vector_scales, lidar_dist / encoder_dist)
    return EncoderWindowScaleStats(
        window_frames=window_frames,
        count_x=len(x_scales),
        count_y=len(y_scales),
        count_vector=len(vector_scales),
        median_x=_median_or_none(x_scales),
        median_y=_median_or_none(y_scales),
        median_vector=_median_or_none(vector_scales),
        std_vector=_std(vector_scales),
    )


def _append_plausible_scale(out: list[float], value: float) -> None:
    if isfinite(value) and 0.5 <= value <= 1.5:
        out.append(value)


def _least_squares_scale(source: list[float], target: list[float]) -> float | None:
    source_span = max(source, default=0.0) - min(source, default=0.0)
    target_span = max(target, default=0.0) - min(target, default=0.0)
    if source_span < 20.0 or target_span < 20.0:
        return None
    denom = sum(value * value for value in source)
    if denom <= 1.0e-9:
        return None
    scale = sum(s * t for s, t in zip(source, target)) / denom
    if not isfinite(scale) or scale <= 0.0 or scale < 0.5 or scale > 1.5:
        return None
    return scale


def _notes(frames: list[RobotFrame], rows: list[DT35FrameResidualRow], sensor: DatalogSensorCounts) -> list[str]:
    notes: list[str] = []
    if not frames:
        return ["No frames parsed."]
    if sensor.lidar_valid < max(5, int(len(frames) * 0.2)):
        notes.append("Too few lidar-valid frames; cannot use lidar as truth for this run.")
    if sensor.h30_valid < max(5, int(len(frames) * 0.2)):
        notes.append("Too few H30-valid frames; yaw-based DT35 modeling is weak.")
    if sensor.dt35_1_valid == 0 or sensor.dt35_2_valid == 0:
        notes.append("At least one DT35 sensor has no valid frames.")
    floor_hits = sum(1 for row in rows if row.floor_hit_suspect)
    if floor_hits:
        notes.append(f"{floor_hits} DT35 rays look like floor/near-object hits and are excluded from correction.")
    no_hit = sum(1 for row in rows if row.sensor_valid and not row.expected_target)
    if no_hit:
        notes.append(f"{no_hit} valid DT35 rays have no modeled target; likely missing practice-field walls or out-of-map targets.")
    ignored = sum(1 for row in rows if row.sensor_valid and row.expected_target_type == "ignore")
    if ignored:
        notes.append(f"{ignored} DT35 rays hit ignored geometry such as racks/poles and are excluded.")
    return notes


def _aggregate(summaries: list[DatalogSummary]) -> dict[str, Any]:
    aggregate = {
        "logs": len(summaries),
        "frames": sum(item.frames for item in summaries),
        "lidar_valid_frames": sum(item.sensor_counts.lidar_valid for item in summaries),
        "h30_valid_frames": sum(item.sensor_counts.h30_valid for item in summaries),
        "dt35_valid_rays": sum(int(item.dt35_residuals.get("valid_rays") or 0) for item in summaries),
        "dt35_fusion_usable_rays": sum(int(item.dt35_residuals.get("fusion_usable_rays") or 0) for item in summaries),
        "dt35_floor_hit_suspect_rays": sum(int(item.dt35_residuals.get("floor_hit_suspect_rays") or 0) for item in summaries),
        "dt35_residual_gate_rejected_rays": sum(
            int(item.dt35_residuals.get("residual_gate_rejected_rays") or 0) for item in summaries
        ),
        "encoder_lidar_rms_xy_cm_mean": _mean(
            [
                item.encoder_vs_lidar_delta_cm.rms_xy
                for item in summaries
                if item.encoder_vs_lidar_delta_cm.rms_xy is not None
            ]
        ),
        "h30_lidar_yaw_delta_std_deg_mean": _mean(
            [
                item.h30_vs_lidar_yaw_delta_deg.std
                for item in summaries
                if item.h30_vs_lidar_yaw_delta_deg.std is not None
            ]
        ),
    }
    aggregate["calibration_recommendations"] = _calibration_recommendations(summaries)
    return aggregate


def _calibration_recommendations(summaries: list[DatalogSummary]) -> dict[str, Any]:
    stable_h30_biases = [
        float(item.h30_initial_bias_deg.mean)
        for item in summaries
        if item.h30_initial_bias_deg.mean is not None
        and (item.h30_initial_bias_deg.std is None or item.h30_initial_bias_deg.std <= 0.5)
        and item.h30_initial_bias_deg.count >= 3
    ]
    h30_bias = _median_or_none(stable_h30_biases)
    h30_delta = -h30_bias if h30_bias is not None else None
    vector_scales = [
        float(item.encoder_window_scale.median_vector)
        for item in summaries
        if item.encoder_window_scale.median_vector is not None
        and item.encoder_window_scale.count_vector >= 20
    ]
    x_scales = [
        float(item.encoder_window_scale.median_x)
        for item in summaries
        if item.encoder_window_scale.median_x is not None
        and item.encoder_window_scale.count_x >= 10
    ]
    y_scales = [
        float(item.encoder_window_scale.median_y)
        for item in summaries
        if item.encoder_window_scale.median_y is not None
        and item.encoder_window_scale.count_y >= 10
    ]
    encoder_vector = _median_or_none(vector_scales)
    encoder_x = _median_or_none(x_scales)
    encoder_y = _median_or_none(y_scales)
    return {
        "h30": {
            "stable_log_count": len(stable_h30_biases),
            "initial_bias_deg_median": h30_bias,
            "suggested_additional_yaw_offset_deg": h30_delta,
            "apply_to_firmware": bool(h30_delta is not None and abs(h30_delta) >= 0.2 and len(stable_h30_biases) >= 2),
            "note": "Add suggested_additional_yaw_offset_deg to LOCATER_H30_YAW_OFFSET_DEG only after confirming the robot was north-facing during startup.",
        },
        "encoder": {
            "window_vector_scale_median": encoder_vector,
            "window_x_scale_median": encoder_x,
            "window_y_scale_median": encoder_y,
            "apply_to_firmware": False,
            "note": "Current practice logs include rotation, missing walls, and possible slip; use window scales as diagnostics only until a straight-line calibration run is captured.",
        },
        "dt35": {
            "range_bias_apply_to_firmware": False,
            "note": "DT35 official scale/offset remains unchanged. Use residual gates and field-target filtering before considering software distance_bias_mm.",
        },
    }


def _recommendation_lines(recommendations: dict[str, Any]) -> list[str]:
    if not recommendations:
        return ["- No calibration recommendations were generated."]
    h30 = recommendations.get("h30", {})
    encoder = recommendations.get("encoder", {})
    dt35 = recommendations.get("dt35", {})
    return [
        f"- H30 startup bias median: {_fmt(h30.get('initial_bias_deg_median'))} deg; "
        f"suggested firmware offset delta: {_fmt(h30.get('suggested_additional_yaw_offset_deg'))} deg; "
        f"apply: {bool(h30.get('apply_to_firmware'))}.",
        f"  - {h30.get('note', '')}",
        f"- Encoder window scale median lidar/encoder: vector={_fmt(encoder.get('window_vector_scale_median'))}, "
        f"x={_fmt(encoder.get('window_x_scale_median'))}, y={_fmt(encoder.get('window_y_scale_median'))}; "
        f"apply: {bool(encoder.get('apply_to_firmware'))}.",
        f"  - {encoder.get('note', '')}",
        f"- DT35 firmware scale/offset apply: {bool(dt35.get('range_bias_apply_to_firmware'))}.",
        f"  - {dt35.get('note', '')}",
    ]


def _resolve_log_csv(path: str | Path) -> Path:
    item = Path(path)
    if item.is_file():
        return item
    for candidate in (
        item / "sensor_data" / "raw_frames.csv",
        item / "raw_frames.csv",
        item / "sensor_data" / "parsed_frames.csv",
        item / "parsed_frames.csv",
    ):
        if candidate.exists():
            return candidate
    raise FileNotFoundError(f"Cannot find raw_frames.csv under {item}")


def _resolve_output_dir(config: dict[str, Any], output_dir: str | Path | None) -> Path:
    if output_dir:
        return Path(output_dir)
    root = Path(config.get("_project_root", Path.cwd()))
    return root / "logs" / "RL_data" / f"{datetime.now().strftime('%Y%m%d_%H%M%S')}_analysis"


def _write_per_log_dt35_csv(path: Path, rows: list[DT35FrameResidualRow]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(DT35FrameResidualRow.__dataclass_fields__.keys())
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(asdict(row))


def _duration_s(frames: list[RobotFrame]) -> float | None:
    times = [float(frame.pc_time) for frame in frames if frame.pc_time > 0.0]
    if len(times) < 2:
        return None
    return max(times) - min(times)


def _range(values: list[float]) -> dict[str, float | None]:
    values = [float(value) for value in values if isfinite(float(value))]
    if not values:
        return {"min": None, "max": None, "span": None}
    return {"min": min(values), "max": max(values), "span": max(values) - min(values)}


def _series(values: list[float]) -> SeriesStats:
    return SeriesStats(
        count=len(values),
        mean=_mean(values),
        std=_std(values),
        rms=_rms(values),
        min=min(values) if values else None,
        max=max(values) if values else None,
    )


def _mean(values: list[float | None]) -> float | None:
    filtered = [float(value) for value in values if value is not None and isfinite(float(value))]
    if not filtered:
        return None
    return sum(filtered) / len(filtered)


def _std(values: list[float]) -> float | None:
    if len(values) < 2:
        return None
    mean = sum(values) / len(values)
    return sqrt(sum((value - mean) * (value - mean) for value in values) / (len(values) - 1))


def _rms(values: list[float]) -> float | None:
    if not values:
        return None
    return sqrt(sum(value * value for value in values) / len(values))


def _median_or_none(values: list[float]) -> float | None:
    filtered = [float(value) for value in values if isfinite(float(value))]
    if not filtered:
        return None
    return float(median(filtered))


def _unwrap_angles(values: list[float]) -> list[float]:
    if not values:
        return []
    out = [values[0]]
    for value in values[1:]:
        out.append(out[-1] + wrap_deg(value - out[-1]))
    return out


def _is_grazing(row: DT35FrameResidualRow) -> bool:
    return (
        row.expected_target_type in ("usable_wall", "solid_obstacle")
        and row.within_range
        and not row.correction_allowed
    )


def _fmt(value: Any, digits: int = 3) -> str:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return "-"
    if not isfinite(number):
        return "-"
    return f"{number:.{digits}f}"


def _json_safe(value: Any) -> Any:
    if isinstance(value, float):
        return value if isfinite(value) else None
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_json_safe(item) for item in value]
    return value
