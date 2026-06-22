from __future__ import annotations

import copy
import csv
import json
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from typing import Any

from .fusion_model import FusionConfig
from .path_diagnostics import PathDiagnosticRow, generate_path_diagnostic
from .synthetic_sim import SyntheticConfig, generate_synthetic_frames


DEFAULT_OBSTACLE_PATHS = ("forest_side", "ramp_side", "field_patrol")


@dataclass(slots=True)
class ObstacleAblationRow:
    path: str
    frames: int
    full_raw_rms_xy_cm: float | None
    full_fused_rms_xy_cm: float | None
    ablated_fused_rms_xy_cm: float | None
    full_improvement_cm: float | None
    ablation_penalty_cm: float | None
    full_dt35_fusion_allowed_frames: int
    ablated_dt35_fusion_allowed_frames: int
    full_solid_fusion_allowed_rays: int
    ablated_solid_fusion_allowed_rays: int
    full_forest_fusion_allowed_rays: int
    full_ramp_fusion_allowed_rays: int
    full_target_type_counts_json: str
    ablated_target_type_counts_json: str
    solid_obstacle_contributed: bool
    ablation_passed: bool

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True)
class ObstacleAblationSummary:
    paths: int
    passed_paths: int
    failed_paths: int
    total_full_solid_fusion_allowed_rays: int
    total_full_forest_fusion_allowed_rays: int
    total_full_ramp_fusion_allowed_rays: int
    mean_full_fused_rms_xy_cm: float | None
    mean_ablated_fused_rms_xy_cm: float | None
    mean_ablation_penalty_cm: float | None
    path_rows: list[dict[str, Any]]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def run_obstacle_ablation(
    config: dict[str, Any],
    paths: list[str] | tuple[str, ...] = DEFAULT_OBSTACLE_PATHS,
    *,
    samples: int = 180,
    encoder_x_scale: float = 0.97,
    encoder_y_scale: float = 1.03,
    encoder_yaw_scale: float = 1.0,
    h30_yaw_bias_deg: float = 0.0,
    dt35_noise_mm: float = 5.0,
    seed: int = 31,
    fusion_cfg: FusionConfig | None = None,
) -> tuple[list[ObstacleAblationRow], ObstacleAblationSummary]:
    fusion = fusion_cfg or FusionConfig(lidar_stride=25, lidar_gain=1.0, dt35_gain=1.0, dt35_yaw_gain=0.0)
    ablated_config = _disable_solid_obstacle_correction(config)
    rows: list[ObstacleAblationRow] = []
    for index, path_name in enumerate(paths):
        synthetic_cfg = SyntheticConfig(
            samples=max(2, int(samples)),
            path_name=path_name,
            encoder_x_scale=encoder_x_scale,
            encoder_y_scale=encoder_y_scale,
            encoder_yaw_scale=encoder_yaw_scale,
            h30_yaw_bias_deg=h30_yaw_bias_deg,
            dt35_noise_mm=dt35_noise_mm,
            seed=seed + index,
        )
        truth_frames = generate_synthetic_frames(config, synthetic_cfg)
        firmware_like_frames = [
            replace(frame, pos_x_cm=frame.encoder_x_cm, pos_y_cm=frame.encoder_y_cm, pos_yaw_deg=frame.h30_yaw_deg)
            for frame in truth_frames
        ]
        full_rows, full_summary, _full_fused = generate_path_diagnostic(config, firmware_like_frames, fusion, start_policy="off")
        ablated_rows, ablated_summary, _ablated_fused = generate_path_diagnostic(
            ablated_config,
            firmware_like_frames,
            fusion,
            start_policy="off",
        )
        full_solid = _fusion_allowed_type_count(full_rows, "solid_obstacle")
        ablated_solid = _fusion_allowed_type_count(ablated_rows, "solid_obstacle")
        forest = _fusion_allowed_target_name_count(full_rows, "forest")
        ramp = _fusion_allowed_target_name_count(full_rows, "ramp")
        penalty = _diff(ablated_summary.fused_rms_xy_cm, full_summary.fused_rms_xy_cm)
        contributed = full_solid > 0 and (forest > 0 or ramp > 0)
        rows.append(
            ObstacleAblationRow(
                path=path_name,
                frames=full_summary.frames,
                full_raw_rms_xy_cm=full_summary.raw_rms_xy_cm,
                full_fused_rms_xy_cm=full_summary.fused_rms_xy_cm,
                ablated_fused_rms_xy_cm=ablated_summary.fused_rms_xy_cm,
                full_improvement_cm=_diff(full_summary.raw_rms_xy_cm, full_summary.fused_rms_xy_cm),
                ablation_penalty_cm=penalty,
                full_dt35_fusion_allowed_frames=full_summary.dt35_fusion_allowed_frames,
                ablated_dt35_fusion_allowed_frames=ablated_summary.dt35_fusion_allowed_frames,
                full_solid_fusion_allowed_rays=full_solid,
                ablated_solid_fusion_allowed_rays=ablated_solid,
                full_forest_fusion_allowed_rays=forest,
                full_ramp_fusion_allowed_rays=ramp,
                full_target_type_counts_json=json.dumps(full_summary.dt35_type_counts, ensure_ascii=False, sort_keys=True),
                ablated_target_type_counts_json=json.dumps(ablated_summary.dt35_type_counts, ensure_ascii=False, sort_keys=True),
                solid_obstacle_contributed=contributed,
                ablation_passed=contributed and ablated_solid == 0 and (penalty is None or penalty >= -0.5),
            )
        )
    return rows, _summary(rows)


def write_obstacle_ablation_csv(path: str | Path, rows: list[ObstacleAblationRow]) -> None:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(ObstacleAblationRow.__dataclass_fields__.keys())
    with out.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row.to_dict())


def write_obstacle_ablation_summary(path: str | Path, summary: ObstacleAblationSummary) -> None:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(summary.to_dict(), ensure_ascii=False, indent=2), encoding="utf-8")


def _disable_solid_obstacle_correction(config: dict[str, Any]) -> dict[str, Any]:
    out = copy.deepcopy(config)
    model = out.get("field_model", {})
    for item in model.get("segments", []):
        if str(item.get("target_type", "")) == "solid_obstacle":
            item["correction_weight"] = 0.0
    for item in model.get("rectangles", []):
        if str(item.get("target_type", "")) == "solid_obstacle":
            item["correction_weight"] = 0.0
    return out


def _fusion_allowed_type_count(rows: list[PathDiagnosticRow], target_type: str) -> int:
    count = 0
    for row in rows:
        if row.dt35_1_fusion_allowed and row.dt35_1_type == target_type:
            count += 1
        if row.dt35_2_fusion_allowed and row.dt35_2_type == target_type:
            count += 1
    return count


def _fusion_allowed_target_name_count(rows: list[PathDiagnosticRow], name_part: str) -> int:
    part = name_part.lower()
    count = 0
    for row in rows:
        if row.dt35_1_fusion_allowed and part in row.dt35_1_target.lower():
            count += 1
        if row.dt35_2_fusion_allowed and part in row.dt35_2_target.lower():
            count += 1
    return count


def _diff(a: float | None, b: float | None) -> float | None:
    if a is None or b is None:
        return None
    return a - b


def _mean(values: list[float]) -> float | None:
    return sum(values) / len(values) if values else None


def _summary(rows: list[ObstacleAblationRow]) -> ObstacleAblationSummary:
    full = [row.full_fused_rms_xy_cm for row in rows if row.full_fused_rms_xy_cm is not None]
    ablated = [row.ablated_fused_rms_xy_cm for row in rows if row.ablated_fused_rms_xy_cm is not None]
    penalties = [row.ablation_penalty_cm for row in rows if row.ablation_penalty_cm is not None]
    passed = sum(1 for row in rows if row.ablation_passed)
    return ObstacleAblationSummary(
        paths=len(rows),
        passed_paths=passed,
        failed_paths=len(rows) - passed,
        total_full_solid_fusion_allowed_rays=sum(row.full_solid_fusion_allowed_rays for row in rows),
        total_full_forest_fusion_allowed_rays=sum(row.full_forest_fusion_allowed_rays for row in rows),
        total_full_ramp_fusion_allowed_rays=sum(row.full_ramp_fusion_allowed_rays for row in rows),
        mean_full_fused_rms_xy_cm=_mean(full),
        mean_ablated_fused_rms_xy_cm=_mean(ablated),
        mean_ablation_penalty_cm=_mean(penalties),
        path_rows=[row.to_dict() for row in rows],
    )
