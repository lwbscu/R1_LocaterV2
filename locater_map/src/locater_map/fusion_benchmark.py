from __future__ import annotations

import csv
import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

from .fusion_model import FusionConfig
from .path_diagnostics import generate_synthetic_path_diagnostic
from .synthetic_sim import SyntheticConfig


DEFAULT_BENCHMARK_PATHS = (
    "top_corridor",
    "forest_side",
    "ramp_side",
    "center_divider",
    "start_corner_yaw_sweep",
    "field_patrol",
)

STATIC_RAW_RMS_THRESHOLD_CM = 1.0
STATIC_FUSED_RMS_LIMIT_CM = 1.0


@dataclass(slots=True)
class BenchmarkPathRow:
    path: str
    frames: int
    raw_rms_xy_cm: float | None
    fused_rms_xy_cm: float | None
    raw_max_xy_cm: float | None
    fused_max_xy_cm: float | None
    improvement_rms_cm: float | None
    improved_frames: int
    worsened_frames: int
    dt35_valid_frames: int
    dt35_allowed_frames: int
    dt35_fusion_allowed_frames: int
    dt35_gate_rejected_frames: int
    dt35_corner_frames: int
    dt35_rank_counts_json: str
    dt35_constraint_state_counts_json: str
    dt35_type_counts_json: str
    validation_passed: bool
    benchmark_passed: bool
    benchmark_reason: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True)
class BenchmarkSummary:
    paths: int
    passed_paths: int
    failed_paths: int
    mean_raw_rms_xy_cm: float | None
    mean_fused_rms_xy_cm: float | None
    mean_improvement_rms_cm: float | None
    max_fused_rms_xy_cm: float | None
    total_dt35_valid_frames: int
    total_dt35_fusion_allowed_frames: int
    total_dt35_gate_rejected_frames: int
    path_rows: list[dict[str, Any]]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def run_synthetic_benchmark(
    config: dict[str, Any],
    paths: list[str] | tuple[str, ...] = DEFAULT_BENCHMARK_PATHS,
    *,
    samples: int = 240,
    encoder_x_scale: float = 0.97,
    encoder_y_scale: float = 1.03,
    encoder_yaw_scale: float = 1.0,
    h30_yaw_bias_deg: float = 0.0,
    dt35_noise_mm: float = 5.0,
    seed: int = 7,
    fusion_cfg: FusionConfig | None = None,
) -> tuple[list[BenchmarkPathRow], BenchmarkSummary]:
    fusion = fusion_cfg or FusionConfig(lidar_stride=25, lidar_gain=1.0, dt35_gain=1.0, dt35_yaw_gain=0.0)
    rows: list[BenchmarkPathRow] = []
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
        _detail_rows, path_summary, _fused = generate_synthetic_path_diagnostic(config, synthetic_cfg, fusion)
        improvement = _improvement(path_summary.raw_rms_xy_cm, path_summary.fused_rms_xy_cm)
        validation_passed = bool(path_summary.model_validation.get("gates", {}).get("passed", False))
        benchmark_passed, benchmark_reason = _benchmark_gate(path_summary.raw_rms_xy_cm, path_summary.fused_rms_xy_cm, validation_passed)
        rows.append(
            BenchmarkPathRow(
                path=path_name,
                frames=path_summary.frames,
                raw_rms_xy_cm=path_summary.raw_rms_xy_cm,
                fused_rms_xy_cm=path_summary.fused_rms_xy_cm,
                raw_max_xy_cm=path_summary.raw_max_xy_cm,
                fused_max_xy_cm=path_summary.fused_max_xy_cm,
                improvement_rms_cm=improvement,
                improved_frames=path_summary.improved_frames,
                worsened_frames=path_summary.worsened_frames,
                dt35_valid_frames=path_summary.dt35_valid_frames,
                dt35_allowed_frames=path_summary.dt35_allowed_frames,
                dt35_fusion_allowed_frames=path_summary.dt35_fusion_allowed_frames,
                dt35_gate_rejected_frames=path_summary.dt35_residual_gate_rejected_frames,
                dt35_corner_frames=path_summary.dt35_corner_frames,
                dt35_rank_counts_json=json.dumps(path_summary.dt35_rank_counts, ensure_ascii=False, sort_keys=True),
                dt35_constraint_state_counts_json=json.dumps(
                    path_summary.dt35_constraint_state_counts,
                    ensure_ascii=False,
                    sort_keys=True,
                ),
                dt35_type_counts_json=json.dumps(path_summary.dt35_type_counts, ensure_ascii=False, sort_keys=True),
                validation_passed=validation_passed,
                benchmark_passed=benchmark_passed,
                benchmark_reason=benchmark_reason,
            )
        )
    return rows, _summary(rows)


def write_benchmark_csv(path: str | Path, rows: list[BenchmarkPathRow]) -> None:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(BenchmarkPathRow.__dataclass_fields__.keys())
    with out.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row.to_dict())


def write_benchmark_summary(path: str | Path, summary: BenchmarkSummary) -> None:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(summary.to_dict(), ensure_ascii=False, indent=2), encoding="utf-8")


def _summary(rows: list[BenchmarkPathRow]) -> BenchmarkSummary:
    raw = [row.raw_rms_xy_cm for row in rows if row.raw_rms_xy_cm is not None]
    fused = [row.fused_rms_xy_cm for row in rows if row.fused_rms_xy_cm is not None]
    improvements = [row.improvement_rms_cm for row in rows if row.improvement_rms_cm is not None]
    passed = sum(1 for row in rows if row.benchmark_passed)
    return BenchmarkSummary(
        paths=len(rows),
        passed_paths=passed,
        failed_paths=len(rows) - passed,
        mean_raw_rms_xy_cm=_mean(raw),
        mean_fused_rms_xy_cm=_mean(fused),
        mean_improvement_rms_cm=_mean(improvements),
        max_fused_rms_xy_cm=max(fused, default=None),
        total_dt35_valid_frames=sum(row.dt35_valid_frames for row in rows),
        total_dt35_fusion_allowed_frames=sum(row.dt35_fusion_allowed_frames for row in rows),
        total_dt35_gate_rejected_frames=sum(row.dt35_gate_rejected_frames for row in rows),
        path_rows=[row.to_dict() for row in rows],
    )


def _improvement(raw: float | None, fused: float | None) -> float | None:
    if raw is None or fused is None:
        return None
    return raw - fused


def _benchmark_gate(raw_rms: float | None, fused_rms: float | None, validation_passed: bool) -> tuple[bool, str]:
    if validation_passed:
        return True, "moving_path_improved"
    if raw_rms is not None and fused_rms is not None:
        if raw_rms <= STATIC_RAW_RMS_THRESHOLD_CM and fused_rms <= STATIC_FUSED_RMS_LIMIT_CM:
            return True, "static_or_rotation_path_stable"
    return False, "failed_validation"


def _mean(values: list[float]) -> float | None:
    return sum(values) / len(values) if values else None
