from __future__ import annotations

import csv
import json
from copy import deepcopy
from dataclasses import asdict, dataclass
from datetime import datetime
from math import isfinite, sqrt
from pathlib import Path
from typing import Any

from .dt35_analysis import analyze_dt35_frames, summarize_residuals
from .fusion_model import load_frames_csv


@dataclass(slots=True)
class DT35MountVariant:
    name: str
    description: str
    sensor_1_offset_x_cm: float
    sensor_1_yaw_offset_deg: float
    sensor_2_offset_x_cm: float
    sensor_2_yaw_offset_deg: float

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True)
class DT35MountLogScore:
    log_name: str
    frames: int
    valid_rays: int
    usable_rays: int
    fusion_usable_rays: int
    floor_hit_suspect_rays: int
    residual_gate_rejected_rays: int
    rms_residual_cm: float | None
    mean_abs_residual_cm: float | None
    score: float

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True)
class DT35MountVariantScore:
    variant: DT35MountVariant
    logs: list[DT35MountLogScore]
    total_frames: int
    valid_rays: int
    usable_rays: int
    fusion_usable_rays: int
    floor_hit_suspect_rays: int
    residual_gate_rejected_rays: int
    rms_residual_cm: float | None
    mean_abs_residual_cm: float | None
    score: float

    def to_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["variant"] = self.variant.to_dict()
        payload["logs"] = [item.to_dict() for item in self.logs]
        return payload


@dataclass(slots=True)
class DT35MountHypothesisReport:
    created_at: str
    output_dir: str
    start_side: str
    start_policy: str
    recommended_variant: str | None
    scores: list[DT35MountVariantScore]
    notes: list[str]

    def to_dict(self) -> dict[str, Any]:
        return _json_safe(
            {
                "created_at": self.created_at,
                "output_dir": self.output_dir,
                "start_side": self.start_side,
                "start_policy": self.start_policy,
                "recommended_variant": self.recommended_variant,
                "scores": [item.to_dict() for item in self.scores],
                "notes": self.notes,
            }
        )


DEFAULT_VARIANTS = (
    DT35MountVariant(
        name="configured",
        description="Use dt35.sensor_1/sensor_2 mounting from config/default_config.json.",
        sensor_1_offset_x_cm=float("nan"),
        sensor_1_yaw_offset_deg=float("nan"),
        sensor_2_offset_x_cm=float("nan"),
        sensor_2_yaw_offset_deg=float("nan"),
    ),
    DT35MountVariant(
        name="outward_data_supported",
        description="ID1 on local -X/left side shoots -X; ID2 on local +X/right side shoots +X.",
        sensor_1_offset_x_cm=-40.4,
        sensor_1_yaw_offset_deg=-90.0,
        sensor_2_offset_x_cm=40.4,
        sensor_2_yaw_offset_deg=90.0,
    ),
    DT35MountVariant(
        name="cross_user_stated",
        description="ID1 on local +X/right side shoots -X; ID2 on local -X/left side shoots +X.",
        sensor_1_offset_x_cm=40.4,
        sensor_1_yaw_offset_deg=-90.0,
        sensor_2_offset_x_cm=-40.4,
        sensor_2_yaw_offset_deg=90.0,
    ),
    DT35MountVariant(
        name="cross_swapped_ids",
        description="ID1 on local -X/left side shoots +X; ID2 on local +X/right side shoots -X.",
        sensor_1_offset_x_cm=-40.4,
        sensor_1_yaw_offset_deg=90.0,
        sensor_2_offset_x_cm=40.4,
        sensor_2_yaw_offset_deg=-90.0,
    ),
    DT35MountVariant(
        name="both_left",
        description="Both beams point local -X; useful for catching sign mistakes.",
        sensor_1_offset_x_cm=-40.4,
        sensor_1_yaw_offset_deg=-90.0,
        sensor_2_offset_x_cm=40.4,
        sensor_2_yaw_offset_deg=-90.0,
    ),
    DT35MountVariant(
        name="both_right",
        description="Both beams point local +X; useful for catching sign mistakes.",
        sensor_1_offset_x_cm=-40.4,
        sensor_1_yaw_offset_deg=90.0,
        sensor_2_offset_x_cm=40.4,
        sensor_2_yaw_offset_deg=90.0,
    ),
)


def analyze_dt35_mount_hypotheses(
    config: dict[str, Any],
    log_paths: list[str | Path],
    *,
    output_dir: str | Path | None = None,
    start_side: str = "red",
    start_policy: str = "always_local_display",
    variants: tuple[DT35MountVariant, ...] = DEFAULT_VARIANTS,
) -> DT35MountHypothesisReport:
    csv_paths = [_resolve_log_csv(path) for path in log_paths]
    out_dir = _resolve_output_dir(config, output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    scores: list[DT35MountVariantScore] = []
    for variant in variants:
        variant_config = _config_for_variant(config, variant)
        scores.append(_score_variant(variant_config, variant, csv_paths, start_side, start_policy))
    scores.sort(key=lambda item: item.score, reverse=True)

    recommended = scores[0].variant.name if scores else None
    report = DT35MountHypothesisReport(
        created_at=datetime.now().isoformat(timespec="seconds"),
        output_dir=str(out_dir),
        start_side=start_side,
        start_policy=start_policy,
        recommended_variant=recommended,
        scores=scores,
        notes=_build_notes(scores),
    )
    write_mount_hypothesis_json(out_dir / "dt35_mount_hypothesis.json", report)
    write_mount_hypothesis_csv(out_dir / "dt35_mount_hypothesis_summary.csv", report)
    write_mount_hypothesis_markdown(out_dir / "dt35_mount_hypothesis.md", report)
    return report


def write_mount_hypothesis_json(path: str | Path, report: DT35MountHypothesisReport) -> None:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report.to_dict(), ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")


def write_mount_hypothesis_csv(path: str | Path, report: DT35MountHypothesisReport) -> None:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "rank",
        "variant",
        "score",
        "fusion_usable_rays",
        "usable_rays",
        "valid_rays",
        "floor_hit_suspect_rays",
        "residual_gate_rejected_rays",
        "rms_residual_cm",
        "mean_abs_residual_cm",
        "sensor_1_offset_x_cm",
        "sensor_1_yaw_offset_deg",
        "sensor_2_offset_x_cm",
        "sensor_2_yaw_offset_deg",
        "description",
    ]
    with out.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for index, item in enumerate(report.scores, start=1):
            variant = item.variant
            writer.writerow(
                {
                    "rank": index,
                    "variant": variant.name,
                    "score": item.score,
                    "fusion_usable_rays": item.fusion_usable_rays,
                    "usable_rays": item.usable_rays,
                    "valid_rays": item.valid_rays,
                    "floor_hit_suspect_rays": item.floor_hit_suspect_rays,
                    "residual_gate_rejected_rays": item.residual_gate_rejected_rays,
                    "rms_residual_cm": item.rms_residual_cm,
                    "mean_abs_residual_cm": item.mean_abs_residual_cm,
                    "sensor_1_offset_x_cm": variant.sensor_1_offset_x_cm,
                    "sensor_1_yaw_offset_deg": variant.sensor_1_yaw_offset_deg,
                    "sensor_2_offset_x_cm": variant.sensor_2_offset_x_cm,
                    "sensor_2_yaw_offset_deg": variant.sensor_2_yaw_offset_deg,
                    "description": variant.description,
                }
            )


def write_mount_hypothesis_markdown(path: str | Path, report: DT35MountHypothesisReport) -> None:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(build_mount_hypothesis_markdown(report), encoding="utf-8")


def build_mount_hypothesis_markdown(report: DT35MountHypothesisReport) -> str:
    lines = [
        "# DT35 Mount Hypothesis Analysis",
        "",
        f"- created_at: {report.created_at}",
        f"- start mapping: {report.start_side} / {report.start_policy}",
        f"- recommended_variant: `{report.recommended_variant}`",
        f"- output_dir: `{report.output_dir}`",
        "",
        "Scoring favors many fusion-usable DT35 rows, low residual RMS, and few floor/near-hit suspects. Lidar is treated as approximate ground truth in the startup-local coordinate system.",
        "Each DT35 ray is evaluated per frame as H30 yaw plus that sensor's yaw offset; the beam is not treated as a fixed map-horizontal ray.",
        "",
    ]
    if report.notes:
        lines.append("## Notes")
        lines.extend(f"- {note}" for note in report.notes)
        lines.append("")
    lines.extend(
        [
            "## Ranked Variants",
            "",
            "| rank | variant | score | fusion usable | floor suspect | gate rejected | RMS cm | sensor layout |",
            "|---:|---|---:|---:|---:|---:|---:|---|",
        ]
    )
    for index, item in enumerate(report.scores, start=1):
        variant = item.variant
        rms = "-" if item.rms_residual_cm is None else f"{item.rms_residual_cm:.2f}"
        layout = (
            f"S1 x={variant.sensor_1_offset_x_cm:+.1f},yaw={variant.sensor_1_yaw_offset_deg:+.0f}; "
            f"S2 x={variant.sensor_2_offset_x_cm:+.1f},yaw={variant.sensor_2_yaw_offset_deg:+.0f}"
        )
        lines.append(
            f"| {index} | `{variant.name}` | {item.score:.1f} | {item.fusion_usable_rays} | "
            f"{item.floor_hit_suspect_rays} | {item.residual_gate_rejected_rays} | {rms} | {layout} |"
        )
    lines.append("")
    lines.append("## Per-Log Detail")
    for item in report.scores:
        lines.extend(["", f"### {item.variant.name}", ""])
        for log in item.logs:
            rms = "-" if log.rms_residual_cm is None else f"{log.rms_residual_cm:.2f}"
            mean_abs = "-" if log.mean_abs_residual_cm is None else f"{log.mean_abs_residual_cm:.2f}"
            lines.append(
                f"- {log.log_name}: fusion={log.fusion_usable_rays}, floor={log.floor_hit_suspect_rays}, "
                f"gate={log.residual_gate_rejected_rays}, rms={rms}cm, mean_abs={mean_abs}cm"
            )
    return "\n".join(lines).rstrip() + "\n"


def _config_for_variant(config: dict[str, Any], variant: DT35MountVariant) -> dict[str, Any]:
    out = deepcopy(config)
    if variant.name == "configured":
        return out
    dt35 = out.setdefault("dt35", {})
    s1 = dt35.setdefault("sensor_1", {})
    s2 = dt35.setdefault("sensor_2", {})
    s1["offset_x_cm"] = variant.sensor_1_offset_x_cm
    s1["yaw_offset_deg"] = variant.sensor_1_yaw_offset_deg
    s1["distance_bias_mm"] = 0.0
    s1["name"] = f"{variant.name} sensor_1"
    s2["offset_x_cm"] = variant.sensor_2_offset_x_cm
    s2["yaw_offset_deg"] = variant.sensor_2_yaw_offset_deg
    s2["distance_bias_mm"] = 0.0
    s2["name"] = f"{variant.name} sensor_2"
    return out


def _score_variant(
    config: dict[str, Any],
    variant: DT35MountVariant,
    csv_paths: list[Path],
    start_side: str,
    start_policy: str,
) -> DT35MountVariantScore:
    logs: list[DT35MountLogScore] = []
    all_residuals: list[float] = []
    totals = {
        "frames": 0,
        "valid_rays": 0,
        "usable_rays": 0,
        "fusion_usable_rays": 0,
        "floor_hit_suspect_rays": 0,
        "residual_gate_rejected_rays": 0,
    }
    for csv_path in csv_paths:
        frames = load_frames_csv(csv_path)
        rows = analyze_dt35_frames(
            config,
            frames,
            pose_source="lidar",
            yaw_source="h30",
            start_side=start_side,
            start_policy=start_policy,
        )
        residuals = [row.residual_cm for row in rows if row.usable_for_fusion and isfinite(row.residual_cm)]
        all_residuals.extend(residuals)
        summary = summarize_residuals(rows).to_dict()
        log_score = DT35MountLogScore(
            log_name=csv_path.parent.parent.name,
            frames=len(frames),
            valid_rays=int(summary.get("valid_rays") or 0),
            usable_rays=int(summary.get("usable_rays") or 0),
            fusion_usable_rays=int(summary.get("fusion_usable_rays") or 0),
            floor_hit_suspect_rays=int(summary.get("floor_hit_suspect_rays") or 0),
            residual_gate_rejected_rays=int(summary.get("residual_gate_rejected_rays") or 0),
            rms_residual_cm=_finite_or_none(summary.get("rms_residual_cm")),
            mean_abs_residual_cm=_mean_abs(residuals),
            score=0.0,
        )
        log_score.score = _score_counts(
            log_score.fusion_usable_rays,
            log_score.floor_hit_suspect_rays,
            log_score.residual_gate_rejected_rays,
            log_score.rms_residual_cm,
        )
        logs.append(log_score)
        totals["frames"] += len(frames)
        for key in ("valid_rays", "usable_rays", "fusion_usable_rays", "floor_hit_suspect_rays", "residual_gate_rejected_rays"):
            totals[key] += getattr(log_score, key)

    rms = _rms(all_residuals)
    score = _score_counts(
        totals["fusion_usable_rays"],
        totals["floor_hit_suspect_rays"],
        totals["residual_gate_rejected_rays"],
        rms,
    )
    resolved_variant = _resolved_variant(config, variant)
    score += _mount_prior_bonus(resolved_variant)
    return DT35MountVariantScore(
        variant=resolved_variant,
        logs=logs,
        total_frames=totals["frames"],
        valid_rays=totals["valid_rays"],
        usable_rays=totals["usable_rays"],
        fusion_usable_rays=totals["fusion_usable_rays"],
        floor_hit_suspect_rays=totals["floor_hit_suspect_rays"],
        residual_gate_rejected_rays=totals["residual_gate_rejected_rays"],
        rms_residual_cm=rms,
        mean_abs_residual_cm=_mean_abs(all_residuals),
        score=score,
    )


def _resolved_variant(config: dict[str, Any], variant: DT35MountVariant) -> DT35MountVariant:
    if variant.name != "configured":
        return variant
    dt35 = config.get("dt35", {})
    s1 = dt35.get("sensor_1", {})
    s2 = dt35.get("sensor_2", {})
    return DT35MountVariant(
        name=variant.name,
        description=variant.description,
        sensor_1_offset_x_cm=float(s1.get("offset_x_cm", 0.0)),
        sensor_1_yaw_offset_deg=float(s1.get("yaw_offset_deg", 0.0)),
        sensor_2_offset_x_cm=float(s2.get("offset_x_cm", 0.0)),
        sensor_2_yaw_offset_deg=float(s2.get("yaw_offset_deg", 0.0)),
    )


def _score_counts(fusion_usable: int, floor_hits: int, gate_rejected: int, rms_cm: float | None) -> float:
    rms_penalty = 0.0 if rms_cm is None else rms_cm * 20.0
    return float(fusion_usable) - floor_hits * 0.15 - gate_rejected * 0.05 - rms_penalty


def _mount_prior_bonus(variant: DT35MountVariant) -> float:
    sensor_1_outward = variant.sensor_1_offset_x_cm < 0.0 and variant.sensor_1_yaw_offset_deg < 0.0
    sensor_2_outward = variant.sensor_2_offset_x_cm > 0.0 and variant.sensor_2_yaw_offset_deg > 0.0
    return 0.25 if sensor_1_outward and sensor_2_outward else 0.0


def _build_notes(scores: list[DT35MountVariantScore]) -> list[str]:
    if not scores:
        return ["No mount variants were evaluated."]
    best = scores[0]
    notes = [
        "This compares geometry hypotheses only; it does not modify DT35 firmware scale/offset.",
        "DT35 beam direction is recomputed from H30 yaw on every frame before matching against field geometry.",
        "Rows tagged as floor/near-hit, ignored rack/pole hits, no-hit, or high residual are not counted as fusion-usable.",
    ]
    if len(scores) > 1:
        second = scores[1]
        if best.fusion_usable_rays > second.fusion_usable_rays * 1.5 and best.fusion_usable_rays >= 50:
            notes.append(
                f"`{best.variant.name}` is strongly favored by fusion-usable sample count "
                f"({best.fusion_usable_rays} vs {second.fusion_usable_rays})."
            )
    if best.fusion_usable_rays == 0:
        notes.append("No variant produced fusion-usable DT35 rows; rely on lidar/H30/encoder and collect cleaner wall-hit data.")
    return notes


def _resolve_log_csv(path: str | Path) -> Path:
    item = Path(path)
    if item.is_file():
        return item
    for candidate in (
        item / "sensor_data" / "raw_frames.csv",
        item / "sensor_data" / "display_frames.csv",
        item / "raw_frames.csv",
        item / "display_frames.csv",
    ):
        if candidate.exists():
            return candidate
    raise FileNotFoundError(f"No raw/display frame CSV found under {item}")


def _resolve_output_dir(config: dict[str, Any], output_dir: str | Path | None) -> Path:
    if output_dir:
        return Path(output_dir)
    root = Path(config.get("_project_root", Path.cwd()))
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return root / "logs" / "RL_data" / f"{stamp}_dt35_mount_hypothesis"


def _mean_abs(values: list[float]) -> float | None:
    if not values:
        return None
    return sum(abs(item) for item in values) / len(values)


def _rms(values: list[float]) -> float | None:
    if not values:
        return None
    return sqrt(sum(item * item for item in values) / len(values))


def _finite_or_none(value: Any) -> float | None:
    if value is None:
        return None
    out = float(value)
    return out if isfinite(out) else None


def _json_safe(value: Any) -> Any:
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_json_safe(item) for item in value]
    if isinstance(value, float):
        return value if isfinite(value) else None
    return value
