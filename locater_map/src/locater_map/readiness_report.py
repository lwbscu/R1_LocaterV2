from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from datetime import datetime
from math import isfinite
from pathlib import Path
from typing import Any

from .dt35_analysis import DEFAULT_YAW_MATRIX_BASE_POSES, analyze_dt35_hits, generate_yaw_matrix_poses
from .dt35_field_sweep import run_dt35_field_sweep, write_field_sweep_csv, write_field_sweep_summary
from .dt35_observability_report import generate_dt35_observability_report, write_dt35_observability_report
from .dt35_role_report import (
    build_dt35_role_rows,
    summarize_dt35_roles,
    write_dt35_role_csv,
    write_dt35_role_markdown,
    write_dt35_role_summary,
)
from .dt35_validation_plan import (
    generate_dt35_validation_plan,
    write_validation_plan_csv,
    write_validation_plan_markdown,
    write_validation_plan_summary,
)
from .field_model_audit import build_field_model_audit, write_field_model_audit
from .field_model_export import write_field_model_svg
from .fusion_benchmark import DEFAULT_BENCHMARK_PATHS, run_synthetic_benchmark, write_benchmark_csv, write_benchmark_summary
from .fusion_model import FusionConfig
from .obstacle_ablation import DEFAULT_OBSTACLE_PATHS, run_obstacle_ablation, write_obstacle_ablation_csv, write_obstacle_ablation_summary
from .real_validation_suite import run_real_validation_suite


@dataclass(slots=True)
class ReadinessArtifacts:
    report_json: str
    report_md: str
    objective_coverage_json: str
    objective_coverage_md: str
    field_model_audit_json: str
    field_model_overlay_svg: str
    dt35_field_sweep_csv: str
    dt35_field_sweep_json: str
    dt35_observability_csv: str
    dt35_observability_json: str
    dt35_observability_md: str
    dt35_role_matrix_csv: str
    dt35_role_matrix_json: str
    dt35_role_matrix_md: str
    dt35_validation_plan_csv: str
    dt35_validation_plan_json: str
    dt35_validation_plan_md: str
    synthetic_benchmark_csv: str
    synthetic_benchmark_json: str
    obstacle_ablation_csv: str
    obstacle_ablation_json: str
    real_validation_dir: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True)
class ReadinessReport:
    generated_at: str
    output_dir: str
    offline_readiness_passed: bool
    completion_verified: bool
    checks: dict[str, bool]
    assumptions: dict[str, Any]
    objective_coverage: list[dict[str, Any]]
    field_model: dict[str, Any]
    dt35_field_sweep: dict[str, Any]
    dt35_observability: dict[str, Any]
    dt35_role_matrix: dict[str, Any]
    dt35_validation_plan: dict[str, Any]
    synthetic_benchmark: dict[str, Any]
    obstacle_ablation: dict[str, Any]
    real_validation: dict[str, Any] | None
    next_actions: list[str]
    artifacts: ReadinessArtifacts

    def to_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["artifacts"] = self.artifacts.to_dict()
        return _json_safe(payload)


def run_readiness_report(
    config: dict[str, Any],
    output_dir: str | Path | None,
    fusion_cfg: FusionConfig,
    *,
    real_csv: str | Path | None = None,
    samples: int = 180,
    grid_step_cm: float = 100.0,
) -> ReadinessReport:
    out_dir = _resolve_output_dir(config, output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    audit_path = out_dir / "field_model_audit.json"
    overlay_svg_path = out_dir / "field_model_overlay.svg"
    sweep_csv_path = out_dir / "dt35_field_sweep.csv"
    sweep_summary_path = out_dir / "dt35_field_sweep.json"
    observability_csv_path = out_dir / "dt35_observability.csv"
    observability_summary_path = out_dir / "dt35_observability.json"
    observability_md_path = out_dir / "dt35_observability.md"
    role_csv_path = out_dir / "dt35_role_matrix.csv"
    role_summary_path = out_dir / "dt35_role_matrix.json"
    role_md_path = out_dir / "dt35_role_matrix.md"
    plan_csv_path = out_dir / "dt35_validation_plan.csv"
    plan_summary_path = out_dir / "dt35_validation_plan.json"
    plan_md_path = out_dir / "dt35_validation_plan.md"
    benchmark_csv_path = out_dir / "synthetic_benchmark.csv"
    benchmark_summary_path = out_dir / "synthetic_benchmark.json"
    obstacle_csv_path = out_dir / "obstacle_ablation.csv"
    obstacle_summary_path = out_dir / "obstacle_ablation.json"
    objective_coverage_json_path = out_dir / "objective_coverage.json"
    objective_coverage_md_path = out_dir / "objective_coverage.md"
    report_json_path = out_dir / "readiness_report.json"
    report_md_path = out_dir / "readiness_report.md"

    write_field_model_audit(audit_path, config)
    write_field_model_svg(overlay_svg_path, config)
    field_audit = build_field_model_audit(config)

    sweep_rows, sweep_summary = run_dt35_field_sweep(config, step_cm=grid_step_cm)
    write_field_sweep_csv(sweep_csv_path, sweep_rows)
    write_field_sweep_summary(sweep_summary_path, sweep_summary)

    observability_rows, observability_summary = generate_dt35_observability_report(config, step_cm=grid_step_cm)
    write_dt35_observability_report(
        observability_csv_path,
        observability_summary_path,
        observability_md_path,
        observability_rows,
        observability_summary,
    )

    role_poses = generate_yaw_matrix_poses(list(DEFAULT_YAW_MATRIX_BASE_POSES), [-180.0, -90.0, 0.0, 90.0, 180.0])
    role_rows = build_dt35_role_rows(analyze_dt35_hits(config, role_poses))
    role_summary = summarize_dt35_roles(role_rows)
    write_dt35_role_csv(role_csv_path, role_rows)
    write_dt35_role_summary(role_summary_path, role_summary)
    write_dt35_role_markdown(role_md_path, role_rows, role_summary)

    plan_cases, plan_summary = generate_dt35_validation_plan(config, step_cm=max(40.0, grid_step_cm), per_category=3)
    write_validation_plan_csv(plan_csv_path, plan_cases)
    write_validation_plan_summary(plan_summary_path, plan_summary)
    write_validation_plan_markdown(plan_md_path, plan_cases, plan_summary)

    benchmark_rows, benchmark_summary = run_synthetic_benchmark(
        config,
        DEFAULT_BENCHMARK_PATHS,
        samples=max(2, int(samples)),
        encoder_x_scale=0.97,
        encoder_y_scale=1.03,
        dt35_noise_mm=0.0,
        fusion_cfg=fusion_cfg,
    )
    write_benchmark_csv(benchmark_csv_path, benchmark_rows)
    write_benchmark_summary(benchmark_summary_path, benchmark_summary)

    obstacle_rows, obstacle_summary = run_obstacle_ablation(
        config,
        DEFAULT_OBSTACLE_PATHS,
        samples=max(2, int(samples)),
        encoder_x_scale=0.97,
        encoder_y_scale=1.03,
        dt35_noise_mm=0.0,
        fusion_cfg=fusion_cfg,
    )
    write_obstacle_ablation_csv(obstacle_csv_path, obstacle_rows)
    write_obstacle_ablation_summary(obstacle_summary_path, obstacle_summary)

    real_validation = None
    real_validation_dir: str | None = None
    if real_csv:
        real_dir = out_dir / "real_validation"
        real_result = run_real_validation_suite(config, real_csv, real_dir, fusion_cfg, start_policy="off")
        real_validation = real_result.to_dict()
        real_validation_dir = str(real_dir)

    checks = _checks(
        field_audit,
        sweep_summary.to_dict(),
        observability_summary.to_dict(),
        role_summary.to_dict(),
        plan_summary.to_dict(),
        benchmark_summary.to_dict(),
        obstacle_summary.to_dict(),
        real_validation,
        fusion_cfg,
    )
    offline_keys = [
        "h30_yaw_is_authority",
        "dt35_translation_enabled",
        "field_model_self_check",
        "manual_dimensions_passed",
        "dt35_sweep_model_passed",
        "forest_constraints_present",
        "ramp_constraints_present",
        "ignored_interference_modeled_but_not_corrected",
        "observability_has_x_y_and_diagonal_rank1",
        "observability_documents_underconstrained_cases",
        "dt35_role_matrix_explains_forest_ramp_ignore",
        "dt35_role_matrix_has_x_y_axes",
        "validation_plan_complete",
        "synthetic_all_paths_passed",
        "synthetic_dt35_used",
        "synthetic_fused_better_than_raw",
        "forest_ramp_ablation_passed",
        "forest_ramp_ablation_has_forest_and_ramp_rays",
    ]
    offline_passed = all(checks.get(key, False) for key in offline_keys)
    completion_verified = bool(real_validation and checks.get("real_validation_passed", False))
    objective_coverage = build_objective_coverage(checks, real_validation is not None)

    artifacts = ReadinessArtifacts(
        report_json=str(report_json_path),
        report_md=str(report_md_path),
        objective_coverage_json=str(objective_coverage_json_path),
        objective_coverage_md=str(objective_coverage_md_path),
        field_model_audit_json=str(audit_path),
        field_model_overlay_svg=str(overlay_svg_path),
        dt35_field_sweep_csv=str(sweep_csv_path),
        dt35_field_sweep_json=str(sweep_summary_path),
        dt35_observability_csv=str(observability_csv_path),
        dt35_observability_json=str(observability_summary_path),
        dt35_observability_md=str(observability_md_path),
        dt35_role_matrix_csv=str(role_csv_path),
        dt35_role_matrix_json=str(role_summary_path),
        dt35_role_matrix_md=str(role_md_path),
        dt35_validation_plan_csv=str(plan_csv_path),
        dt35_validation_plan_json=str(plan_summary_path),
        dt35_validation_plan_md=str(plan_md_path),
        synthetic_benchmark_csv=str(benchmark_csv_path),
        synthetic_benchmark_json=str(benchmark_summary_path),
        obstacle_ablation_csv=str(obstacle_csv_path),
        obstacle_ablation_json=str(obstacle_summary_path),
        real_validation_dir=real_validation_dir,
    )
    report = ReadinessReport(
        generated_at=datetime.now().isoformat(timespec="seconds"),
        output_dir=str(out_dir),
        offline_readiness_passed=offline_passed,
        completion_verified=completion_verified,
        checks=checks,
        assumptions={
            "lidar": "absolute world-frame XY/YAW reference for recorded validation",
            "h30": "trusted yaw authority for DT35 raycasting and fused yaw; do not tune H30 yaw to hide a DT35 residual",
            "dt35": "trusted side-distance measurement; do not tune DT35 scale/offset to hide field-model residuals",
            "encoder": "high-rate XY interpolation between lidar anchors",
            "residual_policy": "persistent DT35 residuals indicate field geometry, mounting offset, lidar/world alignment, or unmodeled objects",
            "synthetic_dt35_noise_mm": 0.0,
            "dt35_correct_lidar_frames": fusion_cfg.dt35_correct_lidar_frames,
            "dt35_yaw_gain": fusion_cfg.dt35_yaw_gain,
            "dt35_gain": fusion_cfg.dt35_gain,
        },
        objective_coverage=objective_coverage,
        field_model={
            "model_self_check": field_audit.get("model_self_check", {}),
            "manual_dimension_checks": field_audit.get("manual_dimension_checks", []),
            "default_pose_behavior": field_audit.get("default_pose_behavior", {}),
        },
        dt35_field_sweep=sweep_summary.to_dict(),
        dt35_observability=observability_summary.to_dict(),
        dt35_role_matrix=role_summary.to_dict(),
        dt35_validation_plan=plan_summary.to_dict(),
        synthetic_benchmark=benchmark_summary.to_dict(),
        obstacle_ablation=obstacle_summary.to_dict(),
        real_validation=real_validation,
        next_actions=_next_actions(checks, real_csv),
        artifacts=artifacts,
    )
    objective_coverage_json_path.write_text(json.dumps(_json_safe(objective_coverage), ensure_ascii=False, indent=2), encoding="utf-8")
    objective_coverage_md_path.write_text(build_objective_coverage_markdown(objective_coverage), encoding="utf-8")
    report_json_path.write_text(json.dumps(report.to_dict(), ensure_ascii=False, indent=2), encoding="utf-8")
    report_md_path.write_text(build_readiness_markdown(report), encoding="utf-8")
    return report


def build_objective_coverage(checks: dict[str, bool], has_real_log: bool) -> list[dict[str, Any]]:
    items = [
        _coverage_item(
            "field_geometry_from_docs",
            "Field size, walls, ignored interference, forest, and ramp geometry are modeled from the map documents.",
            ["field_model_overlay.svg", "field_model_audit.json", "dt35_field_sweep.json"],
            checks["field_model_self_check"]
            and checks["manual_dimensions_passed"]
            and checks["dt35_sweep_model_passed"]
            and checks["ignored_interference_modeled_but_not_corrected"],
        ),
        _coverage_item(
            "yaw_rotates_dt35_rays",
            "H30 yaw rotates both side-facing DT35 rays into the world frame before raycasting.",
            ["field_model_overlay.svg", "dt35_role_matrix.md", "dt35_observability.md"],
            checks["h30_yaw_is_authority"]
            and checks["dt35_role_matrix_has_x_y_axes"]
            and checks["observability_has_x_y_and_diagonal_rank1"],
        ),
        _coverage_item(
            "lidar_absolute_world_anchor",
            "Lidar is treated as the absolute world-coordinate pose reference; encoder/H30/DT35 are compared against it.",
            ["synthetic_benchmark.json", "real_validation/real_validation_suite.json"],
            checks["synthetic_all_paths_passed"] and checks["synthetic_fused_better_than_raw"],
            requires_real=True,
            has_real_log=has_real_log,
            real_passed=checks["real_validation_passed"],
        ),
        _coverage_item(
            "encoder_h30_interpolation",
            "Current encoder XY algorithm and H30 yaw provide high-rate interpolation between lidar anchors.",
            ["synthetic_benchmark.json", "path_report.csv"],
            checks["synthetic_dt35_used"] and checks["synthetic_fused_better_than_raw"],
            requires_real=True,
            has_real_log=has_real_log,
            real_passed=checks["real_validation_passed"],
        ),
        _coverage_item(
            "dt35_meaning_by_pose_and_yaw",
            "The report distinguishes what DT35-1 and DT35-2 measure at different lidar XY and H30 yaw poses.",
            ["dt35_role_matrix.md", "dt35_observability.md"],
            checks["dt35_role_matrix_explains_forest_ramp_ignore"]
            and checks["dt35_role_matrix_has_x_y_axes"]
            and checks["observability_documents_underconstrained_cases"],
        ),
        _coverage_item(
            "forest_and_ramp_are_dt35_blockers",
            "Forest and ramp zones block DT35 rays and are allowed to correct translation with lower confidence than flat walls.",
            ["field_model_overlay.svg", "dt35_field_sweep.json", "obstacle_ablation.json"],
            checks["forest_constraints_present"]
            and checks["ramp_constraints_present"]
            and checks["forest_ramp_ablation_passed"]
            and checks["forest_ramp_ablation_has_forest_and_ramp_rays"],
        ),
        _coverage_item(
            "ignored_interference_is_not_used",
            "Blue long-pole/gap interference is modeled so rays can be ignored instead of incorrectly using the back wall.",
            ["dt35_field_sweep.json", "dt35_role_matrix.md"],
            checks["ignored_interference_modeled_but_not_corrected"],
        ),
        _coverage_item(
            "full_real_robot_validation",
            "A real parsed_frames.csv with lidar, H30, both encoders, and both DT35 sensors passes the validation suite.",
            ["real_validation/real_validation_suite.json", "real_validation/dt35_advice.md"],
            checks["real_validation_passed"],
            requires_real=True,
            has_real_log=has_real_log,
            real_passed=checks["real_validation_passed"],
        ),
    ]
    return items


def build_objective_coverage_markdown(items: list[dict[str, Any]]) -> str:
    lines = ["# Objective Coverage", ""]
    for item in items:
        lines.extend(
            [
                f"## {item['id']}",
                "",
                f"- Status: {item['status']}",
                f"- Requirement: {item['requirement']}",
                f"- Evidence: {', '.join(item['evidence'])}",
                f"- Note: {item['note']}",
                "",
            ]
        )
    return "\n".join(lines).rstrip() + "\n"


def _coverage_item(
    item_id: str,
    requirement: str,
    evidence: list[str],
    offline_passed: bool,
    *,
    requires_real: bool = False,
    has_real_log: bool = False,
    real_passed: bool = False,
) -> dict[str, Any]:
    if requires_real and real_passed:
        status = "passed_real"
        note = "Real-log validation passed."
    elif requires_real and not has_real_log and offline_passed:
        status = "passed_offline_needs_real_log"
        note = "Offline model evidence passed; capture real parsed_frames.csv for final proof."
    elif requires_real and not has_real_log:
        status = "needs_real_log"
        note = "This requirement can only be proven with a real parsed_frames.csv."
    elif requires_real and has_real_log and not real_passed:
        status = "failed_real_log"
        note = "Real log was provided but the validation suite did not pass."
    elif offline_passed:
        status = "passed_offline"
        note = "Covered by generated offline model/simulation artifacts."
    else:
        status = "failed_offline"
        note = "Offline readiness check did not pass; inspect the listed artifacts."
    return {
        "id": item_id,
        "requirement": requirement,
        "status": status,
        "evidence": evidence,
        "requires_real_log": requires_real,
        "offline_passed": bool(offline_passed),
        "real_passed": bool(real_passed),
        "note": note,
    }


def build_readiness_markdown(report: ReadinessReport) -> str:
    checks = report.checks
    lines = [
        "# R1 Locater Readiness Report",
        "",
        f"- Generated: {report.generated_at}",
        f"- Offline readiness passed: {report.offline_readiness_passed}",
        f"- Completion verified with real log: {report.completion_verified}",
        "",
        "## Assumptions",
        "",
    ]
    for key, value in report.assumptions.items():
        lines.append(f"- {key}: {value}")
    lines.extend(["", "## Objective Coverage", ""])
    for item in report.objective_coverage:
        lines.append(f"- {item['status']}: {item['id']}")
    lines.extend(["", "## Checks", ""])
    for key, passed in checks.items():
        mark = "PASS" if passed else "FAIL"
        lines.append(f"- {mark}: {key}")
    lines.extend(
        [
            "",
            "## DT35 Field Sweep",
            "",
            f"- Model passed: {report.dt35_field_sweep.get('model_passed')}",
            f"- Required categories: {report.dt35_field_sweep.get('required_category_counts')}",
            f"- Constraint states: {report.dt35_field_sweep.get('constraint_state_counts')}",
            f"- Risk counts: {report.dt35_field_sweep.get('risk_counts')}",
            "",
            "## DT35 Observability",
            "",
            f"- Rank counts: {report.dt35_observability.get('rank_counts')}",
            f"- Constraint states: {report.dt35_observability.get('constraint_state_counts')}",
            f"- Principal axes: {report.dt35_observability.get('principal_axis_counts')}",
            f"- Underconstrained poses: {report.dt35_observability.get('underconstrained_poses')}",
            f"- Two-dimensional poses: {report.dt35_observability.get('two_dim_poses')}",
            "",
            "## DT35 Role Matrix",
            "",
            f"- Usable rows: {report.dt35_role_matrix.get('usable_rows')}",
            f"- Usable forest rows: {report.dt35_role_matrix.get('usable_forest_rows')}",
            f"- Usable ramp rows: {report.dt35_role_matrix.get('usable_ramp_rows')}",
            f"- Ignored rows: {report.dt35_role_matrix.get('ignored_rows')}",
            f"- Sensor axes: {report.dt35_role_matrix.get('sensor_axis_counts')}",
            "",
            "## Synthetic Benchmark",
            "",
            f"- Paths: {report.synthetic_benchmark.get('paths')}",
            f"- Passed paths: {report.synthetic_benchmark.get('passed_paths')}",
            f"- Mean raw RMS XY cm: {report.synthetic_benchmark.get('mean_raw_rms_xy_cm')}",
            f"- Mean fused RMS XY cm: {report.synthetic_benchmark.get('mean_fused_rms_xy_cm')}",
            f"- Total DT35 fusion frames: {report.synthetic_benchmark.get('total_dt35_fusion_allowed_frames')}",
            "",
            "## Forest/Ramp Ablation",
            "",
            f"- Paths: {report.obstacle_ablation.get('paths')}",
            f"- Passed paths: {report.obstacle_ablation.get('passed_paths')}",
            f"- Forest fusion rays: {report.obstacle_ablation.get('total_full_forest_fusion_allowed_rays')}",
            f"- Ramp fusion rays: {report.obstacle_ablation.get('total_full_ramp_fusion_allowed_rays')}",
            f"- Full fused RMS XY cm: {report.obstacle_ablation.get('mean_full_fused_rms_xy_cm')}",
            f"- Ablated fused RMS XY cm: {report.obstacle_ablation.get('mean_ablated_fused_rms_xy_cm')}",
            f"- Mean ablation penalty cm: {report.obstacle_ablation.get('mean_ablation_penalty_cm')}",
            "",
            "## Next Actions",
            "",
        ]
    )
    lines.extend(f"- {item}" for item in report.next_actions)
    lines.extend(["", "## Artifacts", ""])
    for key, value in report.artifacts.to_dict().items():
        lines.append(f"- {key}: {value}")
    return "\n".join(lines).rstrip() + "\n"


def _resolve_output_dir(config: dict[str, Any], output_dir: str | Path | None) -> Path:
    if output_dir:
        return Path(output_dir)
    root = Path(config.get("_project_root", Path.cwd()))
    return root / "logs" / f"{datetime.now().strftime('%Y%m%d_%H%M%S')}_readiness"


def _checks(
    field_audit: dict[str, Any],
    sweep: dict[str, Any],
    observability: dict[str, Any],
    role_matrix: dict[str, Any],
    plan: dict[str, Any],
    benchmark: dict[str, Any],
    obstacle_ablation: dict[str, Any],
    real_validation: dict[str, Any] | None,
    fusion_cfg: FusionConfig,
) -> dict[str, bool]:
    manual_checks = field_audit.get("manual_dimension_checks", [])
    required_counts = sweep.get("required_category_counts", {})
    constraint_states = observability.get("constraint_state_counts", {})
    sensor_axes = role_matrix.get("sensor_axis_counts", {})
    mean_raw = benchmark.get("mean_raw_rms_xy_cm")
    mean_fused = benchmark.get("mean_fused_rms_xy_cm")
    checks = {
        "h30_yaw_is_authority": abs(float(fusion_cfg.dt35_yaw_gain)) <= 1.0e-9,
        "dt35_translation_enabled": float(fusion_cfg.dt35_gain) > 0.0,
        "field_model_self_check": bool(field_audit.get("model_self_check", {}).get("passed", False)),
        "manual_dimensions_passed": all(bool(item.get("passed", False)) for item in manual_checks),
        "dt35_sweep_model_passed": bool(sweep.get("model_passed", False)),
        "forest_constraints_present": int(required_counts.get("forest_constraint") or 0) > 0,
        "ramp_constraints_present": int(required_counts.get("ramp_constraint") or 0) > 0,
        "ignored_interference_modeled_but_not_corrected": bool(sweep.get("ignored_targets_never_corrected", False)),
        "observability_has_x_y_and_diagonal_rank1": (
            int(constraint_states.get("rank1_x") or 0) > 0
            and int(constraint_states.get("rank1_y") or 0) > 0
            and int(constraint_states.get("rank1_xy") or 0) > 0
        ),
        "observability_documents_underconstrained_cases": (
            int(observability.get("underconstrained_poses") or 0) > 0
            and int(observability.get("no_dt35_poses") or 0) > 0
            and int(observability.get("one_dim_poses") or 0) > 0
        ),
        "dt35_role_matrix_explains_forest_ramp_ignore": (
            int(role_matrix.get("usable_forest_rows") or 0) > 0
            and int(role_matrix.get("usable_ramp_rows") or 0) > 0
            and int(role_matrix.get("ignored_rows") or 0) > 0
        ),
        "dt35_role_matrix_has_x_y_axes": (
            any(str(key).endswith(":x") and int(value) > 0 for key, value in sensor_axes.items())
            and any(str(key).endswith(":y") and int(value) > 0 for key, value in sensor_axes.items())
        ),
        "validation_plan_complete": not plan.get("missing_categories"),
        "synthetic_all_paths_passed": int(benchmark.get("failed_paths") or 0) == 0 and int(benchmark.get("paths") or 0) > 0,
        "synthetic_dt35_used": int(benchmark.get("total_dt35_fusion_allowed_frames") or 0) > 0,
        "synthetic_fused_better_than_raw": _is_number(mean_raw) and _is_number(mean_fused) and float(mean_fused) < float(mean_raw),
        "forest_ramp_ablation_passed": int(obstacle_ablation.get("failed_paths") or 0) == 0 and int(obstacle_ablation.get("paths") or 0) > 0,
        "forest_ramp_ablation_has_forest_and_ramp_rays": (
            int(obstacle_ablation.get("total_full_forest_fusion_allowed_rays") or 0) > 0
            and int(obstacle_ablation.get("total_full_ramp_fusion_allowed_rays") or 0) > 0
        ),
        "real_log_provided": real_validation is not None,
        "real_validation_passed": bool(real_validation and real_validation.get("real_validation_passed", False)),
    }
    return checks


def _next_actions(checks: dict[str, bool], real_csv: str | Path | None) -> list[str]:
    actions: list[str] = []
    if not checks.get("field_model_self_check", False):
        actions.append("Fix field model self-check failures in field_model_audit.json.")
    if not checks.get("forest_constraints_present", False) or not checks.get("ramp_constraints_present", False):
        actions.append("Adjust forest/ramp modeled rectangles so DT35 rays can hit them as solid obstacles.")
    if not checks.get("ignored_interference_modeled_but_not_corrected", False):
        actions.append("Ensure blue ignored-interference targets are modeled but never allowed to correct pose.")
    if not checks.get("observability_has_x_y_and_diagonal_rank1", False):
        actions.append("Inspect dt35_observability.md; field/yaw sweep should demonstrate rank1_x, rank1_y, and rank1_xy DT35 constraints.")
    if not checks.get("observability_documents_underconstrained_cases", False):
        actions.append("Inspect dt35_observability.md; report should explicitly document rank0 and rank1 cases so lidar/encoder dependency is clear.")
    if not checks.get("dt35_role_matrix_explains_forest_ramp_ignore", False):
        actions.append("Inspect dt35_role_matrix.md; it should include usable forest/ramp hits and ignored-interference examples.")
    if not checks.get("dt35_role_matrix_has_x_y_axes", False):
        actions.append("Inspect dt35_role_matrix.md; selected lidar/yaw poses should demonstrate both world-X and world-Y DT35 constraints.")
    if not checks.get("synthetic_all_paths_passed", False):
        actions.append("Inspect synthetic_benchmark.json failed paths before changing firmware behavior.")
    if not checks.get("synthetic_fused_better_than_raw", False):
        actions.append("Tune field model/fusion gates until synthetic fused XY improves over encoder/H30 prediction.")
    if not checks.get("forest_ramp_ablation_passed", False):
        actions.append("Inspect obstacle_ablation.json; forest/ramp solid-obstacle correction should contribute without making fusion worse.")
    if not checks.get("forest_ramp_ablation_has_forest_and_ramp_rays", False):
        actions.append("Inspect obstacle_ablation.json; the simulated paths should include both forest and ramp DT35 fusion rays.")
    if real_csv is None:
        actions.append("Capture a real parsed_frames.csv with lidar, H30, both encoders, and both DT35 sensors, then rerun with --readiness-real-csv.")
    elif not checks.get("real_validation_passed", False):
        actions.append("Inspect real_validation/real_validation_suite.json and dt35_advice.md; keep H30/DT35 calibration fixed unless hardware evidence contradicts it.")
    if not actions:
        actions.append("Offline and real-log gates passed; this report can be used as completion evidence.")
    return actions


def _is_number(value: Any) -> bool:
    try:
        return isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _json_safe(value: Any) -> Any:
    if isinstance(value, float):
        return value if isfinite(value) else None
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_json_safe(item) for item in value]
    return value
