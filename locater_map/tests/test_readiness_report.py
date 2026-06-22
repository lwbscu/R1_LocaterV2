import json
from pathlib import Path

from locater_map.fusion_model import FusionConfig
from locater_map.readiness_report import run_readiness_report


def _default_config() -> dict:
    config_path = Path(__file__).resolve().parents[1] / "config" / "default_config.json"
    return json.loads(config_path.read_text(encoding="utf-8"))


def test_readiness_report_generates_offline_gate_artifacts(tmp_path):
    config = _default_config()

    report = run_readiness_report(
        config,
        tmp_path / "readiness",
        FusionConfig(lidar_stride=25, lidar_gain=1.0, dt35_gain=1.0, dt35_yaw_gain=0.0),
        samples=80,
        grid_step_cm=100.0,
    )

    assert report.offline_readiness_passed is True
    assert report.completion_verified is False
    assert report.checks["h30_yaw_is_authority"] is True
    assert report.checks["dt35_translation_enabled"] is True
    assert report.checks["forest_constraints_present"] is True
    assert report.checks["ramp_constraints_present"] is True
    assert report.checks["ignored_interference_modeled_but_not_corrected"] is True
    assert report.checks["observability_has_x_y_and_diagonal_rank1"] is True
    assert report.checks["observability_documents_underconstrained_cases"] is True
    assert report.checks["dt35_role_matrix_explains_forest_ramp_ignore"] is True
    assert report.checks["dt35_role_matrix_has_x_y_axes"] is True
    assert report.checks["synthetic_all_paths_passed"] is True
    assert report.checks["synthetic_dt35_used"] is True
    assert report.checks["forest_ramp_ablation_passed"] is True
    assert report.checks["forest_ramp_ablation_has_forest_and_ramp_rays"] is True
    assert report.checks["real_log_provided"] is False
    coverage = {item["id"]: item for item in report.objective_coverage}
    assert coverage["field_geometry_from_docs"]["status"] == "passed_offline"
    assert coverage["dt35_meaning_by_pose_and_yaw"]["status"] == "passed_offline"
    assert coverage["forest_and_ramp_are_dt35_blockers"]["status"] == "passed_offline"
    assert coverage["lidar_absolute_world_anchor"]["status"] == "passed_offline_needs_real_log"
    assert coverage["full_real_robot_validation"]["status"] == "needs_real_log"
    assert report.synthetic_benchmark["mean_fused_rms_xy_cm"] < report.synthetic_benchmark["mean_raw_rms_xy_cm"]
    assert report.obstacle_ablation["total_full_forest_fusion_allowed_rays"] > 0
    assert report.obstacle_ablation["total_full_ramp_fusion_allowed_rays"] > 0

    payload = json.loads(Path(report.artifacts.report_json).read_text(encoding="utf-8"))
    assert payload["offline_readiness_passed"] is True
    assert payload["completion_verified"] is False
    assert Path(payload["artifacts"]["objective_coverage_json"]).exists()
    assert Path(payload["artifacts"]["objective_coverage_md"]).exists()
    assert Path(payload["artifacts"]["field_model_overlay_svg"]).exists()
    assert Path(payload["artifacts"]["dt35_field_sweep_json"]).exists()
    assert Path(payload["artifacts"]["dt35_observability_md"]).exists()
    assert Path(payload["artifacts"]["dt35_role_matrix_md"]).exists()
    assert Path(payload["artifacts"]["dt35_validation_plan_md"]).exists()
    assert Path(payload["artifacts"]["synthetic_benchmark_json"]).exists()
    assert Path(payload["artifacts"]["obstacle_ablation_json"]).exists()
    assert "DT35 Role Matrix" in Path(report.artifacts.dt35_role_matrix_md).read_text(encoding="utf-8")
    assert "DT35 Observability Matrix" in Path(report.artifacts.dt35_observability_md).read_text(encoding="utf-8")
    overlay_svg = Path(report.artifacts.field_model_overlay_svg).read_text(encoding="utf-8")
    assert "DT35 field model overlay" in overlay_svg
    assert "usable wall" in overlay_svg
    assert "ignored interference" in overlay_svg
    assert "solid obstacle blocks laser" in overlay_svg
    assert "sensor_1" in overlay_svg
    assert "sensor_2" in overlay_svg
    assert "Objective Coverage" in Path(report.artifacts.objective_coverage_md).read_text(encoding="utf-8")
    assert "Capture a real parsed_frames.csv" in Path(report.artifacts.report_md).read_text(encoding="utf-8")
    assert report.assumptions["h30"].startswith("trusted yaw authority")
    assert "do not tune DT35 scale/offset" in report.assumptions["dt35"]
    assert report.assumptions["synthetic_dt35_noise_mm"] == 0.0
    assert report.assumptions["dt35_correct_lidar_frames"] is False
