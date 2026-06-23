import json
from dataclasses import replace
from pathlib import Path

from locater_map.dt35_analysis import DT35FrameResidualRow
from locater_map.fusion_model import FusionConfig
from locater_map.path_diagnostics import (
    _sensor_fields,
    generate_path_diagnostic,
    generate_synthetic_path_diagnostic,
    write_path_diagnostic_csv,
    write_path_diagnostic_summary,
)
from locater_map.synthetic_sim import SyntheticConfig, generate_synthetic_frames


def test_synthetic_path_diagnostic_reports_frame_level_targets(tmp_path):
    config_path = Path(__file__).resolve().parents[1] / "config" / "default_config.json"
    config = json.loads(config_path.read_text(encoding="utf-8"))

    rows, summary, fused_frames = generate_synthetic_path_diagnostic(
        config,
        SyntheticConfig(samples=120, path_name="field_patrol", encoder_x_scale=0.97, encoder_y_scale=1.02),
        FusionConfig(lidar_stride=25, lidar_gain=1.0, dt35_gain=1.0, dt35_yaw_gain=0.0),
    )

    assert len(rows) == 120
    assert len(fused_frames) == 120
    assert summary.raw_rms_xy_cm is not None
    assert summary.fused_rms_xy_cm is not None
    assert summary.fused_rms_xy_cm < summary.raw_rms_xy_cm
    assert summary.no_dt35_rms_xy_cm is not None
    assert summary.dt35_active_frames > 0
    assert summary.dt35_helped_frames > 0
    assert summary.dt35_max_correction_cm is not None
    assert summary.dt35_max_correction_cm > 0.0
    assert summary.dt35_valid_frames > 0
    assert summary.dt35_allowed_frames > 0
    assert summary.dt35_fusion_allowed_frames > 0
    assert summary.dt35_rank_counts
    assert summary.dt35_constraint_state_counts
    assert summary.dt35_rank_error_stats
    assert summary.dt35_constraint_state_error_stats
    assert summary.dt35_principal_axis_error_stats
    assert "2" not in summary.dt35_rank_counts
    assert summary.dt35_rank_counts.get("1", 0) > 0
    assert sum(item["frames"] for item in summary.dt35_rank_error_stats.values()) == len(rows)
    assert sum(item["frames"] for item in summary.dt35_constraint_state_error_stats.values()) == len(rows)
    assert summary.dt35_constraint_state_error_stats["rank1_x"]["fused_rms_xy_cm"] is not None
    assert "solid_obstacle" in summary.dt35_type_counts
    assert any(row.dt35_1_target or row.dt35_2_target for row in rows)
    assert any(row.dt35_translation_rank == 1 for row in rows)

    csv_path = tmp_path / "path.csv"
    json_path = tmp_path / "path.json"
    write_path_diagnostic_csv(csv_path, rows)
    write_path_diagnostic_summary(json_path, summary)

    assert "fused_improvement_cm" in csv_path.read_text(encoding="utf-8").splitlines()[0]
    header = csv_path.read_text(encoding="utf-8").splitlines()[0]
    assert "dt35_1_ray_dx" in header
    assert "dt35_1_constraint_axis" in header
    assert "dt35_2_correction_dy_per_cm" in header
    assert "dt35_1_fusion_allowed" in header
    assert "dt35_1_residual_gate_cm" in header
    assert "dt35_1_floor_hit_suspect" in header
    assert "dt35_translation_rank" in header
    assert "dt35_constraint_state" in header
    assert "no_dt35_xy_error_cm" in header
    assert "dt35_correction_dx_cm" in header
    assert "dt35_improvement_cm" in header
    payload = json.loads(json_path.read_text(encoding="utf-8"))
    assert payload["frames"] == 120
    assert payload["model_validation"]["gates"]["passed"] is True
    assert payload["dt35_rank_counts"]["1"] > 0
    assert payload["dt35_rank_error_stats"]["1"]["frames"] == payload["dt35_rank_counts"]["1"]
    assert "rank1_x" in payload["dt35_constraint_state_error_stats"]
    assert payload["dt35_active_frames"] > 0
    assert payload["dt35_max_correction_cm"] > 0.0


def test_real_csv_style_path_diagnostic_uses_existing_frames():
    config_path = Path(__file__).resolve().parents[1] / "config" / "default_config.json"
    config = json.loads(config_path.read_text(encoding="utf-8"))
    truth = generate_synthetic_frames(
        config,
        SyntheticConfig(samples=80, path_name="center_divider", encoder_x_scale=0.97, encoder_y_scale=1.02),
    )
    frames = [replace(frame, pos_x_cm=frame.encoder_x_cm, pos_y_cm=frame.encoder_y_cm, pos_yaw_deg=frame.h30_yaw_deg) for frame in truth]

    rows, summary, fused_frames = generate_path_diagnostic(
        config,
        frames,
        FusionConfig(lidar_stride=20, lidar_gain=1.0, dt35_gain=1.0, dt35_yaw_gain=0.0),
        start_policy="off",
    )

    assert len(rows) == len(frames)
    assert len(fused_frames) == len(frames)
    assert summary.fused_rms_xy_cm is not None
    assert summary.raw_rms_xy_cm is not None
    assert summary.fused_rms_xy_cm < summary.raw_rms_xy_cm
    assert summary.no_dt35_rms_xy_cm is not None
    assert summary.dt35_allowed_frames > 0
    assert summary.dt35_fusion_allowed_frames > 0
    assert summary.dt35_rank_counts.get("1", 0) > 0
    assert sum(item["frames"] for item in summary.dt35_principal_axis_error_stats.values()) == len(rows)


def test_path_diagnostic_dt35_constraint_axis_follows_h30_yaw():
    left_ray = _sensor_fields("dt35_1", _residual_row(ray_yaw_deg=-90.0))
    assert round(left_ray["dt35_1_ray_dx"], 3) == -1.0
    assert round(left_ray["dt35_1_ray_dy"], 3) == 0.0
    assert left_ray["dt35_1_constraint_axis"] == "x"
    assert round(left_ray["dt35_1_correction_dx_per_cm"], 3) == 1.0
    assert round(left_ray["dt35_1_correction_dy_per_cm"], 3) == -0.0

    front_ray = _sensor_fields("dt35_1", _residual_row(ray_yaw_deg=0.0))
    assert round(front_ray["dt35_1_ray_dx"], 3) == 0.0
    assert round(front_ray["dt35_1_ray_dy"], 3) == 1.0
    assert front_ray["dt35_1_constraint_axis"] == "y"
    assert round(front_ray["dt35_1_correction_dx_per_cm"], 3) == -0.0
    assert round(front_ray["dt35_1_correction_dy_per_cm"], 3) == -1.0


def test_path_diagnostic_reports_dt35_contribution_against_no_dt35_baseline():
    config_path = Path(__file__).resolve().parents[1] / "config" / "default_config.json"
    config = json.loads(config_path.read_text(encoding="utf-8"))

    rows, summary, _fused_frames = generate_synthetic_path_diagnostic(
        config,
        SyntheticConfig(samples=100, path_name="forest_side", encoder_x_scale=0.97, encoder_y_scale=1.02),
        FusionConfig(lidar_stride=25, lidar_gain=1.0, dt35_gain=1.0, dt35_yaw_gain=0.0, dt35_correct_lidar_frames=True),
    )

    active_rows = [row for row in rows if row.dt35_correction_mag_cm > 1.0e-6]
    assert active_rows
    assert summary.dt35_active_frames == len(active_rows)
    assert summary.dt35_helped_frames > 0
    assert any(row.dt35_correction_axis in ("x", "y", "xy") for row in active_rows)
    assert any(row.no_dt35_xy_error_cm != row.fused_xy_error_cm for row in active_rows)


def _residual_row(ray_yaw_deg: float) -> DT35FrameResidualRow:
    return DT35FrameResidualRow(
        seq=1,
        source_time_ms=0,
        pose_source="lidar",
        yaw_source="h30",
        pose_x_cm=0.0,
        pose_y_cm=0.0,
        pose_yaw_deg=0.0,
        sensor_key="sensor_1",
        sensor_name="DT35-1",
        sensor_valid=True,
        measured_distance_cm=100.0,
        expected_distance_cm=90.0,
        residual_cm=10.0,
        residual_gate_cm=40.0,
        residual_within_gate=True,
        floor_hit_suspect=False,
        abs_residual_cm=10.0,
        expected_target="field_left",
        expected_target_type="usable_wall",
        correction_allowed=True,
        corner_ambiguous=False,
        within_range=True,
        usable_for_correction=True,
        usable_for_fusion=True,
        incidence_deg=0.0,
        incidence_scale=1.0,
        correction_weight=1.0,
        sensor_x_cm=0.0,
        sensor_y_cm=0.0,
        ray_yaw_deg=ray_yaw_deg,
        measured_hit_x_cm=0.0,
        measured_hit_y_cm=0.0,
        expected_hit_x_cm=0.0,
        expected_hit_y_cm=0.0,
    )
