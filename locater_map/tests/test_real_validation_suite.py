import json
from dataclasses import replace
from pathlib import Path

from locater_map.fusion_model import FusionConfig, write_frames_csv
from locater_map.real_validation_suite import run_real_validation_suite
from locater_map.synthetic_sim import SyntheticConfig, generate_synthetic_frames


def _default_config() -> dict:
    config_path = Path(__file__).resolve().parents[1] / "config" / "default_config.json"
    return json.loads(config_path.read_text(encoding="utf-8"))


def _firmware_like_frames(config: dict, *, protocol: str):
    truth_frames = generate_synthetic_frames(
        config,
        SyntheticConfig(
            samples=160,
            path_name="field_patrol",
            encoder_x_scale=0.97,
            encoder_y_scale=1.02,
            dt35_noise_mm=0.0,
        ),
    )
    return [
        replace(
            frame,
            pos_x_cm=frame.encoder_x_cm,
            pos_y_cm=frame.encoder_y_cm,
            pos_yaw_deg=frame.h30_yaw_deg,
            protocol=protocol,
        )
        for frame in truth_frames
    ]


def test_real_validation_suite_rejects_synthetic_as_completion(tmp_path):
    config = _default_config()
    csv_path = tmp_path / "synthetic_frames.csv"
    write_frames_csv(csv_path, _firmware_like_frames(config, protocol="synthetic_r1_csv_v3"))

    result = run_real_validation_suite(
        config,
        csv_path,
        tmp_path / "suite",
        FusionConfig(lidar_stride=25, lidar_gain=1.0, dt35_gain=1.0, dt35_yaw_gain=0.0),
        start_policy="off",
    )

    assert result.is_synthetic is True
    assert result.real_validation_passed is False
    assert result.completion_candidate is False
    assert result.checks["not_synthetic_input"] is False
    assert result.checks["path_report_dt35_active"] is True
    assert result.checks["path_report_dt35_not_worse_than_no_dt35"] is True
    assert result.checks["dt35_geometry_has_no_actionable_target_shift"] is True
    assert result.dt35_advice["actionable_targets"] == 0
    assert Path(result.artifacts.suite_report_json).exists()
    assert Path(result.artifacts.dt35_advice_csv).exists()
    assert Path(result.artifacts.dt35_advice_json).exists()
    assert Path(result.artifacts.dt35_advice_md).exists()


def test_real_validation_suite_accepts_real_like_complete_log(tmp_path):
    config = _default_config()
    csv_path = tmp_path / "real_like_frames.csv"
    write_frames_csv(csv_path, _firmware_like_frames(config, protocol="r1_csv_v3"))

    result = run_real_validation_suite(
        config,
        csv_path,
        tmp_path / "suite",
        FusionConfig(lidar_stride=25, lidar_gain=1.0, dt35_gain=1.0, dt35_yaw_gain=0.0),
        start_policy="off",
    )

    assert result.is_synthetic is False
    assert result.real_validation_passed is True
    assert result.completion_candidate is True
    assert result.checks["has_lidar_reference"] is True
    assert result.checks["has_h30_yaw"] is True
    assert result.checks["has_dt35_1_measurements"] is True
    assert result.checks["has_dt35_2_measurements"] is True
    assert result.checks["has_both_encoder_pulse_flags"] is True
    assert result.checks["path_report_dt35_active"] is True
    assert result.checks["path_report_dt35_not_worse_than_no_dt35"] is True
    assert result.checks["dt35_geometry_has_no_actionable_target_shift"] is True
    assert result.dt35_residuals["fusion_usable_rays"] > 0
    assert result.dt35_advice["actionable_targets"] == 0
    assert result.path_summary["dt35_active_frames"] > 0
    assert result.path_summary["fused_rms_xy_cm"] <= result.path_summary["no_dt35_rms_xy_cm"]

    payload = json.loads(Path(result.artifacts.suite_report_json).read_text(encoding="utf-8"))
    assert payload["real_validation_passed"] is True
    assert payload["checks"]["path_report_dt35_active"] is True
    assert payload["checks"]["dt35_geometry_has_no_actionable_target_shift"] is True
    assert payload["dt35_advice"]["actionable_targets"] == 0
    assert Path(payload["artifacts"]["path_report_csv"]).exists()
    assert Path(payload["artifacts"]["dt35_advice_md"]).exists()
