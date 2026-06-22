import json
from dataclasses import replace
from pathlib import Path

from locater_map.fusion_model import FusionConfig
from locater_map.model_validation import validate_model_log, write_validation_report
from locater_map.synthetic_sim import SyntheticConfig, generate_synthetic_frames


def test_model_validation_reports_fused_pose_improvement(tmp_path):
    config_path = Path(__file__).resolve().parents[1] / "config" / "default_config.json"
    config = json.loads(config_path.read_text(encoding="utf-8"))
    truth_frames = generate_synthetic_frames(
        config,
        SyntheticConfig(samples=160, path_name="field_patrol", encoder_x_scale=0.97, encoder_y_scale=1.02, dt35_noise_mm=0.0),
    )
    firmware_like_frames = [
        replace(frame, pos_x_cm=frame.encoder_x_cm, pos_y_cm=frame.encoder_y_cm, pos_yaw_deg=frame.h30_yaw_deg)
        for frame in truth_frames
    ]

    report, fused_frames = validate_model_log(
        config,
        firmware_like_frames,
        FusionConfig(lidar_stride=25, lidar_gain=1.0, dt35_gain=1.0, dt35_yaw_gain=0.0),
        start_policy="off",
    )

    assert len(fused_frames) == len(firmware_like_frames)
    assert report.pose_error.raw_rms_xy_cm is not None
    assert report.pose_error.fused_rms_xy_cm is not None
    assert report.pose_error.fused_rms_xy_cm < report.pose_error.raw_rms_xy_cm
    assert report.dt35_residuals["valid_rays"] > 0
    assert report.dt35_residuals["usable_rays"] > 0
    assert report.gates.passed
    assert report.gates.checks["has_lidar_reference"]
    assert report.gates.checks["has_h30_yaw"]
    assert "sensor_1" in report.dt35_breakdown["by_sensor"]
    assert "sensor_2" in report.dt35_breakdown["by_sensor"]
    assert report.dt35_breakdown["by_target_type"]["usable_wall"]["usable_rays"] > 0
    assert "dt35_quality" in report.to_dict()
    assert report.dt35_quality["good_target_count"] > 0
    assert isinstance(report.dt35_quality["bad_targets"], list)

    out = tmp_path / "report.json"
    write_validation_report(out, report)
    payload = json.loads(out.read_text(encoding="utf-8"))
    assert payload["pose_error"]["fused_rms_xy_cm"] < payload["pose_error"]["raw_rms_xy_cm"]
    assert payload["gates"]["passed"] is True
    assert payload["dt35_breakdown"]["by_sensor"]["sensor_1"]["valid_rays"] > 0
    assert "dt35_quality" in payload
