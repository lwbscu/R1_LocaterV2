import csv
import json
from pathlib import Path

from locater_map.left_forest_loop_experiment import _field_pose_to_local, run_left_forest_loop_experiment


def _default_config() -> dict:
    config_path = Path(__file__).resolve().parents[1] / "config" / "default_config.json"
    config = json.loads(config_path.read_text(encoding="utf-8"))
    config["_project_root"] = str(config_path.parents[1])
    return config


def test_left_forest_loop_experiment_writes_10hz_validation_package(tmp_path):
    result = run_left_forest_loop_experiment(
        _default_config(),
        output_dir=tmp_path / "loop",
        duration_s=2.0,
        hz=10.0,
        screenshot_hz=1.0,
        render_png=False,
    )

    assert result.frames == 21
    assert result.stage == "ideal"
    assert result.hz == 10.0
    assert result.screenshot_count == 3
    assert Path(result.ground_truth_csv).exists()
    assert Path(result.report_md).exists()
    assert Path(result.report_json).exists()
    assert Path(result.overview_svg).exists()
    assert Path(result.raw_serial_log).exists()
    assert Path(result.dt35_residual_csv).exists()

    truth_rows = list(csv.DictReader(Path(result.truth_csv).open("r", encoding="utf-8", newline="")))
    assert int(truth_rows[1]["source_time_ms"]) - int(truth_rows[0]["source_time_ms"]) == 100
    firmware_rows = list(csv.DictReader(Path(result.firmware_like_csv).open("r", encoding="utf-8", newline="")))
    first = firmware_rows[0]
    assert abs(float(first["pos_x_cm"])) < 1.0e-6
    assert abs(float(first["pos_y_cm"])) < 1.0e-6
    assert abs(float(first["pos_yaw_deg"])) < 1.0e-6
    assert abs(float(first["lidar_x_cm"])) < 1.0e-6
    assert abs(float(first["lidar_y_cm"])) < 1.0e-6
    assert abs(float(first["lidar_yaw_deg"])) < 1.0e-6

    raw_lines = Path(result.raw_serial_log).read_text(encoding="utf-8").strip().splitlines()
    assert len(raw_lines) == result.frames
    assert all(len(line.split(",")) == 12 for line in raw_lines)
    assert raw_lines[0].startswith("0.000,0.000,0.000,0.000,0.000,0.000")

    summary = json.loads(Path(result.dt35_residual_summary_json).read_text(encoding="utf-8"))
    assert summary["valid_rays"] > 0
    assert summary["usable_rays"] > 0
    assert summary["rms_residual_cm"] is not None
    assert summary["rms_residual_cm"] < 1.0e-6


def test_left_forest_lidar_noise_stage_reports_ground_truth_metrics(tmp_path):
    result = run_left_forest_loop_experiment(
        _default_config(),
        output_dir=tmp_path / "loop_noise",
        duration_s=2.0,
        hz=10.0,
        screenshot_hz=1.0,
        stage="lidar_noise",
        render_png=False,
    )

    payload = json.loads(Path(result.report_json).read_text(encoding="utf-8"))
    assert payload["stage"] == "lidar_noise"
    assert payload["ground_truth_error_metrics"]["lidar_rms_xy_cm"] is not None
    assert payload["ground_truth_error_metrics"]["lidar_rms_xy_cm"] > 0.0
    assert payload["ground_truth_error_metrics"]["fused_rms_xy_cm"] is not None
    first_line = Path(result.raw_serial_log).read_text(encoding="utf-8").splitlines()[0]
    assert first_line.startswith("0.000,0.000,0.000,0.000,0.000,0.000")


def test_red_start_field_to_local_axes_match_robot_protocol():
    start_tf = (-552.5, 549.0, 0.0, True)

    assert _field_pose_to_local(-552.5, 549.0, 0.0, start_tf) == (0.0, 0.0, 0.0)
    east_x, east_y, east_yaw = _field_pose_to_local(-542.5, 549.0, 0.0, start_tf)
    south_x, south_y, south_yaw = _field_pose_to_local(-552.5, 539.0, 0.0, start_tf)

    assert round(east_x, 6) == 10.0
    assert round(east_y, 6) == 0.0
    assert round(east_yaw, 6) == 0.0
    assert round(south_x, 6) == 0.0
    assert round(south_y, 6) == -10.0
    assert round(south_yaw, 6) == 0.0
