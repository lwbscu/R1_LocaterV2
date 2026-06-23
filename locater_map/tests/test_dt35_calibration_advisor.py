import json
from pathlib import Path

from locater_map.data_model import RobotFrame
from locater_map.dt35_analysis import analyze_dt35_frames
from locater_map.dt35_calibration_advisor import (
    build_calibration_advice,
    build_calibration_advice_markdown,
    write_calibration_advice_csv,
    write_calibration_advice_markdown,
    write_calibration_advice_summary,
)


def test_dt35_calibration_advice_suggests_axis_aligned_target_shift(tmp_path):
    config = {
        "map": {"field_width_cm": 200.0, "field_height_cm": 200.0},
        "robot": {"start_pose_policy": "off"},
        "display": {"live_fusion_dt35_residual_gate_cm": 40.0},
        "field_model": {"enabled": True, "use_field_boundary": True, "field_width_cm": 200.0, "field_height_cm": 200.0},
        "dt35": {
            "sensor_1": {"name": "left ray", "enabled": True, "offset_x_cm": 0.0, "offset_y_cm": 0.0, "yaw_offset_deg": -90.0, "max_range_cm": 250.0},
            "sensor_2": {"enabled": False},
        },
    }
    frames = [
        RobotFrame(
            seq=index,
            lidar_valid=True,
            lidar_online=True,
            lidar_x_cm=0.0,
            lidar_y_cm=0.0,
            h30_valid=True,
            h30_yaw_deg=0.0,
            dt35_1_valid=True,
            dt35_1_mm=1050.0,
        )
        for index in range(1, 5)
    ]
    rows = analyze_dt35_frames(config, frames, pose_source="lidar", yaw_source="h30", start_policy="off")

    advice, summary = build_calibration_advice(rows, min_frames=3, actionable_residual_cm=3.0)

    assert summary.targets == 1
    assert summary.actionable_targets == 1
    assert summary.worst_target == "field_left"
    item = advice[0]
    assert item.expected_target == "field_left"
    assert item.suggestion_axis == "x"
    assert round(item.suggested_shift_cm, 2) == -5.0
    assert item.rms_residual_cm == 5.0

    csv_path = tmp_path / "advice.csv"
    json_path = tmp_path / "advice.json"
    md_path = tmp_path / "advice.md"
    write_calibration_advice_csv(csv_path, advice)
    write_calibration_advice_summary(json_path, summary)
    write_calibration_advice_markdown(md_path, advice, summary)

    assert "suggested_shift_cm" in csv_path.read_text(encoding="utf-8").splitlines()[0]
    payload = json.loads(json_path.read_text(encoding="utf-8"))
    assert payload["actionable_targets"] == 1
    markdown = md_path.read_text(encoding="utf-8")
    assert "DT35 Calibration Advice" in markdown
    assert "field_left" in markdown
    assert build_calibration_advice_markdown(advice, summary).startswith("# DT35 Calibration Advice")


def test_dt35_calibration_advice_skips_floor_or_near_hit_suspects():
    config = {
        "map": {"field_width_cm": 200.0, "field_height_cm": 200.0},
        "robot": {"start_pose_policy": "off"},
        "display": {"live_fusion_dt35_residual_gate_cm": 40.0},
        "field_model": {
            "enabled": True,
            "use_field_boundary": True,
            "field_width_cm": 200.0,
            "field_height_cm": 200.0,
            "floor_hit_negative_residual_gate_cm": 12.0,
        },
        "dt35": {
            "sensor_1": {"name": "left ray", "enabled": True, "offset_x_cm": 0.0, "offset_y_cm": 0.0, "yaw_offset_deg": -90.0, "max_range_cm": 250.0},
            "sensor_2": {"enabled": False},
        },
    }
    frames = [
        RobotFrame(
            seq=index,
            lidar_valid=True,
            lidar_online=True,
            lidar_x_cm=0.0,
            lidar_y_cm=0.0,
            h30_valid=True,
            h30_yaw_deg=0.0,
            dt35_1_valid=True,
            dt35_1_mm=200.0,
        )
        for index in range(1, 5)
    ]
    rows = analyze_dt35_frames(config, frames, pose_source="lidar", yaw_source="h30", start_policy="off")

    advice, summary = build_calibration_advice(rows, min_frames=3, actionable_residual_cm=3.0)

    assert all(row.floor_hit_suspect for row in rows if row.sensor_key == "sensor_1")
    assert advice == []
    assert summary.targets == 0
    assert summary.actionable_targets == 0
