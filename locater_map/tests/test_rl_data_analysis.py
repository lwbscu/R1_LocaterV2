from __future__ import annotations

import csv

from locater_map.config_loader import load_config
from locater_map.data_model import RobotFrame
from locater_map.rl_data_analysis import analyze_rl_logs


def _write_frames(path, frames: list[RobotFrame]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=RobotFrame.field_names())
        writer.writeheader()
        for frame in frames:
            writer.writerow(frame.to_row())


def test_rl_data_analysis_counts_floor_hit_suspects(tmp_path):
    config = load_config()
    log_dir = tmp_path / "20260622_000000_log"
    csv_path = log_dir / "sensor_data" / "raw_frames.csv"
    frames = [
        RobotFrame(
            pc_time=1.0,
            pos_x_cm=0.0,
            pos_y_cm=0.0,
            pos_yaw_deg=0.0,
            lidar_x_cm=0.0,
            lidar_y_cm=0.0,
            lidar_yaw_deg=0.0,
            encoder_x_cm=0.0,
            encoder_y_cm=0.0,
            h30_yaw_deg=0.0,
            dt35_1_mm=10.0,
            dt35_2_mm=5201.0,
            dt35_1_valid=True,
            dt35_2_valid=True,
            h30_valid=True,
            lidar_valid=True,
            lidar_online=True,
            x_pulse_seen=True,
            y_pulse_seen=True,
            status=(1 << 1) | (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 10) | (1 << 11),
            protocol="r1_csv_v3",
        ),
        RobotFrame(
            pc_time=2.0,
            pos_x_cm=5.0,
            pos_y_cm=0.0,
            pos_yaw_deg=0.0,
            lidar_x_cm=5.0,
            lidar_y_cm=0.0,
            lidar_yaw_deg=0.0,
            encoder_x_cm=5.0,
            encoder_y_cm=0.0,
            h30_yaw_deg=0.0,
            dt35_1_mm=10.0,
            dt35_2_mm=5151.0,
            dt35_1_valid=True,
            dt35_2_valid=True,
            h30_valid=True,
            lidar_valid=True,
            lidar_online=True,
            x_pulse_seen=True,
            y_pulse_seen=True,
            status=(1 << 1) | (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 10) | (1 << 11),
            protocol="r1_csv_v3",
        ),
    ]
    _write_frames(csv_path, frames)

    report = analyze_rl_logs(config, [log_dir], output_dir=tmp_path / "analysis")

    assert report.aggregate["logs"] == 1
    assert report.aggregate["frames"] == 2
    assert report.aggregate["dt35_floor_hit_suspect_rays"] >= 1
    assert (tmp_path / "analysis" / "rl_data_analysis.md").exists()
    assert (tmp_path / "analysis" / "rl_data_analysis_summary.csv").exists()


def test_rl_data_analysis_reports_h30_and_encoder_calibration_recommendations(tmp_path):
    config = load_config()
    log_dir = tmp_path / "20260622_000001_log"
    csv_path = log_dir / "sensor_data" / "raw_frames.csv"
    frames: list[RobotFrame] = []
    for index in range(40):
        lidar_x = float(index * 2.0)
        encoder_x = float(index * 1.8)
        frames.append(
            RobotFrame(
                pc_time=float(index) * 0.1,
                pos_x_cm=lidar_x,
                pos_y_cm=0.0,
                pos_yaw_deg=0.0,
                lidar_x_cm=lidar_x,
                lidar_y_cm=0.0,
                lidar_yaw_deg=0.0,
                encoder_x_cm=encoder_x,
                encoder_y_cm=0.0,
                h30_yaw_deg=0.4,
                h30_valid=True,
                lidar_valid=True,
                lidar_online=True,
                x_pulse_seen=True,
                y_pulse_seen=True,
                status=(1 << 1) | (1 << 2) | (1 << 3) | (1 << 10) | (1 << 11),
                protocol="r1_csv_v3",
            )
        )
    _write_frames(csv_path, frames)

    report = analyze_rl_logs(config, [log_dir], output_dir=tmp_path / "analysis")
    recommendation = report.aggregate["calibration_recommendations"]

    assert round(float(recommendation["h30"]["initial_bias_deg_median"]), 3) == 0.4
    assert round(float(recommendation["h30"]["suggested_additional_yaw_offset_deg"]), 3) == -0.4
    assert recommendation["h30"]["apply_to_firmware"] is False
    assert round(float(recommendation["encoder"]["window_vector_scale_median"]), 3) == 1.111
    assert recommendation["encoder"]["apply_to_firmware"] is False
