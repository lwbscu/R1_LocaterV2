from __future__ import annotations

import csv

from locater_map.config_loader import load_config
from locater_map.data_model import RobotFrame
from locater_map.dt35_mount_hypothesis import analyze_dt35_mount_hypotheses


def _write_frames(path, frames: list[RobotFrame]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=RobotFrame.field_names())
        writer.writeheader()
        for frame in frames:
            writer.writerow(frame.to_row())


def test_dt35_mount_hypothesis_prefers_outward_start_observation(tmp_path):
    config = load_config()
    config["dt35"]["sensor_1"]["offset_x_cm"] = 40.4
    config["dt35"]["sensor_1"]["yaw_offset_deg"] = -90.0
    config["dt35"]["sensor_2"]["offset_x_cm"] = -40.4
    config["dt35"]["sensor_2"]["yaw_offset_deg"] = 90.0
    log_dir = tmp_path / "20260622_000000_log"
    csv_path = log_dir / "sensor_data" / "raw_frames.csv"
    status = (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 10) | (1 << 11)
    frames = [
        RobotFrame(
            seq=i,
            pc_time=float(i) * 0.02,
            pos_x_cm=0.0,
            pos_y_cm=0.0,
            pos_yaw_deg=0.0,
            lidar_x_cm=0.0,
            lidar_y_cm=0.0,
            lidar_yaw_deg=0.0,
            encoder_x_cm=0.0,
            encoder_y_cm=0.0,
            h30_yaw_deg=0.0,
            h30_valid=True,
            lidar_valid=True,
            lidar_online=True,
            dt35_1_valid=True,
            dt35_2_valid=True,
            dt35_1_mm=146.0,
            dt35_2_mm=5121.0,
            x_pulse_seen=True,
            y_pulse_seen=True,
            status=status,
            protocol="r1_csv_v3",
        )
        for i in range(6)
    ]
    _write_frames(csv_path, frames)

    report = analyze_dt35_mount_hypotheses(config, [log_dir], output_dir=tmp_path / "analysis")

    assert report.recommended_variant == "outward_data_supported"
    assert report.scores[0].score > report.scores[1].score
    assert (tmp_path / "analysis" / "dt35_mount_hypothesis.md").exists()
