import csv

from locater_map.replay import ReplaySource


def test_synthetic_replay_csv_uses_start_pose_display_transform(tmp_path):
    path = tmp_path / "firmware_like_frames.csv"
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "source_time_ms",
                "pc_time",
                "seq",
                "pos_x_cm",
                "pos_y_cm",
                "pos_yaw_deg",
                "protocol",
            ],
        )
        writer.writeheader()
        writer.writerow({
            "source_time_ms": "0",
            "pc_time": "0",
            "seq": "0",
            "pos_x_cm": "0.0",
            "pos_y_cm": "0.0",
            "pos_yaw_deg": "0.0",
            "protocol": "synthetic_r1_csv_v3",
        })

    replay = ReplaySource(path)

    assert replay.display_ready is False
    frame = replay.step()
    assert frame.pos_x_cm == 0.0
    assert frame.pos_y_cm == 0.0
    assert frame.pos_yaw_deg == 0.0


def test_raw_firmware_replay_csv_still_uses_display_transform(tmp_path):
    path = tmp_path / "raw_frames.csv"
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "source_time_ms",
                "pc_time",
                "seq",
                "pos_x_cm",
                "pos_y_cm",
                "pos_yaw_deg",
                "protocol",
            ],
        )
        writer.writeheader()
        writer.writerow({
            "source_time_ms": "0",
            "pc_time": "0",
            "seq": "0",
            "pos_x_cm": "0.0",
            "pos_y_cm": "0.0",
            "pos_yaw_deg": "0.0",
            "protocol": "r1_csv_v3",
        })

    replay = ReplaySource(path)

    assert replay.display_ready is False
