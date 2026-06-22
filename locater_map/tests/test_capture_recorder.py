import json
import re

from locater_map.capture_recorder import CaptureRecorder, capture_timestamp
from locater_map.data_model import RobotFrame


def test_capture_timestamp_is_file_safe_datetime():
    stamp = capture_timestamp()

    assert re.fullmatch(r"\d{8}_\d{6}", stamp)


def test_capture_recorder_writes_rl_data_layout_and_metadata(tmp_path):
    recorder = CaptureRecorder(
        tmp_path,
        {"display": {"capture_data_interval_s": 60.0, "capture_map_snapshot_interval_s": 1.0}},
    )
    session = recorder.start()
    raw = RobotFrame(
        seq=1,
        lidar_x_cm=10.0,
        lidar_y_cm=20.0,
        lidar_yaw_deg=30.0,
        encoder_x_cm=1.0,
        encoder_y_cm=2.0,
        h30_yaw_deg=3.0,
        dt35_1_mm=400.0,
        dt35_2_mm=500.0,
        status=0x0C7F,
        raw_line="10,20,30,10,20,30,1,2,3,400,500,3199",
    )
    display = RobotFrame(seq=1, pos_x_cm=10.0, pos_y_cm=20.0, pos_yaw_deg=3.0)

    recorder.raw_line(raw.raw_line)
    recorder.frame_pair(raw, display)
    recorder.frame_pair(raw, display)
    recorder.frame_pair(raw, display)
    path = recorder.screenshot_path()
    assert path is not None
    path.write_text("fake png", encoding="utf-8")
    summary = recorder.stop()

    sensor_dir = session / "sensor_data"
    png_dir = session / "png"

    assert summary.raw_frame_count == 4
    assert summary.display_frame_count == 4
    assert summary.raw_line_count == 1
    assert summary.screenshot_count == 1
    assert session.parent.name == "RL_data"
    assert session.name.endswith("_log")
    assert re.fullmatch(r"\d{8}_\d{6}(?:_\d{2})?_log", session.name)
    assert sensor_dir.is_dir()
    assert png_dir.is_dir()
    assert (sensor_dir / "raw_frames.csv").exists()
    assert (sensor_dir / "display_frames.csv").exists()
    raw_rows = (sensor_dir / "raw_frames.csv").read_text(encoding="utf-8").splitlines()
    assert raw_rows[0].startswith("capture_sample,capture_elapsed_ms,capture_wall_time,")
    assert raw_rows[1].startswith("1,")
    assert raw_rows[2].startswith("2,")
    assert raw_rows[3].startswith("3,")
    assert raw_rows[4].startswith("4,")
    assert path.parent == png_dir
    assert path.name.startswith("map_sample_000003_")
    assert "_t" in path.name
    assert path.name.endswith(".png")
    assert (sensor_dir / "raw_serial.log").read_text(encoding="utf-8").strip() == raw.raw_line
    metadata = json.loads((session / "metadata.json").read_text(encoding="utf-8"))
    assert metadata["raw_frame_count"] == 4
    assert metadata["display_frame_count"] == 4
    assert metadata["screenshot_count"] == 1
    assert metadata["files"]["sensor_data_dir"] == "sensor_data"
    assert metadata["files"]["png_dir"] == "png"
    assert metadata["files"]["raw_frames_csv"] == "sensor_data/raw_frames.csv"
    assert metadata["files"]["display_frames_csv"] == "sensor_data/display_frames.csv"
    assert metadata["files"]["raw_serial_log"] == "sensor_data/raw_serial.log"
    assert metadata["files"]["screenshots"] == "png"
    assert metadata["capture"]["data_interval_s"] == 60.0
    assert metadata["capture"]["map_snapshot_interval_s"] == 1.0
