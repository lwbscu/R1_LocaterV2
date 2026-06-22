from locater_map.calibration_recorder import CalibrationRecorder
from locater_map.data_model import RobotFrame


def test_calibration_summary_compares_lidar_encoder_and_h30(tmp_path):
    recorder = CalibrationRecorder(tmp_path)
    recorder.frames = [
        RobotFrame(
            pos_x_cm=0.0,
            pos_y_cm=0.0,
            pos_yaw_deg=0.0,
            lidar_x_cm=0.0,
            lidar_y_cm=0.0,
            lidar_yaw_deg=0.0,
            encoder_x_cm=0.0,
            encoder_y_cm=0.0,
            h30_yaw_deg=0.0,
            lidar_valid=True,
            h30_valid=True,
            x_pulse_seen=True,
            y_pulse_seen=True,
        ),
        RobotFrame(
            pos_x_cm=100.0,
            pos_y_cm=50.0,
            pos_yaw_deg=10.0,
            lidar_x_cm=100.0,
            lidar_y_cm=50.0,
            lidar_yaw_deg=10.0,
            encoder_x_cm=101.0,
            encoder_y_cm=48.0,
            h30_yaw_deg=10.5,
            lidar_valid=True,
            h30_valid=True,
            x_pulse_seen=True,
            y_pulse_seen=True,
        ),
    ]
    recorder.stats.frames = len(recorder.frames)
    recorder.stats.raw_lines = len(recorder.frames)
    recorder._refresh_valid_counts()

    summary = recorder.analyze()

    assert summary.stats.lidar_valid_frames == 2
    assert summary.stats.h30_valid_frames == 2
    assert summary.stats.encoder_1_seen_frames == 2
    assert round(float(summary.lidar_encoder_delta_error_cm["mean_x"]), 3) == 0.5
    assert round(float(summary.lidar_encoder_delta_error_cm["mean_y"]), 3) == -1.0
    assert round(float(summary.lidar_h30_delta_error_deg["mean"]), 3) == 0.25
    assert summary.lidar_encoder_delta_error_cm["rms_xy"] is not None


def test_calibration_summary_handles_empty_session(tmp_path):
    recorder = CalibrationRecorder(tmp_path)
    summary = recorder.analyze()

    assert summary.stats.frames == 0
    assert summary.ranges["lidar_x_cm"] is None
    assert summary.notes
