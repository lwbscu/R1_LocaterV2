import json
from pathlib import Path

from locater_map.encoder_sim import FirmwareEncoderSimulator
from locater_map.synthetic_sim import SyntheticConfig, generate_synthetic_frames


def _default_config() -> dict:
    config_path = Path(__file__).resolve().parents[1] / "config" / "default_config.json"
    return json.loads(config_path.read_text(encoding="utf-8"))


def test_firmware_encoder_sim_reconstructs_world_delta_with_unit_scale():
    sim = FirmwareEncoderSimulator(initial_x_cm=0.0, initial_y_cm=0.0)

    sample = sim.sample_for_truth_delta(12.5, -4.0, 17.0)

    assert abs(sample.x_cm - 12.5) < 0.01
    assert abs(sample.y_cm + 4.0) < 0.01
    assert sample.x_delta_count != 0
    assert sample.y_delta_count != 0


def test_synthetic_encoder_uses_firmware_rotation_compensation():
    frames = generate_synthetic_frames(
        _default_config(),
        SyntheticConfig(samples=120, path_name="yaw_sweep"),
    )
    max_xy_error = max(
        ((frame.encoder_x_cm - frame.lidar_x_cm) ** 2 + (frame.encoder_y_cm - frame.lidar_y_cm) ** 2) ** 0.5
        for frame in frames
    )

    assert max_xy_error < 0.05
    assert any(frame.x_delta_count != 0 for frame in frames[1:])
    assert any(frame.y_delta_count != 0 for frame in frames[1:])
    assert frames[-1].x_pulse_seen is True
    assert frames[-1].y_pulse_seen is True


def test_synthetic_encoder_status_bits_wait_for_real_pulses():
    frames = generate_synthetic_frames(
        _default_config(),
        SyntheticConfig(samples=3, path_name="static_start"),
    )

    assert frames[0].status & (1 << 10) == 0
    assert frames[0].status & (1 << 11) == 0
    assert frames[-1].x_pulse_seen is False
    assert frames[-1].y_pulse_seen is False
