import csv
import json
from pathlib import Path

from locater_map.data_model import RobotFrame
from locater_map.fusion_model import FusionConfig, LiveFusionFilter, simulate_fusion, write_frames_csv
from locater_map.synthetic_sim import SyntheticConfig, generate_synthetic_frames


def test_default_fusion_trusts_h30_yaw_and_dt35_translation():
    cfg = FusionConfig()

    assert cfg.dt35_gain == 1.0
    assert cfg.dt35_yaw_gain == 0.0
    assert cfg.dt35_correct_lidar_frames is False
    assert cfg.dt35_max_translation_step_cm >= 10.0
    assert cfg.dt35_damping <= 0.1


def test_lidar_anchor_does_not_override_valid_h30_yaw():
    frames = [
        RobotFrame(
            lidar_x_cm=0.0,
            lidar_y_cm=0.0,
            lidar_yaw_deg=30.0,
            encoder_x_cm=0.0,
            encoder_y_cm=0.0,
            h30_yaw_deg=0.0,
            h30_valid=True,
            lidar_valid=True,
        ),
        RobotFrame(
            lidar_x_cm=10.0,
            lidar_y_cm=0.0,
            lidar_yaw_deg=35.0,
            encoder_x_cm=10.0,
            encoder_y_cm=0.0,
            h30_yaw_deg=5.0,
            h30_valid=True,
            lidar_valid=True,
        ),
    ]

    result = simulate_fusion(frames, FusionConfig(lidar_stride=1, lidar_gain=1.0, dt35_gain=0.0))

    assert result.frames[0].pos_yaw_deg == 0.0
    assert result.frames[1].pos_yaw_deg == 5.0


def test_lidar_first_fusion_interpolates_with_encoder_and_h30():
    frames = [
        RobotFrame(
            lidar_x_cm=0.0,
            lidar_y_cm=0.0,
            lidar_yaw_deg=0.0,
            encoder_x_cm=0.0,
            encoder_y_cm=0.0,
            h30_yaw_deg=0.0,
            h30_valid=True,
            lidar_valid=True,
        ),
        RobotFrame(
            lidar_x_cm=10.0,
            lidar_y_cm=0.0,
            lidar_yaw_deg=5.0,
            encoder_x_cm=9.0,
            encoder_y_cm=1.0,
            h30_yaw_deg=4.0,
            h30_valid=True,
            lidar_valid=True,
        ),
        RobotFrame(
            lidar_x_cm=20.0,
            lidar_y_cm=0.0,
            lidar_yaw_deg=10.0,
            encoder_x_cm=20.0,
            encoder_y_cm=0.0,
            h30_yaw_deg=10.0,
            h30_valid=True,
            lidar_valid=True,
        ),
    ]

    result = simulate_fusion(frames, FusionConfig(lidar_stride=3, lidar_gain=1.0))

    assert len(result.frames) == 3
    assert result.frames[0].pos_x_cm == 0.0
    assert result.frames[1].pos_x_cm == 9.0
    assert result.frames[1].pos_y_cm == 1.0
    assert result.frames[1].pos_yaw_deg == 4.0
    assert result.metrics.lidar_used_frames == 1
    assert result.metrics.lidar_holdout_frames == 2
    assert result.metrics.rms_xy_cm is not None


def test_write_fusion_frames_are_replay_compatible(tmp_path):
    out = tmp_path / "sim.csv"
    write_frames_csv(out, [RobotFrame(pos_x_cm=1.0, pos_y_cm=2.0, pos_yaw_deg=3.0)])

    with out.open("r", encoding="utf-8", newline="") as f:
        rows = list(csv.DictReader(f))

    assert rows[0]["pos_x_cm"] == "1.0"
    assert rows[0]["pos_y_cm"] == "2.0"


def test_dt35_reduces_encoder_translation_drift_when_yaw_is_trusted():
    config_path = Path(__file__).resolve().parents[1] / "config" / "default_config.json"
    config = json.loads(config_path.read_text(encoding="utf-8"))
    frames = generate_synthetic_frames(
        config,
        SyntheticConfig(samples=160, path_name="top_corridor", encoder_x_scale=0.95, encoder_y_scale=1.0),
    )

    no_dt35 = simulate_fusion(frames, FusionConfig(lidar_stride=25, lidar_gain=1.0, dt35_gain=0.0), config)
    with_dt35 = simulate_fusion(frames, FusionConfig(lidar_stride=25, lidar_gain=1.0, dt35_gain=0.4, dt35_yaw_gain=0.0), config)

    assert no_dt35.metrics.rms_xy_cm is not None
    assert with_dt35.metrics.rms_xy_cm is not None
    assert with_dt35.metrics.rms_xy_cm < no_dt35.metrics.rms_xy_cm


def test_accurate_dt35_improves_ramp_side_path():
    config_path = Path(__file__).resolve().parents[1] / "config" / "default_config.json"
    config = json.loads(config_path.read_text(encoding="utf-8"))
    frames = generate_synthetic_frames(
        config,
        SyntheticConfig(samples=160, path_name="ramp_side", encoder_x_scale=0.97, encoder_y_scale=1.02, dt35_noise_mm=0.0),
    )

    no_dt35 = simulate_fusion(frames, FusionConfig(lidar_stride=25, lidar_gain=1.0, dt35_gain=0.0), config)
    with_dt35 = simulate_fusion(frames, FusionConfig(lidar_stride=25, lidar_gain=1.0, dt35_gain=1.0, dt35_yaw_gain=0.0), config)

    assert no_dt35.metrics.rms_xy_cm is not None
    assert with_dt35.metrics.rms_xy_cm is not None
    assert with_dt35.metrics.rms_xy_cm < no_dt35.metrics.rms_xy_cm


def test_accurate_dt35_improves_key_field_paths_with_h30_yaw():
    config_path = Path(__file__).resolve().parents[1] / "config" / "default_config.json"
    config = json.loads(config_path.read_text(encoding="utf-8"))

    for path_name in ("top_corridor", "forest_side", "ramp_side", "center_divider", "field_patrol"):
        frames = generate_synthetic_frames(
            config,
            SyntheticConfig(samples=180, path_name=path_name, encoder_x_scale=0.97, encoder_y_scale=1.02, dt35_noise_mm=0.0),
        )
        no_dt35 = simulate_fusion(frames, FusionConfig(lidar_stride=25, lidar_gain=1.0, dt35_gain=0.0), config)
        with_dt35 = simulate_fusion(frames, FusionConfig(lidar_stride=25, lidar_gain=1.0, dt35_gain=1.0, dt35_yaw_gain=0.0), config)

        assert no_dt35.metrics.rms_xy_cm is not None
        assert with_dt35.metrics.rms_xy_cm is not None
        assert with_dt35.metrics.rms_xy_cm < no_dt35.metrics.rms_xy_cm, path_name


def test_lidar_anchors_learn_encoder_scale_for_high_rate_interpolation():
    config_path = Path(__file__).resolve().parents[1] / "config" / "default_config.json"
    config = json.loads(config_path.read_text(encoding="utf-8"))
    frames = generate_synthetic_frames(
        config,
        SyntheticConfig(samples=240, path_name="field_patrol", encoder_x_scale=0.97, encoder_y_scale=1.03, dt35_noise_mm=0.0),
    )

    fixed_scale = simulate_fusion(
        frames,
        FusionConfig(lidar_stride=25, dt35_gain=0.0, encoder_scale_learning=False),
        config,
    )
    learned_scale = simulate_fusion(
        frames,
        FusionConfig(lidar_stride=25, dt35_gain=0.0, encoder_scale_learning=True),
        config,
    )
    learned_scale_with_dt35 = simulate_fusion(
        frames,
        FusionConfig(lidar_stride=25, dt35_gain=1.0, encoder_scale_learning=True),
        config,
    )

    assert fixed_scale.metrics.rms_xy_cm is not None
    assert learned_scale.metrics.rms_xy_cm is not None
    assert learned_scale_with_dt35.metrics.rms_xy_cm is not None
    assert learned_scale.metrics.rms_xy_cm < fixed_scale.metrics.rms_xy_cm
    assert learned_scale_with_dt35.metrics.rms_xy_cm < learned_scale.metrics.rms_xy_cm


def test_dt35_translation_correction_keeps_h30_yaw_fixed():
    config = {
        "map": {"field_width_cm": 200.0, "field_height_cm": 200.0},
        "robot": {"start_pose_policy": "off"},
        "field_model": {"enabled": True, "use_field_boundary": True, "field_width_cm": 200.0, "field_height_cm": 200.0},
        "dt35": {
            "sensor_1": {"enabled": True, "offset_x_cm": 0.0, "offset_y_cm": 0.0, "yaw_offset_deg": -90.0, "max_range_cm": 250.0},
            "sensor_2": {"enabled": False},
        },
    }
    frame = RobotFrame(
        pos_x_cm=10.0,
        pos_y_cm=0.0,
        pos_yaw_deg=0.0,
        encoder_x_cm=10.0,
        encoder_y_cm=0.0,
        h30_yaw_deg=0.0,
        dt35_1_mm=1000.0,
        dt35_1_valid=True,
    )

    result = simulate_fusion(
        [frame],
        FusionConfig(
            lidar_stride=10,
            lidar_gain=1.0,
            dt35_gain=1.0,
            dt35_yaw_gain=0.0,
            dt35_max_translation_step_cm=20.0,
            dt35_damping=1.0e-6,
        ),
        config,
    )

    assert abs(result.frames[0].pos_x_cm) < 0.1
    assert result.frames[0].pos_yaw_deg == 0.0


def test_dt35_does_not_override_current_lidar_anchor_by_default():
    config = {
        "map": {"field_width_cm": 200.0, "field_height_cm": 200.0},
        "robot": {"start_pose_policy": "off"},
        "field_model": {"enabled": True, "use_field_boundary": True, "field_width_cm": 200.0, "field_height_cm": 200.0},
        "dt35": {
            "sensor_1": {"enabled": True, "offset_x_cm": 0.0, "offset_y_cm": 0.0, "yaw_offset_deg": -90.0, "max_range_cm": 250.0},
            "sensor_2": {"enabled": False},
        },
    }
    frame = RobotFrame(
        lidar_x_cm=0.0,
        lidar_y_cm=0.0,
        lidar_yaw_deg=0.0,
        lidar_valid=True,
        pos_x_cm=30.0,
        pos_y_cm=0.0,
        encoder_x_cm=30.0,
        encoder_y_cm=0.0,
        h30_yaw_deg=0.0,
        h30_valid=True,
        dt35_1_mm=800.0,
        dt35_1_valid=True,
    )

    result = simulate_fusion(
        [frame],
        FusionConfig(
            lidar_stride=1,
            lidar_gain=1.0,
            dt35_gain=1.0,
            dt35_yaw_gain=0.0,
            dt35_max_translation_step_cm=50.0,
            dt35_damping=1.0e-6,
        ),
        config,
    )

    assert result.frames[0].pos_x_cm == 0.0
    assert result.frames[0].pos_y_cm == 0.0


def test_dt35_can_correct_current_lidar_anchor_when_explicitly_enabled():
    config = {
        "map": {"field_width_cm": 200.0, "field_height_cm": 200.0},
        "robot": {"start_pose_policy": "off"},
        "field_model": {"enabled": True, "use_field_boundary": True, "field_width_cm": 200.0, "field_height_cm": 200.0},
        "dt35": {
            "sensor_1": {"enabled": True, "offset_x_cm": 0.0, "offset_y_cm": 0.0, "yaw_offset_deg": -90.0, "max_range_cm": 250.0},
            "sensor_2": {"enabled": False},
        },
    }
    frame = RobotFrame(
        lidar_x_cm=0.0,
        lidar_y_cm=0.0,
        lidar_yaw_deg=0.0,
        lidar_valid=True,
        encoder_x_cm=0.0,
        encoder_y_cm=0.0,
        h30_yaw_deg=0.0,
        h30_valid=True,
        dt35_1_mm=800.0,
        dt35_1_valid=True,
    )

    result = simulate_fusion(
        [frame],
        FusionConfig(
            lidar_stride=1,
            lidar_gain=1.0,
            dt35_gain=1.0,
            dt35_yaw_gain=0.0,
            dt35_correct_lidar_frames=True,
            dt35_max_translation_step_cm=50.0,
            dt35_damping=1.0e-6,
        ),
        config,
    )

    assert result.frames[0].pos_x_cm < -15.0
    assert result.frames[0].pos_yaw_deg == 0.0


def test_live_fusion_filter_corrects_display_pose_with_dt35():
    config = {
        "map": {"field_width_cm": 200.0, "field_height_cm": 200.0},
        "robot": {"start_pose_policy": "off"},
        "field_model": {"enabled": True, "use_field_boundary": True, "field_width_cm": 200.0, "field_height_cm": 200.0},
        "dt35": {
            "sensor_1": {"enabled": True, "offset_x_cm": 0.0, "offset_y_cm": 0.0, "yaw_offset_deg": -90.0, "max_range_cm": 250.0},
            "sensor_2": {"enabled": False},
        },
    }
    filt = LiveFusionFilter(
        FusionConfig(
            lidar_stride=10,
            lidar_gain=1.0,
            dt35_gain=1.0,
            dt35_yaw_gain=0.0,
            dt35_damping=1.0e-6,
            dt35_max_translation_step_cm=20.0,
        ),
        config,
        use_start_transform=False,
    )
    frame = RobotFrame(
        seq=1,
        pos_x_cm=10.0,
        pos_y_cm=0.0,
        pos_yaw_deg=0.0,
        encoder_x_cm=10.0,
        encoder_y_cm=0.0,
        h30_yaw_deg=0.0,
        h30_valid=True,
        dt35_1_mm=1000.0,
        dt35_1_valid=True,
    )

    fused = filt.process(frame)

    assert abs(fused.pos_x_cm) < 0.1
    assert fused.pos_yaw_deg == 0.0
    assert fused.status & (1 << 6)


def test_live_fusion_filter_resets_on_sequence_rewind():
    filt = LiveFusionFilter(FusionConfig(dt35_gain=0.0), {"field_model": {"enabled": False}}, use_start_transform=False)
    first = filt.process(RobotFrame(seq=10, pos_x_cm=100.0, encoder_x_cm=100.0))
    second = filt.process(RobotFrame(seq=1, pos_x_cm=5.0, encoder_x_cm=5.0))

    assert first.pos_x_cm == 100.0
    assert second.pos_x_cm == 5.0
