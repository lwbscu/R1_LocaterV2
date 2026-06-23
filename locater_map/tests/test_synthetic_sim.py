from locater_map.synthetic_sim import SyntheticConfig, generate_synthetic_frames
from locater_map.dt35_analysis import analyze_dt35_frames, summarize_residuals


def test_synthetic_static_start_generates_valid_side_dt35():
    config = {
        "map": {"field_width_cm": 200.0, "field_height_cm": 100.0},
        "robot": {"start_pose_policy": "off"},
        "field_model": {"enabled": True, "use_field_boundary": True, "field_width_cm": 200.0, "field_height_cm": 100.0},
        "dt35": {
            "sensor_1": {"enabled": True, "offset_x_cm": -40.0, "offset_y_cm": 0.0, "yaw_offset_deg": -90.0, "max_range_cm": 250.0},
            "sensor_2": {"enabled": True, "offset_x_cm": 40.0, "offset_y_cm": 0.0, "yaw_offset_deg": 90.0, "max_range_cm": 250.0},
        },
    }

    frames = generate_synthetic_frames(config, SyntheticConfig(samples=3, path_name="static_start"))

    assert len(frames) == 3
    assert frames[0].dt35_1_valid is True
    assert frames[0].dt35_2_valid is True
    assert round(frames[0].dt35_1_mm, 3) == 600.0
    assert round(frames[0].dt35_2_mm, 3) == 600.0
    assert frames[0].h30_valid is True
    assert frames[0].lidar_valid is True


def test_synthetic_ignore_target_does_not_emit_dt35_valid():
    config = {
        "map": {"field_width_cm": 200.0, "field_height_cm": 100.0},
        "robot": {"start_pose_policy": "off"},
        "field_model": {
            "enabled": True,
            "use_field_boundary": True,
            "field_width_cm": 200.0,
            "field_height_cm": 100.0,
            "segments": [{"name": "pole_gap", "target_type": "ignore", "x1_cm": -20.0, "y1_cm": -20.0, "x2_cm": -20.0, "y2_cm": 20.0}],
        },
        "dt35": {
            "sensor_1": {"enabled": True, "offset_x_cm": 0.0, "offset_y_cm": 0.0, "yaw_offset_deg": -90.0, "max_range_cm": 250.0},
            "sensor_2": {"enabled": False},
        },
    }

    frames = generate_synthetic_frames(config, SyntheticConfig(samples=2, path_name="static_start"))

    assert frames[0].dt35_1_valid is False
    assert frames[0].dt35_1_mm == 0.0


def test_default_forest_and_ramp_paths_exercise_dt35_obstacles():
    import json
    from pathlib import Path

    config_path = Path(__file__).resolve().parents[1] / "config" / "default_config.json"
    config = json.loads(config_path.read_text(encoding="utf-8"))

    forest = generate_synthetic_frames(config, SyntheticConfig(samples=20, path_name="forest_side"))
    ramp = generate_synthetic_frames(config, SyntheticConfig(samples=20, path_name="ramp_side"))

    assert sum(frame.dt35_1_valid for frame in forest) > 0
    assert sum(frame.dt35_2_valid for frame in forest) > 0
    assert sum(frame.dt35_1_valid for frame in ramp) > 0


def test_field_patrol_exercises_multiple_dt35_target_types():
    import json
    from pathlib import Path

    config_path = Path(__file__).resolve().parents[1] / "config" / "default_config.json"
    config = json.loads(config_path.read_text(encoding="utf-8"))

    frames = generate_synthetic_frames(config, SyntheticConfig(samples=180, path_name="field_patrol"))
    rows = analyze_dt35_frames(config, frames, pose_source="lidar", yaw_source="h30", start_policy="off")
    summary = summarize_residuals(rows)

    assert any(frame.dt35_1_valid for frame in frames)
    assert any(frame.dt35_2_valid for frame in frames)
    assert summary.usable_rays > 0
    assert "solid_obstacle" in summary.target_type_counts
    assert "usable_wall" in summary.target_type_counts


def test_left_forest_loop_is_10hz_capable_and_returns_to_start():
    import json
    from pathlib import Path

    config_path = Path(__file__).resolve().parents[1] / "config" / "default_config.json"
    config = json.loads(config_path.read_text(encoding="utf-8"))

    frames = generate_synthetic_frames(
        config,
        SyntheticConfig(samples=101, path_name="left_forest_loop", sample_period_s=0.1),
    )
    rows = analyze_dt35_frames(config, frames, pose_source="lidar", yaw_source="h30", start_policy="off")
    summary = summarize_residuals(rows)

    assert frames[1].source_time_ms - frames[0].source_time_ms == 100
    assert round(frames[0].lidar_x_cm, 3) == -552.5
    assert round(frames[0].lidar_y_cm, 3) == 549.0
    assert round(frames[0].lidar_yaw_deg, 1) == 0.0
    assert round(frames[-1].lidar_x_cm, 3) == -552.5
    assert round(frames[-1].lidar_y_cm, 3) == 549.0
    assert round(frames[-1].lidar_yaw_deg, 1) == 0.0
    yaw_values = {round(frame.lidar_yaw_deg / 10.0) * 10 for frame in frames}
    assert 180 in yaw_values or -180 in yaw_values
    assert 90 in yaw_values
    assert 0 in yaw_values
    assert -90 in yaw_values
    assert any(row.expected_target.startswith("red_forest_obstacle") for row in rows)
    assert summary.valid_rays > 0
    assert summary.usable_rays > 0
    assert summary.rms_residual_cm is not None
    assert summary.rms_residual_cm < 1.0e-6


def test_left_forest_pid_wiggle_keeps_start_end_and_changes_mid_path():
    import json
    from pathlib import Path

    config_path = Path(__file__).resolve().parents[1] / "config" / "default_config.json"
    config = json.loads(config_path.read_text(encoding="utf-8"))

    plain = generate_synthetic_frames(config, SyntheticConfig(samples=121, path_name="left_forest_loop", sample_period_s=0.1))
    wiggle = generate_synthetic_frames(
        config,
        SyntheticConfig(
            samples=121,
            path_name="left_forest_loop",
            sample_period_s=0.1,
            path_wiggle_cm=6.0,
            path_wiggle_cycles=9.0,
            path_yaw_wiggle_deg=2.0,
        ),
    )

    assert round(wiggle[0].lidar_x_cm, 3) == round(plain[0].lidar_x_cm, 3)
    assert round(wiggle[0].lidar_y_cm, 3) == round(plain[0].lidar_y_cm, 3)
    assert round(wiggle[-1].lidar_x_cm, 3) == round(plain[-1].lidar_x_cm, 3)
    assert round(wiggle[-1].lidar_y_cm, 3) == round(plain[-1].lidar_y_cm, 3)
    assert max(abs(a.lidar_x_cm - b.lidar_x_cm) + abs(a.lidar_y_cm - b.lidar_y_cm) for a, b in zip(plain, wiggle)) > 2.0


def test_dt35_occlusion_and_dropout_create_screenable_disturbances():
    import json
    from pathlib import Path

    config_path = Path(__file__).resolve().parents[1] / "config" / "default_config.json"
    config = json.loads(config_path.read_text(encoding="utf-8"))
    frames = generate_synthetic_frames(
        config,
        SyntheticConfig(
            samples=200,
            path_name="left_forest_loop",
            dt35_dropout_rate=0.04,
            dt35_occlusion_rate=0.08,
            seed=19,
        ),
    )

    valid_distances = [frame.dt35_1_mm for frame in frames if frame.dt35_1_valid]
    invalid_count = sum(1 for frame in frames if not frame.dt35_1_valid or not frame.dt35_2_valid)
    assert invalid_count > 0
    assert any(180.0 <= distance <= 950.0 for distance in valid_distances)


def test_random_patrol_depends_on_seed_and_has_dt35_observations():
    import json
    from pathlib import Path

    config_path = Path(__file__).resolve().parents[1] / "config" / "default_config.json"
    config = json.loads(config_path.read_text(encoding="utf-8"))

    frames_a = generate_synthetic_frames(config, SyntheticConfig(samples=120, path_name="random_patrol", seed=11))
    frames_b = generate_synthetic_frames(config, SyntheticConfig(samples=120, path_name="random_patrol", seed=12))

    assert frames_a[20].lidar_x_cm != frames_b[20].lidar_x_cm
    assert any(frame.dt35_1_valid or frame.dt35_2_valid for frame in frames_a)


def test_default_synthetic_paths_stay_inside_field_coordinates():
    import json
    from pathlib import Path

    config_path = Path(__file__).resolve().parents[1] / "config" / "default_config.json"
    config = json.loads(config_path.read_text(encoding="utf-8"))
    half_w = float(config["map"]["field_width_cm"]) * 0.5
    half_h = float(config["map"]["field_height_cm"]) * 0.5

    for path_name in (
        "top_corridor",
        "forest_side",
        "left_forest_loop",
        "ramp_side",
        "center_divider",
        "yaw_sweep",
        "start_corner_yaw_sweep",
        "field_patrol",
        "random_patrol",
    ):
        frames = generate_synthetic_frames(config, SyntheticConfig(samples=120, path_name=path_name, seed=21))
        assert all(-half_w <= frame.lidar_x_cm <= half_w for frame in frames), path_name
        assert all(-half_h <= frame.lidar_y_cm <= half_h for frame in frames), path_name
