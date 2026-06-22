import json
from pathlib import Path

from locater_map.dt35_analysis import (
    PoseSpec,
    analyze_observability,
    analyze_dt35_frames,
    analyze_dt35_hits,
    apply_display_policy,
    generate_grid_poses,
    generate_yaw_matrix_poses,
    parse_pose_specs,
    parse_xy_pose_specs,
    summarize_observability,
    summarize_residuals,
    summarize_coverage,
)
from locater_map.data_model import RobotFrame
from locater_map.synthetic_sim import SyntheticConfig, generate_synthetic_frames


def _default_config() -> dict:
    config_path = Path(__file__).resolve().parents[1] / "config" / "default_config.json"
    return json.loads(config_path.read_text(encoding="utf-8"))


def test_parse_dt35_pose_specs():
    poses = parse_pose_specs("1,2,3,a;4,5,6")

    assert poses[0] == PoseSpec(1.0, 2.0, 3.0, "a")
    assert poses[1] == PoseSpec(4.0, 5.0, 6.0, "pose_2")


def test_parse_xy_pose_specs_and_generate_yaw_matrix():
    bases = parse_xy_pose_specs("1,2,a;3,4")
    poses = generate_yaw_matrix_poses(bases, [-90.0, 0.0, 90.0])

    assert bases[0] == PoseSpec(1.0, 2.0, 0.0, "a")
    assert bases[1] == PoseSpec(3.0, 4.0, 0.0, "xy_pose_2")
    assert len(poses) == 6
    assert poses[0] == PoseSpec(1.0, 2.0, -90.0, "a_yaw-90")
    assert poses[2] == PoseSpec(1.0, 2.0, 90.0, "a_yaw90")


def test_dt35_hit_table_marks_top_pole_as_ignore():
    rows = analyze_dt35_hits(_default_config(), [PoseSpec(-360.0, 520.0, 90.0, "ignore")])

    assert rows[0].expected_target.startswith("top_red_long_pole")
    assert rows[0].expected_target_type == "ignore"
    assert rows[0].correction_allowed is False
    assert rows[0].usable_for_correction is False


def test_dt35_horizontal_ray_marks_thick_top_pole_rack_as_ignore():
    rows = analyze_dt35_hits(_default_config(), [PoseSpec(-300.0, 560.0, 0.0, "horizontal_top_rack")])
    targets = {row.sensor_key: row for row in rows}

    left_ray = targets["sensor_1"]
    assert left_ray.expected_target.startswith("top_red_long_pole_rack_ignore")
    assert left_ray.expected_target_type == "ignore"
    assert left_ray.correction_allowed is False
    assert left_ray.usable_for_correction is False


def test_dt35_hit_table_marks_forest_as_correctable_obstacle():
    rows = analyze_dt35_hits(_default_config(), [PoseSpec(-550.0, 50.0, 0.0, "forest")])
    targets = {row.expected_target: row for row in rows}

    forest = targets["red_forest_obstacle_left"]
    assert forest.expected_target_type == "solid_obstacle"
    assert forest.correction_allowed is True
    assert forest.within_range is True
    assert forest.usable_for_correction is True
    assert forest.correction_weight == 0.65


def test_dt35_hit_table_marks_ramp_as_correctable_obstacle():
    rows = analyze_dt35_hits(_default_config(), [PoseSpec(-400.0, -420.0, 0.0, "ramp")])
    targets = {row.expected_target: row for row in rows}

    ramp = targets["red_left_ramp_zone_450h_right"]
    assert ramp.expected_target_type == "solid_obstacle"
    assert ramp.correction_allowed is True
    assert ramp.within_range is True
    assert ramp.usable_for_correction is True
    assert ramp.correction_weight == 0.35


def test_dt35_targets_change_with_h30_yaw_direction():
    config = _default_config()
    north_rows = analyze_dt35_hits(config, [PoseSpec(0.0, 0.0, 0.0, "north")])
    east_rows = analyze_dt35_hits(config, [PoseSpec(0.0, 0.0, 90.0, "east")])
    west_rows = analyze_dt35_hits(config, [PoseSpec(0.0, 0.0, -90.0, "west")])

    north = {row.sensor_key: row for row in north_rows}
    east = {row.sensor_key: row for row in east_rows}
    west = {row.sensor_key: row for row in west_rows}

    assert north["sensor_1"].expected_target == "center_divider_wall"
    assert north["sensor_2"].expected_target == "center_divider_wall"
    assert east["sensor_1"].expected_target == "upper_red_r1_r2_wall"
    assert east["sensor_2"].expected_target == "lower_used_weapon_wall"
    assert west["sensor_1"].expected_target == "lower_used_weapon_wall"
    assert west["sensor_2"].expected_target == "upper_blue_r1_r2_wall"
    assert north["sensor_1"].constraint_axis == "x"
    assert east["sensor_1"].constraint_axis == "y"
    assert west["sensor_1"].constraint_axis == "y"
    assert round(north["sensor_1"].ray_dx, 3) == -1.0
    assert round(east["sensor_1"].ray_dy, 3) == 1.0


def test_dt35_yaw_matrix_explains_what_side_sensors_measure():
    config = _default_config()
    poses = generate_yaw_matrix_poses([PoseSpec(0.0, 0.0, 0.0, "center")], [-90.0, 0.0, 90.0])
    rows = analyze_dt35_hits(config, poses)
    by_pose_sensor = {(row.pose_label, row.sensor_key): row for row in rows}

    assert by_pose_sensor[("center_yaw0", "sensor_1")].constraint_axis == "x"
    assert by_pose_sensor[("center_yaw0", "sensor_2")].constraint_axis == "x"
    assert by_pose_sensor[("center_yaw90", "sensor_1")].constraint_axis == "y"
    assert by_pose_sensor[("center_yaw-90", "sensor_1")].constraint_axis == "y"
    assert round(by_pose_sensor[("center_yaw0", "sensor_1")].correction_dx_per_cm, 3) == 1.0
    assert round(by_pose_sensor[("center_yaw90", "sensor_1")].correction_dy_per_cm, 3) == -1.0


def test_dt35_ramp_uses_top_view_square_not_side_view_length():
    rows = analyze_dt35_hits(_default_config(), [PoseSpec(-400.0, -530.0, 0.0, "below_ramp")])
    targets = {row.sensor_key: row for row in rows}

    assert targets["sensor_1"].expected_target != "red_left_ramp_zone_450h_right"
    assert targets["sensor_2"].expected_target != "red_left_ramp_zone_450h_right"


def test_default_display_policy_keeps_online_lidar_absolute():
    config = _default_config()
    frame = RobotFrame(
        pos_x_cm=236.4,
        pos_y_cm=1.58,
        pos_yaw_deg=0.52,
        lidar_x_cm=236.4,
        lidar_y_cm=1.58,
        lidar_yaw_deg=0.52,
        lidar_valid=True,
        lidar_online=True,
    )

    display = apply_display_policy(config, frame)

    assert display.pos_x_cm == frame.pos_x_cm
    assert display.pos_y_cm == frame.pos_y_cm
    assert display.pos_yaw_deg == frame.pos_yaw_deg
    assert display.lidar_x_cm == frame.lidar_x_cm
    assert display.lidar_y_cm == frame.lidar_y_cm
    assert display.lidar_yaw_deg == frame.lidar_yaw_deg


def test_default_display_policy_offsets_local_pose_when_lidar_offline():
    config = _default_config()
    frame = RobotFrame(pos_x_cm=4.15, pos_y_cm=0.02, pos_yaw_deg=0.33, lidar_online=False)

    display = apply_display_policy(config, frame)
    red_start = config["robot"]["start_pose_red"]

    assert round(display.pos_x_cm, 3) == round(red_start["x_cm"] + frame.pos_x_cm, 3)
    assert round(display.pos_y_cm, 3) == round(red_start["y_cm"] + frame.pos_y_cm, 3)
    assert round(display.pos_yaw_deg, 3) == round(red_start["yaw_deg"] + frame.pos_yaw_deg, 3)


def test_dt35_hit_table_marks_out_of_range_as_not_usable():
    rows = analyze_dt35_hits(_default_config(), [PoseSpec(-400.0, -420.0, 0.0, "ramp")])
    targets = {row.sensor_key: row for row in rows}

    far_wall = targets["sensor_2"]
    assert far_wall.expected_target == "bottom_center_barrier_wall_left"
    assert far_wall.correction_allowed is True
    assert far_wall.within_range is False
    assert far_wall.usable_for_correction is False


def test_dt35_manual_feature_matrix_matches_expected_target_classes():
    config = _default_config()
    rows = analyze_dt35_hits(
        config,
        [
            PoseSpec(-230.0, 330.0, 90.0, "red_upper_wall"),
            PoseSpec(-360.0, 520.0, 90.0, "top_long_pole_ignore"),
            PoseSpec(-550.0, 50.0, 0.0, "red_forest_left_face"),
            PoseSpec(-70.0, 50.0, 0.0, "red_forest_right_face"),
            PoseSpec(-400.0, -420.0, 0.0, "red_ramp_face"),
            PoseSpec(40.0, 0.0, 0.0, "center_divider"),
        ],
    )
    by_pose = {}
    for row in rows:
        by_pose.setdefault(row.pose_label, []).append(row)

    assert _has_target(by_pose["red_upper_wall"], "upper_red_r1_r2_wall", "usable_wall", True)
    assert _has_target(by_pose["top_long_pole_ignore"], "top_red_long_pole", "ignore", False)
    assert _has_target(by_pose["red_forest_left_face"], "red_forest_obstacle_left", "solid_obstacle", True)
    assert _has_target(by_pose["red_forest_right_face"], "red_forest_obstacle_right", "solid_obstacle", True)
    assert _has_target(by_pose["red_ramp_face"], "red_left_ramp_zone_450h_right", "solid_obstacle", True)
    assert _has_target(by_pose["center_divider"], "center_divider_wall", "usable_wall", True)


def test_dt35_lidar_reference_samples_have_interpretable_targets():
    config = _default_config()
    rows = analyze_dt35_hits(
        config,
        [
            PoseSpec(-3.797, 11.322, 0.574, "lidar_origin_sample"),
            PoseSpec(236.403, 1.580, 0.516, "lidar_forest_side_sample"),
            PoseSpec(1.480, -162.786, -1.203, "lidar_bottom_corridor_sample"),
            PoseSpec(154.289, -92.150, -92.188, "lidar_rotated_sample"),
        ],
    )
    by_pose = {}
    for row in rows:
        by_pose.setdefault(row.pose_label, {})[row.sensor_key] = row

    assert by_pose["lidar_origin_sample"]["sensor_1"].expected_target == "center_divider_wall"
    assert by_pose["lidar_origin_sample"]["sensor_2"].expected_target == "center_divider_wall"
    assert by_pose["lidar_origin_sample"]["sensor_1"].usable_for_correction is True
    assert by_pose["lidar_bottom_corridor_sample"]["sensor_1"].expected_target == "center_divider_wall"
    assert by_pose["lidar_bottom_corridor_sample"]["sensor_2"].expected_target == "center_divider_wall"
    assert by_pose["lidar_forest_side_sample"]["sensor_1"].expected_target.startswith("blue_forest_obstacle")
    assert by_pose["lidar_forest_side_sample"]["sensor_1"].usable_for_correction is True
    assert by_pose["lidar_rotated_sample"]["sensor_1"].expected_target.startswith("blue_forest_obstacle")
    assert by_pose["lidar_rotated_sample"]["sensor_1"].constraint_axis == "y"


def test_dt35_grid_summary_counts_usable_and_ignored_rays():
    poses = generate_grid_poses(-400.0, -300.0, 520.0, 520.0, 100.0, [90.0])
    rows = analyze_dt35_hits(_default_config(), poses)
    summary = summarize_coverage(rows)

    assert summary.poses == 2
    assert summary.rays == 4
    assert summary.ignored_rays > 0
    assert summary.usable_rays < summary.rays
    assert summary.risk_counts["ignored_geometry"] > 0
    assert summary.constraint_axis_counts["y"] == 4
    assert summary.sensor_axis_counts["sensor_1:y"] == 2


def test_dt35_observability_reports_current_side_sensors_as_one_dimensional():
    rows = analyze_dt35_hits(_default_config(), [PoseSpec(0.0, 0.0, 0.0, "north")])
    observability = analyze_observability(rows)
    summary = summarize_observability(observability, rows)

    assert len(observability) == 1
    pose = observability[0]
    assert pose.usable_sensor_count == 2
    assert pose.translation_rank == 1
    assert pose.constraint_state == "rank1_x"
    assert summary.one_dim_poses == 1
    assert summary.two_dim_poses == 0


def test_dt35_observability_detects_two_dimensional_perpendicular_sensor_layout():
    config = {
        "map": {"field_width_cm": 200.0, "field_height_cm": 200.0},
        "field_model": {"enabled": True, "use_field_boundary": True},
        "dt35": {
            "sensor_1": {"enabled": True, "offset_x_cm": 0.0, "offset_y_cm": 0.0, "yaw_offset_deg": 0.0, "max_range_cm": 250.0},
            "sensor_2": {"enabled": True, "offset_x_cm": 0.0, "offset_y_cm": 0.0, "yaw_offset_deg": 90.0, "max_range_cm": 250.0},
        },
    }
    rows = analyze_dt35_hits(config, [PoseSpec(0.0, 0.0, 0.0, "perpendicular")])
    observability = analyze_observability(rows)
    summary = summarize_observability(observability, rows)

    assert observability[0].usable_sensor_count == 2
    assert observability[0].translation_rank == 2
    assert observability[0].constraint_state == "rank2_xy"
    assert summary.two_dim_poses == 1


def test_dt35_grid_summary_counts_grazing_filtered_rays():
    config = {
        "map": {"field_width_cm": 200.0, "field_height_cm": 100.0},
        "field_model": {
            "enabled": True,
            "use_field_boundary": False,
            "max_correction_incidence_deg": 75.0,
            "segments": [
                {"name": "top_wall", "target_type": "usable_wall", "x1_cm": -100.0, "y1_cm": 10.0, "x2_cm": 100.0, "y2_cm": 10.0}
            ],
        },
        "dt35": {
            "sensor_1": {"enabled": True, "offset_x_cm": 0.0, "offset_y_cm": 0.0, "yaw_offset_deg": 0.0, "max_range_cm": 250.0},
            "sensor_2": {"enabled": False, "offset_x_cm": 0.0, "offset_y_cm": 0.0, "yaw_offset_deg": 180.0, "max_range_cm": 250.0},
        },
    }
    rows = analyze_dt35_hits(config, [PoseSpec(0.0, 0.0, 80.0, "grazing")])
    summary = summarize_coverage(rows)

    assert summary.grazing_filtered_rays == 1
    assert summary.usable_rays == 0


def test_dt35_corner_hit_is_not_used_for_correction():
    config = {
        "map": {"field_width_cm": 300.0, "field_height_cm": 300.0},
        "field_model": {
            "enabled": True,
            "use_field_boundary": False,
            "corner_ambiguity_cm": 3.0,
            "rectangles": [
                {
                    "name": "box",
                    "target_type": "solid_obstacle",
                    "correction_weight": 1.0,
                    "center_x_cm": 0.0,
                    "center_y_cm": 0.0,
                    "width_cm": 100.0,
                    "height_cm": 100.0,
                }
            ],
        },
        "dt35": {
            "sensor_1": {"enabled": True, "offset_x_cm": 0.0, "offset_y_cm": 0.0, "yaw_offset_deg": 0.0, "max_range_cm": 250.0},
            "sensor_2": {"enabled": False, "offset_x_cm": 0.0, "offset_y_cm": 0.0, "yaw_offset_deg": 180.0, "max_range_cm": 250.0},
        },
    }
    rows = analyze_dt35_hits(config, [PoseSpec(-100.0, -100.0, 45.0, "corner")])
    summary = summarize_coverage(rows)
    sensor_1 = {row.sensor_key: row for row in rows}["sensor_1"]

    assert sensor_1.expected_target.startswith("box_")
    assert sensor_1.corner_ambiguous is True
    assert sensor_1.correction_allowed is False
    assert sensor_1.usable_for_correction is False
    assert summary.corner_ambiguous_rays == 1


def test_dt35_frame_residuals_are_small_for_synthetic_truth():
    config = _default_config()
    frames = generate_synthetic_frames(config, SyntheticConfig(samples=20, path_name="center_divider"))
    rows = analyze_dt35_frames(config, frames, pose_source="lidar", yaw_source="h30", start_policy="off")
    summary = summarize_residuals(rows)

    assert summary.valid_rays > 0
    assert summary.usable_rays > 0
    assert summary.rms_residual_cm is not None
    assert summary.rms_residual_cm < 1.0e-6


def test_dt35_residual_gate_separates_geometry_from_fusion_use():
    config = {
        "map": {"field_width_cm": 200.0, "field_height_cm": 200.0},
        "robot": {"start_pose_policy": "off"},
        "display": {"live_fusion_dt35_residual_gate_cm": 40.0},
        "field_model": {"enabled": True, "use_field_boundary": True, "field_width_cm": 200.0, "field_height_cm": 200.0},
        "dt35": {
            "sensor_1": {"enabled": True, "offset_x_cm": 0.0, "offset_y_cm": 0.0, "yaw_offset_deg": -90.0, "max_range_cm": 250.0},
            "sensor_2": {"enabled": False},
        },
    }
    frame = RobotFrame(
        seq=1,
        lidar_valid=True,
        lidar_x_cm=0.0,
        lidar_y_cm=0.0,
        h30_valid=True,
        h30_yaw_deg=0.0,
        dt35_1_valid=True,
        dt35_1_mm=200.0,
    )

    rows = analyze_dt35_frames(config, [frame], pose_source="lidar", yaw_source="h30", start_policy="off")
    sensor_1 = [row for row in rows if row.sensor_key == "sensor_1"][0]
    summary = summarize_residuals(rows)

    assert sensor_1.expected_target == "field_left"
    assert sensor_1.usable_for_correction is True
    assert sensor_1.residual_within_gate is False
    assert sensor_1.usable_for_fusion is False
    assert summary.usable_rays == 1
    assert summary.fusion_usable_rays == 0
    assert summary.residual_gate_rejected_rays == 1
    assert summary.rms_residual_cm is None


def _has_target(rows, target_prefix: str, target_type: str, usable: bool) -> bool:
    return any(
        row.expected_target.startswith(target_prefix)
        and row.expected_target_type == target_type
        and row.usable_for_correction is usable
        for row in rows
    )
