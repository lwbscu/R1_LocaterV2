import json
from pathlib import Path

from locater_map.data_model import RobotFrame
from locater_map.utils_transform import (
    dt35_ray,
    dt35_yaw_from_frame,
    expected_dt35_hit,
    robot_local_to_world,
    transform_frame,
    transform_xy_yaw,
)


def test_transform_xy_yaw():
    cfg = {
        "data_x_sign": -1,
        "data_y_sign": 1,
        "data_yaw_sign": -1,
        "data_x_offset_cm": 10,
        "data_y_offset_cm": -5,
        "data_yaw_offset_deg": 90,
    }
    assert transform_xy_yaw(2, 3, 30, cfg) == (8, -2, 60)


def test_transform_frame_applies_all_pose_groups():
    frame = RobotFrame(pos_x_cm=1, pos_y_cm=2, pos_yaw_deg=3, lidar_x_cm=4, lidar_y_cm=5, lidar_yaw_deg=6)
    out = transform_frame(frame, {"data_x_offset_cm": 10, "data_y_offset_cm": 20, "data_yaw_offset_deg": 30})
    assert out.pos_x_cm == 11
    assert out.pos_y_cm == 22
    assert out.pos_yaw_deg == 33
    assert out.lidar_x_cm == 14
    assert out.lidar_y_cm == 25
    assert out.lidar_yaw_deg == 36


def test_dt35_ray_formula():
    cfg = {"name": "s1", "enabled": True, "offset_x_cm": 10, "offset_y_cm": 0, "yaw_offset_deg": 90, "max_range_cm": 100}
    ray = dt35_ray(0, 0, 0, cfg, 500)
    assert round(float(ray["sensor_x_cm"]), 3) == 10
    assert round(float(ray["sensor_y_cm"]), 3) == 0
    assert round(float(ray["hit_x_cm"]), 3) == 60
    assert round(float(ray["hit_y_cm"]), 3) == 0
    assert ray["valid"] is True


def test_dt35_yaw_uses_h30_when_attitude_is_valid():
    assert dt35_yaw_from_frame(RobotFrame(pos_yaw_deg=30.0, h30_yaw_deg=5.0, h30_valid=True)) == 5.0
    assert dt35_yaw_from_frame(RobotFrame(pos_yaw_deg=30.0, h30_yaw_deg=6.0, h30_has_attitude=True)) == 6.0
    assert dt35_yaw_from_frame(RobotFrame(pos_yaw_deg=30.0, h30_yaw_deg=5.0)) == 30.0


def test_robot_local_to_world_yaw_front_definition():
    assert tuple(round(v, 3) for v in robot_local_to_world(0, 0, 0, 40.4, -3.3)) == (40.4, -3.3)
    assert tuple(round(v, 3) for v in robot_local_to_world(0, 0, 90, 40.4, -3.3)) == (-3.3, -40.4)


def test_dt35_v2_mounting_left_and_right_rays():
    s1 = {"enabled": True, "offset_x_cm": -40.4, "offset_y_cm": -3.3, "yaw_offset_deg": -90, "max_range_cm": 1000}
    s2 = {"enabled": True, "offset_x_cm": 40.4, "offset_y_cm": -3.3, "yaw_offset_deg": 90, "max_range_cm": 1000}
    r1 = dt35_ray(0, 0, 0, s1, 1000)
    r2 = dt35_ray(0, 0, 0, s2, 1000)
    assert round(float(r1["sensor_x_cm"]), 3) == -40.4
    assert round(float(r1["sensor_y_cm"]), 3) == -3.3
    assert round(float(r1["hit_x_cm"]), 3) == -140.4
    assert round(float(r1["hit_y_cm"]), 3) == -3.3
    assert round(float(r2["sensor_x_cm"]), 3) == 40.4
    assert round(float(r2["sensor_y_cm"]), 3) == -3.3
    assert round(float(r2["hit_x_cm"]), 3) == 140.4
    assert round(float(r2["hit_y_cm"]), 3) == -3.3


def test_dt35_expected_hit_with_field_boundary():
    model = {"enabled": True, "use_field_boundary": True, "field_width_cm": 200, "field_height_cm": 100}
    hit = expected_dt35_hit(40, 0, -90, model)
    assert hit is not None
    assert hit["name"] == "field_left"
    assert hit["target_type"] == "usable_wall"
    assert hit["correction_allowed"] == "1"
    assert round(float(hit["incidence_deg"]), 3) == 0.0
    assert round(float(hit["incidence_scale"]), 3) == 1.0
    assert round(float(hit["distance_cm"]), 3) == 140
    assert round(float(hit["hit_x_cm"]), 3) == -100
    assert round(float(hit["hit_y_cm"]), 3) == 0


def test_dt35_blocker_stops_before_wall_and_disables_correction():
    model = {
        "enabled": True,
        "use_field_boundary": True,
        "field_width_cm": 200,
        "field_height_cm": 100,
        "rectangles": [
            {"name": "forest", "target_type": "blocker", "center_x_cm": -30, "center_y_cm": 0, "width_cm": 10, "height_cm": 20}
        ],
    }
    hit = expected_dt35_hit(0, 0, -90, model)
    assert hit is not None
    assert hit["name"] == "forest_right"
    assert hit["target_type"] == "blocker"
    assert hit["correction_allowed"] == "0"
    assert round(float(hit["distance_cm"]), 3) == 25


def test_dt35_solid_obstacle_stops_before_wall_and_allows_weighted_correction():
    model = {
        "enabled": True,
        "use_field_boundary": True,
        "field_width_cm": 200,
        "field_height_cm": 100,
        "rectangles": [
            {
                "name": "forest",
                "target_type": "solid_obstacle",
                "correction_weight": 0.65,
                "center_x_cm": -30,
                "center_y_cm": 0,
                "width_cm": 10,
                "height_cm": 20,
            }
        ],
    }
    hit = expected_dt35_hit(0, 0, -90, model)
    assert hit is not None
    assert hit["name"] == "forest_right"
    assert hit["target_type"] == "solid_obstacle"
    assert hit["correction_allowed"] == "1"
    assert float(hit["correction_weight"]) == 0.65
    assert round(float(hit["distance_cm"]), 3) == 25


def test_dt35_display_ray_clips_at_first_modeled_obstacle():
    model = {
        "enabled": True,
        "use_field_boundary": True,
        "field_width_cm": 200,
        "field_height_cm": 100,
        "rectangles": [
            {
                "name": "forest",
                "target_type": "solid_obstacle",
                "center_x_cm": -30,
                "center_y_cm": 0,
                "width_cm": 10,
                "height_cm": 20,
            }
        ],
    }

    ray = dt35_ray(0, 0, 0, {"enabled": True, "yaw_offset_deg": -90, "max_range_cm": 1000}, 1000, model)

    assert round(float(ray["hit_x_cm"]), 3) == -100.0
    assert round(float(ray["display_hit_x_cm"]), 3) == -25.0
    assert round(float(ray["display_distance_cm"]), 3) == 25.0
    assert ray["display_clipped_by_model"] is True
    assert ray["display_clip_target"] == "forest_right"


def test_dt35_missing_target_inference_does_not_skip_solid_obstacle():
    model = {
        "enabled": True,
        "use_field_boundary": False,
        "infer_missing_targets": True,
        "missing_target_residual_gate_cm": 5.0,
        "missing_target_skippable_types": ["usable_wall"],
        "rectangles": [
            {
                "name": "forest",
                "target_type": "solid_obstacle",
                "center_x_cm": -30,
                "center_y_cm": 0,
                "width_cm": 10,
                "height_cm": 20,
            }
        ],
        "segments": [
            {
                "name": "far_wall",
                "target_type": "usable_wall",
                "x1_cm": -100.0,
                "y1_cm": -50.0,
                "x2_cm": -100.0,
                "y2_cm": 50.0,
            }
        ],
    }

    ray = dt35_ray(0, 0, 0, {"enabled": True, "yaw_offset_deg": -90, "max_range_cm": 1000}, 1000, model)

    assert ray["expected_target"] == "forest_right"
    assert ray["nearest_target"] == "forest_right"
    assert ray["inferred_missing_target"] is False


def test_dt35_ignore_segment_disables_correction():
    model = {
        "enabled": True,
        "use_field_boundary": True,
        "field_width_cm": 200,
        "field_height_cm": 100,
        "segments": [
            {"name": "pole_gap", "target_type": "ignore", "x1_cm": -10, "y1_cm": -10, "x2_cm": -10, "y2_cm": 10}
        ],
    }
    hit = expected_dt35_hit(0, 0, -90, model)
    assert hit is not None
    assert hit["name"] == "pole_gap"
    assert hit["target_type"] == "ignore"
    assert hit["correction_allowed"] == "0"


def test_dt35_grazing_incidence_disables_correction():
    model = {
        "enabled": True,
        "use_field_boundary": False,
        "max_correction_incidence_deg": 75.0,
        "segments": [
            {"name": "top_wall", "target_type": "usable_wall", "x1_cm": -100.0, "y1_cm": 10.0, "x2_cm": 100.0, "y2_cm": 10.0}
        ],
    }
    hit = expected_dt35_hit(0, 0, 80, model)

    assert hit is not None
    assert hit["name"] == "top_wall"
    assert round(float(hit["incidence_deg"]), 3) == 80.0
    assert hit["correction_allowed"] == "0"


def test_default_config_ramp_and_bottom_barrier_match_prior_map_visible_footprint():
    config_path = Path(__file__).resolve().parents[1] / "config" / "default_config.json"
    config = json.loads(config_path.read_text(encoding="utf-8"))
    rectangles = {item["name"]: item for item in config["field_model"]["rectangles"]}

    expected = {
        "red_left_ramp_zone_450h": (155.0, 148.0, -528.75, -404.0),
        "blue_right_ramp_zone_450h": (148.5, 148.5, 524.5, -404.0),
    }
    for name, (width, height, center_x, center_y) in expected.items():
        ramp = rectangles[name]
        assert ramp["target_type"] == "solid_obstacle"
        assert ramp["width_cm"] == width
        assert ramp["height_cm"] == height
        assert ramp["center_x_cm"] == center_x
        assert ramp["center_y_cm"] == center_y
        assert ramp["correction_weight"] == 0.35

    bottom_barrier = rectangles["bottom_center_barrier_wall"]
    assert bottom_barrier["target_type"] == "usable_wall"
    assert bottom_barrier["center_x_cm"] == -1.25
    assert bottom_barrier["center_y_cm"] == -473.75
    assert bottom_barrier["width_cm"] == 28.0
    assert bottom_barrier["height_cm"] == 161.0


def test_default_config_top_center_end_rack_blocks_red_start_side_ray():
    config_path = Path(__file__).resolve().parents[1] / "config" / "default_config.json"
    config = json.loads(config_path.read_text(encoding="utf-8"))
    model = dict(config["field_model"])
    model.setdefault("field_width_cm", config["map"]["field_width_cm"])
    model.setdefault("field_height_cm", config["map"]["field_height_cm"])

    ray = dt35_ray(-552.5, 520.0, 0.0, config["dt35"]["sensor_2"], 10000, model)

    assert ray["expected_target"] == "top_center_end_rack_wall_left"
    assert ray["display_clipped_by_model"] is True
