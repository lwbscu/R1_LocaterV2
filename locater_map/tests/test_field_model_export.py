import json
from pathlib import Path

from locater_map.dt35_analysis import PoseSpec
from locater_map.field_model_export import build_field_model_svg, write_field_model_svg


def _default_config() -> dict:
    config_path = Path(__file__).resolve().parents[1] / "config" / "default_config.json"
    config = json.loads(config_path.read_text(encoding="utf-8"))
    config["_project_root"] = str(config_path.parents[1])
    return config


def test_field_model_svg_exports_algorithm_geometry_and_dt35_rays(tmp_path):
    svg = build_field_model_svg(
        _default_config(),
        [PoseSpec(236.403, 1.58, 0.516, "lidar_forest_side_sample")],
    )

    assert "field_prior_map_clean_labeled_1215x1210cm.png" in svg
    assert "field_left [usable_wall]" in svg
    assert "field_right [usable_wall]" in svg
    assert "top_blue_start_zone_marker" in svg
    assert "red_start_left_wall" in svg
    assert "top_red_long_pole_rack_ignore" in svg
    assert "red_forest_obstacle" in svg
    assert "red_left_ramp_zone_450h" in svg
    assert "lidar_forest_side_sample sensor_1" in svg
    assert "lidar_forest_side_sample sensor_2" in svg
    assert "#ffd24a" in svg
    assert "#29d4ff" in svg
    assert "upper_r1_r2_wall" not in svg
    assert "upper_red_r1_r2_wall" not in svg
    assert "upper_blue_r1_r2_wall" not in svg
    assert "usable wall" in svg
    assert "ignored interference" in svg

    out = tmp_path / "field_model.svg"
    write_field_model_svg(out, _default_config(), [PoseSpec(-400.0, -420.0, 0.0, "ramp")])

    assert out.exists()
    assert out.read_text(encoding="utf-8").startswith("<svg")
