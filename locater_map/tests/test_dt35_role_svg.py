import json
from pathlib import Path

from locater_map.dt35_analysis import PoseSpec, analyze_dt35_hits, generate_yaw_matrix_poses
from locater_map.dt35_role_report import build_dt35_role_rows
from locater_map.dt35_role_svg import build_dt35_role_svg, write_dt35_role_svg


def _default_config() -> dict:
    return json.loads((Path(__file__).resolve().parents[1] / "config" / "default_config.json").read_text(encoding="utf-8"))


def test_dt35_role_svg_contains_yaw_panels_and_target_classes(tmp_path):
    config = _default_config()
    base = [
        PoseSpec(-360.0, 520.0, 0.0, "top_ignore"),
        PoseSpec(-550.0, 50.0, 0.0, "forest"),
        PoseSpec(-400.0, -420.0, 0.0, "ramp"),
    ]
    poses = generate_yaw_matrix_poses(base, [0.0, 90.0])
    rows = build_dt35_role_rows(analyze_dt35_hits(config, poses))

    svg = build_dt35_role_svg(rows, config, title="test role map")
    assert "<svg" in svg
    assert "H30 yaw 0" in svg
    assert "H30 yaw 90" in svg
    assert "red_forest_obstacle" in svg
    assert "red_left_ramp_zone_450h" in svg
    assert "ignore/noisy" in svg
    assert "green stroke=forest/ramp" in svg

    out = tmp_path / "roles.svg"
    write_dt35_role_svg(out, rows, config)
    assert out.read_text(encoding="utf-8").startswith("<svg")
