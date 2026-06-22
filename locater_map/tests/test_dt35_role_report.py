import json
from pathlib import Path

from locater_map.dt35_analysis import PoseSpec, analyze_dt35_hits
from locater_map.dt35_role_report import (
    build_dt35_role_markdown,
    build_dt35_role_rows,
    summarize_dt35_roles,
    write_dt35_role_csv,
    write_dt35_role_markdown,
    write_dt35_role_summary,
)


def _default_config() -> dict:
    return json.loads((Path(__file__).resolve().parents[1] / "config" / "default_config.json").read_text(encoding="utf-8"))


def test_dt35_role_report_explains_obstacle_and_ignore_targets(tmp_path):
    config = _default_config()
    poses = [
        PoseSpec(-360.0, 520.0, 90.0, "top_ignore"),
        PoseSpec(-550.0, 50.0, 0.0, "forest"),
        PoseSpec(-400.0, -420.0, 0.0, "ramp"),
        PoseSpec(40.0, 0.0, 0.0, "center"),
    ]
    rows = build_dt35_role_rows(analyze_dt35_hits(config, poses))
    summary = summarize_dt35_roles(rows)

    assert summary.rows == len(poses) * 2
    assert summary.ignored_rows > 0
    assert summary.usable_forest_rows > 0
    assert summary.usable_ramp_rows > 0
    assert any(row.world_constraint_axis == "x" for row in rows)
    assert any("Positive residual moves pose" in row.explanation for row in rows if row.usable_for_fusion)
    assert any(row.risk == "ignored_interference" for row in rows)

    csv_path = tmp_path / "roles.csv"
    json_path = tmp_path / "roles.json"
    md_path = tmp_path / "roles.md"
    write_dt35_role_csv(csv_path, rows)
    write_dt35_role_summary(json_path, summary)
    write_dt35_role_markdown(md_path, rows, summary)

    header = csv_path.read_text(encoding="utf-8").splitlines()[0]
    assert "world_constraint_axis" in header
    payload = json.loads(json_path.read_text(encoding="utf-8"))
    assert payload["usable_forest_rows"] > 0
    assert payload["usable_ramp_rows"] > 0
    markdown = md_path.read_text(encoding="utf-8")
    assert "DT35 Role Matrix" in markdown
    assert "forest" in markdown
    assert "ramp" in markdown
    assert "ignored interference" in markdown.lower()
    assert build_dt35_role_markdown(rows, summary).startswith("# DT35 Role Matrix")
