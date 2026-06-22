import json
from pathlib import Path

from locater_map.dt35_field_sweep import (
    REQUIRED_SWEEP_CATEGORIES,
    run_dt35_field_sweep,
    write_field_sweep_csv,
    write_field_sweep_summary,
)


def _default_config() -> dict:
    return json.loads((Path(__file__).resolve().parents[1] / "config" / "default_config.json").read_text(encoding="utf-8"))


def test_dt35_field_sweep_covers_walls_obstacles_and_ignored_geometry(tmp_path):
    rows, summary = run_dt35_field_sweep(
        _default_config(),
        x_min_cm=-580.0,
        x_max_cm=580.0,
        y_min_cm=-580.0,
        y_max_cm=580.0,
        step_cm=100.0,
        yaws_deg=[-180.0, -135.0, -90.0, -45.0, 0.0, 45.0, 90.0, 135.0],
    )

    assert rows
    assert summary.model_passed is True
    assert summary.missing_categories == []
    assert set(REQUIRED_SWEEP_CATEGORIES).issubset(summary.required_category_counts)
    assert summary.usable_wall_x_poses > 0
    assert summary.usable_wall_y_poses > 0
    assert summary.forest_constraint_poses > 0
    assert summary.ramp_constraint_poses > 0
    assert summary.ignored_interference_poses > 0
    assert summary.ignored_targets_never_corrected is True
    assert summary.one_dim_poses > 0
    assert "solid_obstacle" in summary.target_type_counts
    assert "ignore" in summary.target_type_counts
    assert any(row.has_forest_constraint for row in rows)
    assert any(row.has_ramp_constraint for row in rows)
    assert any(row.has_ignored_interference for row in rows)

    csv_path = tmp_path / "sweep.csv"
    json_path = tmp_path / "sweep.json"
    write_field_sweep_csv(csv_path, rows)
    write_field_sweep_summary(json_path, summary)

    assert "primary_note" in csv_path.read_text(encoding="utf-8").splitlines()[0]
    payload = json.loads(json_path.read_text(encoding="utf-8"))
    assert payload["model_passed"] is True
    assert payload["missing_categories"] == []
