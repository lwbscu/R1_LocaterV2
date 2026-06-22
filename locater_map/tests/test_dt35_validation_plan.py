import json
from pathlib import Path

from locater_map.dt35_validation_plan import (
    REQUIRED_CATEGORIES,
    build_validation_plan_markdown,
    generate_dt35_validation_plan,
    write_validation_plan_csv,
    write_validation_plan_markdown,
    write_validation_plan_summary,
)


def _default_config() -> dict:
    return json.loads((Path(__file__).resolve().parents[1] / "config" / "default_config.json").read_text(encoding="utf-8"))


def test_dt35_validation_plan_selects_real_car_checkpoints(tmp_path):
    cases, summary = generate_dt35_validation_plan(
        _default_config(),
        x_min_cm=-580.0,
        x_max_cm=580.0,
        y_min_cm=-580.0,
        y_max_cm=580.0,
        step_cm=100.0,
        yaws_deg=[-180.0, -135.0, -90.0, -45.0, 0.0, 45.0, 90.0, 135.0],
        per_category=2,
    )

    categories = {case.category for case in cases}
    assert set(REQUIRED_CATEGORIES).issubset(categories)
    assert summary.missing_categories == []
    assert summary.selected_cases == len(cases)
    assert summary.sensor_counts
    assert "solid_obstacle" in summary.target_type_counts
    assert "ignore" in summary.target_type_counts
    assert any(case.expected_target_type == "usable_wall" for case in cases)
    assert any("forest" in case.expected_target for case in cases)
    assert any("ramp" in case.expected_target for case in cases)
    assert any(case.risk == "ignored_interference" for case in cases)

    csv_path = tmp_path / "plan.csv"
    json_path = tmp_path / "plan.json"
    write_validation_plan_csv(csv_path, cases)
    write_validation_plan_summary(json_path, summary)

    header = csv_path.read_text(encoding="utf-8").splitlines()[0]
    assert "operator_note" in header
    payload = json.loads(json_path.read_text(encoding="utf-8"))
    assert payload["missing_categories"] == []

    md_path = tmp_path / "plan.md"
    write_validation_plan_markdown(md_path, cases, summary)
    markdown = md_path.read_text(encoding="utf-8")
    assert "DT35 Real-Car Validation Checklist" in markdown
    assert "python main.py --record" in markdown
    assert "Expected distance" in markdown
    assert cases[0].pose_label in markdown
    assert build_validation_plan_markdown(cases[:1], summary).startswith("# DT35 Real-Car")
