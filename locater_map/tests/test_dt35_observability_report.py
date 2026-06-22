import json
from pathlib import Path

from locater_map.dt35_observability_report import (
    build_observability_markdown,
    generate_dt35_observability_report,
    write_dt35_observability_report,
)


def _default_config() -> dict:
    return json.loads((Path(__file__).resolve().parents[1] / "config" / "default_config.json").read_text(encoding="utf-8"))


def test_dt35_observability_report_documents_rank_limits(tmp_path):
    rows, summary = generate_dt35_observability_report(
        _default_config(),
        step_cm=100.0,
        yaws_deg=[-180.0, -135.0, -90.0, -45.0, 0.0, 45.0, 90.0, 135.0],
    )

    assert rows
    assert summary.underconstrained_poses > 0
    assert summary.no_dt35_poses > 0
    assert summary.one_dim_poses > 0
    assert summary.constraint_state_counts["rank1_x"] > 0
    assert summary.constraint_state_counts["rank1_y"] > 0
    assert summary.constraint_state_counts["rank1_xy"] > 0

    csv_path = tmp_path / "observability.csv"
    json_path = tmp_path / "observability.json"
    md_path = tmp_path / "observability.md"
    write_dt35_observability_report(csv_path, json_path, md_path, rows, summary)

    assert "translation_rank" in csv_path.read_text(encoding="utf-8").splitlines()[0]
    payload = json.loads(json_path.read_text(encoding="utf-8"))
    assert payload["underconstrained_poses"] > 0
    markdown = md_path.read_text(encoding="utf-8")
    assert "rank1_x" in markdown
    assert "rank0_no_dt35" in markdown
    assert build_observability_markdown(rows, summary).startswith("# DT35 Observability Matrix")
