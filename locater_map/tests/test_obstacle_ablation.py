import json
from pathlib import Path

from locater_map.fusion_model import FusionConfig
from locater_map.obstacle_ablation import (
    DEFAULT_OBSTACLE_PATHS,
    run_obstacle_ablation,
    write_obstacle_ablation_csv,
    write_obstacle_ablation_summary,
)


def _default_config() -> dict:
    return json.loads((Path(__file__).resolve().parents[1] / "config" / "default_config.json").read_text(encoding="utf-8"))


def test_obstacle_ablation_proves_forest_and_ramp_correction_is_used(tmp_path):
    rows, summary = run_obstacle_ablation(
        _default_config(),
        DEFAULT_OBSTACLE_PATHS,
        samples=120,
        encoder_x_scale=0.97,
        encoder_y_scale=1.03,
        dt35_noise_mm=0.0,
        fusion_cfg=FusionConfig(lidar_stride=25, lidar_gain=1.0, dt35_gain=1.0, dt35_yaw_gain=0.0),
    )

    assert summary.paths == 3
    assert summary.failed_paths == 0
    assert summary.total_full_solid_fusion_allowed_rays > 0
    assert summary.total_full_forest_fusion_allowed_rays > 0
    assert summary.total_full_ramp_fusion_allowed_rays > 0
    assert all(row.solid_obstacle_contributed for row in rows)
    assert all(row.ablated_solid_fusion_allowed_rays == 0 for row in rows)
    assert any(row.full_forest_fusion_allowed_rays > 0 for row in rows)
    assert any(row.full_ramp_fusion_allowed_rays > 0 for row in rows)

    csv_path = tmp_path / "ablation.csv"
    json_path = tmp_path / "ablation.json"
    write_obstacle_ablation_csv(csv_path, rows)
    write_obstacle_ablation_summary(json_path, summary)

    header = csv_path.read_text(encoding="utf-8").splitlines()[0]
    assert "full_solid_fusion_allowed_rays" in header
    payload = json.loads(json_path.read_text(encoding="utf-8"))
    assert payload["failed_paths"] == 0
