import json
from pathlib import Path

from locater_map.fusion_benchmark import DEFAULT_BENCHMARK_PATHS, run_synthetic_benchmark, write_benchmark_csv, write_benchmark_summary
from locater_map.fusion_model import FusionConfig


def _default_config() -> dict:
    config_path = Path(__file__).resolve().parents[1] / "config" / "default_config.json"
    return json.loads(config_path.read_text(encoding="utf-8"))


def test_synthetic_benchmark_reports_path_level_dt35_fusion_quality(tmp_path):
    rows, summary = run_synthetic_benchmark(
        _default_config(),
        DEFAULT_BENCHMARK_PATHS,
        samples=180,
        encoder_x_scale=0.97,
        encoder_y_scale=1.03,
        dt35_noise_mm=5.0,
        fusion_cfg=FusionConfig(lidar_stride=25, lidar_gain=1.0, dt35_gain=1.0, dt35_yaw_gain=0.0),
    )

    assert len(rows) == 6
    assert summary.paths == 6
    assert summary.passed_paths == 6
    assert summary.mean_raw_rms_xy_cm is not None
    assert summary.mean_fused_rms_xy_cm is not None
    assert summary.mean_fused_rms_xy_cm < summary.mean_raw_rms_xy_cm
    assert summary.total_dt35_fusion_allowed_frames > 0
    assert all(row.benchmark_passed for row in rows)
    moving_rows = [row for row in rows if row.benchmark_reason == "moving_path_improved"]
    stable_rows = [row for row in rows if row.benchmark_reason == "static_or_rotation_path_stable"]
    assert moving_rows
    assert stable_rows
    assert all(row.improvement_rms_cm is not None and row.improvement_rms_cm > 0.0 for row in moving_rows)

    csv_path = tmp_path / "benchmark.csv"
    json_path = tmp_path / "benchmark.json"
    write_benchmark_csv(csv_path, rows)
    write_benchmark_summary(json_path, summary)

    header = csv_path.read_text(encoding="utf-8").splitlines()[0]
    assert "dt35_rank_counts_json" in header
    assert "benchmark_reason" in header
    payload = json.loads(json_path.read_text(encoding="utf-8"))
    assert payload["failed_paths"] == 0
