import json
from copy import deepcopy
from pathlib import Path

from locater_map.dt35_analysis import PoseSpec
from locater_map.field_model_audit import build_field_model_audit, write_field_model_audit


def _default_config() -> dict:
    config_path = Path(__file__).resolve().parents[1] / "config" / "default_config.json"
    config = json.loads(config_path.read_text(encoding="utf-8"))
    config["_project_root"] = str(config_path.parents[1])
    return config


def test_field_model_audit_reports_dimensions_targets_and_missing_real_evidence(tmp_path):
    audit = build_field_model_audit(_default_config())

    assert audit["map"]["field_width_cm"] == 1215.0
    assert audit["map"]["field_height_cm"] == 1210.0
    assert audit["map"]["pixels_per_cm_x"] == 2.0
    assert audit["map"]["pixels_per_cm_y"] == 2.0
    assert audit["dt35"]["sensor_1"]["offset_x_cm"] == 40.4
    assert audit["dt35"]["sensor_2"]["offset_x_cm"] == -40.4
    assert "usable_wall" in audit["field_model"]["target_type_counts"]
    assert "ignore" in audit["field_model"]["target_type_counts"]
    assert "solid_obstacle" in audit["field_model"]["target_type_counts"]
    assert audit["default_pose_coverage"]["rays"] > 2
    assert audit["default_pose_observability"]["poses"] > 1
    assert audit["default_pose_hits"]
    assert audit["default_pose_behavior"]["usable_solid_obstacle_ray_count"] > 0
    assert audit["default_pose_behavior"]["usable_forest_ray_count"] > 0
    assert audit["default_pose_behavior"]["usable_ramp_ray_count"] > 0
    assert audit["default_pose_behavior"]["usable_forest_targets"]
    assert audit["default_pose_behavior"]["usable_ramp_targets"]
    assert audit["field_sweep"]["summary"]["model_passed"] is True
    assert audit["field_sweep"]["summary"]["forest_constraint_poses"] > 0
    assert audit["field_sweep"]["summary"]["ramp_constraint_poses"] > 0
    assert audit["field_sweep"]["summary"]["ignored_interference_poses"] > 0
    assert audit["field_sweep"]["summary"]["ignored_targets_never_corrected"] is True
    assert audit["field_sweep"]["weak_pose_examples"]
    assert audit["field_sweep"]["sample_rows"]
    assert audit["manual_dimension_checks"]
    assert all(item["passed"] for item in audit["manual_dimension_checks"])
    names = {item["name"] for item in audit["manual_dimension_checks"]}
    assert "red_forest_obstacle_width_cm" in names
    assert "blue_forest_obstacle_height_cm" in names
    assert "red_left_ramp_zone_450h_width_cm" in names
    assert "dt35_sensor_1_offset_x_cm" in names
    assert audit["model_self_check"]["passed"] is True
    assert audit["model_self_check"]["failed_checks"] == []
    assert audit["completion_evidence_missing"]
    assert audit["completion_verified"] is False

    out = tmp_path / "audit.json"
    write_field_model_audit(out, _default_config(), [PoseSpec(-400.0, -420.0, 0.0, "ramp")])

    payload = json.loads(out.read_text(encoding="utf-8"))
    assert payload["manual_dimension_interpretation"]["field_outer_size_cm"] == [1215.0, 1210.0]


def test_field_model_audit_self_check_fails_for_missing_required_geometry():
    config = _default_config()
    broken = deepcopy(config)
    broken["field_model"]["segments"] = [
        item for item in broken["field_model"]["segments"] if item.get("target_type") != "ignore"
    ]
    broken["field_model"]["rectangles"] = [
        item for item in broken["field_model"]["rectangles"] if item.get("target_type") != "ignore"
    ]
    broken["field_model"]["rectangles"] = [
        item for item in broken["field_model"]["rectangles"] if "ramp" not in item.get("name", "")
    ]

    audit = build_field_model_audit(broken)

    assert audit["model_self_check"]["passed"] is False
    assert "has_ignore_targets" in audit["model_self_check"]["failed_checks"]
    assert "has_ramp_blockers" in audit["model_self_check"]["failed_checks"]
    assert "default_poses_have_usable_ramp_rays" in audit["model_self_check"]["failed_checks"]
    assert "field_sweep_passed" in audit["model_self_check"]["failed_checks"]


def test_field_model_audit_self_check_fails_for_bad_manual_dimensions():
    config = _default_config()
    broken = deepcopy(config)
    for item in broken["field_model"]["rectangles"]:
        if item.get("name") == "red_forest_obstacle":
            item["width_cm"] = 330.0
            break

    audit = build_field_model_audit(broken)

    failed_dimensions = [item["name"] for item in audit["manual_dimension_checks"] if not item["passed"]]
    assert "red_forest_obstacle_width_cm" in failed_dimensions
    assert audit["model_self_check"]["passed"] is False
    assert "manual_dimension_checks_passed" in audit["model_self_check"]["failed_checks"]
