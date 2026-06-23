import json
from pathlib import Path

from locater_map.data_model import RobotFrame
from locater_map.dt35_range_calibrator import estimate_dt35_range_bias
from locater_map.utils_transform import dt35_ray


def _default_config() -> dict:
    config_path = Path(__file__).resolve().parents[1] / "config" / "default_config.json"
    return json.loads(config_path.read_text(encoding="utf-8"))


def test_range_bias_estimator_uses_start_local_lidar_as_field_pose():
    config = _default_config()
    config["dt35"]["sensor_1"]["max_range_cm"] = 2000.0
    config["dt35"]["sensor_2"]["max_range_cm"] = 2000.0
    config["dt35"]["sensor_1"]["distance_bias_mm"] = 0.0
    config["dt35"]["sensor_2"]["distance_bias_mm"] = 0.0
    red = config["robot"]["start_pose_red"]
    field_model = dict(config["field_model"])
    field_model.setdefault("field_width_cm", config["map"]["field_width_cm"])
    field_model.setdefault("field_height_cm", config["map"]["field_height_cm"])

    expected_1 = dt35_ray(
        red["x_cm"],
        red["y_cm"],
        red["yaw_deg"],
        config["dt35"]["sensor_1"],
        1.0,
        field_model,
    )
    expected_2 = dt35_ray(
        red["x_cm"],
        red["y_cm"],
        red["yaw_deg"],
        config["dt35"]["sensor_2"],
        1.0,
        field_model,
    )
    frame = RobotFrame(
        pos_x_cm=0.0,
        pos_y_cm=0.0,
        pos_yaw_deg=0.0,
        lidar_x_cm=0.0,
        lidar_y_cm=0.0,
        lidar_yaw_deg=0.0,
        lidar_valid=True,
        lidar_online=True,
        h30_yaw_deg=0.0,
        h30_valid=True,
        dt35_1_mm=float(expected_1["expected_distance_cm"]) * 10.0 + 20.0,
        dt35_2_mm=float(expected_2["expected_distance_cm"]) * 10.0 - 30.0,
        dt35_1_valid=True,
        dt35_2_valid=True,
    )

    estimates, summary = estimate_dt35_range_bias(config, [frame], min_frames=1)
    by_sensor = {item.sensor_key: item for item in estimates}

    assert summary.start_policy == "always_local_display"
    assert set(by_sensor) == {"sensor_1", "sensor_2"}
    assert round(by_sensor["sensor_1"].mean_residual_cm, 3) == 2.0
    assert round(by_sensor["sensor_1"].suggested_distance_bias_mm, 3) == -20.0
    assert round(by_sensor["sensor_2"].mean_residual_cm, 3) == -3.0
    assert round(by_sensor["sensor_2"].suggested_distance_bias_mm, 3) == 30.0
