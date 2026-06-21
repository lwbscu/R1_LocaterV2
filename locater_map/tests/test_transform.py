from locater_map.data_model import RobotFrame
from locater_map.utils_transform import dt35_ray, transform_frame, transform_xy_yaw


def test_transform_xy_yaw():
    cfg = {
        "data_x_sign": -1,
        "data_y_sign": 1,
        "data_yaw_sign": -1,
        "data_x_offset_cm": 10,
        "data_y_offset_cm": -5,
        "data_yaw_offset_deg": 90,
    }
    assert transform_xy_yaw(2, 3, 30, cfg) == (8, -2, 60)


def test_transform_frame_applies_all_pose_groups():
    frame = RobotFrame(pos_x_cm=1, pos_y_cm=2, pos_yaw_deg=3, lidar_x_cm=4, lidar_y_cm=5, lidar_yaw_deg=6)
    out = transform_frame(frame, {"data_x_offset_cm": 10, "data_y_offset_cm": 20, "data_yaw_offset_deg": 30})
    assert out.pos_x_cm == 11
    assert out.pos_y_cm == 22
    assert out.pos_yaw_deg == 33
    assert out.lidar_x_cm == 14
    assert out.lidar_y_cm == 25
    assert out.lidar_yaw_deg == 36


def test_dt35_ray_formula():
    cfg = {"name": "s1", "enabled": True, "offset_x_cm": 10, "offset_y_cm": 0, "yaw_offset_deg": 0, "max_range_cm": 100}
    ray = dt35_ray(0, 0, 0, cfg, 500)
    assert round(float(ray["sensor_x_cm"]), 3) == 10
    assert round(float(ray["sensor_y_cm"]), 3) == 0
    assert round(float(ray["hit_x_cm"]), 3) == 60
    assert round(float(ray["hit_y_cm"]), 3) == 0
    assert ray["valid"] is True
