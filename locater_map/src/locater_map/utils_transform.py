from __future__ import annotations

from dataclasses import replace
from math import cos, radians, sin
from typing import Any

from .data_model import RobotFrame


def transform_xy_yaw(x: float, y: float, yaw: float, cfg: dict[str, Any]) -> tuple[float, float, float]:
    sx = float(cfg.get("data_x_sign", 1.0))
    sy = float(cfg.get("data_y_sign", 1.0))
    syaw = float(cfg.get("data_yaw_sign", 1.0))
    ox = float(cfg.get("data_x_offset_cm", 0.0))
    oy = float(cfg.get("data_y_offset_cm", 0.0))
    oyaw = float(cfg.get("data_yaw_offset_deg", 0.0))
    return x * sx + ox, y * sy + oy, yaw * syaw + oyaw


def transform_frame(frame: RobotFrame, cfg: dict[str, Any]) -> RobotFrame:
    px, py, pyaw = transform_xy_yaw(frame.pos_x_cm, frame.pos_y_cm, frame.pos_yaw_deg, cfg)
    cx, cy, cyaw = transform_xy_yaw(frame.calib_x_cm, frame.calib_y_cm, frame.calib_yaw_deg, cfg)
    hx, hy, hyaw = transform_xy_yaw(frame.h30_x_cm, frame.h30_y_cm, frame.h30_yaw_deg, cfg)
    lx, ly, lyaw = transform_xy_yaw(frame.lidar_x_cm, frame.lidar_y_cm, frame.lidar_yaw_deg, cfg)
    return replace(
        frame,
        pos_x_cm=px,
        pos_y_cm=py,
        pos_yaw_deg=pyaw,
        calib_x_cm=cx,
        calib_y_cm=cy,
        calib_yaw_deg=cyaw,
        encoder_x_cm=cx,
        encoder_y_cm=cy,
        h30_x_cm=hx,
        h30_y_cm=hy,
        h30_yaw_deg=hyaw,
        lidar_x_cm=lx,
        lidar_y_cm=ly,
        lidar_yaw_deg=lyaw,
    )


def dt35_ray(
    robot_x_cm: float,
    robot_y_cm: float,
    robot_yaw_deg: float,
    sensor_cfg: dict[str, Any],
    distance_mm: float,
) -> dict[str, float | bool | str]:
    ox = float(sensor_cfg.get("offset_x_cm", 0.0))
    oy = float(sensor_cfg.get("offset_y_cm", 0.0))
    yaw_offset = float(sensor_cfg.get("yaw_offset_deg", 0.0))
    max_range_cm = float(sensor_cfg.get("max_range_cm", 999999.0))
    enabled = bool(sensor_cfg.get("enabled", True))
    yaw = radians(robot_yaw_deg)
    sensor_x = robot_x_cm + ox * cos(yaw) - oy * sin(yaw)
    sensor_y = robot_y_cm + ox * sin(yaw) + oy * cos(yaw)
    d_cm = max(0.0, float(distance_mm) / 10.0)
    valid = enabled and 0.0 < d_cm <= max_range_cm
    draw_d = d_cm if valid else max_range_cm
    ray_yaw = radians(robot_yaw_deg + yaw_offset)
    hit_x = sensor_x + draw_d * cos(ray_yaw)
    hit_y = sensor_y + draw_d * sin(ray_yaw)
    return {
        "name": str(sensor_cfg.get("name", "DT35")),
        "enabled": enabled,
        "valid": valid,
        "sensor_x_cm": sensor_x,
        "sensor_y_cm": sensor_y,
        "hit_x_cm": hit_x,
        "hit_y_cm": hit_y,
        "distance_cm": d_cm,
        "max_range_cm": max_range_cm,
    }
