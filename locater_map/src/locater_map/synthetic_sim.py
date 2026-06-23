from __future__ import annotations

from dataclasses import dataclass
from math import cos, hypot, pi, radians, sin
from random import Random
from typing import Any

from .data_model import RobotFrame
from .encoder_sim import FirmwareEncoderSimulator
from .fusion_model import wrap_deg
from .utils_transform import expected_dt35_hit, robot_local_to_world


@dataclass(slots=True)
class SyntheticConfig:
    samples: int = 240
    path_name: str = "top_corridor"
    sample_period_s: float = 0.02
    encoder_x_scale: float = 1.0
    encoder_y_scale: float = 1.0
    encoder_yaw_scale: float = 1.0
    h30_yaw_bias_deg: float = 0.0
    lidar_xy_noise_cm: float = 0.0
    lidar_yaw_noise_deg: float = 0.0
    dt35_noise_mm: float = 0.0
    path_wiggle_cm: float = 0.0
    path_wiggle_cycles: float = 8.0
    path_yaw_wiggle_deg: float = 0.0
    h30_delay_frames: int = 0
    dt35_delay_frames: int = 0
    dt35_dropout_rate: float = 0.0
    dt35_occlusion_rate: float = 0.0
    dt35_occlusion_min_cm: float = 18.0
    dt35_occlusion_max_cm: float = 95.0
    use_start_transform: bool = False
    seed: int = 7


def generate_synthetic_frames(app_config: dict[str, Any], cfg: SyntheticConfig) -> list[RobotFrame]:
    rng = Random(cfg.seed)
    poses = _pose_path(cfg.path_name, max(2, int(cfg.samples)), cfg.seed)
    poses = _apply_path_wiggle(
        poses,
        amplitude_cm=float(cfg.path_wiggle_cm),
        cycles=float(cfg.path_wiggle_cycles),
        yaw_amplitude_deg=float(cfg.path_yaw_wiggle_deg),
    )
    field_model = _field_model_from_config(app_config)
    start_tf = _start_transform(app_config) if cfg.use_start_transform else (0.0, 0.0, 0.0, False)

    frames: list[RobotFrame] = []
    first_x, first_y, first_yaw = poses[0]
    encoder = FirmwareEncoderSimulator(
        initial_x_cm=first_x,
        initial_y_cm=first_y,
        x_scale=cfg.encoder_x_scale,
        y_scale=cfg.encoder_y_scale,
    )
    prev_truth_x = first_x
    prev_truth_y = first_y
    for seq, (x_cm, y_cm, yaw_deg) in enumerate(poses):
        lidar_x = x_cm + _noise(rng, cfg.lidar_xy_noise_cm)
        lidar_y = y_cm + _noise(rng, cfg.lidar_xy_noise_cm)
        lidar_yaw = wrap_deg(yaw_deg + _noise(rng, cfg.lidar_yaw_noise_deg))
        h30_index = max(0, seq - max(0, int(cfg.h30_delay_frames)))
        h30_truth_yaw = poses[h30_index][2]
        h30_yaw = wrap_deg(first_yaw + wrap_deg(h30_truth_yaw - first_yaw) * cfg.encoder_yaw_scale + cfg.h30_yaw_bias_deg)
        if seq == 0:
            encoder_sample = encoder.sample_static(h30_yaw)
        else:
            encoder_sample = encoder.sample_for_truth_delta(x_cm - prev_truth_x, y_cm - prev_truth_y, h30_yaw)
        prev_truth_x = x_cm
        prev_truth_y = y_cm
        dt35_index = max(0, seq - max(0, int(cfg.dt35_delay_frames)))
        dt35_x, dt35_y, dt35_yaw = poses[dt35_index]
        dt35_1_mm, dt35_1_valid = _synthetic_dt35_distance(
            dt35_x, dt35_y, dt35_yaw, app_config.get("dt35", {}).get("sensor_1", {}), field_model, start_tf, rng, cfg.dt35_noise_mm
        )
        dt35_1_mm, dt35_1_valid = _apply_dt35_disturbance(dt35_1_mm, dt35_1_valid, rng, cfg)
        dt35_2_mm, dt35_2_valid = _synthetic_dt35_distance(
            dt35_x, dt35_y, dt35_yaw, app_config.get("dt35", {}).get("sensor_2", {}), field_model, start_tf, rng, cfg.dt35_noise_mm
        )
        dt35_2_mm, dt35_2_valid = _apply_dt35_disturbance(dt35_2_mm, dt35_2_valid, rng, cfg)
        status = _status_mask(dt35_1_valid, dt35_2_valid, encoder_sample.x_pulse_seen, encoder_sample.y_pulse_seen)
        sample_period_s = max(0.001, float(cfg.sample_period_s))
        frames.append(
            RobotFrame(
                source_time_ms=int(round(seq * sample_period_s * 1000.0)),
                pc_time=seq * sample_period_s,
                seq=seq,
                pos_x_cm=lidar_x,
                pos_y_cm=lidar_y,
                pos_yaw_deg=lidar_yaw,
                lidar_x_cm=lidar_x,
                lidar_y_cm=lidar_y,
                lidar_yaw_deg=lidar_yaw,
                calib_x_cm=encoder_sample.x_cm,
                calib_y_cm=encoder_sample.y_cm,
                calib_yaw_deg=h30_yaw,
                encoder_x_cm=encoder_sample.x_cm,
                encoder_y_cm=encoder_sample.y_cm,
                h30_yaw_deg=h30_yaw,
                dt35_1_mm=dt35_1_mm,
                dt35_2_mm=dt35_2_mm,
                dt35_1_valid=dt35_1_valid,
                dt35_2_valid=dt35_2_valid,
                h30_valid=True,
                h30_has_attitude=True,
                lidar_valid=True,
                lidar_online=True,
                x_delta_count=encoder_sample.x_delta_count,
                y_delta_count=encoder_sample.y_delta_count,
                x_total_count=encoder_sample.x_total_count,
                y_total_count=encoder_sample.y_total_count,
                x_raw_count=encoder_sample.x_total_count,
                y_raw_count=encoder_sample.y_total_count,
                x_pulse_seen=encoder_sample.x_pulse_seen,
                y_pulse_seen=encoder_sample.y_pulse_seen,
                encoder_dis_p_mm=encoder_sample.encoder_dis_p_mm,
                encoder_dis_q_mm=encoder_sample.encoder_dis_q_mm,
                status=status,
                protocol="synthetic_r1_csv_v3",
            )
        )
    return frames


def _apply_path_wiggle(
    poses: list[tuple[float, float, float]],
    *,
    amplitude_cm: float,
    cycles: float,
    yaw_amplitude_deg: float,
) -> list[tuple[float, float, float]]:
    if len(poses) < 3 or abs(amplitude_cm) <= 1.0e-9 and abs(yaw_amplitude_deg) <= 1.0e-9:
        return poses
    out: list[tuple[float, float, float]] = []
    last = len(poses) - 1
    for index, (x_cm, y_cm, yaw_deg) in enumerate(poses):
        if index == 0 or index == last:
            out.append((x_cm, y_cm, yaw_deg))
            continue
        prev_x, prev_y, _ = poses[index - 1]
        next_x, next_y, _ = poses[index + 1]
        tx = next_x - prev_x
        ty = next_y - prev_y
        length = hypot(tx, ty)
        if length <= 1.0e-9:
            out.append((x_cm, y_cm, yaw_deg))
            continue
        tx /= length
        ty /= length
        nx = -ty
        ny = tx
        phase = index / max(1, last)
        envelope = sin(pi * phase)
        wave = sin(2.0 * pi * cycles * phase)
        yaw_wave = cos(2.0 * pi * cycles * phase)
        out.append((
            x_cm + amplitude_cm * envelope * wave * nx,
            y_cm + amplitude_cm * envelope * wave * ny,
            wrap_deg(yaw_deg + yaw_amplitude_deg * envelope * yaw_wave),
        ))
    return out


def _apply_dt35_disturbance(
    distance_mm: float,
    valid: bool,
    rng: Random,
    cfg: SyntheticConfig,
) -> tuple[float, bool]:
    dropout_rate = min(1.0, max(0.0, float(cfg.dt35_dropout_rate)))
    if dropout_rate > 0.0 and rng.random() < dropout_rate:
        return 0.0, False
    occlusion_rate = min(1.0, max(0.0, float(cfg.dt35_occlusion_rate)))
    if occlusion_rate <= 0.0 or rng.random() >= occlusion_rate:
        return distance_mm, valid
    low_mm = max(10.0, float(cfg.dt35_occlusion_min_cm) * 10.0)
    high_mm = max(low_mm + 1.0, float(cfg.dt35_occlusion_max_cm) * 10.0)
    if valid and distance_mm > low_mm + 30.0:
        high_mm = min(high_mm, distance_mm - 20.0)
    return rng.uniform(low_mm, high_mm), True


def _pose_path(name: str, samples: int, seed: int = 7) -> list[tuple[float, float, float]]:
    if name == "static_start":
        return [(0.0, 0.0, 0.0) for _ in range(samples)]
    if name == "forest_side":
        return _polyline_path(
            [
                (-550.0, -160.0, 0.0),
                (-550.0, 220.0, 0.0),
                (-70.0, 300.0, 0.0),
                (-70.0, -160.0, 0.0),
            ],
            samples,
        )
    if name == "left_forest_loop":
        return _left_forest_loop_path(samples)
    if name == "ramp_side":
        return _line_path(-430.0, -420.0, -360.0, -420.0, 0.0, samples)
    if name == "center_divider":
        return _line_path(120.0, 0.0, 40.0, 0.0, 0.0, samples)
    if name == "yaw_sweep":
        return [(160.0, -90.0, wrap_deg(-120.0 + 240.0 * i / max(1, samples - 1))) for i in range(samples)]
    if name == "start_corner_yaw_sweep":
        return [(0.0, 0.0, wrap_deg(-120.0 + 240.0 * i / max(1, samples - 1))) for i in range(samples)]
    if name == "field_patrol":
        return _polyline_path(
            [
                (0.0, 0.0, 0.0),
                (-230.0, 330.0, 90.0),
                (-360.0, 520.0, 90.0),
                (-550.0, 180.0, 0.0),
                (-550.0, -160.0, 0.0),
                (-70.0, -160.0, 0.0),
                (-400.0, -420.0, 0.0),
                (-120.0, -320.0, -90.0),
                (40.0, -180.0, 0.0),
                (40.0, 220.0, 0.0),
                (260.0, 330.0, -90.0),
                (80.0, -20.0, 0.0),
            ],
            samples,
        )
    if name == "random_patrol":
        return _random_patrol_path(samples, seed)
    return _top_corridor_path(samples)


def _top_corridor_path(samples: int) -> list[tuple[float, float, float]]:
    poses: list[tuple[float, float, float]] = []
    for i in range(samples):
        t = i / max(1, samples - 1)
        x = 0.0 + 330.0 * t
        y = 0.0 + 4.0 * sin(t * 2.0 * pi)
        yaw = 2.0 * sin(t * 2.0 * pi)
        poses.append((x, y, yaw))
    return poses


def _left_forest_loop_path(samples: int) -> list[tuple[float, float, float]]:
    """Clockwise loop around the red-side forest obstacle.

    Coordinates are field-center based cm. The red forest model spans roughly
    x=[-487.5,-127.5], y=[-195,285]; this path leaves clearance for an 83 cm
    chassis while still exercising all four faces with the side-mounted DT35s.
    The robot starts and ends at the top-left red start-zone center with yaw=0
    so replayed synthetic data uses the same convention as lidar startup.
    The straight legs keep yaw fixed; short corner phases rotate in place so
    the DT35 rays remain physically interpretable.
    """
    start_x = -552.5
    start_y = 549.0
    start_yaw = 0.0
    x_left = -552.5
    x_right = -62.5
    y_top = 330.0
    y_bottom = -245.0
    turn_weight = 70.0
    return _weighted_piecewise_path(
        [
            (start_x, start_y, start_yaw, start_x, start_y, 180.0, turn_weight),
            (start_x, start_y, 180.0, x_left, y_top, 180.0, abs(start_y - y_top)),
            (x_left, y_top, 180.0, x_left, y_bottom, 180.0, abs(y_top - y_bottom)),
            (x_left, y_bottom, 180.0, x_left, y_bottom, 90.0, turn_weight),
            (x_left, y_bottom, 90.0, x_right, y_bottom, 90.0, abs(x_right - x_left)),
            (x_right, y_bottom, 90.0, x_right, y_bottom, 0.0, turn_weight),
            (x_right, y_bottom, 0.0, x_right, y_top, 0.0, abs(y_top - y_bottom)),
            (x_right, y_top, 0.0, x_right, y_top, -90.0, turn_weight),
            (x_right, y_top, -90.0, x_left, y_top, -90.0, abs(x_right - x_left)),
            (x_left, y_top, -90.0, x_left, y_top, 0.0, turn_weight),
            (x_left, y_top, 0.0, start_x, start_y, start_yaw, abs(start_y - y_top)),
        ],
        samples,
    )


def _line_path(x0: float, y0: float, x1: float, y1: float, yaw_deg: float, samples: int) -> list[tuple[float, float, float]]:
    poses: list[tuple[float, float, float]] = []
    for i in range(samples):
        t = i / max(1, samples - 1)
        poses.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, yaw_deg))
    return poses


def _polyline_path(points: list[tuple[float, float, float]], samples: int) -> list[tuple[float, float, float]]:
    if len(points) < 2:
        return points * max(1, samples)
    segment_lengths: list[float] = []
    total = 0.0
    for (x0, y0, _), (x1, y1, _) in zip(points, points[1:]):
        length = ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
        segment_lengths.append(length)
        total += length
    if total <= 1.0e-6:
        return [points[0] for _ in range(samples)]

    poses: list[tuple[float, float, float]] = []
    for i in range(samples):
        distance = total * i / max(1, samples - 1)
        acc = 0.0
        segment_index = len(segment_lengths) - 1
        for index, length in enumerate(segment_lengths):
            if distance <= acc + length or index == len(segment_lengths) - 1:
                segment_index = index
                break
            acc += length
        length = max(1.0e-6, segment_lengths[segment_index])
        t = min(1.0, max(0.0, (distance - acc) / length))
        x0, y0, yaw0 = points[segment_index]
        x1, y1, yaw1 = points[segment_index + 1]
        poses.append((
            x0 + (x1 - x0) * t,
            y0 + (y1 - y0) * t,
            wrap_deg(yaw0 + wrap_deg(yaw1 - yaw0) * t),
        ))
    return poses


def _weighted_piecewise_path(
    segments: list[tuple[float, float, float, float, float, float, float]],
    samples: int,
) -> list[tuple[float, float, float]]:
    if not segments:
        return [(0.0, 0.0, 0.0) for _ in range(samples)]
    total = sum(max(0.0, segment[6]) for segment in segments)
    if total <= 1.0e-6:
        x0, y0, yaw0 = segments[0][:3]
        return [(x0, y0, yaw0) for _ in range(samples)]

    poses: list[tuple[float, float, float]] = []
    for i in range(samples):
        distance = total * i / max(1, samples - 1)
        acc = 0.0
        segment = segments[-1]
        for candidate in segments:
            weight = max(0.0, candidate[6])
            if distance <= acc + weight or candidate is segments[-1]:
                segment = candidate
                break
            acc += weight
        weight = max(1.0e-6, segment[6])
        t = min(1.0, max(0.0, (distance - acc) / weight))
        x0, y0, yaw0, x1, y1, yaw1, _ = segment
        poses.append((
            x0 + (x1 - x0) * t,
            y0 + (y1 - y0) * t,
            wrap_deg(yaw0 + wrap_deg(yaw1 - yaw0) * t),
        ))
    return poses


def _random_patrol_path(samples: int, seed: int) -> list[tuple[float, float, float]]:
    rng = Random(seed)
    anchors = [
        (0.0, 0.0, 0.0),
        (-230.0, 330.0, 90.0),
        (-360.0, 520.0, 90.0),
        (-550.0, 180.0, 0.0),
        (-550.0, -160.0, 0.0),
        (-70.0, -160.0, 0.0),
        (-400.0, -420.0, 0.0),
        (-120.0, -320.0, -90.0),
        (40.0, -180.0, 0.0),
        (40.0, 220.0, 0.0),
        (260.0, 330.0, -90.0),
        (520.0, -360.0, -90.0),
        (80.0, -20.0, 0.0),
    ]
    waypoint_count = rng.randint(7, 10)
    points = [anchors[0]]
    pool = anchors[1:-1]
    for _ in range(waypoint_count - 2):
        x, y, yaw = rng.choice(pool)
        points.append((
            x + rng.uniform(-35.0, 35.0),
            y + rng.uniform(-35.0, 35.0),
            wrap_deg(yaw + rng.uniform(-25.0, 25.0)),
        ))
    points.append(anchors[-1])
    return _polyline_path(points, samples)


def _synthetic_dt35_distance(
    local_x_cm: float,
    local_y_cm: float,
    local_yaw_deg: float,
    sensor_cfg: dict[str, Any],
    field_model: dict[str, Any],
    start_tf: tuple[float, float, float, bool],
    rng: Random,
    noise_mm: float,
) -> tuple[float, bool]:
    if not bool(sensor_cfg.get("enabled", True)):
        return 0.0, False
    field_x, field_y, field_yaw = _local_pose_to_field(local_x_cm, local_y_cm, local_yaw_deg, start_tf)
    sensor_x, sensor_y = robot_local_to_world(
        field_x,
        field_y,
        field_yaw,
        float(sensor_cfg.get("offset_x_cm", 0.0)),
        float(sensor_cfg.get("offset_y_cm", 0.0)),
    )
    ray_yaw = field_yaw + float(sensor_cfg.get("yaw_offset_deg", 0.0))
    hit = expected_dt35_hit(sensor_x, sensor_y, ray_yaw, field_model)
    if hit is None or str(hit["target_type"]) == "ignore":
        return 0.0, False
    distance_cm = float(hit["distance_cm"])
    max_range_cm = float(sensor_cfg.get("max_range_cm", 250.0))
    if not (0.0 < distance_cm <= max_range_cm):
        return 0.0, False
    distance_bias_mm = float(sensor_cfg.get("distance_bias_mm", 0.0))
    raw_distance_mm = distance_cm * 10.0 - distance_bias_mm
    return max(0.0, raw_distance_mm + _noise(rng, noise_mm)), True


def _field_model_from_config(config: dict[str, Any]) -> dict[str, Any]:
    model = dict(config.get("field_model", {}))
    map_cfg = config.get("map", {})
    model.setdefault("enabled", True)
    model.setdefault("use_field_boundary", True)
    model.setdefault("field_width_cm", map_cfg.get("field_width_cm", 1215.0))
    model.setdefault("field_height_cm", map_cfg.get("field_height_cm", 1210.0))
    return model


def _start_transform(config: dict[str, Any]) -> tuple[float, float, float, bool]:
    robot = config.get("robot", {})
    policy = str(robot.get("start_pose_policy", "off"))
    side = str(robot.get("default_start_side", "none"))
    if policy == "off" or side not in ("red", "blue"):
        return 0.0, 0.0, 0.0, False
    pose = robot.get(f"start_pose_{side}", {})
    return float(pose.get("x_cm", 0.0)), float(pose.get("y_cm", 0.0)), float(pose.get("yaw_deg", 0.0)), True


def _local_pose_to_field(x_cm: float, y_cm: float, yaw_deg: float,
                         start_tf: tuple[float, float, float, bool]) -> tuple[float, float, float]:
    ox, oy, oyaw, enabled = start_tf
    if not enabled:
        return x_cm, y_cm, yaw_deg
    yaw = radians(oyaw)
    return (
        ox + x_cm * cos(yaw) - y_cm * sin(yaw),
        oy + x_cm * sin(yaw) + y_cm * cos(yaw),
        yaw_deg + oyaw,
    )


def _status_mask(dt35_1_valid: bool, dt35_2_valid: bool, encoder_1_seen: bool, encoder_2_seen: bool) -> int:
    status = 0
    for bit in (1, 2, 3, 6):
        status |= 1 << bit
    if encoder_1_seen and encoder_2_seen:
        status |= 1 << 0
    if dt35_1_valid:
        status |= 1 << 4
    if dt35_2_valid:
        status |= 1 << 5
    if encoder_1_seen:
        status |= 1 << 10
    if encoder_2_seen:
        status |= 1 << 11
    return status


def _noise(rng: Random, amplitude: float) -> float:
    if amplitude <= 0.0:
        return 0.0
    return rng.uniform(-amplitude, amplitude)

