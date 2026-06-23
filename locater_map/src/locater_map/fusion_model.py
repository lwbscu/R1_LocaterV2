from __future__ import annotations

import csv
import json
from dataclasses import asdict, dataclass, replace
from math import atan2, cos, degrees, hypot, isfinite, radians, sin
from pathlib import Path
from typing import Any

from .data_model import RobotFrame
from .utils_transform import dt35_ray, heading_vector_from_front_yaw


def wrap_deg(angle: float) -> float:
    while angle > 180.0:
        angle -= 360.0
    while angle <= -180.0:
        angle += 360.0
    return angle


def angle_lerp_deg(a: float, b: float, gain: float) -> float:
    return wrap_deg(a + wrap_deg(b - a) * gain)


@dataclass(slots=True)
class FusionConfig:
    lidar_stride: int = 1
    lidar_gain: float = 1.0
    encoder_scale_learning: bool = True
    encoder_scale_learning_gain: float = 0.35
    encoder_scale_min_delta_cm: float = 20.0
    encoder_scale_min: float = 0.85
    encoder_scale_max: float = 1.15
    dt35_gain: float = 1.0
    dt35_yaw_gain: float = 0.0
    dt35_correct_lidar_frames: bool = False
    dt35_residual_gate_cm: float = 40.0
    dt35_max_translation_step_cm: float = 12.0
    dt35_max_yaw_step_deg: float = 3.0
    dt35_numeric_step_cm: float = 1.0
    dt35_numeric_step_yaw_deg: float = 0.5
    dt35_damping: float = 0.05
    use_dt35: bool = True


@dataclass(slots=True)
class FusionMetrics:
    frames: int = 0
    lidar_used_frames: int = 0
    lidar_holdout_frames: int = 0
    rms_x_cm: float | None = None
    rms_y_cm: float | None = None
    rms_xy_cm: float | None = None
    rms_yaw_deg: float | None = None
    max_xy_cm: float | None = None
    heading_delta_deg: float | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True)
class FusionResult:
    frames: list[RobotFrame]
    metrics: FusionMetrics


class LiveFusionFilter:
    def __init__(
        self,
        cfg: FusionConfig | None = None,
        app_config: dict[str, Any] | None = None,
        *,
        use_start_transform: bool = False,
    ) -> None:
        self.cfg = cfg or FusionConfig()
        self.app_config = app_config or {}
        self.field_model = _field_model_from_config(self.app_config)
        self.dt35_cfg = self.app_config.get("dt35", {})
        self.start_tf = _start_transform(self.app_config) if use_start_transform else (0.0, 0.0, 0.0, False)
        self.anchor: _Anchor | None = None
        self.encoder_scale = _EncoderScale()
        self._frame_index = 0
        self._last_seq: int | None = None

    def reset(self) -> None:
        self.anchor = None
        self.encoder_scale = _EncoderScale()
        self._frame_index = 0
        self._last_seq = None

    def process(self, frame: RobotFrame) -> RobotFrame:
        if self._last_seq is not None and frame.seq < self._last_seq:
            self.reset()
        self._last_seq = frame.seq

        stride = max(1, int(self.cfg.lidar_stride))
        lidar_gain = min(1.0, max(0.0, float(self.cfg.lidar_gain)))
        lidar_valid = frame.lidar_valid or frame.lidar_online
        use_lidar = lidar_valid and self._frame_index % stride == 0

        if self.anchor is None:
            base_x = frame.lidar_x_cm if lidar_valid else frame.pos_x_cm
            base_y = frame.lidar_y_cm if lidar_valid else frame.pos_y_cm
            h30_yaw_valid = frame.h30_valid or frame.h30_has_attitude
            base_yaw = frame.lidar_yaw_deg if lidar_valid else frame.h30_yaw_deg if h30_yaw_valid else frame.pos_yaw_deg
            self.anchor = _Anchor(base_x, base_y, base_yaw, frame.encoder_x_cm, frame.encoder_y_cm, frame.h30_yaw_deg)

        pred_x = self.anchor.x_cm + (frame.encoder_x_cm - self.anchor.encoder_x_cm) * self.encoder_scale.x
        pred_y = self.anchor.y_cm + (frame.encoder_y_cm - self.anchor.encoder_y_cm) * self.encoder_scale.y
        if frame.h30_valid or frame.h30_has_attitude:
            pred_yaw = wrap_deg(self.anchor.yaw_deg + wrap_deg(frame.h30_yaw_deg - self.anchor.h30_yaw_deg))
        else:
            pred_yaw = frame.pos_yaw_deg

        if use_lidar:
            self.encoder_scale = _learn_encoder_scale(self.encoder_scale, self.anchor, frame, self.cfg)
            fused_x = pred_x + (frame.lidar_x_cm - pred_x) * lidar_gain
            fused_y = pred_y + (frame.lidar_y_cm - pred_y) * lidar_gain
            fused_yaw = angle_lerp_deg(pred_yaw, frame.lidar_yaw_deg, lidar_gain)
            self.anchor = _Anchor(fused_x, fused_y, fused_yaw, frame.encoder_x_cm, frame.encoder_y_cm, frame.h30_yaw_deg)
        else:
            fused_x, fused_y, fused_yaw = pred_x, pred_y, pred_yaw

        if self.cfg.use_dt35 and self.cfg.dt35_gain > 0.0 and (not use_lidar or self.cfg.dt35_correct_lidar_frames):
            fused_x, fused_y, fused_yaw = _apply_dt35_correction(
                fused_x,
                fused_y,
                fused_yaw,
                frame,
                self.dt35_cfg,
                self.field_model,
                self.start_tf,
                self.cfg,
            )

        self._frame_index += 1
        return replace(frame, pos_x_cm=fused_x, pos_y_cm=fused_y, pos_yaw_deg=fused_yaw, status=frame.status | (1 << 6))


@dataclass(slots=True)
class _Anchor:
    x_cm: float
    y_cm: float
    yaw_deg: float
    encoder_x_cm: float
    encoder_y_cm: float
    h30_yaw_deg: float


@dataclass(slots=True)
class _EncoderScale:
    x: float = 1.0
    y: float = 1.0


def load_frames_csv(path: str | Path) -> list[RobotFrame]:
    with Path(path).open("r", encoding="utf-8", newline="") as f:
        return [RobotFrame.from_row(row) for row in csv.DictReader(f)]


def write_frames_csv(path: str | Path, frames: list[RobotFrame]) -> None:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=RobotFrame.field_names())
        writer.writeheader()
        for frame in frames:
            writer.writerow(frame.to_row())


def write_metrics_json(path: str | Path, metrics: FusionMetrics, config: FusionConfig) -> None:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    payload = {"config": asdict(config), "metrics": metrics.to_dict()}
    with out.open("w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)


def simulate_fusion(
    frames: list[RobotFrame],
    cfg: FusionConfig,
    app_config: dict[str, Any] | None = None,
) -> FusionResult:
    if not frames:
        return FusionResult([], FusionMetrics())

    stride = max(1, int(cfg.lidar_stride))
    lidar_gain = min(1.0, max(0.0, float(cfg.lidar_gain)))
    field_model = _field_model_from_config(app_config or {})
    dt35_cfg = (app_config or {}).get("dt35", {})
    start_tf = (0.0, 0.0, 0.0, False)

    out: list[RobotFrame] = []
    anchor: _Anchor | None = None
    encoder_scale = _EncoderScale()
    lidar_used = 0
    holdout_errors: list[tuple[float, float, float]] = []

    for index, frame in enumerate(frames):
        lidar_valid = frame.lidar_valid or frame.lidar_online
        use_lidar = lidar_valid and index % stride == 0

        if anchor is None:
            base_x = frame.lidar_x_cm if lidar_valid else frame.pos_x_cm
            base_y = frame.lidar_y_cm if lidar_valid else frame.pos_y_cm
            h30_yaw_valid = frame.h30_valid or frame.h30_has_attitude
            base_yaw = frame.lidar_yaw_deg if lidar_valid else frame.h30_yaw_deg if h30_yaw_valid else frame.pos_yaw_deg
            anchor = _Anchor(base_x, base_y, base_yaw, frame.encoder_x_cm, frame.encoder_y_cm, frame.h30_yaw_deg)

        pred_x = anchor.x_cm + (frame.encoder_x_cm - anchor.encoder_x_cm) * encoder_scale.x
        pred_y = anchor.y_cm + (frame.encoder_y_cm - anchor.encoder_y_cm) * encoder_scale.y
        if frame.h30_valid or frame.h30_has_attitude:
            pred_yaw = wrap_deg(anchor.yaw_deg + wrap_deg(frame.h30_yaw_deg - anchor.h30_yaw_deg))
        else:
            pred_yaw = frame.pos_yaw_deg

        if use_lidar:
            lidar_used += 1
            encoder_scale = _learn_encoder_scale(encoder_scale, anchor, frame, cfg)
            fused_x = pred_x + (frame.lidar_x_cm - pred_x) * lidar_gain
            fused_y = pred_y + (frame.lidar_y_cm - pred_y) * lidar_gain
            fused_yaw = angle_lerp_deg(pred_yaw, frame.lidar_yaw_deg, lidar_gain)
            anchor = _Anchor(fused_x, fused_y, fused_yaw, frame.encoder_x_cm, frame.encoder_y_cm, frame.h30_yaw_deg)
        else:
            fused_x, fused_y, fused_yaw = pred_x, pred_y, pred_yaw

        if cfg.use_dt35 and cfg.dt35_gain > 0.0 and (not use_lidar or cfg.dt35_correct_lidar_frames):
            fused_x, fused_y, fused_yaw = _apply_dt35_correction(
                fused_x,
                fused_y,
                fused_yaw,
                frame,
                dt35_cfg,
                field_model,
                start_tf,
                cfg,
            )

        sim = replace(frame, pos_x_cm=fused_x, pos_y_cm=fused_y, pos_yaw_deg=fused_yaw)
        out.append(sim)

        if lidar_valid and not use_lidar:
            holdout_errors.append((fused_x - frame.lidar_x_cm, fused_y - frame.lidar_y_cm, wrap_deg(fused_yaw - frame.lidar_yaw_deg)))

    metrics = _metrics(frames, out, holdout_errors, lidar_used)
    return FusionResult(out, metrics)


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
    return (
        float(pose.get("x_cm", 0.0)),
        float(pose.get("y_cm", 0.0)),
        float(pose.get("yaw_deg", 0.0)),
        True,
    )


def _learn_encoder_scale(scale: _EncoderScale, anchor: _Anchor, frame: RobotFrame, cfg: FusionConfig) -> _EncoderScale:
    if not cfg.encoder_scale_learning:
        return scale
    gain = min(1.0, max(0.0, float(cfg.encoder_scale_learning_gain)))
    if gain <= 0.0:
        return scale
    min_delta = max(0.0, float(cfg.encoder_scale_min_delta_cm))
    low = min(float(cfg.encoder_scale_min), float(cfg.encoder_scale_max))
    high = max(float(cfg.encoder_scale_min), float(cfg.encoder_scale_max))
    x = _learn_axis_scale(
        current=scale.x,
        encoder_delta=frame.encoder_x_cm - anchor.encoder_x_cm,
        lidar_delta=frame.lidar_x_cm - anchor.x_cm,
        gain=gain,
        min_delta=min_delta,
        low=low,
        high=high,
    )
    y = _learn_axis_scale(
        current=scale.y,
        encoder_delta=frame.encoder_y_cm - anchor.encoder_y_cm,
        lidar_delta=frame.lidar_y_cm - anchor.y_cm,
        gain=gain,
        min_delta=min_delta,
        low=low,
        high=high,
    )
    return _EncoderScale(x, y)


def _learn_axis_scale(
    *,
    current: float,
    encoder_delta: float,
    lidar_delta: float,
    gain: float,
    min_delta: float,
    low: float,
    high: float,
) -> float:
    if abs(encoder_delta) < min_delta or abs(lidar_delta) < min_delta:
        return current
    measured = lidar_delta / encoder_delta
    if not isfinite(measured) or measured <= 0.0:
        return current
    measured = _clamp(measured, low, high)
    return _clamp(current + (measured - current) * gain, low, high)


def _local_pose_to_field(x_cm: float, y_cm: float, yaw_deg: float,
                         start_tf: tuple[float, float, float, bool]) -> tuple[float, float, float]:
    ox, oy, oyaw, enabled = start_tf
    if not enabled:
        return x_cm, y_cm, yaw_deg
    start_yaw = radians(oyaw)
    start_sin = sin(start_yaw)
    start_cos = cos(start_yaw)
    return (
        ox + x_cm * start_cos - y_cm * start_sin,
        oy + x_cm * start_sin + y_cm * start_cos,
        yaw_deg + oyaw,
    )


def _field_vector_to_local(dx_cm: float, dy_cm: float,
                           start_tf: tuple[float, float, float, bool]) -> tuple[float, float]:
    _, _, oyaw, enabled = start_tf
    if not enabled:
        return dx_cm, dy_cm
    start_yaw = radians(oyaw)
    start_sin = sin(start_yaw)
    start_cos = cos(start_yaw)
    return dx_cm * start_cos + dy_cm * start_sin, -dx_cm * start_sin + dy_cm * start_cos


def _apply_dt35_correction(
    x_cm: float,
    y_cm: float,
    yaw_deg: float,
    frame: RobotFrame,
    dt35_cfg: dict[str, Any],
    field_model: dict[str, Any],
    start_tf: tuple[float, float, float, bool],
    cfg: FusionConfig,
) -> tuple[float, float, float]:
    observations = _dt35_observations(x_cm, y_cm, yaw_deg, frame, dt35_cfg, field_model, start_tf, cfg.dt35_residual_gate_cm)
    if not observations:
        return x_cm, y_cm, yaw_deg

    yaw_gain = min(1.0, max(0.0, float(cfg.dt35_yaw_gain)))
    if yaw_gain <= 0.0:
        return _apply_dt35_translation_only_correction(x_cm, y_cm, yaw_deg, frame, dt35_cfg, field_model, start_tf, cfg, observations)

    step_x = max(0.1, float(cfg.dt35_numeric_step_cm))
    step_y = step_x
    step_yaw = max(0.05, float(cfg.dt35_numeric_step_yaw_deg))

    rows: list[tuple[list[float], float, float]] = []
    for obs in observations:
        key = str(obs["key"])
        residual = float(obs["residual_cm"])
        weight = float(obs["weight"])
        jac = [
            _dt35_residual_derivative(x_cm, y_cm, yaw_deg, frame, dt35_cfg, field_model, start_tf, key, 0, step_x, cfg.dt35_residual_gate_cm),
            _dt35_residual_derivative(x_cm, y_cm, yaw_deg, frame, dt35_cfg, field_model, start_tf, key, 1, step_y, cfg.dt35_residual_gate_cm),
            _dt35_residual_derivative(x_cm, y_cm, yaw_deg, frame, dt35_cfg, field_model, start_tf, key, 2, step_yaw, cfg.dt35_residual_gate_cm),
        ]
        if all(isfinite(item) for item in jac):
            rows.append((jac, residual, weight))

    if not rows:
        return _apply_dt35_ray_fallback(x_cm, y_cm, yaw_deg, observations, start_tf, cfg)

    damping = max(1.0e-6, float(cfg.dt35_damping))
    normal = [[0.0, 0.0, 0.0] for _ in range(3)]
    rhs = [0.0, 0.0, 0.0]
    for jac, residual, weight in rows:
        w = max(0.0, weight)
        for i in range(3):
            rhs[i] += -w * jac[i] * residual
            for j in range(3):
                normal[i][j] += w * jac[i] * jac[j]
    normal[0][0] += damping
    normal[1][1] += damping
    normal[2][2] += damping

    delta = _solve_3x3(normal, rhs)
    if delta is None:
        return _apply_dt35_ray_fallback(x_cm, y_cm, yaw_deg, observations, start_tf, cfg)

    translation_gain = min(1.0, max(0.0, float(cfg.dt35_gain)))
    dx = _clamp(delta[0] * translation_gain, -float(cfg.dt35_max_translation_step_cm), float(cfg.dt35_max_translation_step_cm))
    dy = _clamp(delta[1] * translation_gain, -float(cfg.dt35_max_translation_step_cm), float(cfg.dt35_max_translation_step_cm))
    dyaw = _clamp(delta[2] * yaw_gain, -float(cfg.dt35_max_yaw_step_deg), float(cfg.dt35_max_yaw_step_deg))
    return x_cm + dx, y_cm + dy, wrap_deg(yaw_deg + dyaw)


def _apply_dt35_translation_only_correction(
    x_cm: float,
    y_cm: float,
    yaw_deg: float,
    frame: RobotFrame,
    dt35_cfg: dict[str, Any],
    field_model: dict[str, Any],
    start_tf: tuple[float, float, float, bool],
    cfg: FusionConfig,
    observations: list[dict[str, float | str]],
) -> tuple[float, float, float]:
    step = max(0.1, float(cfg.dt35_numeric_step_cm))
    rows: list[tuple[tuple[float, float], float, float]] = []
    for obs in observations:
        key = str(obs["key"])
        residual = float(obs["residual_cm"])
        weight = float(obs["weight"])
        jx = _dt35_residual_derivative(x_cm, y_cm, yaw_deg, frame, dt35_cfg, field_model, start_tf, key, 0, step, cfg.dt35_residual_gate_cm)
        jy = _dt35_residual_derivative(x_cm, y_cm, yaw_deg, frame, dt35_cfg, field_model, start_tf, key, 1, step, cfg.dt35_residual_gate_cm)
        if isfinite(jx) and isfinite(jy):
            rows.append(((jx, jy), residual, weight))

    if not rows:
        return _apply_dt35_ray_fallback(x_cm, y_cm, yaw_deg, observations, start_tf, cfg)

    damping = max(1.0e-6, float(cfg.dt35_damping))
    a00 = damping
    a01 = 0.0
    a11 = damping
    b0 = 0.0
    b1 = 0.0
    for (jx, jy), residual, weight in rows:
        w = max(0.0, weight)
        b0 += -w * jx * residual
        b1 += -w * jy * residual
        a00 += w * jx * jx
        a01 += w * jx * jy
        a11 += w * jy * jy

    delta = _solve_2x2(a00, a01, a11, b0, b1)
    if delta is None:
        return _apply_dt35_ray_fallback(x_cm, y_cm, yaw_deg, observations, start_tf, cfg)

    gain = min(1.0, max(0.0, float(cfg.dt35_gain)))
    dx = _clamp(delta[0] * gain, -float(cfg.dt35_max_translation_step_cm), float(cfg.dt35_max_translation_step_cm))
    dy = _clamp(delta[1] * gain, -float(cfg.dt35_max_translation_step_cm), float(cfg.dt35_max_translation_step_cm))
    return x_cm + dx, y_cm + dy, yaw_deg


def _dt35_observations(
    x_cm: float,
    y_cm: float,
    yaw_deg: float,
    frame: RobotFrame,
    dt35_cfg: dict[str, Any],
    field_model: dict[str, Any],
    start_tf: tuple[float, float, float, bool],
    residual_gate_cm: float,
) -> list[dict[str, float | str]]:
    observations: list[dict[str, float | str]] = []
    field_x, field_y, field_yaw = _local_pose_to_field(x_cm, y_cm, yaw_deg, start_tf)
    for key, distance_mm, valid in (
        ("sensor_1", frame.dt35_1_mm, frame.dt35_1_valid),
        ("sensor_2", frame.dt35_2_mm, frame.dt35_2_valid),
    ):
        if not valid:
            continue
        sensor_cfg = dt35_cfg.get(key, {})
        ray = dt35_ray(field_x, field_y, field_yaw, sensor_cfg, distance_mm, field_model)
        if not bool(ray.get("correction_allowed", False)):
            continue
        if bool(ray.get("floor_hit_suspect", False)):
            continue
        residual = float(ray["residual_cm"])
        if not isfinite(residual) or abs(residual) > residual_gate_cm:
            continue
        observations.append({
            "key": key,
            "residual_cm": residual,
            "weight": float(ray.get("correction_weight", 1.0)),
            "ray_yaw_deg": float(ray["ray_yaw_deg"]),
        })
    return observations


def _dt35_single_residual(
    x_cm: float,
    y_cm: float,
    yaw_deg: float,
    frame: RobotFrame,
    dt35_cfg: dict[str, Any],
    field_model: dict[str, Any],
    start_tf: tuple[float, float, float, bool],
    key: str,
    residual_gate_cm: float,
) -> float | None:
    if key == "sensor_1":
        distance_mm = frame.dt35_1_mm
        valid = frame.dt35_1_valid
    else:
        distance_mm = frame.dt35_2_mm
        valid = frame.dt35_2_valid
    if not valid:
        return None
    field_x, field_y, field_yaw = _local_pose_to_field(x_cm, y_cm, yaw_deg, start_tf)
    ray = dt35_ray(field_x, field_y, field_yaw, dt35_cfg.get(key, {}), distance_mm, field_model)
    if not bool(ray.get("correction_allowed", False)):
        return None
    if bool(ray.get("floor_hit_suspect", False)):
        return None
    residual = float(ray["residual_cm"])
    if not isfinite(residual) or abs(residual) > residual_gate_cm:
        return None
    return residual


def _dt35_residual_derivative(
    x_cm: float,
    y_cm: float,
    yaw_deg: float,
    frame: RobotFrame,
    dt35_cfg: dict[str, Any],
    field_model: dict[str, Any],
    start_tf: tuple[float, float, float, bool],
    key: str,
    axis: int,
    step: float,
    residual_gate_cm: float,
) -> float:
    plus = [x_cm, y_cm, yaw_deg]
    minus = [x_cm, y_cm, yaw_deg]
    plus[axis] += step
    minus[axis] -= step
    r_plus = _dt35_single_residual(plus[0], plus[1], plus[2], frame, dt35_cfg, field_model, start_tf, key, residual_gate_cm)
    r_minus = _dt35_single_residual(minus[0], minus[1], minus[2], frame, dt35_cfg, field_model, start_tf, key, residual_gate_cm)
    if r_plus is not None and r_minus is not None:
        return (r_plus - r_minus) / (2.0 * step)
    center = _dt35_single_residual(x_cm, y_cm, yaw_deg, frame, dt35_cfg, field_model, start_tf, key, residual_gate_cm)
    if center is not None and r_plus is not None:
        return (r_plus - center) / step
    if center is not None and r_minus is not None:
        return (center - r_minus) / step
    return float("nan")


def _apply_dt35_ray_fallback(
    x_cm: float,
    y_cm: float,
    yaw_deg: float,
    observations: list[dict[str, float | str]],
    start_tf: tuple[float, float, float, bool],
    cfg: FusionConfig,
) -> tuple[float, float, float]:
    corrections: list[tuple[float, float]] = []
    for obs in observations:
        residual = float(obs["residual_cm"])
        weight = float(obs.get("weight", 1.0))
        dx, dy = heading_vector_from_front_yaw(float(obs["ray_yaw_deg"]))
        local_dx, local_dy = _field_vector_to_local(-residual * dx * weight, -residual * dy * weight, start_tf)
        corrections.append((local_dx, local_dy))

    if not corrections:
        return x_cm, y_cm, yaw_deg
    avg_x = sum(item[0] for item in corrections) / len(corrections)
    avg_y = sum(item[1] for item in corrections) / len(corrections)
    gain = min(1.0, max(0.0, float(cfg.dt35_gain)))
    dx = _clamp(avg_x * gain, -float(cfg.dt35_max_translation_step_cm), float(cfg.dt35_max_translation_step_cm))
    dy = _clamp(avg_y * gain, -float(cfg.dt35_max_translation_step_cm), float(cfg.dt35_max_translation_step_cm))
    return x_cm + dx, y_cm + dy, yaw_deg


def _solve_3x3(matrix: list[list[float]], vector: list[float]) -> list[float] | None:
    aug = [matrix[i][:] + [vector[i]] for i in range(3)]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda row: abs(aug[row][col]))
        if abs(aug[pivot][col]) < 1.0e-9:
            return None
        if pivot != col:
            aug[col], aug[pivot] = aug[pivot], aug[col]
        pivot_value = aug[col][col]
        for k in range(col, 4):
            aug[col][k] /= pivot_value
        for row in range(3):
            if row == col:
                continue
            factor = aug[row][col]
            for k in range(col, 4):
                aug[row][k] -= factor * aug[col][k]
    return [aug[i][3] for i in range(3)]


def _solve_2x2(a00: float, a01: float, a11: float, b0: float, b1: float) -> tuple[float, float] | None:
    det = a00 * a11 - a01 * a01
    if abs(det) < 1.0e-9:
        return None
    return (b0 * a11 - a01 * b1) / det, (a00 * b1 - a01 * b0) / det


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _metrics(
    raw_frames: list[RobotFrame],
    sim_frames: list[RobotFrame],
    holdout_errors: list[tuple[float, float, float]],
    lidar_used_frames: int,
) -> FusionMetrics:
    metrics = FusionMetrics(
        frames=len(sim_frames),
        lidar_used_frames=lidar_used_frames,
        lidar_holdout_frames=len(holdout_errors),
    )
    if holdout_errors:
        metrics.rms_x_cm = _rms([e[0] for e in holdout_errors])
        metrics.rms_y_cm = _rms([e[1] for e in holdout_errors])
        metrics.rms_xy_cm = (sum(x * x + y * y for x, y, _ in holdout_errors) / len(holdout_errors)) ** 0.5
        metrics.rms_yaw_deg = _rms([e[2] for e in holdout_errors])
        metrics.max_xy_cm = max(hypot(e[0], e[1]) for e in holdout_errors)

    first = raw_frames[0]
    last = raw_frames[-1]
    sim_first = sim_frames[0]
    sim_last = sim_frames[-1]
    lidar_dx = last.lidar_x_cm - first.lidar_x_cm
    lidar_dy = last.lidar_y_cm - first.lidar_y_cm
    sim_dx = sim_last.pos_x_cm - sim_first.pos_x_cm
    sim_dy = sim_last.pos_y_cm - sim_first.pos_y_cm
    if hypot(lidar_dx, lidar_dy) >= 10.0 and hypot(sim_dx, sim_dy) >= 10.0:
        lidar_heading = degrees(atan2(lidar_dx, lidar_dy))
        sim_heading = degrees(atan2(sim_dx, sim_dy))
        metrics.heading_delta_deg = wrap_deg(sim_heading - lidar_heading)
    return metrics


def _rms(values: list[float]) -> float | None:
    if not values:
        return None
    return (sum(v * v for v in values) / len(values)) ** 0.5
