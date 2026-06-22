from __future__ import annotations

from dataclasses import dataclass
from math import cos, radians, sin

from .fusion_model import wrap_deg


@dataclass(slots=True)
class FirmwareEncoderConfig:
    noise_threshold_count: int = 1
    scale_p_mm: float = 0.00771069429
    scale_q_mm: float = 0.00771069429
    offset_angle_deg: float = 225.0
    offset_l_mm: float = 268.16
    cross_p_from_q: float = 0.0
    cross_q_from_p: float = 0.0
    rot_comp_gain: float = 0.100
    pos_x_corr: float = -1.035
    pos_y_corr: float = -1.0366


@dataclass(slots=True)
class FirmwareEncoderSample:
    x_cm: float
    y_cm: float
    x_delta_count: int
    y_delta_count: int
    x_total_count: int
    y_total_count: int
    encoder_dis_p_mm: float
    encoder_dis_q_mm: float
    x_pulse_seen: bool
    y_pulse_seen: bool


class FirmwareEncoderSimulator:
    """Python mirror of Core/Application/task_locater.c:update_encoder_odometry."""

    def __init__(
        self,
        *,
        initial_x_cm: float,
        initial_y_cm: float,
        x_scale: float = 1.0,
        y_scale: float = 1.0,
        cfg: FirmwareEncoderConfig | None = None,
    ) -> None:
        self.cfg = cfg or FirmwareEncoderConfig()
        self.x_cm = float(initial_x_cm)
        self.y_cm = float(initial_y_cm)
        self.x_scale = float(x_scale)
        self.y_scale = float(y_scale)
        self.prev_yaw_deg = 0.0
        self.prev_yaw_ready = False
        self.x_total_count = 0
        self.y_total_count = 0
        self.x_pulse_seen = False
        self.y_pulse_seen = False

    def sample_for_truth_delta(self, dx_truth_cm: float, dy_truth_cm: float, h30_yaw_deg: float) -> FirmwareEncoderSample:
        enc_p, enc_q = self._counts_for_world_delta(
            dx_truth_mm=float(dx_truth_cm) * 10.0 * self.x_scale,
            dy_truth_mm=float(dy_truth_cm) * 10.0 * self.y_scale,
            yaw_deg=float(h30_yaw_deg),
        )
        return self.apply_counts(enc_p, enc_q, h30_yaw_deg)

    def sample_static(self, h30_yaw_deg: float) -> FirmwareEncoderSample:
        return self.apply_counts(0, 0, h30_yaw_deg)

    def apply_counts(self, enc_p: int, enc_q: int, yaw_deg: float) -> FirmwareEncoderSample:
        cfg = self.cfg
        dis_p_raw = float(enc_p) * cfg.scale_p_mm if abs(enc_p) > cfg.noise_threshold_count else 0.0
        dis_q_raw = -float(enc_q) * cfg.scale_q_mm if abs(enc_q) > cfg.noise_threshold_count else 0.0
        dis_p_base = dis_p_raw + dis_q_raw * cfg.cross_p_from_q
        dis_q_base = dis_q_raw + dis_p_raw * cfg.cross_q_from_p

        d_yaw = wrap_deg(float(yaw_deg) - self.prev_yaw_deg) if self.prev_yaw_ready else 0.0
        self.prev_yaw_ready = True
        self.prev_yaw_deg = float(yaw_deg)

        correction = cfg.offset_l_mm * radians(d_yaw) * 0.70710678118 * cfg.rot_comp_gain
        live_dis_p = dis_p_base - correction
        live_dis_q = dis_q_base + correction
        angle_rad = radians(wrap_deg(float(yaw_deg) + cfg.offset_angle_deg))
        dx_mm = live_dis_p * cos(angle_rad) - live_dis_q * sin(angle_rad)
        dy_mm = live_dis_p * sin(angle_rad) + live_dis_q * cos(angle_rad)
        dx_mm *= cfg.pos_x_corr
        dy_mm *= cfg.pos_y_corr

        self.x_cm += dx_mm * 0.1
        self.y_cm += dy_mm * 0.1
        self.x_total_count += int(enc_p)
        self.y_total_count += int(enc_q)
        if enc_p != 0:
            self.x_pulse_seen = True
        if enc_q != 0:
            self.y_pulse_seen = True

        return FirmwareEncoderSample(
            x_cm=self.x_cm,
            y_cm=self.y_cm,
            x_delta_count=int(enc_p),
            y_delta_count=int(enc_q),
            x_total_count=self.x_total_count,
            y_total_count=self.y_total_count,
            encoder_dis_p_mm=live_dis_p,
            encoder_dis_q_mm=live_dis_q,
            x_pulse_seen=self.x_pulse_seen,
            y_pulse_seen=self.y_pulse_seen,
        )

    def _counts_for_world_delta(self, *, dx_truth_mm: float, dy_truth_mm: float, yaw_deg: float) -> tuple[int, int]:
        cfg = self.cfg
        d_yaw = wrap_deg(float(yaw_deg) - self.prev_yaw_deg) if self.prev_yaw_ready else 0.0
        correction = cfg.offset_l_mm * radians(d_yaw) * 0.70710678118 * cfg.rot_comp_gain

        dx_before_corr = _safe_div(dx_truth_mm, cfg.pos_x_corr)
        dy_before_corr = _safe_div(dy_truth_mm, cfg.pos_y_corr)
        angle_rad = radians(wrap_deg(float(yaw_deg) + cfg.offset_angle_deg))
        live_p = dx_before_corr * cos(angle_rad) + dy_before_corr * sin(angle_rad)
        live_q = -dx_before_corr * sin(angle_rad) + dy_before_corr * cos(angle_rad)

        dis_p_base = live_p + correction
        dis_q_base = live_q - correction
        dis_p_raw, dis_q_raw = _invert_cross_coupling(dis_p_base, dis_q_base, cfg)
        enc_p = int(round(_safe_div(dis_p_raw, cfg.scale_p_mm)))
        enc_q = int(round(_safe_div(-dis_q_raw, cfg.scale_q_mm)))
        return enc_p, enc_q


def _safe_div(value: float, denom: float) -> float:
    if abs(denom) <= 1.0e-12:
        return 0.0
    return value / denom


def _invert_cross_coupling(dis_p_base: float, dis_q_base: float, cfg: FirmwareEncoderConfig) -> tuple[float, float]:
    det = 1.0 - cfg.cross_p_from_q * cfg.cross_q_from_p
    if abs(det) <= 1.0e-12:
        return dis_p_base, dis_q_base
    dis_p_raw = (dis_p_base - cfg.cross_p_from_q * dis_q_base) / det
    dis_q_raw = (-cfg.cross_q_from_p * dis_p_base + dis_q_base) / det
    return dis_p_raw, dis_q_raw
