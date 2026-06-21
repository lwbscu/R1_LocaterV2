from __future__ import annotations

from math import cos, radians, sin
from time import monotonic, time

from .data_model import RobotFrame


class DemoSource:
    def __init__(self) -> None:
        self._t0 = monotonic()
        self._seq = 0

    def next_frame(self) -> RobotFrame:
        t = monotonic() - self._t0
        self._seq += 1
        yaw = (t * 24.0) % 360.0
        x = 230.0 * cos(t * 0.22) - 120.0
        y = 180.0 * sin(t * 0.17) + 120.0
        enc_x = x + 4.0 * sin(t * 1.7)
        enc_y = y + 4.0 * cos(t * 1.3)
        lidar_x = x + 1.5 * sin(t * 0.8)
        lidar_y = y + 1.5 * cos(t * 0.9)
        h30_yaw = yaw + 1.2 * sin(t * 0.6)
        return RobotFrame(
            source_time_ms=int(t * 1000),
            pc_time=time(),
            seq=self._seq,
            pos_x_cm=x,
            pos_y_cm=y,
            pos_yaw_deg=yaw,
            calib_x_cm=enc_x,
            calib_y_cm=enc_y,
            calib_yaw_deg=h30_yaw,
            h30_x_cm=enc_x * 0.1,
            h30_y_cm=enc_y * 0.1,
            h30_yaw_deg=h30_yaw,
            encoder_x_cm=enc_x,
            encoder_y_cm=enc_y,
            lidar_x_cm=lidar_x,
            lidar_y_cm=lidar_y,
            lidar_yaw_deg=yaw - 0.8 * sin(radians(yaw)),
            h30_valid=True,
            h30_has_attitude=True,
            h30_has_accel=True,
            lidar_valid=True,
            lidar_online=True,
            h30_packet_count=self._seq * 2,
            h30_rx_byte_count=self._seq * 64,
            lidar_packet_count=self._seq,
            lidar_rx_byte_count=self._seq * 24,
            x_raw_count=self._seq * 8,
            y_raw_count=self._seq * 7,
            x_delta_count=8,
            y_delta_count=7,
            x_total_count=self._seq * 8,
            y_total_count=self._seq * 7,
            x_pulse_seen=True,
            y_pulse_seen=True,
            encoder_dis_p_mm=0.062,
            encoder_dis_q_mm=0.054,
            status=0x0C4F,
            raw_line="demo",
            protocol="r1_csv_v3",
        )
