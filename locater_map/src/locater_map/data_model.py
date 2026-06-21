from __future__ import annotations

from dataclasses import asdict, dataclass, fields
from time import time
from typing import Any


STATUS_BITS = {
    0: "encoder_valid",
    1: "h30_valid",
    2: "lidar_valid",
    3: "lidar_online",
    4: "dt35_1_valid",
    5: "dt35_2_valid",
    6: "fusion_valid",
    7: "h30_error",
    8: "lidar_error",
    9: "encoder_index_seen",
    10: "encoder_1_valid",
    11: "encoder_2_valid",
}


def _row_get(row: dict[str, str], name: str, default: str = "") -> str:
    value = row.get(name)
    if value not in (None, ""):
        return value
    aliases = {
        "pos_x_cm": "fuse_x_cm",
        "pos_y_cm": "fuse_y_cm",
        "pos_yaw_deg": "fuse_yaw_deg",
        "calib_x_cm": "enc_x_cm",
        "calib_y_cm": "enc_y_cm",
        "calib_yaw_deg": "enc_yaw_deg",
    }
    alias = aliases.get(name)
    if alias:
        value = row.get(alias)
        if value not in (None, ""):
            return value
    return default


def _to_bool(value: str) -> bool:
    return str(value).strip().lower() in ("1", "true", "yes", "ok")


@dataclass(slots=True)
class RobotFrame:
    source_time_ms: int = 0
    pc_time: float = 0.0
    seq: int = 0

    pos_x_cm: float = 0.0
    pos_y_cm: float = 0.0
    pos_yaw_deg: float = 0.0
    calib_x_cm: float = 0.0
    calib_y_cm: float = 0.0
    calib_yaw_deg: float = 0.0
    h30_yaw_deg: float = 0.0
    h30_x_cm: float = 0.0
    h30_y_cm: float = 0.0
    encoder_x_cm: float = 0.0
    encoder_y_cm: float = 0.0
    lidar_x_cm: float = 0.0
    lidar_y_cm: float = 0.0
    lidar_yaw_deg: float = 0.0
    dt35_1_mm: float = 0.0
    dt35_2_mm: float = 0.0
    dt35_1_valid: bool = False
    dt35_2_valid: bool = False

    h30_valid: bool = False
    h30_has_attitude: bool = False
    h30_has_accel: bool = False
    lidar_valid: bool = False
    lidar_online: bool = False
    h30_packet_count: int = 0
    h30_rx_byte_count: int = 0
    lidar_packet_count: int = 0
    lidar_rx_byte_count: int = 0
    h30_crc_error_count: int = 0
    h30_frame_error_count: int = 0
    h30_last_update_ms: int = 0
    lidar_checksum_error_count: int = 0
    lidar_frame_error_count: int = 0
    lidar_last_update_ms: int = 0

    x_raw_count: int = 0
    y_raw_count: int = 0
    x_delta_count: int = 0
    y_delta_count: int = 0
    x_total_count: int = 0
    y_total_count: int = 0
    x_index_seen: bool = False
    y_index_seen: bool = False
    x_pulse_seen: bool = False
    y_pulse_seen: bool = False
    encoder_dis_p_mm: float = 0.0
    encoder_dis_q_mm: float = 0.0

    status: int = 0
    crc_ok: bool = True
    crc_state: str = "ok"
    raw_line: str = ""
    protocol: str = "r1m"

    @property
    def fuse_x_cm(self) -> float:
        return self.pos_x_cm

    @property
    def fuse_y_cm(self) -> float:
        return self.pos_y_cm

    @property
    def fuse_yaw_deg(self) -> float:
        return self.pos_yaw_deg

    @property
    def enc_x_cm(self) -> float:
        return self.calib_x_cm

    @property
    def enc_y_cm(self) -> float:
        return self.calib_y_cm

    @property
    def enc_yaw_deg(self) -> float:
        return self.calib_yaw_deg

    @classmethod
    def field_names(cls) -> list[str]:
        return [f.name for f in fields(cls)]

    def to_row(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_row(cls, row: dict[str, str]) -> "RobotFrame":
        kwargs: dict[str, Any] = {}
        for f in fields(cls):
            value = _row_get(row, f.name)
            if f.type in (int, "int"):
                kwargs[f.name] = int(float(value or 0))
            elif f.type in (float, "float"):
                kwargs[f.name] = float(value or 0)
            elif f.type in (bool, "bool"):
                kwargs[f.name] = _to_bool(value)
            else:
                kwargs[f.name] = value
        return cls(**kwargs)

    def status_dict(self) -> dict[str, bool]:
        return {name: bool(self.status & (1 << bit)) for bit, name in STATUS_BITS.items()}


def now_frame(**kwargs: Any) -> RobotFrame:
    kwargs.setdefault("pc_time", time())
    return RobotFrame(**kwargs)


@dataclass(slots=True)
class SerialStats:
    rx_bytes: int = 0
    rx_bytes_per_s: float = 0.0
    frames: int = 0
    frames_per_s: float = 0.0
    crc_errors: int = 0
    dropped_frames: int = 0
    parse_errors: int = 0
    last_frame_pc_time: float = 0.0
    last_interval_ms: float = 0.0
    connected: bool = False
    port: str = ""

    def copy(self) -> "SerialStats":
        return SerialStats(**asdict(self))
