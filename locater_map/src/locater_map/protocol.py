from __future__ import annotations

from dataclasses import dataclass
from time import time

from .crc16 import crc16_ccitt_false
from .data_model import RobotFrame


R1_CSV_V2_FIELD_COUNT = 25
R1_CSV_V2_DIAG_FIELD_COUNT = 41
R1_CSV_V3_FIELD_COUNT = 12


@dataclass(slots=True)
class ParseResult:
    frame: RobotFrame | None
    error: str | None = None
    raw_line: str = ""
    crc_error: bool = False
    no_crc: bool = False

    @property
    def ok(self) -> bool:
        return self.frame is not None and self.error is None


def _to_float(value: str) -> float:
    return float(value.strip())


def _to_int(value: str) -> int:
    text = value.strip()
    if text.lower().startswith("0x"):
        return int(text, 16)
    return int(float(text))


def _to_bool_num(value: float) -> bool:
    return int(value) != 0


def _value(values: list[float], index: int, default: float = 0.0) -> float:
    if index < len(values):
        return values[index]
    return default


def _parse_r1m(line: str, allow_no_crc: bool) -> ParseResult:
    raw = line.strip("\r\n")
    if not raw.startswith("$R1M,"):
        return ParseResult(None, "not_r1m", raw)

    crc_ok = True
    crc_state = "ok"
    no_crc = False
    payload = raw[1:]

    if "*" in payload:
        body, crc_text = payload.rsplit("*", 1)
        crc_text = crc_text.strip()
        try:
            received_crc = int(crc_text, 16)
        except ValueError:
            return ParseResult(None, "bad_crc_text", raw, crc_error=True)
        calc_crc = crc16_ccitt_false(body)
        if calc_crc != received_crc:
            return ParseResult(None, f"crc_mismatch calc={calc_crc:04X} got={received_crc:04X}", raw, crc_error=True)
        payload = body
    else:
        if not allow_no_crc:
            return ParseResult(None, "missing_crc", raw)
        no_crc = True
        crc_ok = False
        crc_state = "no_crc"

    payload = payload.rstrip(",")
    parts = [p.strip() for p in payload.split(",")]
    if len(parts) != 19 or parts[0] != "R1M":
        return ParseResult(None, f"r1m_field_count={len(parts)}", raw)

    try:
        status = _to_int(parts[18])
        frame = RobotFrame(
            source_time_ms=_to_int(parts[2]),
            pc_time=time(),
            seq=_to_int(parts[3]),
            pos_x_cm=_to_float(parts[4]),
            pos_y_cm=_to_float(parts[5]),
            pos_yaw_deg=_to_float(parts[6]),
            calib_x_cm=_to_float(parts[7]),
            calib_y_cm=_to_float(parts[8]),
            calib_yaw_deg=_to_float(parts[9]),
            encoder_x_cm=_to_float(parts[7]),
            encoder_y_cm=_to_float(parts[8]),
            h30_x_cm=_to_float(parts[10]),
            h30_y_cm=_to_float(parts[11]),
            h30_yaw_deg=_to_float(parts[12]),
            lidar_x_cm=_to_float(parts[13]),
            lidar_y_cm=_to_float(parts[14]),
            lidar_yaw_deg=_to_float(parts[15]),
            dt35_1_mm=_to_float(parts[16]),
            dt35_2_mm=_to_float(parts[17]),
            x_pulse_seen=bool(status & (1 << 10)),
            y_pulse_seen=bool(status & (1 << 11)),
            status=status,
            crc_ok=crc_ok,
            crc_state=crc_state,
            raw_line=raw,
            protocol="r1m",
        )
    except (TypeError, ValueError) as exc:
        return ParseResult(None, f"r1m_parse_error={exc}", raw)

    return ParseResult(frame, raw_line=raw, no_crc=no_crc)


def _parse_r1_csv_v2(values: list[float], raw: str) -> ParseResult:
    if len(values) < R1_CSV_V2_FIELD_COUNT:
        return ParseResult(None, f"r1_csv_v2_field_count={len(values)}", raw)

    status = int(values[24])
    x_delta_count = int(_value(values, 33))
    y_delta_count = int(_value(values, 34))
    x_total_count = int(_value(values, 35))
    y_total_count = int(_value(values, 36))
    frame = RobotFrame(
        pc_time=time(),
        pos_x_cm=values[0],
        pos_y_cm=values[1],
        pos_yaw_deg=values[2],
        lidar_x_cm=values[3],
        lidar_y_cm=values[4],
        lidar_yaw_deg=values[5],
        calib_x_cm=values[6],
        calib_y_cm=values[7],
        calib_yaw_deg=values[8],
        h30_yaw_deg=values[9],
        h30_x_cm=values[10],
        h30_y_cm=values[11],
        encoder_x_cm=values[12],
        encoder_y_cm=values[13],
        h30_valid=_to_bool_num(values[14]),
        h30_has_attitude=_to_bool_num(values[15]),
        lidar_valid=_to_bool_num(values[16]),
        lidar_online=_to_bool_num(values[17]),
        h30_packet_count=int(values[18]),
        lidar_packet_count=int(values[19]),
        h30_crc_error_count=int(values[20]),
        h30_frame_error_count=int(values[21]),
        lidar_checksum_error_count=int(values[22]),
        lidar_frame_error_count=int(values[23]),
        source_time_ms=int(_value(values, 25)),
        h30_rx_byte_count=int(_value(values, 26)),
        h30_has_accel=_to_bool_num(_value(values, 27)),
        h30_last_update_ms=int(_value(values, 28)),
        lidar_rx_byte_count=int(_value(values, 29)),
        lidar_last_update_ms=int(_value(values, 30)),
        x_raw_count=int(_value(values, 31)),
        y_raw_count=int(_value(values, 32)),
        x_delta_count=x_delta_count,
        y_delta_count=y_delta_count,
        x_total_count=x_total_count,
        y_total_count=y_total_count,
        x_index_seen=_to_bool_num(_value(values, 37)),
        y_index_seen=_to_bool_num(_value(values, 38)),
        x_pulse_seen=bool(status & (1 << 10)) or x_delta_count != 0 or x_total_count != 0,
        y_pulse_seen=bool(status & (1 << 11)) or y_delta_count != 0 or y_total_count != 0,
        encoder_dis_p_mm=_value(values, 39),
        encoder_dis_q_mm=_value(values, 40),
        status=status,
        crc_ok=False,
        crc_state="no_crc",
        raw_line=raw,
        protocol="r1_csv_v2",
    )
    return ParseResult(frame, raw_line=raw, no_crc=True)


def _parse_r1_csv_v3(values: list[float], raw: str) -> ParseResult:
    if len(values) != R1_CSV_V3_FIELD_COUNT:
        return ParseResult(None, f"r1_csv_v3_field_count={len(values)}", raw)

    status = int(values[11])
    h30_valid = bool(status & (1 << 1))
    lidar_valid = bool(status & (1 << 2))
    lidar_online = bool(status & (1 << 3))
    dt35_1_valid = bool(status & (1 << 4))
    dt35_2_valid = bool(status & (1 << 5))
    x_pulse_seen = bool(status & (1 << 10))
    y_pulse_seen = bool(status & (1 << 11))
    encoder_x = values[6]
    encoder_y = values[7]
    h30_yaw = values[8]
    frame = RobotFrame(
        pc_time=time(),
        pos_x_cm=values[0],
        pos_y_cm=values[1],
        pos_yaw_deg=values[2],
        lidar_x_cm=values[3],
        lidar_y_cm=values[4],
        lidar_yaw_deg=values[5],
        calib_x_cm=encoder_x,
        calib_y_cm=encoder_y,
        calib_yaw_deg=h30_yaw,
        encoder_x_cm=encoder_x,
        encoder_y_cm=encoder_y,
        h30_yaw_deg=h30_yaw,
        dt35_1_mm=values[9],
        dt35_2_mm=values[10],
        dt35_1_valid=dt35_1_valid,
        dt35_2_valid=dt35_2_valid,
        x_pulse_seen=x_pulse_seen,
        y_pulse_seen=y_pulse_seen,
        h30_valid=h30_valid,
        h30_has_attitude=h30_valid,
        lidar_valid=lidar_valid,
        lidar_online=lidar_online,
        status=status,
        crc_ok=False,
        crc_state="no_crc",
        raw_line=raw,
        protocol="r1_csv_v3",
    )
    return ParseResult(frame, raw_line=raw, no_crc=True)


def _parse_legacy_csv(line: str) -> ParseResult:
    raw = line.strip("\r\n")
    try:
        values = [float(p.strip()) for p in raw.split(",") if p.strip() != ""]
    except ValueError as exc:
        return ParseResult(None, f"legacy_parse_error={exc}", raw)

    if len(values) >= R1_CSV_V2_FIELD_COUNT:
        return _parse_r1_csv_v2(values, raw)
    if len(values) == R1_CSV_V3_FIELD_COUNT:
        return _parse_r1_csv_v3(values, raw)

    if len(values) == 5:
        yaw, h30_x, h30_y, enc_x, enc_y = values
        frame = RobotFrame(
            pc_time=time(),
            pos_x_cm=enc_x,
            pos_y_cm=enc_y,
            pos_yaw_deg=yaw,
            calib_x_cm=enc_x,
            calib_y_cm=enc_y,
            calib_yaw_deg=yaw,
            encoder_x_cm=enc_x,
            encoder_y_cm=enc_y,
            h30_x_cm=h30_x,
            h30_y_cm=h30_y,
            h30_yaw_deg=yaw,
            h30_valid=True,
            h30_has_attitude=True,
            status=0x0027,
            crc_ok=False,
            crc_state="no_crc",
            raw_line=raw,
            protocol="legacy_csv",
        )
    elif len(values) == 6:
        frame = RobotFrame(
            pc_time=time(),
            pos_x_cm=values[0],
            pos_y_cm=values[1],
            pos_yaw_deg=values[2],
            lidar_x_cm=values[3],
            lidar_y_cm=values[4],
            lidar_yaw_deg=values[5],
            lidar_valid=True,
            status=0x0038,
            crc_ok=False,
            crc_state="no_crc",
            raw_line=raw,
            protocol="legacy_csv",
        )
    elif len(values) >= 9:
        frame = RobotFrame(
            pc_time=time(),
            pos_x_cm=values[0],
            pos_y_cm=values[1],
            pos_yaw_deg=values[2],
            lidar_x_cm=values[3],
            lidar_y_cm=values[4],
            lidar_yaw_deg=values[5],
            calib_x_cm=values[6],
            calib_y_cm=values[7],
            calib_yaw_deg=values[8],
            encoder_x_cm=values[6],
            encoder_y_cm=values[7],
            h30_yaw_deg=values[8],
            h30_valid=True,
            h30_has_attitude=True,
            lidar_valid=True,
            status=0x002F,
            crc_ok=False,
            crc_state="no_crc",
            raw_line=raw,
            protocol="legacy_csv",
        )
    else:
        return ParseResult(None, f"legacy_field_count={len(values)}", raw)
    return ParseResult(frame, raw_line=raw, no_crc=True)


def parse_line(
    line: str | bytes,
    mode: str = "auto",
    allow_no_crc: bool = True,
    allow_legacy_csv: bool = True,
) -> ParseResult:
    if isinstance(line, bytes):
        text = line.decode("ascii", errors="ignore")
    else:
        text = line
    text = text.strip("\x00\r\n ")
    if not text:
        return ParseResult(None, "empty", "")

    if "$R1M," in text and not text.startswith("$R1M,"):
        text = text[text.find("$R1M,") :]

    if mode in ("r1m", "auto") and text.startswith("$R1M,"):
        return _parse_r1m(text, allow_no_crc=allow_no_crc)

    if mode in ("r1_csv_v3", "r1_csv_v2", "legacy_csv", "auto") and allow_legacy_csv:
        return _parse_legacy_csv(text)

    return ParseResult(None, "unsupported_protocol", text)


def make_r1m_line(frame: RobotFrame, version: int = 1, with_crc: bool = True) -> str:
    fields = [
        "R1M",
        str(version),
        str(int(frame.source_time_ms)),
        str(int(frame.seq)),
        f"{frame.pos_x_cm:.2f}",
        f"{frame.pos_y_cm:.2f}",
        f"{frame.pos_yaw_deg:.2f}",
        f"{frame.calib_x_cm:.2f}",
        f"{frame.calib_y_cm:.2f}",
        f"{frame.calib_yaw_deg:.2f}",
        f"{frame.h30_x_cm:.2f}",
        f"{frame.h30_y_cm:.2f}",
        f"{frame.h30_yaw_deg:.2f}",
        f"{frame.lidar_x_cm:.2f}",
        f"{frame.lidar_y_cm:.2f}",
        f"{frame.lidar_yaw_deg:.2f}",
        f"{frame.dt35_1_mm:.0f}",
        f"{frame.dt35_2_mm:.0f}",
        f"0x{frame.status:04X}",
    ]
    body = ",".join(fields) + ","
    if with_crc:
        return f"${body}*{crc16_ccitt_false(body):04X}\r\n"
    return f"${body}\r\n"


def make_r1_csv_v2_line(frame: RobotFrame) -> str:
    fields = [
        f"{frame.pos_x_cm:.3f}",
        f"{frame.pos_y_cm:.3f}",
        f"{frame.pos_yaw_deg:.3f}",
        f"{frame.lidar_x_cm:.3f}",
        f"{frame.lidar_y_cm:.3f}",
        f"{frame.lidar_yaw_deg:.3f}",
        f"{frame.calib_x_cm:.3f}",
        f"{frame.calib_y_cm:.3f}",
        f"{frame.calib_yaw_deg:.3f}",
        f"{frame.h30_yaw_deg:.3f}",
        f"{frame.h30_x_cm:.3f}",
        f"{frame.h30_y_cm:.3f}",
        f"{frame.encoder_x_cm:.3f}",
        f"{frame.encoder_y_cm:.3f}",
        str(int(frame.h30_valid)),
        str(int(frame.h30_has_attitude)),
        str(int(frame.lidar_valid)),
        str(int(frame.lidar_online)),
        str(int(frame.h30_packet_count)),
        str(int(frame.lidar_packet_count)),
        str(int(frame.h30_crc_error_count)),
        str(int(frame.h30_frame_error_count)),
        str(int(frame.lidar_checksum_error_count)),
        str(int(frame.lidar_frame_error_count)),
        str(int(frame.status)),
        str(int(frame.source_time_ms)),
        str(int(frame.h30_rx_byte_count)),
        str(int(frame.h30_has_accel)),
        str(int(frame.h30_last_update_ms)),
        str(int(frame.lidar_rx_byte_count)),
        str(int(frame.lidar_last_update_ms)),
        str(int(frame.x_raw_count)),
        str(int(frame.y_raw_count)),
        str(int(frame.x_delta_count)),
        str(int(frame.y_delta_count)),
        str(int(frame.x_total_count)),
        str(int(frame.y_total_count)),
        str(int(frame.x_index_seen)),
        str(int(frame.y_index_seen)),
        f"{frame.encoder_dis_p_mm:.3f}",
        f"{frame.encoder_dis_q_mm:.3f}",
    ]
    return ",".join(fields) + "\r\n"


def make_r1_csv_v3_line(frame: RobotFrame) -> str:
    fields = [
        f"{frame.pos_x_cm:.3f}",
        f"{frame.pos_y_cm:.3f}",
        f"{frame.pos_yaw_deg:.3f}",
        f"{frame.lidar_x_cm:.3f}",
        f"{frame.lidar_y_cm:.3f}",
        f"{frame.lidar_yaw_deg:.3f}",
        f"{frame.encoder_x_cm:.3f}",
        f"{frame.encoder_y_cm:.3f}",
        f"{frame.h30_yaw_deg:.3f}",
        f"{frame.dt35_1_mm:.3f}",
        f"{frame.dt35_2_mm:.3f}",
        str(int(frame.status)),
    ]
    return ",".join(fields) + "\r\n"
