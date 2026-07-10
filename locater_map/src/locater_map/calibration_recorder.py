from __future__ import annotations

import csv
import json
from dataclasses import asdict, dataclass
from datetime import datetime
from math import atan2, degrees, hypot
from pathlib import Path
from time import monotonic, time
from typing import Any, Iterable

try:
    import serial
except Exception:  # pragma: no cover
    serial = None

from .data_model import RobotFrame
from .protocol import parse_line


def _wrap_deg(angle: float) -> float:
    while angle > 180.0:
        angle -= 360.0
    while angle <= -180.0:
        angle += 360.0
    return angle


def _unwrap_series(values: Iterable[float]) -> list[float]:
    out: list[float] = []
    last: float | None = None
    offset = 0.0
    for value in values:
        if last is not None:
            delta = value - last
            if delta > 180.0:
                offset -= 360.0
            elif delta < -180.0:
                offset += 360.0
        out.append(value + offset)
        last = value
    return out


def _range(values: list[float]) -> dict[str, float] | None:
    if not values:
        return None
    return {"min": min(values), "max": max(values), "span": max(values) - min(values)}


def _mean(values: list[float]) -> float | None:
    if not values:
        return None
    return sum(values) / len(values)


def _std(values: list[float]) -> float | None:
    if len(values) < 2:
        return None
    avg = sum(values) / len(values)
    return (sum((v - avg) ** 2 for v in values) / (len(values) - 1)) ** 0.5


@dataclass(slots=True)
class CalibrationStats:
    raw_lines: int = 0
    frames: int = 0
    parse_errors: int = 0
    crc_errors: int = 0
    rx_bytes: int = 0
    duration_s: float = 0.0
    fps: float = 0.0
    lidar_valid_frames: int = 0
    h30_valid_frames: int = 0
    encoder_1_seen_frames: int = 0
    encoder_2_seen_frames: int = 0
    dt35_1_valid_frames: int = 0
    dt35_2_valid_frames: int = 0


@dataclass(slots=True)
class CalibrationSummary:
    stats: CalibrationStats
    ranges: dict[str, dict[str, float] | None]
    lidar_encoder_delta_error_cm: dict[str, float | None]
    lidar_h30_delta_error_deg: dict[str, float | None]
    lidar_encoder_heading_delta_deg: float | None
    dt35: dict[str, dict[str, float | None]]
    notes: list[str]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


class CalibrationRecorder:
    def __init__(
        self,
        output_root: str | Path,
        protocol_cfg: dict[str, Any] | None = None,
        session_name: str | None = None,
    ) -> None:
        self.output_root = Path(output_root)
        stamp = session_name or datetime.now().strftime("%Y%m%d_%H%M%S_calib")
        self.session_dir = self.output_root / stamp
        self.protocol_cfg = protocol_cfg or {"mode": "auto", "allow_no_crc": True, "allow_legacy_csv": True}
        self.frames: list[RobotFrame] = []
        self.stats = CalibrationStats()

    def record_serial(self, port: str, baudrate: int, duration_s: float) -> CalibrationSummary:
        if serial is None:
            raise RuntimeError("pyserial is not installed")

        self.session_dir.mkdir(parents=True, exist_ok=True)
        start = monotonic()
        buffer = bytearray()
        with serial.Serial(port, baudrate, timeout=0.05) as ser, \
                (self.session_dir / "raw_serial.log").open("w", encoding="utf-8", buffering=1) as raw_file, \
                (self.session_dir / "parse_errors.log").open("w", encoding="utf-8", buffering=1) as error_file, \
                (self.session_dir / "parsed_frames.csv").open("w", encoding="utf-8", newline="") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=RobotFrame.field_names())
            writer.writeheader()
            drop_first_line = True
            while monotonic() - start < duration_s:
                chunk = ser.read(512)
                if not chunk:
                    continue
                self.stats.rx_bytes += len(chunk)
                buffer.extend(chunk)
                while b"\n" in buffer:
                    raw, _, rest = buffer.partition(b"\n")
                    buffer = bytearray(rest)
                    line = raw.decode("ascii", errors="ignore").strip("\r\x00 ")
                    if drop_first_line:
                        drop_first_line = False
                        continue
                    self._handle_line(line, raw_file, error_file, writer)

        self.stats.duration_s = max(0.001, monotonic() - start)
        self.stats.fps = self.stats.frames / self.stats.duration_s
        summary = self.analyze()
        self._write_summary(summary)
        return summary

    def load_csv(self, path: str | Path) -> CalibrationSummary:
        self.session_dir.mkdir(parents=True, exist_ok=True)
        with Path(path).open("r", encoding="utf-8", newline="") as f:
            reader = csv.DictReader(f)
            self.frames = [RobotFrame.from_row(row) for row in reader]
        self.stats.frames = len(self.frames)
        self.stats.raw_lines = len(self.frames)
        self._refresh_valid_counts()
        summary = self.analyze()
        self._write_summary(summary)
        return summary

    def _handle_line(self, line: str, raw_file: Any, error_file: Any, writer: csv.DictWriter) -> None:
        if not line:
            return
        self.stats.raw_lines += 1
        raw_file.write(line + "\n")
        result = parse_line(
            line,
            mode=str(self.protocol_cfg.get("mode", "auto")),
            allow_no_crc=bool(self.protocol_cfg.get("allow_no_crc", True)),
            allow_legacy_csv=bool(self.protocol_cfg.get("allow_legacy_csv", True)),
        )
        if result.crc_error:
            self.stats.crc_errors += 1
        if result.frame is None:
            self.stats.parse_errors += 1
            error_file.write(f"{result.error}: {line}\n")
            return
        frame = result.frame
        frame.pc_time = time()
        self.frames.append(frame)
        writer.writerow(frame.to_row())
        self.stats.frames += 1
        self._count_valid(frame)

    def _count_valid(self, frame: RobotFrame) -> None:
        if frame.lidar_valid or frame.lidar_online:
            self.stats.lidar_valid_frames += 1
        if frame.h30_valid or frame.h30_has_attitude:
            self.stats.h30_valid_frames += 1
        if frame.x_pulse_seen:
            self.stats.encoder_1_seen_frames += 1
        if frame.y_pulse_seen:
            self.stats.encoder_2_seen_frames += 1
        if frame.dt35_1_valid:
            self.stats.dt35_1_valid_frames += 1
        if frame.dt35_2_valid:
            self.stats.dt35_2_valid_frames += 1

    def _refresh_valid_counts(self) -> None:
        self.stats.lidar_valid_frames = 0
        self.stats.h30_valid_frames = 0
        self.stats.encoder_1_seen_frames = 0
        self.stats.encoder_2_seen_frames = 0
        self.stats.dt35_1_valid_frames = 0
        self.stats.dt35_2_valid_frames = 0
        for frame in self.frames:
            self._count_valid(frame)

    def analyze(self) -> CalibrationSummary:
        frames = self.frames
        lidar_frames = [f for f in frames if f.lidar_valid or f.lidar_online]
        ranges = {
            "pos_x_cm": _range([f.pos_x_cm for f in frames]),
            "pos_y_cm": _range([f.pos_y_cm for f in frames]),
            "pos_yaw_deg": _range([f.pos_yaw_deg for f in frames]),
            "lidar_x_cm": _range([f.lidar_x_cm for f in lidar_frames]),
            "lidar_y_cm": _range([f.lidar_y_cm for f in lidar_frames]),
            "lidar_yaw_deg": _range([f.lidar_yaw_deg for f in lidar_frames]),
            "encoder_x_cm": _range([f.encoder_x_cm for f in frames]),
            "encoder_y_cm": _range([f.encoder_y_cm for f in frames]),
            "h30_yaw_deg": _range([f.h30_yaw_deg for f in frames]),
        }
        delta_error, yaw_error, heading_delta = self._compare_lidar_local(lidar_frames)
        dt35 = self._dt35_summary(frames)
        notes = self._make_notes(lidar_frames, delta_error, yaw_error, dt35)
        return CalibrationSummary(
            stats=self.stats,
            ranges=ranges,
            lidar_encoder_delta_error_cm=delta_error,
            lidar_h30_delta_error_deg=yaw_error,
            lidar_encoder_heading_delta_deg=heading_delta,
            dt35=dt35,
            notes=notes,
        )

    def _compare_lidar_local(
        self,
        lidar_frames: list[RobotFrame],
    ) -> tuple[dict[str, float | None], dict[str, float | None], float | None]:
        if len(lidar_frames) < 2:
            return (
                {"mean_x": None, "mean_y": None, "std_x": None, "std_y": None, "rms_xy": None},
                {"mean": None, "std": None},
                None,
            )
        first = lidar_frames[0]
        x_errors: list[float] = []
        y_errors: list[float] = []
        yaw_errors: list[float] = []
        lidar_yaws = _unwrap_series([f.lidar_yaw_deg for f in lidar_frames])
        h30_yaws = _unwrap_series([f.h30_yaw_deg for f in lidar_frames])
        for i, frame in enumerate(lidar_frames):
            lidar_dx = frame.lidar_x_cm - first.lidar_x_cm
            lidar_dy = frame.lidar_y_cm - first.lidar_y_cm
            enc_dx = frame.encoder_x_cm - first.encoder_x_cm
            enc_dy = frame.encoder_y_cm - first.encoder_y_cm
            x_errors.append(enc_dx - lidar_dx)
            y_errors.append(enc_dy - lidar_dy)
            yaw_errors.append((h30_yaws[i] - h30_yaws[0]) - (lidar_yaws[i] - lidar_yaws[0]))
        rms_xy = (sum(x * x + y * y for x, y in zip(x_errors, y_errors)) / len(x_errors)) ** 0.5
        heading_delta = self._heading_delta_deg(first, lidar_frames[-1])
        return (
            {
                "mean_x": _mean(x_errors),
                "mean_y": _mean(y_errors),
                "std_x": _std(x_errors),
                "std_y": _std(y_errors),
                "rms_xy": rms_xy,
            },
            {"mean": _mean(yaw_errors), "std": _std(yaw_errors)},
            heading_delta,
        )

    @staticmethod
    def _heading_delta_deg(first: RobotFrame, last: RobotFrame) -> float | None:
        lidar_dx = last.lidar_x_cm - first.lidar_x_cm
        lidar_dy = last.lidar_y_cm - first.lidar_y_cm
        enc_dx = last.encoder_x_cm - first.encoder_x_cm
        enc_dy = last.encoder_y_cm - first.encoder_y_cm
        if hypot(lidar_dx, lidar_dy) < 10.0 or hypot(enc_dx, enc_dy) < 10.0:
            return None
        lidar_heading = degrees(atan2(lidar_dx, lidar_dy))
        enc_heading = degrees(atan2(enc_dx, enc_dy))
        return _wrap_deg(enc_heading - lidar_heading)

    @staticmethod
    def _dt35_summary(frames: list[RobotFrame]) -> dict[str, dict[str, float | None]]:
        result: dict[str, dict[str, float | None]] = {}
        for name, getter in (
            ("dt35_1_mm", lambda f: f.dt35_1_mm if f.dt35_1_valid else None),
            ("dt35_2_mm", lambda f: f.dt35_2_mm if f.dt35_2_valid else None),
        ):
            values = [float(v) for f in frames if (v := getter(f)) is not None and v > 0.0]
            result[name] = {
                "min_mm": min(values) if values else None,
                "max_mm": max(values) if values else None,
                "mean_mm": _mean(values),
                "std_mm": _std(values),
                "valid_ratio": len(values) / max(1, len(frames)),
            }
        return result

    def _make_notes(
        self,
        lidar_frames: list[RobotFrame],
        delta_error: dict[str, float | None],
        yaw_error: dict[str, float | None],
        dt35: dict[str, dict[str, float | None]],
    ) -> list[str]:
        notes: list[str] = []
        if not self.frames:
            return ["No parsed frames. Check serial port, baudrate, and USART1 telemetry output."]
        if len(lidar_frames) < max(10, int(self.stats.frames * 0.2)):
            notes.append("Too few valid lidar frames; this capture is weak for map/encoder alignment.")
        if self.stats.h30_valid_frames == 0:
            notes.append("No valid H30 state; this capture cannot calibrate yaw.")
        if self.stats.encoder_1_seen_frames == 0 or self.stats.encoder_2_seen_frames == 0:
            notes.append("At least one orthogonal encoder has not received pulses in this capture.")
        rms = delta_error.get("rms_xy")
        if rms is not None:
            notes.append(f"Encoder-vs-lidar translation RMS error is about {rms:.2f} cm.")
        yaw_std = yaw_error.get("std")
        if yaw_std is not None:
            notes.append(f"H30-vs-lidar yaw-delta standard deviation is about {yaw_std:.3f} deg.")
        for key, item in dt35.items():
            ratio = float(item.get("valid_ratio") or 0.0)
            if ratio <= 0.0:
                notes.append(f"{key} has no valid distance in this capture; field-boundary matching is disabled for it.")
        return notes

    def _write_summary(self, summary: CalibrationSummary) -> None:
        self.session_dir.mkdir(parents=True, exist_ok=True)
        with (self.session_dir / "calibration_summary.json").open("w", encoding="utf-8") as f:
            json.dump(summary.to_dict(), f, ensure_ascii=False, indent=2)
        with (self.session_dir / "calibration_notes.md").open("w", encoding="utf-8") as f:
            f.write("# R1 Calibration Notes\n\n")
            f.write(f"- frames: {summary.stats.frames}\n")
            f.write(f"- duration_s: {summary.stats.duration_s:.3f}\n")
            f.write(f"- fps: {summary.stats.fps:.2f}\n")
            for note in summary.notes:
                f.write(f"- {note}\n")


def record_to_session(
    port: str,
    baudrate: int,
    duration_s: float,
    output_root: str | Path,
    protocol_cfg: dict[str, Any] | None = None,
) -> tuple[Path, CalibrationSummary]:
    recorder = CalibrationRecorder(output_root=output_root, protocol_cfg=protocol_cfg)
    summary = recorder.record_serial(port=port, baudrate=baudrate, duration_s=duration_s)
    return recorder.session_dir, summary
