from __future__ import annotations

import csv
import json
import threading
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from time import time
from typing import Any, TextIO

from .data_model import RobotFrame


def capture_timestamp(dt: datetime | None = None) -> str:
    value = dt or datetime.now()
    return value.strftime("%Y%m%d_%H%M%S")


@dataclass(slots=True)
class CaptureSummary:
    session_dir: Path
    raw_frame_count: int
    display_frame_count: int
    raw_line_count: int
    screenshot_count: int
    started_at: str
    stopped_at: str | None = None


class CaptureRecorder:
    def __init__(self, root: str | Path, config: dict[str, Any]) -> None:
        self.root = Path(root)
        self.config = config
        self.active = False
        self.session_dir: Path | None = None
        self.screenshot_dir: Path | None = None
        self.sensor_data_dir: Path | None = None
        self.started_at = ""
        self.started_pc_time = 0.0
        self.raw_frame_count = 0
        self.display_frame_count = 0
        self.raw_line_count = 0
        self.screenshot_count = 0
        self._raw_csv_file: TextIO | None = None
        self._display_csv_file: TextIO | None = None
        self._raw_log_file: TextIO | None = None
        self._events_file: TextIO | None = None
        self._raw_csv: csv.DictWriter | None = None
        self._display_csv: csv.DictWriter | None = None
        self._lock = threading.RLock()
        self._sample_stop = threading.Event()
        self._sample_thread: threading.Thread | None = None
        self._latest_raw_frame: RobotFrame | None = None
        self._latest_display_frame: RobotFrame | None = None
        display = config.get("display", {})
        self.data_interval_s = float(display.get("capture_data_interval_s", 0.1))
        self.snapshot_interval_s = float(display.get("capture_map_snapshot_interval_s", 1.0))
        self._sample_index = 0
        self._last_sample_index = 0
        self._last_sample_elapsed_ms = 0
        self._last_sample_wall_stamp = ""

    def start(self) -> Path:
        if self.active:
            raise RuntimeError("capture already active")

        stamp = capture_timestamp()
        self.session_dir = self._unique_session_dir(stamp)
        self.screenshot_dir = self.session_dir / "png"
        self.sensor_data_dir = self.session_dir / "sensor_data"
        self.session_dir.mkdir(parents=True, exist_ok=False)
        self.screenshot_dir.mkdir(parents=True, exist_ok=True)
        self.sensor_data_dir.mkdir(parents=True, exist_ok=True)

        self.started_at = datetime.now().isoformat(timespec="milliseconds")
        self.started_pc_time = time()
        self.raw_frame_count = 0
        self.display_frame_count = 0
        self.raw_line_count = 0
        self.screenshot_count = 0
        self._sample_index = 0
        self._last_sample_index = 0
        self._last_sample_elapsed_ms = 0
        self._last_sample_wall_stamp = self._wall_stamp()
        self._latest_raw_frame = None
        self._latest_display_frame = None
        self._sample_stop.clear()

        self._raw_csv_file = (self.sensor_data_dir / "raw_frames.csv").open(
            "w", encoding="utf-8", newline="", buffering=1
        )
        self._display_csv_file = (self.sensor_data_dir / "display_frames.csv").open(
            "w", encoding="utf-8", newline="", buffering=1
        )
        self._raw_log_file = (self.sensor_data_dir / "raw_serial.log").open("w", encoding="utf-8", buffering=1)
        self._events_file = (self.sensor_data_dir / "events.log").open("w", encoding="utf-8", buffering=1)

        fieldnames = self._capture_field_names()
        self._raw_csv = csv.DictWriter(self._raw_csv_file, fieldnames=fieldnames)
        self._display_csv = csv.DictWriter(self._display_csv_file, fieldnames=fieldnames)
        self._raw_csv.writeheader()
        self._display_csv.writeheader()

        self.event("capture started")
        self.active = True
        self._sample_thread = threading.Thread(target=self._sample_loop, name="r1-capture-sampler", daemon=True)
        self._sample_thread.start()
        self._write_metadata(stopped_at=None)
        return self.session_dir

    def _unique_session_dir(self, stamp: str) -> Path:
        base = self.root / "RL_data" / f"{stamp}_log"
        if not base.exists():
            return base
        for index in range(2, 1000):
            candidate = self.root / "RL_data" / f"{stamp}_{index:02d}_log"
            if not candidate.exists():
                return candidate
        raise RuntimeError("too many capture sessions in the same second")

    @staticmethod
    def _wall_stamp() -> str:
        return datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]

    @staticmethod
    def _capture_field_names() -> list[str]:
        return ["capture_sample", "capture_elapsed_ms", "capture_wall_time"] + RobotFrame.field_names()

    def _capture_row(self, frame: RobotFrame, sample_index: int, elapsed_ms: int, wall_stamp: str) -> dict[str, Any]:
        row = frame.to_row()
        row.update(
            {
                "capture_sample": sample_index,
                "capture_elapsed_ms": elapsed_ms,
                "capture_wall_time": wall_stamp,
            }
        )
        return row

    def stop(self) -> CaptureSummary:
        if not self.active or self.session_dir is None:
            raise RuntimeError("capture not active")
        self.sample_latest()
        self._sample_stop.set()
        if self._sample_thread is not None:
            self._sample_thread.join(timeout=2.0)
            self._sample_thread = None
        stopped_at = datetime.now().isoformat(timespec="milliseconds")
        self.event("capture stopped")
        self.active = False
        self._write_metadata(stopped_at=stopped_at)
        with self._lock:
            for handle in (self._raw_csv_file, self._display_csv_file, self._raw_log_file, self._events_file):
                if handle:
                    handle.close()
        self._raw_csv_file = None
        self._display_csv_file = None
        self._raw_log_file = None
        self._events_file = None
        self._raw_csv = None
        self._display_csv = None
        return CaptureSummary(
            session_dir=self.session_dir,
            raw_frame_count=self.raw_frame_count,
            display_frame_count=self.display_frame_count,
            raw_line_count=self.raw_line_count,
            screenshot_count=self.screenshot_count,
            started_at=self.started_at,
            stopped_at=stopped_at,
        )

    def raw_line(self, line: str) -> None:
        if not self.active or self._raw_log_file is None:
            return
        with self._lock:
            if not self.active or self._raw_log_file is None:
                return
            self.raw_line_count += 1
            self._raw_log_file.write(str(line).rstrip("\n") + "\n")

    def update_frame_pair(self, raw_frame: RobotFrame, display_frame: RobotFrame) -> None:
        with self._lock:
            self._latest_raw_frame = raw_frame
            self._latest_display_frame = display_frame

    def frame_pair(self, raw_frame: RobotFrame, display_frame: RobotFrame) -> None:
        self.update_frame_pair(raw_frame, display_frame)
        self.sample_latest()

    def sample_latest(self) -> None:
        if not self.active:
            return
        with self._lock:
            raw_frame = self._latest_raw_frame
            display_frame = self._latest_display_frame
            if raw_frame is None or display_frame is None:
                return
            now = time()
            self._sample_index += 1
            elapsed_ms = int(round((now - self.started_pc_time) * 1000.0))
            wall_stamp = self._wall_stamp()
            self._last_sample_index = self._sample_index
            self._last_sample_elapsed_ms = elapsed_ms
            self._last_sample_wall_stamp = wall_stamp

            if self._raw_csv is not None:
                self._raw_csv.writerow(self._capture_row(raw_frame, self._sample_index, elapsed_ms, wall_stamp))
                self.raw_frame_count += 1
            if self._display_csv is not None:
                self._display_csv.writerow(self._capture_row(display_frame, self._sample_index, elapsed_ms, wall_stamp))
                self.display_frame_count += 1

    def _sample_loop(self) -> None:
        interval = max(0.02, self.data_interval_s)
        while not self._sample_stop.wait(interval):
            self.sample_latest()

    def screenshot_path(self) -> Path | None:
        if not self.active or self.screenshot_dir is None:
            return None
        with self._lock:
            self.screenshot_count += 1
            sample = self._last_sample_index
            elapsed = self._last_sample_elapsed_ms
            stamp = self._last_sample_wall_stamp or self._wall_stamp()
        return self.screenshot_dir / (
            f"map_sample_{sample:06d}_t{elapsed:08d}ms_shot_{self.screenshot_count:06d}_{stamp}.png"
        )

    def event(self, text: str) -> None:
        with self._lock:
            if self._events_file is not None:
                self._events_file.write(f"{datetime.now().isoformat(timespec='milliseconds')} {text}\n")

    def _write_metadata(self, stopped_at: str | None) -> None:
        if self.session_dir is None:
            return
        display = self.config.get("display", {})
        payload = {
            "started_at": self.started_at,
            "stopped_at": stopped_at,
            "duration_s": max(0.0, time() - self.started_pc_time),
            "raw_frame_count": self.raw_frame_count,
            "display_frame_count": self.display_frame_count,
            "raw_line_count": self.raw_line_count,
            "screenshot_count": self.screenshot_count,
            "files": {
                "sensor_data_dir": "sensor_data",
                "png_dir": "png",
                "raw_frames_csv": "sensor_data/raw_frames.csv",
                "display_frames_csv": "sensor_data/display_frames.csv",
                "raw_serial_log": "sensor_data/raw_serial.log",
                "events_log": "sensor_data/events.log",
                "screenshots": "png",
            },
            "notes": [
                "sensor_data/raw_frames.csv stores parsed STM32 data before display transform/live fusion.",
                "sensor_data/display_frames.csv stores the map-displayed frame after transform, start policy, and live fusion.",
                "png stores synchronized map screenshots for visual comparison against the real field.",
                "Frame rows are sampled at capture_data_interval_s; screenshots use the latest sample id and elapsed time.",
                "Real field may be incomplete outside Zone 1; compare DT35 residuals against raw_serial and screenshots before changing sensor calibration.",
            ],
            "capture": {
                "data_interval_s": self.data_interval_s,
                "map_snapshot_interval_s": self.snapshot_interval_s,
                "live_fusion_enabled_by_default": bool(display.get("apply_live_fusion", True)),
                "dt35_correct_lidar_frames": bool(display.get("live_fusion_dt35_correct_lidar_frames", False)),
            },
        }
        (self.session_dir / "metadata.json").write_text(
            json.dumps(payload, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
