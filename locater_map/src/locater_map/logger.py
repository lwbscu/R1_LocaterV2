from __future__ import annotations

import csv
import queue
import threading
from datetime import datetime
from pathlib import Path
from typing import TextIO

from .data_model import RobotFrame


class SessionLogger:
    def __init__(self, root: str | Path, enabled: bool = True) -> None:
        self.enabled = enabled
        self.root = Path(root)
        self.session_dir = self.root / datetime.now().strftime("%Y%m%d_%H%M%S")
        self._queue: queue.Queue[tuple[str, object]] = queue.Queue()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._raw: TextIO | None = None
        self._events: TextIO | None = None
        self._csv_file: TextIO | None = None
        self._csv: csv.DictWriter | None = None

    def start(self) -> None:
        if not self.enabled:
            return
        self.session_dir.mkdir(parents=True, exist_ok=True)
        self._raw = (self.session_dir / "raw_serial.log").open("a", encoding="utf-8", buffering=1)
        self._events = (self.session_dir / "events.log").open("a", encoding="utf-8", buffering=1)
        self._csv_file = (self.session_dir / "parsed_frames.csv").open("a", encoding="utf-8", newline="")
        self._csv = csv.DictWriter(self._csv_file, fieldnames=RobotFrame.field_names())
        self._csv.writeheader()
        self._thread = threading.Thread(target=self._run, name="SessionLogger", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        if not self.enabled:
            return
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        for handle in (self._raw, self._events, self._csv_file):
            if handle:
                handle.close()

    def raw(self, line: str) -> None:
        if self.enabled:
            self._queue.put(("raw", line))

    def frame(self, frame: RobotFrame) -> None:
        if self.enabled:
            self._queue.put(("frame", frame))

    def event(self, text: str) -> None:
        if self.enabled:
            self._queue.put(("event", f"{datetime.now().isoformat(timespec='milliseconds')} {text}"))

    def _run(self) -> None:
        while not self._stop.is_set() or not self._queue.empty():
            try:
                kind, value = self._queue.get(timeout=0.1)
            except queue.Empty:
                continue
            if kind == "raw" and self._raw:
                self._raw.write(str(value).rstrip("\n") + "\n")
            elif kind == "event" and self._events:
                self._events.write(str(value).rstrip("\n") + "\n")
            elif kind == "frame" and self._csv and isinstance(value, RobotFrame):
                self._csv.writerow(value.to_row())
