from __future__ import annotations

import csv
from pathlib import Path

from .data_model import RobotFrame


class ReplaySource:
    def __init__(self, csv_path: str | Path) -> None:
        self.path = Path(csv_path)
        with self.path.open("r", encoding="utf-8", newline="") as f:
            self.frames = [RobotFrame.from_row(row) for row in csv.DictReader(f)]
        self.display_ready = self.path.name.lower() == "display_frames.csv"
        self.index = 0
        self.playing = False
        self.speed = 1.0

    def reset(self) -> None:
        self.index = 0

    def step(self) -> RobotFrame | None:
        if self.index >= len(self.frames):
            self.playing = False
            return None
        frame = self.frames[self.index]
        self.index += 1
        return frame
