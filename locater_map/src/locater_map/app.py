from __future__ import annotations

import sys

from PySide6.QtWidgets import QApplication

from .config_loader import load_config
from .main_window import MainWindow


def run_app(
    config_path: str | None = None,
    demo: bool = False,
    replay_path: str | None = None,
    serial_port: str | None = None,
    baudrate: int | None = None,
    duration_s: float | None = None,
    screenshot_path: str | None = None,
    capture_on_start: bool = False,
) -> int:
    config = load_config(config_path)
    app = QApplication(sys.argv)
    app.setApplicationName("R1 Locater Map")
    window = MainWindow(
        config,
        demo=demo,
        replay_path=replay_path,
        serial_port=serial_port,
        baudrate=baudrate,
        duration_s=duration_s,
        screenshot_path=screenshot_path,
        capture_on_start=capture_on_start,
    )
    window.show()
    return app.exec()
