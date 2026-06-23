from __future__ import annotations

import shutil
import subprocess
import threading
from dataclasses import replace
from math import cos, radians, sin
from pathlib import Path
from time import time
from typing import Any

from PySide6.QtCore import QTimer, Qt
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QFileDialog,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPushButton,
    QPlainTextEdit,
    QSpinBox,
    QSplitter,
    QTabWidget,
    QToolButton,
    QVBoxLayout,
    QWidget,
)

from .config_loader import PROJECT_ROOT
from .capture_recorder import CaptureRecorder
from .data_model import RobotFrame, SerialStats
from .demo_source import DemoSource
from .fusion_model import FusionConfig, LiveFusionFilter
from .i18n import normalize_language, tr
from .logger import SessionLogger
from .map_widget import FieldMapView
from .replay import ReplaySource
from .serial_worker import SerialSession, available_ports, preferred_serial_port
from .utils_transform import dt35_ray, dt35_yaw_from_frame, transform_frame


class MainWindow(QMainWindow):
    def __init__(
        self,
        config: dict[str, Any],
        demo: bool = False,
        replay_path: str | None = None,
        serial_port: str | None = None,
        baudrate: int | None = None,
        duration_s: float | None = None,
        screenshot_path: str | None = None,
        capture_on_start: bool = False,
        record_video_path: str | None = None,
        record_gif_path: str | None = None,
        record_fps: float = 10.0,
        replay_speed: float | None = None,
    ) -> None:
        super().__init__()
        self.config = config
        self.language = normalize_language(config.get("ui", {}).get("language", "zh"))
        self._text_widgets: dict[str, list[QLabel | QPushButton | QCheckBox | QGroupBox | QToolButton | QTabWidget]] = {}
        self._form_labels: dict[str, QLabel] = {}
        self._value_name_labels: dict[str, QLabel] = {}
        self._layer_checks: dict[str, QCheckBox] = {}
        self.setWindowTitle(self._t("window_title"))
        self.resize(1680, 980)

        self.serial = SerialSession()
        self.serial.frame_received.connect(self.on_frame)
        self.serial.raw_received.connect(self.on_raw)
        self.serial.stats_changed.connect(self.on_stats)
        self.serial.event.connect(self.on_event)

        log_root = PROJECT_ROOT / str(config.get("logging", {}).get("root", "logs"))
        self.logger = SessionLogger(log_root, bool(config.get("logging", {}).get("enabled", True)))
        self.logger.start()
        self.capture = CaptureRecorder(log_root, config)

        self.demo_source = DemoSource()
        self.replay: ReplaySource | None = ReplaySource(replay_path) if replay_path else None
        self.live_fusion = LiveFusionFilter(self._live_fusion_config(), self.config, use_start_transform=False)
        self._live_fusion_has_lidar_anchor = False
        self.latest_frame: RobotFrame | None = None
        self.latest_local_frame: RobotFrame | None = None
        self._latest_raw_frame: RobotFrame | None = None
        self._latest_display_frame: RobotFrame | None = None
        self._screenshot_threads: list[threading.Thread] = []
        self.stats = SerialStats()
        self._last_serial_args: tuple[str, int, dict[str, Any]] | None = None

        self._auto_serial_port = serial_port
        self._auto_baudrate = baudrate
        self._auto_duration_s = duration_s
        self._auto_screenshot_path = screenshot_path
        self._auto_capture = capture_on_start
        self._auto_replay_speed = replay_speed
        self._record_video_path_text = record_video_path
        self._record_gif_path_text = record_gif_path
        self._record_fps = max(1.0, float(record_fps))
        self._record_frame_dir: Path | None = None
        self._record_video_path: Path | None = None
        self._record_gif_path: Path | None = None
        self._record_frame_index = 0
        self._record_encoding_done = False
        self._record_threads: list[threading.Thread] = []
        self._record_frame_ext = ".jpg"
        self._record_max_width_px = 1920
        self._record_frame_size: tuple[int, int] | None = None

        self._build_ui()
        self._apply_theme()
        self._setup_timers()
        if demo:
            self.demo_check.setChecked(True)
        if replay_path:
            self.replay_file.setText(replay_path)
        self._schedule_startup_actions()

    def _t(self, key: str) -> str:
        return tr(self.language, key)

    def _live_fusion_config(self) -> FusionConfig:
        display = self.config.get("display", {})
        return FusionConfig(
            lidar_stride=max(1, int(display.get("live_fusion_lidar_stride", 1))),
            lidar_gain=float(display.get("live_fusion_lidar_gain", 1.0)),
            encoder_scale_learning=bool(display.get("live_fusion_encoder_scale_learning", True)),
            encoder_scale_learning_gain=float(display.get("live_fusion_encoder_scale_learning_gain", 0.35)),
            encoder_scale_min_delta_cm=float(display.get("live_fusion_encoder_scale_min_delta_cm", 20.0)),
            encoder_scale_min=float(display.get("live_fusion_encoder_scale_min", 0.85)),
            encoder_scale_max=float(display.get("live_fusion_encoder_scale_max", 1.15)),
            dt35_gain=float(display.get("live_fusion_dt35_gain", 1.0)),
            dt35_yaw_gain=float(display.get("live_fusion_dt35_yaw_gain", 0.0)),
            dt35_correct_lidar_frames=bool(display.get("live_fusion_dt35_correct_lidar_frames", False)),
            dt35_residual_gate_cm=float(display.get("live_fusion_dt35_residual_gate_cm", 40.0)),
            dt35_max_translation_step_cm=float(display.get("live_fusion_dt35_max_translation_step_cm", 12.0)),
            dt35_damping=float(display.get("live_fusion_dt35_damping", 0.05)),
            use_dt35=bool(display.get("live_fusion_use_dt35", True)),
        )

    def _reset_live_fusion(self) -> None:
        self.live_fusion.reset()
        self._live_fusion_has_lidar_anchor = False

    def _remember_text(self, key: str, widget: QLabel | QPushButton | QCheckBox | QGroupBox | QToolButton | QTabWidget) -> None:
        self._text_widgets.setdefault(key, []).append(widget)

    def _make_label(self, key: str) -> QLabel:
        label = QLabel(self._t(key))
        self._form_labels[key] = label
        return label

    def _set_text_widgets(self, key: str, text: str) -> None:
        for widget in self._text_widgets.get(key, []):
            if isinstance(widget, QGroupBox):
                widget.setTitle(text)
            elif isinstance(widget, (QPushButton, QCheckBox, QToolButton)):
                widget.setText(text)

    def toggle_language(self) -> None:
        self.language = "en" if self.language == "zh" else "zh"
        self._apply_language()

    def _apply_language(self) -> None:
        self.setWindowTitle(self._t("window_title"))
        for key, label in self._form_labels.items():
            label.setText(self._t(key))
        for key in self._text_widgets:
            self._set_text_widgets(key, self._t(key))
        for key, label in self._value_name_labels.items():
            label.setText(self._t(key))
        if hasattr(self, "tabs"):
            self.tabs.setTabText(0, self._t("raw_serial"))
            self.tabs.setTabText(1, self._t("commands"))
        if hasattr(self, "open_btn"):
            self.open_btn.setText(self._t("close") if self.serial.worker else self._t("open"))
        if hasattr(self, "replay_toggle_btn") and not self.replay_timer.isActive():
            self.replay_toggle_btn.setText(self._t("play"))
        if hasattr(self, "capture_btn"):
            self.capture_btn.setText(self._t("capture_stop") if self.capture.active else self._t("capture_start"))
        if hasattr(self, "capture_status") and not self.capture.active and self.capture_status.text() in ("-", self._t("capture_idle")):
            self.capture_status.setText(self._t("capture_idle"))
        self._update_labels()

    def _schedule_startup_actions(self) -> None:
        if self._auto_replay_speed is not None:
            self._set_replay_speed(float(self._auto_replay_speed))
        if self.replay:
            QTimer.singleShot(250, self.start_startup_replay)
        if self._auto_serial_port:
            QTimer.singleShot(200, self.open_startup_serial)
        if self._auto_capture:
            QTimer.singleShot(700, self.start_capture)
        if self._record_video_path_text or self._record_gif_path_text:
            QTimer.singleShot(350, self.start_window_recording)
        if self._auto_duration_s is not None:
            delay_ms = max(250, int(self._auto_duration_s * 1000.0))
            QTimer.singleShot(delay_ms, self.finish_timed_run)

    def _set_replay_speed(self, speed: float) -> None:
        speed = max(0.1, float(speed))
        text = f"{speed:g}x"
        if self.replay_speed.findText(text) < 0:
            self.replay_speed.addItem(text)
        self.replay_speed.setCurrentText(text)

    def start_startup_replay(self) -> None:
        if not self.replay:
            return
        self.step_replay()
        if self.replay.index < len(self.replay.frames):
            self.toggle_replay()

    def open_startup_serial(self) -> None:
        port = str(self._auto_serial_port)
        if port and self.port_combo.findText(port) < 0:
            self.port_combo.addItem(port)
        if port:
            self.port_combo.setCurrentText(port)
        if self._auto_baudrate:
            self.baud_spin.setValue(int(self._auto_baudrate))
        if self.serial.worker is None:
            self.toggle_serial()

    def finish_timed_run(self) -> None:
        if self.capture.active:
            self.stop_capture()
        self.finish_window_recording()
        if self._auto_screenshot_path:
            out = Path(self._auto_screenshot_path)
            if not out.is_absolute():
                out = PROJECT_ROOT / out
            out.parent.mkdir(parents=True, exist_ok=True)
            self.map_view.save_screenshot(out)
            self.on_event(f"timed screenshot saved: {out}")
        self.close()

    def _build_ui(self) -> None:
        central = QWidget()
        root = QVBoxLayout(central)
        splitter = QSplitter(Qt.Orientation.Horizontal)
        self.map_view = FieldMapView(self.config)
        self.map_view.mouse_position_changed.connect(self.on_mouse_position)
        splitter.addWidget(self._left_panel())
        splitter.addWidget(self.map_view)
        splitter.addWidget(self._right_panel())
        splitter.setStretchFactor(1, 1)

        main_split = QSplitter(Qt.Orientation.Vertical)
        top = QWidget()
        top_layout = QVBoxLayout(top)
        top_layout.setContentsMargins(0, 0, 0, 0)
        top_layout.addWidget(splitter)
        main_split.addWidget(top)
        main_split.addWidget(self._bottom_tabs())
        main_split.setStretchFactor(0, 5)
        main_split.setStretchFactor(1, 1)
        root.addWidget(main_split)
        self.setCentralWidget(central)

    def _left_panel(self) -> QWidget:
        panel = QWidget()
        panel.setMinimumWidth(260)
        layout = QVBoxLayout(panel)

        serial_box = QGroupBox(self._t("serial"))
        self._remember_text("serial", serial_box)
        form = QFormLayout(serial_box)
        self.port_combo = QComboBox()
        self.refresh_ports()
        refresh = QPushButton(self._t("refresh"))
        self._remember_text("refresh", refresh)
        refresh.clicked.connect(self.refresh_ports)
        port_row = QHBoxLayout()
        port_row.addWidget(self.port_combo)
        port_row.addWidget(refresh)
        form.addRow(self._make_label("port"), port_row)
        self.baud_spin = QSpinBox()
        self.baud_spin.setRange(1200, 4000000)
        self.baud_spin.setValue(int(self.config["serial"].get("baudrate", 115200)))
        form.addRow(self._make_label("baud"), self.baud_spin)
        self.protocol_combo = QComboBox()
        self.protocol_combo.addItems(["auto", "r1_csv_v3", "r1_csv_v2", "legacy_csv", "r1m"])
        self.protocol_combo.setCurrentText(str(self.config["protocol"].get("mode", "auto")))
        form.addRow(self._make_label("protocol"), self.protocol_combo)
        self.auto_reconnect_check = QCheckBox(self._t("auto_reconnect"))
        self._remember_text("auto_reconnect", self.auto_reconnect_check)
        self.auto_reconnect_check.setChecked(bool(self.config["serial"].get("auto_reconnect", True)))
        form.addRow(self.auto_reconnect_check)
        self.language_btn = QPushButton(self._t("language_toggle"))
        self._remember_text("language_toggle", self.language_btn)
        self.language_btn.clicked.connect(self.toggle_language)
        form.addRow(self.language_btn)
        self.open_btn = QPushButton(self._t("open"))
        self.open_btn.clicked.connect(self.toggle_serial)
        form.addRow(self.open_btn)
        layout.addWidget(serial_box)

        start_box = QGroupBox(self._t("start_pose"))
        self._remember_text("start_pose", start_box)
        start_form = QFormLayout(start_box)
        self.start_side_combo = QComboBox()
        self.start_side_combo.addItems(["none", "red", "blue"])
        self.start_side_combo.setCurrentText(str(self.config["robot"].get("default_start_side", "none")))
        self.start_policy_combo = QComboBox()
        self.start_policy_combo.addItems(["auto_lidar_offline", "always_local_display", "off"])
        self.start_policy_combo.setCurrentText(str(self.config["robot"].get("start_pose_policy", "auto_lidar_offline")))
        self.start_side_combo.currentTextChanged.connect(lambda _text: self._reset_live_fusion())
        self.start_policy_combo.currentTextChanged.connect(lambda _text: self._reset_live_fusion())
        start_form.addRow(self._make_label("side"), self.start_side_combo)
        start_form.addRow(self._make_label("policy"), self.start_policy_combo)
        self.live_fusion_check = QCheckBox(self._t("live_fusion"))
        self._remember_text("live_fusion", self.live_fusion_check)
        self.live_fusion_check.setChecked(bool(self.config.get("display", {}).get("apply_live_fusion", True)))
        self.live_fusion_check.toggled.connect(lambda _checked: self._reset_live_fusion())
        start_form.addRow(self.live_fusion_check)
        layout.addWidget(start_box)

        mode_box = QGroupBox(self._t("mode_replay"))
        self._remember_text("mode_replay", mode_box)
        mode_layout = QVBoxLayout(mode_box)
        self.demo_check = QCheckBox(self._t("demo_mode"))
        self._remember_text("demo_mode", self.demo_check)
        mode_layout.addWidget(self.demo_check)
        self.replay_file = QLineEdit()
        choose = QPushButton(self._t("choose_csv"))
        self._remember_text("choose_csv", choose)
        choose.clicked.connect(self.choose_replay)
        replay_row = QHBoxLayout()
        replay_row.addWidget(self.replay_file)
        replay_row.addWidget(choose)
        mode_layout.addLayout(replay_row)
        replay_controls = QHBoxLayout()
        self.replay_toggle_btn = QPushButton(self._t("play"))
        self.replay_toggle_btn.clicked.connect(self.toggle_replay)
        step = QPushButton(self._t("step"))
        self._remember_text("step", step)
        step.clicked.connect(self.step_replay)
        self.replay_speed = QComboBox()
        self.replay_speed.addItems(["0.1x", "0.5x", "1x", "2x", "5x"])
        self.replay_speed.setCurrentText("1x")
        replay_controls.addWidget(self.replay_toggle_btn)
        replay_controls.addWidget(step)
        replay_controls.addWidget(self.replay_speed)
        mode_layout.addLayout(replay_controls)
        self.capture_btn = QPushButton(self._t("capture_start"))
        self._remember_text("capture_start", self.capture_btn)
        self.capture_btn.clicked.connect(self.toggle_capture)
        self.capture_status = QLabel(self._t("capture_idle"))
        self.capture_status.setWordWrap(True)
        mode_layout.addWidget(self.capture_btn)
        mode_layout.addWidget(self.capture_status)
        layout.addWidget(mode_box)

        layers = QGroupBox(self._t("layers"))
        self._remember_text("layers", layers)
        layer_layout = QVBoxLayout(layers)
        for key, label in [
            ("pos", "pos_trajectory"),
            ("calib", "calib_trajectory"),
            ("lidar", "lidar_trajectory"),
            ("dt35", "dt35_rays"),
            ("field_model", "field_model"),
            ("grid", "grid"),
            ("axes", "axes"),
        ]:
            cb = QCheckBox(self._t(label))
            self._remember_text(label, cb)
            cb.setChecked(self._default_layer_visible(key))
            cb.toggled.connect(lambda checked, k=key: self.map_view.set_layer_visible(k, checked))
            self.map_view.set_layer_visible(key, cb.isChecked())
            self._layer_checks[key] = cb
            layer_layout.addWidget(cb)
        follow = QCheckBox(self._t("follow_robot"))
        self._remember_text("follow_robot", follow)
        follow.setChecked(bool(self.config["display"].get("follow_robot", True)))
        follow.toggled.connect(lambda v: setattr(self.map_view, "follow_robot", v))
        layer_layout.addWidget(follow)
        clear = QPushButton(self._t("clear_trajectory"))
        self._remember_text("clear_trajectory", clear)
        clear.clicked.connect(self.map_view.clear_trajectories)
        reset = QPushButton(self._t("reset_view"))
        self._remember_text("reset_view", reset)
        reset.clicked.connect(self.map_view.reset_view)
        shot = QPushButton(self._t("save_screenshot"))
        self._remember_text("save_screenshot", shot)
        shot.clicked.connect(self.save_screenshot)
        layer_layout.addWidget(clear)
        layer_layout.addWidget(reset)
        layer_layout.addWidget(shot)
        layout.addWidget(layers)
        layout.addStretch(1)
        return panel

    def _right_panel(self) -> QWidget:
        panel = QWidget()
        panel.setMinimumWidth(310)
        layout = QVBoxLayout(panel)

        self.value_labels: dict[str, QLabel] = {}
        self.sensor_status_labels: dict[str, QLabel] = {}
        values = QGroupBox(self._t("live_data"))
        self._remember_text("live_data", values)
        grid = QGridLayout(values)
        names = [
            "pos", "map_pose", "lidar", "encoder", "h30", "dt35", "diag",
            "dt35_model", "fps", "bytes", "dropped", "crc", "parse", "interval", "mouse",
        ]
        for row, name in enumerate(names):
            name_label = QLabel(self._t(name))
            self._value_name_labels[name] = name_label
            grid.addWidget(name_label, row, 0)
            label = QLabel("-")
            label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
            grid.addWidget(label, row, 1)
            self.value_labels[name] = label
        layout.addWidget(values)

        status_box = QGroupBox(self._t("sensor_status"))
        self._remember_text("sensor_status", status_box)
        status_layout = QGridLayout(status_box)
        for i, key in enumerate(("encoder_1", "encoder_2", "h30", "lidar", "dt35_1", "dt35_2")):
            lamp = QLabel("-")
            lamp.setStyleSheet("color:#ff6b6b")
            status_layout.addWidget(lamp, i, 0)
            self.sensor_status_labels[key] = lamp
        layout.addWidget(status_box)

        raw_box = QGroupBox(self._t("current_raw"))
        self._remember_text("current_raw", raw_box)
        raw_layout = QVBoxLayout(raw_box)
        self.raw_summary = QLabel("-")
        self.raw_summary.setWordWrap(True)
        self.raw_summary.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        raw_layout.addWidget(self.raw_summary)
        layout.addWidget(raw_box)
        layout.addStretch(1)
        return panel

    def _bottom_tabs(self) -> QTabWidget:
        tabs = QTabWidget()
        self.tabs = tabs

        raw_tab = QWidget()
        raw_layout = QVBoxLayout(raw_tab)
        raw_controls = QHBoxLayout()
        self.raw_pause_check = QCheckBox(self._t("pause_scroll"))
        self._remember_text("pause_scroll", self.raw_pause_check)
        clear_raw = QPushButton(self._t("clear"))
        self._remember_text("clear", clear_raw)
        clear_raw.clicked.connect(lambda: self.raw_text.clear())
        save_raw = QPushButton(self._t("save_visible_raw"))
        self._remember_text("save_visible_raw", save_raw)
        save_raw.clicked.connect(self.save_visible_raw)
        raw_controls.addWidget(self.raw_pause_check)
        raw_controls.addWidget(clear_raw)
        raw_controls.addWidget(save_raw)
        raw_controls.addStretch(1)
        raw_layout.addLayout(raw_controls)
        self.raw_text = QPlainTextEdit()
        self.raw_text.setReadOnly(True)
        raw_layout.addWidget(self.raw_text)
        tabs.addTab(raw_tab, self._t("raw_serial"))

        cmd = QWidget()
        cmd_layout = QVBoxLayout(cmd)
        self.cmd_text = QLineEdit()
        send = QPushButton(self._t("send_text"))
        self._remember_text("send_text", send)
        send.clicked.connect(self.send_command_text)
        send_hex = QPushButton(self._t("send_hex"))
        self._remember_text("send_hex", send_hex)
        send_hex.clicked.connect(lambda: self.serial.send_hex(self.cmd_text.text()))
        row = QHBoxLayout()
        row.addWidget(self.cmd_text)
        row.addWidget(send)
        row.addWidget(send_hex)
        cmd_layout.addLayout(row)

        periodic = QHBoxLayout()
        self.periodic_check = QCheckBox(self._t("periodic_text"))
        self._remember_text("periodic_text", self.periodic_check)
        self.periodic_check.toggled.connect(self.toggle_periodic_send)
        self.periodic_ms = QSpinBox()
        self.periodic_ms.setRange(20, 60000)
        self.periodic_ms.setValue(1000)
        periodic.addWidget(self.periodic_check)
        periodic.addWidget(QLabel("ms"))
        periodic.addWidget(self.periodic_ms)
        periodic.addStretch(1)
        cmd_layout.addLayout(periodic)

        common = QHBoxLayout()
        for entry in self.config.get("commands", []):
            btn = QToolButton()
            btn.setText(str(entry.get("name", "cmd")))
            btn.clicked.connect(lambda _=False, text=str(entry.get("text", "")): self.serial.send_text(text + "\n"))
            common.addWidget(btn)
        common.addStretch(1)
        cmd_layout.addLayout(common)
        tabs.addTab(cmd, self._t("commands"))
        return tabs

    def _default_layer_visible(self, key: str) -> bool:
        display = self.config.get("display", {})
        config_keys = {
            "pos": "show_pos_trajectory",
            "calib": "show_calib_trajectory",
            "lidar": "show_lidar_trajectory",
            "dt35": "show_dt35",
            "field_model": "show_field_model",
        }
        return bool(display.get(config_keys.get(key, ""), True))

    def _setup_timers(self) -> None:
        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self.on_ui_tick)
        self.ui_timer.start(int(1000 / int(self.config["display"].get("ui_fps", 30))))

        self.replay_timer = QTimer(self)
        self.replay_timer.timeout.connect(self.step_replay)

        self.periodic_timer = QTimer(self)
        self.periodic_timer.timeout.connect(self.send_command_text)

        self.capture_timer = QTimer(self)
        self.capture_timer.timeout.connect(self.capture_map_snapshot)

        self.window_record_timer = QTimer(self)
        self.window_record_timer.timeout.connect(self.capture_window_record_frame)

    def _apply_theme(self) -> None:
        self.setStyleSheet(
            """
            QMainWindow, QWidget { background:#0f141b; color:#d6dde6; font-size:13px; }
            QGroupBox { border:1px solid #2b3542; border-radius:6px; margin-top:8px; padding:8px; }
            QGroupBox::title { subcontrol-origin: margin; left:8px; padding:0 4px; }
            QPushButton, QToolButton, QComboBox, QSpinBox, QLineEdit { background:#18212b; border:1px solid #334155; border-radius:4px; padding:5px; }
            QPushButton:hover, QToolButton:hover { background:#223044; }
            QPlainTextEdit { background:#080c11; border:1px solid #2b3542; font-family:Consolas; }
            """
        )

    def refresh_ports(self) -> None:
        current = self.port_combo.currentText() if hasattr(self, "port_combo") else ""
        self.port_combo.clear()
        ports = available_ports()
        self.port_combo.addItems(ports)
        default = str(self.config["serial"].get("default_port", ""))
        preferred = preferred_serial_port(current=current, default=default)
        if preferred:
            self.port_combo.setCurrentText(preferred)

    def toggle_serial(self) -> None:
        if self.serial.worker is None:
            self.refresh_ports()
            port = self.port_combo.currentText()
            if not port:
                self.on_event("no serial port selected")
                return
            if port not in available_ports():
                self.on_event(f"serial port not present: {port}")
                return
            protocol_cfg = dict(self.config["protocol"])
            protocol_cfg["mode"] = self.protocol_combo.currentText()
            baudrate = int(self.baud_spin.value())
            self._last_serial_args = (port, baudrate, protocol_cfg)
            self.serial.open(port, baudrate, protocol_cfg)
            self.open_btn.setText(self._t("close"))
        else:
            self.serial.close()
            self.open_btn.setText(self._t("open"))

    def _auto_reconnect(self) -> None:
        if not self.auto_reconnect_check.isChecked() or not self._last_serial_args:
            return
        port, baudrate, protocol_cfg = self._last_serial_args
        if port not in available_ports():
            default = str(self.config["serial"].get("default_port", ""))
            port = preferred_serial_port(default=default)
            if not port:
                self.open_btn.setText(self._t("open"))
                self.on_event("no serial port selected")
                return
            self.port_combo.setCurrentText(port)
            self._last_serial_args = (port, baudrate, protocol_cfg)
        self.serial.close()
        self.serial.open(port, baudrate, protocol_cfg)
        self.open_btn.setText(self._t("close"))

    def choose_replay(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Choose parsed_frames.csv", str(PROJECT_ROOT / "logs"), "CSV (*.csv)")
        if path:
            self.replay_file.setText(path)
            self.replay = ReplaySource(path)

    def toggle_replay(self) -> None:
        if self.replay_timer.isActive():
            self.replay_timer.stop()
            self.replay_toggle_btn.setText(self._t("play"))
            return
        if not self.replay and self.replay_file.text():
            self.replay = ReplaySource(self.replay_file.text())
        if not self.replay:
            self.on_event("no replay CSV selected")
            return
        speed = float(self.replay_speed.currentText().rstrip("x"))
        interval_ms = max(5, int(50 / max(speed, 0.1)))
        self.replay_timer.start(interval_ms)
        self.replay_toggle_btn.setText(self._t("pause"))

    def step_replay(self) -> None:
        if not self.replay and self.replay_file.text():
            self.replay = ReplaySource(self.replay_file.text())
        if self.replay:
            frame = self.replay.step()
            if frame:
                self.on_frame(frame, display_ready=getattr(self.replay, "display_ready", False))
            elif self.replay_timer.isActive():
                self.replay_timer.stop()
                self.replay_toggle_btn.setText(self._t("play"))

    def save_screenshot(self) -> None:
        out = PROJECT_ROOT / "logs" / f"map_{int(time())}.png"
        out.parent.mkdir(parents=True, exist_ok=True)
        self.map_view.save_screenshot(out)
        self.on_event(f"screenshot saved: {out}")

    def save_visible_raw(self) -> None:
        out = PROJECT_ROOT / "logs" / f"visible_raw_{int(time())}.log"
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(self.raw_text.toPlainText(), encoding="utf-8")
        self.on_event(f"visible raw saved: {out}")

    def toggle_capture(self) -> None:
        if self.capture.active:
            self.stop_capture()
        else:
            self.start_capture()

    def start_capture(self) -> None:
        out = self.capture.start()
        self.capture_btn.setText(self._t("capture_stop"))
        self.capture_status.setText(f"{self._t('capture_recording')}: {out}")
        interval_s = float(self.config.get("display", {}).get("capture_map_snapshot_interval_s", 1.0))
        interval_ms = max(250, int(interval_s * 1000.0))
        self.capture_timer.start(interval_ms)
        self.capture_map_snapshot()
        self.on_event(f"capture started: {out}")

    def stop_capture(self) -> None:
        self.capture_map_snapshot()
        self.capture_timer.stop()
        summary = self.capture.stop()
        self._join_screenshot_saves(timeout_s=2.0)
        self.capture_btn.setText(self._t("capture_start"))
        self.capture_status.setText(
            f"{self._t('capture_saved')}: {summary.session_dir} | "
            f"frames={summary.display_frame_count} screenshots={summary.screenshot_count}"
        )
        self.on_event(
            f"capture saved: {summary.session_dir} "
            f"frames={summary.display_frame_count} screenshots={summary.screenshot_count}"
        )

    def capture_map_snapshot(self) -> None:
        path = self.capture.screenshot_path()
        if path is None:
            return
        image = self.map_view.grab().toImage()
        thread = threading.Thread(target=self._save_screenshot_image, args=(image, path), daemon=True)
        thread.start()
        self._screenshot_threads.append(thread)

    @staticmethod
    def _save_screenshot_image(image, path: Path) -> None:  # type: ignore[no-untyped-def]
        image.save(str(path))

    def _join_screenshot_saves(self, timeout_s: float | None = None) -> None:
        alive: list[threading.Thread] = []
        for thread in self._screenshot_threads:
            thread.join(timeout=timeout_s)
            if thread.is_alive():
                alive.append(thread)
        self._screenshot_threads = alive

    def _resolve_output_path(self, path_text: str) -> Path:
        out = Path(path_text)
        if not out.is_absolute():
            out = PROJECT_ROOT / out
        return out

    def start_window_recording(self) -> None:
        if self.window_record_timer.isActive() or self._record_frame_dir is not None:
            return
        primary_text = self._record_video_path_text or self._record_gif_path_text
        if not primary_text:
            return
        primary_path = self._resolve_output_path(primary_text)
        primary_path.parent.mkdir(parents=True, exist_ok=True)
        self._record_video_path = self._resolve_output_path(self._record_video_path_text) if self._record_video_path_text else None
        self._record_gif_path = self._resolve_output_path(self._record_gif_path_text) if self._record_gif_path_text else None
        if self._record_video_path:
            self._record_video_path.parent.mkdir(parents=True, exist_ok=True)
        if self._record_gif_path:
            self._record_gif_path.parent.mkdir(parents=True, exist_ok=True)

        self._record_frame_dir = primary_path.parent / f"{primary_path.stem}_frames"
        if self._record_frame_dir.exists():
            for old in self._record_frame_dir.glob("frame_*.*"):
                old.unlink()
        self._record_frame_dir.mkdir(parents=True, exist_ok=True)
        self._record_frame_index = 0
        self._record_encoding_done = False
        self._record_frame_size = None
        interval_ms = max(20, int(1000.0 / self._record_fps))
        self.window_record_timer.start(interval_ms)
        self.capture_window_record_frame()
        self.on_event(f"window recording started: {self._record_frame_dir}")

    def capture_window_record_frame(self) -> None:
        if self._record_frame_dir is None:
            return
        self._record_threads = [thread for thread in self._record_threads if thread.is_alive()]
        path = self._record_frame_dir / f"frame_{self._record_frame_index:05d}{self._record_frame_ext}"
        self._record_frame_index += 1
        image = self.grab().toImage()
        if self._record_max_width_px > 0 and image.width() > self._record_max_width_px:
            image = image.scaledToWidth(self._record_max_width_px, Qt.TransformationMode.SmoothTransformation)
        if self._record_frame_size is None:
            self._record_frame_size = (image.width(), image.height())
        elif (image.width(), image.height()) != self._record_frame_size:
            width, height = self._record_frame_size
            image = image.scaled(
                width,
                height,
                Qt.AspectRatioMode.IgnoreAspectRatio,
                Qt.TransformationMode.SmoothTransformation,
            )
        thread = threading.Thread(target=self._save_record_frame_image, args=(image, path), daemon=True)
        thread.start()
        self._record_threads.append(thread)

    @staticmethod
    def _save_record_frame_image(image, path: Path) -> None:  # type: ignore[no-untyped-def]
        image.save(str(path), "JPG", 92)

    def finish_window_recording(self) -> None:
        if self.window_record_timer.isActive():
            self.window_record_timer.stop()
            self.capture_window_record_frame()
        if self._record_encoding_done or self._record_frame_dir is None:
            return
        self._record_encoding_done = True
        for thread in self._record_threads:
            thread.join(timeout=10.0)
        self._record_threads = [thread for thread in self._record_threads if thread.is_alive()]
        if self._record_threads:
            self.on_event(f"window recording warning: {len(self._record_threads)} frame writers still running")
        frame_count = len(list(self._record_frame_dir.glob(f"frame_*{self._record_frame_ext}")))
        if frame_count == 0:
            self.on_event("window recording skipped: no frames")
            return
        if self._record_video_path:
            self._encode_window_video(frame_count)
        if self._record_gif_path:
            self._encode_window_gif(frame_count)

    def _encode_window_video(self, frame_count: int) -> None:
        if self._record_frame_dir is None or self._record_video_path is None:
            return
        ffmpeg = shutil.which("ffmpeg")
        if not ffmpeg:
            self.on_event(f"ffmpeg not found; kept {frame_count} PNG frames: {self._record_frame_dir}")
            return
        encode_fps = self._record_encode_fps(frame_count)
        pattern = self._record_frame_dir / f"frame_%05d{self._record_frame_ext}"
        cmd = [
            ffmpeg,
            "-y",
            "-framerate",
            f"{encode_fps:g}",
            "-i",
            str(pattern),
            "-vf",
            "scale=trunc(iw/2)*2:trunc(ih/2)*2",
            "-pix_fmt",
            "yuv420p",
            "-movflags",
            "+faststart",
            str(self._record_video_path),
        ]
        proc = subprocess.run(cmd, cwd=str(PROJECT_ROOT), capture_output=True, text=True)
        if proc.returncode != 0:
            tail = (proc.stderr or proc.stdout or "").strip().splitlines()[-3:]
            self.on_event(f"video encode failed: {' | '.join(tail)}")
            return
        self.on_event(f"window video saved: {self._record_video_path} frames={frame_count} fps={encode_fps:g}")

    def _encode_window_gif(self, frame_count: int) -> None:
        if self._record_frame_dir is None or self._record_gif_path is None:
            return
        ffmpeg = shutil.which("ffmpeg")
        if not ffmpeg:
            self.on_event(f"ffmpeg not found; kept {frame_count} PNG frames: {self._record_frame_dir}")
            return
        encode_fps = self._record_encode_fps(frame_count)
        pattern = self._record_frame_dir / f"frame_%05d{self._record_frame_ext}"
        palette = self._record_frame_dir / "palette.png"
        fps = f"{encode_fps:g}"
        palette_cmd = [
            ffmpeg,
            "-y",
            "-framerate",
            fps,
            "-i",
            str(pattern),
            "-vf",
            "fps=10,scale=960:-1:flags=lanczos,palettegen",
            str(palette),
        ]
        gif_cmd = [
            ffmpeg,
            "-y",
            "-framerate",
            fps,
            "-i",
            str(pattern),
            "-i",
            str(palette),
            "-lavfi",
            "fps=10,scale=960:-1:flags=lanczos[x];[x][1:v]paletteuse",
            str(self._record_gif_path),
        ]
        palette_proc = subprocess.run(palette_cmd, cwd=str(PROJECT_ROOT), capture_output=True, text=True)
        if palette_proc.returncode != 0:
            tail = (palette_proc.stderr or palette_proc.stdout or "").strip().splitlines()[-3:]
            self.on_event(f"gif palette failed: {' | '.join(tail)}")
            return
        gif_proc = subprocess.run(gif_cmd, cwd=str(PROJECT_ROOT), capture_output=True, text=True)
        if gif_proc.returncode != 0:
            tail = (gif_proc.stderr or gif_proc.stdout or "").strip().splitlines()[-3:]
            self.on_event(f"gif encode failed: {' | '.join(tail)}")
            return
        self.on_event(f"window gif saved: {self._record_gif_path} frames={frame_count}")

    def _record_encode_fps(self, frame_count: int) -> float:
        if self._auto_duration_s is not None and self._auto_duration_s > 0 and frame_count > 1:
            return max(1.0, float(frame_count) / float(self._auto_duration_s))
        return self._record_fps

    def send_command_text(self) -> None:
        self.serial.send_text(self.cmd_text.text() + "\n")

    def toggle_periodic_send(self, enabled: bool) -> None:
        if enabled:
            self.periodic_timer.start(int(self.periodic_ms.value()))
        else:
            self.periodic_timer.stop()

    def on_ui_tick(self) -> None:
        if self.demo_check.isChecked():
            self.on_frame(self.demo_source.next_frame())
        if self.capture.active and hasattr(self, "capture_status"):
            self.capture_status.setText(
                f"{self._t('capture_recording')}: "
                f"frames={self.capture.display_frame_count} screenshots={self.capture.screenshot_count}"
            )
        self._update_labels()

    def on_frame(self, frame: RobotFrame, *, display_ready: bool = False) -> None:
        self.logger.frame(frame)
        if display_ready:
            self.latest_frame = frame
            self.latest_local_frame = frame
            self._latest_raw_frame = frame
            self._latest_display_frame = frame
            self.capture.update_frame_pair(frame, frame)
            self.map_view.update_frame(frame)
            return

        local_frame = transform_frame(frame, self.config.get("transform", {}))
        display_frame = self._apply_start_pose(local_frame)
        if local_frame.lidar_online and local_frame.lidar_valid:
            self._live_fusion_has_lidar_anchor = True
        if (
            getattr(self, "live_fusion_check", None) is not None
            and self.live_fusion_check.isChecked()
            and self._live_fusion_has_lidar_anchor
        ):
            display_frame = self.live_fusion.process(display_frame)
        self.latest_frame = local_frame
        self.latest_local_frame = local_frame
        self._latest_raw_frame = frame
        self._latest_display_frame = display_frame
        self.capture.update_frame_pair(frame, display_frame)
        self.map_view.update_frame(display_frame)

    def _apply_start_pose(self, frame: RobotFrame) -> RobotFrame:
        side = self.start_side_combo.currentText() if hasattr(self, "start_side_combo") else "none"
        policy = self.start_policy_combo.currentText() if hasattr(self, "start_policy_combo") else "off"
        if side not in ("red", "blue") or policy == "off":
            return frame
        if policy == "auto_lidar_offline" and frame.lidar_online:
            return frame

        pose = self.config.get("robot", {}).get(f"start_pose_{side}", {})
        ox = float(pose.get("x_cm", 0.0))
        oy = float(pose.get("y_cm", 0.0))
        oyaw = float(pose.get("yaw_deg", 0.0))
        start_yaw = radians(oyaw)
        start_sin = sin(start_yaw)
        start_cos = cos(start_yaw)

        def apply_pose(x_cm: float, y_cm: float, yaw_deg: float) -> tuple[float, float, float]:
            return (
                ox + x_cm * start_cos - y_cm * start_sin,
                oy + x_cm * start_sin + y_cm * start_cos,
                yaw_deg + oyaw,
            )

        pos_x, pos_y, pos_yaw = apply_pose(frame.pos_x_cm, frame.pos_y_cm, frame.pos_yaw_deg)
        calib_x, calib_y, calib_yaw = apply_pose(frame.calib_x_cm, frame.calib_y_cm, frame.calib_yaw_deg)
        h30_x, h30_y, h30_yaw = apply_pose(frame.h30_x_cm, frame.h30_y_cm, frame.h30_yaw_deg)
        lidar_x, lidar_y, lidar_yaw = apply_pose(frame.lidar_x_cm, frame.lidar_y_cm, frame.lidar_yaw_deg)
        return replace(
            frame,
            pos_x_cm=pos_x,
            pos_y_cm=pos_y,
            pos_yaw_deg=pos_yaw,
            calib_x_cm=calib_x,
            calib_y_cm=calib_y,
            calib_yaw_deg=calib_yaw,
            h30_x_cm=h30_x,
            h30_y_cm=h30_y,
            h30_yaw_deg=h30_yaw,
            lidar_x_cm=lidar_x,
            lidar_y_cm=lidar_y,
            lidar_yaw_deg=lidar_yaw,
            encoder_x_cm=calib_x,
            encoder_y_cm=calib_y,
        )

    def on_raw(self, line: str) -> None:
        self.logger.raw(line)
        self.capture.raw_line(line)
        self.raw_summary.setText(line[:240])
        if self.raw_pause_check.isChecked():
            return
        if self.raw_text.blockCount() > 1000:
            self.raw_text.clear()
        self.raw_text.appendPlainText(line)

    def on_stats(self, stats: SerialStats) -> None:
        self.stats = stats

    def on_event(self, text: str) -> None:
        self.logger.event(text)
        if hasattr(self, "raw_text"):
            self.raw_text.appendPlainText(f"[event] {text}")
        if text.startswith("serial error"):
            self.open_btn.setText(self._t("open"))
            self.refresh_ports()
            if self.auto_reconnect_check.isChecked():
                QTimer.singleShot(1000, self._auto_reconnect)

    def on_mouse_position(self, x_cm: float, y_cm: float) -> None:
        self.value_labels["mouse"].setText(f"{x_cm:.1f}, {y_cm:.1f} cm")

    def _diagnosis_text(self, frame: RobotFrame) -> str:
        messages: list[str] = []
        if not frame.h30_valid:
            messages.append(self._t("diagnosis_h30_invalid"))
        if not (frame.status & 1):
            messages.append(self._t("diagnosis_encoder_invalid"))
        if not frame.lidar_online:
            messages.append(self._t("diagnosis_lidar_offline"))
        return " | ".join(messages) if messages else self._t("diagnosis_ok")

    def _set_sensor_status(self, key: str, ok: bool) -> None:
        label = self.sensor_status_labels.get(key)
        if label is None:
            return
        if key in ("encoder_1", "encoder_2"):
            state = self._t("sensor_pulse_received") if ok else self._t("sensor_pulse_missing")
        else:
            state = self._t("sensor_received") if ok else self._t("sensor_missing")
        label.setText(f"{self._t('sensor_' + key)}: {state}")
        label.setStyleSheet("color:#37d67a" if ok else "color:#ff6b6b")

    def _dt35_model_text(self, frame: RobotFrame) -> str:
        field_model = dict(self.config.get("field_model", {}))
        field_model.setdefault("field_width_cm", self.config.get("map", {}).get("field_width_cm", 1215.0))
        field_model.setdefault("field_height_cm", self.config.get("map", {}).get("field_height_cm", 1210.0))
        residual_gate = float(self.config.get("display", {}).get("live_fusion_dt35_residual_gate_cm", 40.0))
        yaw_source = str(self.config.get("dt35", {}).get("display_yaw_source", "pos"))
        yaw_for_dt35 = dt35_yaw_from_frame(frame, yaw_source)
        parts: list[str] = []
        for label, key, distance in (
            ("1", "sensor_1", frame.dt35_1_mm),
            ("2", "sensor_2", frame.dt35_2_mm),
        ):
            sensor_cfg = self.config.get("dt35", {}).get(key, {})
            ray = dt35_ray(frame.pos_x_cm, frame.pos_y_cm, yaw_for_dt35, sensor_cfg, distance, field_model)
            target = str(ray.get("expected_target", "")) or "-"
            target_type = str(ray.get("expected_target_type", ""))
            state = self._dt35_model_state(ray, residual_gate)
            expected = float(ray.get("expected_distance_cm", float("nan")))
            measured = float(ray.get("distance_cm", float("nan")))
            residual = float(ray.get("residual_cm", float("nan")))
            target_text = self._dt35_target_type_text(target_type)
            if bool(ray.get("valid", False)) and expected == expected and residual == residual:
                parts.append(
                    f"DT35-{label}: {target_text} {target} "
                    f"meas={measured:.1f}cm exp={expected:.1f}cm residual={residual:+.1f}cm {state}"
                )
            elif expected == expected:
                parts.append(f"DT35-{label}: {target_text} {target} exp={expected:.1f}cm {state}")
            else:
                parts.append(f"DT35-{label}: {target_text} {target} {state}")
        return " | ".join(parts)

    def _dt35_model_state(self, ray: dict[str, object], residual_gate_cm: float) -> str:
        if bool(ray.get("floor_hit_suspect", False)):
            return "floor/near-hit suspect; skipped"
        if bool(ray.get("valid", False)) and bool(ray.get("correction_allowed", False)):
            residual = float(ray.get("residual_cm", float("nan")))
            if residual == residual and abs(residual) > residual_gate_cm:
                return "large residual; skipped"
            return "usable"
        return self._dt35_skip_reason(ray)

    def _dt35_target_type_text(self, target_type: str) -> str:
        if target_type == "usable_wall":
            return "wall"
        if target_type == "solid_obstacle":
            return "obstacle"
        if target_type == "ignore":
            return "ignore"
        if target_type == "blocker":
            return "blocker"
        return "unknown"

    def _dt35_skip_reason(self, ray: dict[str, object]) -> str:
        if bool(ray.get("floor_hit_suspect", False)):
            return "floor/near-hit suspect; skipped"
        if bool(ray.get("correction_allowed", False)):
            return "usable"
        if bool(ray.get("corner_ambiguous", False)):
            return "corner; skipped"
        if str(ray.get("expected_target_type", "")) == "ignore":
            return "ignored zone; skipped"
        expected = float(ray.get("expected_distance_cm", float("nan")))
        max_range = float(ray.get("max_range_cm", float("nan")))
        if expected != expected:
            return "no modeled hit"
        if max_range == max_range and expected > max_range:
            return "out of range; skipped"
        return "skipped"

    def _update_labels(self) -> None:
        f = self.latest_local_frame
        map_frame = self._latest_display_frame
        if f:
            self.value_labels["pos"].setText(f"{f.pos_x_cm:.2f}, {f.pos_y_cm:.2f}, {f.pos_yaw_deg:.2f}")
            if map_frame:
                self.value_labels["map_pose"].setText(
                    f"{map_frame.pos_x_cm:.2f}, {map_frame.pos_y_cm:.2f}, {map_frame.pos_yaw_deg:.2f}"
                )
            self.value_labels["lidar"].setText(f"{f.lidar_x_cm:.2f}, {f.lidar_y_cm:.2f}, {f.lidar_yaw_deg:.2f}")
            self.value_labels["encoder"].setText(f"{f.encoder_x_cm:.2f}, {f.encoder_y_cm:.2f} cm")
            self.value_labels["h30"].setText(f"{f.h30_yaw_deg:.2f} deg")
            self.value_labels["diag"].setText(self._diagnosis_text(f))
            dt35_1 = f"{f.dt35_1_mm:.0f} mm" if f.dt35_1_valid else self._t("dt35_invalid")
            dt35_2 = f"{f.dt35_2_mm:.0f} mm" if f.dt35_2_valid else self._t("dt35_invalid")
            self.value_labels["dt35"].setText(f"{dt35_1} / {dt35_2}")
            self.value_labels["dt35_model"].setText(self._dt35_model_text(map_frame or f))
            self._set_sensor_status("encoder_1", f.x_pulse_seen or bool(f.status & (1 << 10)))
            self._set_sensor_status("encoder_2", f.y_pulse_seen or bool(f.status & (1 << 11)))
            self._set_sensor_status("h30", f.h30_valid)
            self._set_sensor_status("lidar", f.lidar_online)
            self._set_sensor_status("dt35_1", f.dt35_1_valid)
            self._set_sensor_status("dt35_2", f.dt35_2_valid)
        self.value_labels["fps"].setText(f"{self.stats.frames_per_s:.1f}")
        self.value_labels["bytes"].setText(f"{self.stats.rx_bytes_per_s:.0f} B/s")
        self.value_labels["dropped"].setText(str(self.stats.dropped_frames))
        self.value_labels["crc"].setText(str(self.stats.crc_errors))
        self.value_labels["parse"].setText(str(self.stats.parse_errors))
        self.value_labels["interval"].setText(f"{self.stats.last_interval_ms:.1f} ms")

    def closeEvent(self, event) -> None:  # type: ignore[no-untyped-def]
        self.periodic_timer.stop()
        self.replay_timer.stop()
        self.capture_timer.stop()
        self.finish_window_recording()
        if self.capture.active:
            self.capture.stop()
        self._join_screenshot_saves(timeout_s=2.0)
        self.serial.close()
        self.logger.stop()
        super().closeEvent(event)
