from __future__ import annotations

from time import monotonic, time
from typing import Any

try:
    import serial
    from serial.tools import list_ports
except Exception:  # pragma: no cover
    serial = None
    list_ports = None

from PySide6.QtCore import QObject, QThread, Signal, Slot

from .data_model import RobotFrame, SerialStats
from .protocol import parse_line


def available_ports() -> list[str]:
    if list_ports is None:
        return []
    return [p.device for p in list_ports.comports()]


class SerialWorker(QObject):
    frame_received = Signal(object)
    raw_received = Signal(str)
    stats_changed = Signal(object)
    event = Signal(str)
    finished = Signal()

    def __init__(self, port: str, baudrate: int, protocol_cfg: dict[str, Any]) -> None:
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.protocol_cfg = protocol_cfg
        self._running = False
        self._ser = None
        self._stats = SerialStats(port=port)
        self._last_seq: int | None = None
        self._window_start = monotonic()
        self._window_bytes = 0
        self._window_frames = 0

    @Slot()
    def start(self) -> None:
        if serial is None:
            self.event.emit("pyserial is not installed")
            self.finished.emit()
            return
        self._running = True
        try:
            self._ser = serial.Serial(self.port, self.baudrate, timeout=0.05)
            self._stats.connected = True
            self.event.emit(f"opened {self.port} @ {self.baudrate}")
            buffer = bytearray()
            while self._running:
                chunk = self._ser.read(512)
                if chunk:
                    self._stats.rx_bytes += len(chunk)
                    self._window_bytes += len(chunk)
                    buffer.extend(chunk)
                    while b"\n" in buffer:
                        raw, _, rest = buffer.partition(b"\n")
                        buffer = bytearray(rest)
                        self._handle_line(raw.decode("ascii", errors="ignore").strip("\r"))
                self._update_rates()
        except Exception as exc:
            self.event.emit(f"serial error: {exc}")
        finally:
            self._stats.connected = False
            if self._ser:
                try:
                    self._ser.close()
                except Exception:
                    pass
            self.stats_changed.emit(self._stats.copy())
            self.finished.emit()

    @Slot()
    def stop(self) -> None:
        self._running = False

    @Slot(str)
    def send_text(self, text: str) -> None:
        if self._ser and self._ser.is_open:
            self._ser.write(text.encode("utf-8"))

    @Slot(str)
    def send_hex(self, text: str) -> None:
        if self._ser and self._ser.is_open:
            clean = "".join(text.split())
            self._ser.write(bytes.fromhex(clean))

    def _handle_line(self, line: str) -> None:
        if not line:
            return
        self.raw_received.emit(line)
        result = parse_line(
            line,
            mode=str(self.protocol_cfg.get("mode", "auto")),
            allow_no_crc=bool(self.protocol_cfg.get("allow_no_crc", True)),
            allow_legacy_csv=bool(self.protocol_cfg.get("allow_legacy_csv", True)),
        )
        if result.crc_error:
            self._stats.crc_errors += 1
        if result.frame is None:
            self._stats.parse_errors += 1
            return
        frame: RobotFrame = result.frame
        if self._last_seq is not None and frame.seq > 0:
            expected = self._last_seq + 1
            if frame.seq != expected and frame.seq > expected:
                self._stats.dropped_frames += frame.seq - expected
        if frame.seq > 0:
            self._last_seq = frame.seq
        now = time()
        if self._stats.last_frame_pc_time > 0:
            self._stats.last_interval_ms = (now - self._stats.last_frame_pc_time) * 1000.0
        self._stats.last_frame_pc_time = now
        self._stats.frames += 1
        self._window_frames += 1
        self.frame_received.emit(frame)

    def _update_rates(self) -> None:
        now = monotonic()
        elapsed = now - self._window_start
        if elapsed >= 1.0:
            self._stats.rx_bytes_per_s = self._window_bytes / elapsed
            self._stats.frames_per_s = self._window_frames / elapsed
            self._window_start = now
            self._window_bytes = 0
            self._window_frames = 0
            self.stats_changed.emit(self._stats.copy())


class SerialSession(QObject):
    frame_received = Signal(object)
    raw_received = Signal(str)
    stats_changed = Signal(object)
    event = Signal(str)

    def __init__(self) -> None:
        super().__init__()
        self.thread: QThread | None = None
        self.worker: SerialWorker | None = None

    def open(self, port: str, baudrate: int, protocol_cfg: dict[str, Any]) -> None:
        self.close()
        self.thread = QThread()
        self.worker = SerialWorker(port, baudrate, protocol_cfg)
        self.worker.moveToThread(self.thread)
        self.thread.started.connect(self.worker.start)
        self.worker.finished.connect(self.thread.quit)
        self.worker.frame_received.connect(self.frame_received)
        self.worker.raw_received.connect(self.raw_received)
        self.worker.stats_changed.connect(self.stats_changed)
        self.worker.event.connect(self.event)
        self.thread.start()

    def close(self) -> None:
        if self.worker:
            self.worker.stop()
        if self.thread:
            self.thread.quit()
            self.thread.wait(1000)
        self.thread = None
        self.worker = None

    def send_text(self, text: str) -> None:
        if self.worker:
            self.worker.send_text(text)

    def send_hex(self, text: str) -> None:
        if self.worker:
            self.worker.send_hex(text)
