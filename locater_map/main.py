from __future__ import annotations

import argparse
import sys
from pathlib import Path


def _add_src_to_path() -> None:
    root = Path(__file__).resolve().parent
    src = root / "src"
    if str(src) not in sys.path:
        sys.path.insert(0, str(src))


def main() -> int:
    _add_src_to_path()
    from locater_map.app import run_app

    parser = argparse.ArgumentParser(description="R1 real-time locater map")
    parser.add_argument("--config", default=None, help="Path to config JSON")
    parser.add_argument("--demo", action="store_true", help="Start with mock robot data")
    parser.add_argument("--replay", default=None, help="Replay parsed_frames.csv")
    parser.add_argument("--serial-port", default=None, help="Open this COM port on startup")
    parser.add_argument("--baudrate", type=int, default=None, help="Override serial baudrate")
    parser.add_argument("--duration-s", type=float, default=None, help="Auto-close after N seconds")
    parser.add_argument("--screenshot", default=None, help="Save a screenshot before auto-close")
    args = parser.parse_args()
    return run_app(
        config_path=args.config,
        demo=args.demo,
        replay_path=args.replay,
        serial_port=args.serial_port,
        baudrate=args.baudrate,
        duration_s=args.duration_s,
        screenshot_path=args.screenshot,
    )


if __name__ == "__main__":
    raise SystemExit(main())
