#!/usr/bin/env python3
"""Standalone CLI tester that exercises VWFrequencyController."""

from __future__ import annotations

import argparse
import sys
from typing import Optional

from controller import ControllerError, VWFrequencyController, format_frame


def hexdump(data: bytes) -> str:
    return " ".join(f"{byte:02X}" for byte in data)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Quick tester for the VW frequency controller protocol (Mode 3 focus)."
    )
    parser.add_argument("--port", default="COM9", help="Serial port name.")
    parser.add_argument("--baudrate", type=int, default=38400, help="Serial baud rate.")
    parser.add_argument("--mode", type=int, default=3, choices=[1, 2, 3], help="Measurement mode byte.")
    parser.add_argument("--k", type=int, default=31, help="Integration counter value.")
    parser.add_argument("--timer-mp", type=float, default=0.008192, help="Timer MP parameter (seconds).")
    parser.add_argument("--calibration", type=int, default=2, help="Calibration offset for wait cycles.")
    parser.add_argument("--timeout", type=float, default=3.0, help="Frame read timeout (seconds).")
    parser.add_argument("--interval", type=float, default=0.0, help="Delay between frames (seconds).")
    parser.add_argument("--count", type=int, default=5, help="Frames to capture (0 = continuous).")
    parser.add_argument("--no-auto-trigger", action="store_true", help="Do not send trigger bytes automatically.")
    parser.add_argument("--print-raw", action="store_true", help="Print raw payload bytes for each frame.")
    parser.add_argument("--log-bytes", action="store_true", help="Echo serial traffic (sent/received bytes).")
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    controller = VWFrequencyController(
        baudrate=args.baudrate,
        read_timeout=args.timeout,
        calibration=args.calibration,
        timer_mp=args.timer_mp,
    )

    if args.log_bytes:
        def _logger(direction: str, data: bytes) -> None:
            arrow = ">" if direction == ">" else "<"
            print(f"{arrow} {hexdump(data)}")

        controller.set_bytes_logger(_logger)

    try:
        controller.connect(args.port)
        print(f"Connected on {args.port}")

        controller.configure_measurement(args.mode, args.k)
        window = controller.measurement_window_seconds()
        cycle = controller.measurement_gap_seconds()
        print(
            f"Handshake sent: mode={controller.mode}, k={controller.k_value}. "
            f"window~= {window:.6f}s, cycle~= {cycle:.6f}s"
        )

        count = None if args.count == 0 else max(0, args.count)
        auto_trigger = not args.no_auto_trigger
        for idx, frame in enumerate(
            controller.iter_frames(count=count, timeout=args.timeout, auto_trigger=auto_trigger, interval=args.interval),
            start=1,
        ):
            print(f"[{idx}] {format_frame(frame)}")
            if args.print_raw:
                raw_hex = hexdump(frame.raw_payload)
                print(f"    raw: {raw_hex}")

    except ControllerError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
        return 0
    finally:
        controller.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())

