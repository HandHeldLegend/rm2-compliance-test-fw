#!/usr/bin/env python3
"""USB serial helper for RM2 compliance test firmware.

Examples:
  python tools/rm2_serial.py read --seconds 3
  python tools/rm2_serial.py send --line 1 --wait 2
  python tools/rm2_serial.py session --script tools/serial_scripts/main_menu.txt
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import serial
from serial import SerialException

DEFAULT_PORT = "COM6"
DEFAULT_BAUD = 115200


def open_port(port: str, baud: int) -> serial.Serial:
    try:
        ser = serial.Serial(port, baud, timeout=0.2)
        ser.dtr = True
        ser.rts = True
    except SerialException as exc:
        print(
            f"ERROR: Could not open {port}: {exc}\n"
            "Close any other serial monitor (PuTTY, Arduino IDE, etc.) and retry.",
            file=sys.stderr,
        )
        raise SystemExit(1) from exc

    # Pico USB CDC often needs a moment after open before output appears.
    time.sleep(1.0)
    return ser


def drain(ser: serial.Serial, seconds: float) -> str:
    end = time.time() + seconds
    chunks: list[bytes] = []
    while time.time() < end:
        data = ser.read(4096)
        if data:
            chunks.append(data)
        else:
            time.sleep(0.05)
    text = b"".join(chunks).decode("utf-8", errors="replace")
    if text:
        sys.stdout.write(text)
        sys.stdout.flush()
    return text


def send_line(ser: serial.Serial, line: str) -> None:
    payload = (line.rstrip("\r\n") + "\n").encode("ascii", errors="replace")
    ser.write(payload)
    ser.flush()


def cmd_read(args: argparse.Namespace) -> int:
    with open_port(args.port, args.baud) as ser:
        drain(ser, args.seconds)
    return 0


def cmd_send(args: argparse.Namespace) -> int:
    with open_port(args.port, args.baud) as ser:
        if args.drain_first:
            drain(ser, args.drain_seconds)
        send_line(ser, args.line)
        drain(ser, args.wait)
    return 0


def cmd_session(args: argparse.Namespace) -> int:
    script_path = Path(args.script)
    if not script_path.is_file():
        print(f"ERROR: Script not found: {script_path}", file=sys.stderr)
        return 1

    lines = [
        line.strip()
        for line in script_path.read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.strip().startswith("#")
    ]

    with open_port(args.port, args.baud) as ser:
        drain(ser, args.initial_wait)
        for idx, line in enumerate(lines, start=1):
            print(f"\n--- step {idx}: send {line!r} ---\n", flush=True)
            send_line(ser, line)
            drain(ser, args.step_wait)
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=DEFAULT_PORT, help=f"Serial port (default: {DEFAULT_PORT})")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help=f"Baud rate (default: {DEFAULT_BAUD})")

    sub = parser.add_subparsers(dest="command", required=True)

    read_p = sub.add_parser("read", help="Read from the device for a few seconds")
    read_p.add_argument("--seconds", type=float, default=3.0, help="How long to read")
    read_p.set_defaults(func=cmd_read)

    send_p = sub.add_parser("send", help="Send one line and read the response")
    send_p.add_argument("--line", required=True, help="Line to send (Enter is appended)")
    send_p.add_argument("--wait", type=float, default=2.0, help="Seconds to read after send")
    send_p.add_argument("--drain-first", action="store_true", help="Read existing output before sending")
    send_p.add_argument("--drain-seconds", type=float, default=1.0, help="Seconds to drain when --drain-first")
    send_p.set_defaults(func=cmd_send)

    session_p = sub.add_parser("session", help="Run a scripted menu/input sequence")
    session_p.add_argument("--script", required=True, help="Text file with one input line per step")
    session_p.add_argument("--initial-wait", type=float, default=2.0, help="Seconds to read before first send")
    session_p.add_argument("--step-wait", type=float, default=2.0, help="Seconds to read after each send")
    session_p.set_defaults(func=cmd_session)

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
