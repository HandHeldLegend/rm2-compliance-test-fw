#!/usr/bin/env python3
"""Run all WiFi menu tests over serial and summarize results."""

from __future__ import annotations

import re
import sys
import time
from dataclasses import dataclass, field

import serial

PORT = "COM6"
BAUD = 115200

# Steps after already being in the WiFi submenu.
# Pkteng tests: mode, channel preset 2 (ch6 / ch6 HT40), default Q-value.
# AP beacon: option 5, default SSID.
WIFI_MENU_TESTS: list[tuple[str, list[str]]] = [
    ("802.11b", ["1", "2", "1"]),
    ("802.11g", ["2", "2", "1"]),
    ("802.11n HT20", ["3", "2", "1"]),
    ("802.11n HT40", ["4", "2", "1"]),
    ("WiFi AP Beacon", ["5", "1"]),
]


@dataclass
class TestResult:
    name: str
    status: str = "UNKNOWN"
    steps_ok: list[str] = field(default_factory=list)
    steps_fail: list[str] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)
    raw: str = ""


def open_port() -> serial.Serial:
    ser = serial.Serial(PORT, BAUD, timeout=0.25)
    ser.dtr = True
    ser.rts = True
    time.sleep(1.5)
    return ser


def read_for(ser: serial.Serial, seconds: float) -> str:
    end = time.time() + seconds
    chunks: list[bytes] = []
    while time.time() < end:
        data = ser.read(8192)
        if data:
            chunks.append(data)
        else:
            time.sleep(0.05)
    return b"".join(chunks).decode("utf-8", errors="replace")


def send(ser: serial.Serial, line: str) -> None:
    ser.write((line.strip() + "\n").encode("ascii", errors="replace"))
    ser.flush()


def analyze_output(name: str, text: str) -> TestResult:
    result = TestResult(name=name, raw=text)

    if "STATUS: CONFIRMED RUNNING" in text:
        result.status = "CONFIRMED RUNNING"
    elif "STATUS: FAILED TO START" in text:
        result.status = "FAILED TO START"
    else:
        result.status = "NO STATUS BANNER"

    for match in re.finditer(r"\[\s*OK\s*\]\s*(.+)", text):
        result.steps_ok.append(match.group(1).strip())
    for match in re.finditer(r"\[FAIL\]\s*(.+)", text):
        result.steps_fail.append(match.group(1).strip())

    if "pkteng start" in text and any("pkteng start" in s for s in result.steps_fail):
        result.notes.append("pkteng ioctl failed (likely needs MFG firmware)")
    if "got unexpected packet 0" in text:
        result.notes.append("CYW43 bus contention messages seen")
    if "ERROR:" in text:
        for line in text.splitlines():
            if "ERROR:" in line:
                result.notes.append(line.strip())

    return result


def run_test(ser: serial.Serial, name: str, steps: list[str], per_step_wait: float) -> TestResult:
    print(f"\n{'=' * 60}\nRUNNING: {name}\n{'=' * 60}", flush=True)
    captured: list[str] = []
    for idx, step in enumerate(steps, 1):
        print(f"  step {idx}: {step!r}", flush=True)
        send(ser, step)
        chunk = read_for(ser, per_step_wait)
        captured.append(chunk)
        sys.stdout.write(chunk)
        sys.stdout.flush()
    return analyze_output(name, "".join(captured))


def goto_wifi_menu(ser: serial.Serial) -> None:
    print("Navigating to WiFi submenu...", flush=True)
    for attempt in range(4):
        send(ser, "0")
        text = read_for(ser, 1.5)
        if "Select test category" in text:
            send(ser, "3")
            read_for(ser, 2.0)
            return
        if "--- WiFi Tests" in text:
            return
    # Last resort: assume main menu and enter WiFi.
    send(ser, "3")
    read_for(ser, 2.0)


def reboot_device(ser: serial.Serial) -> None:
    print("Rebooting device for clean state...", flush=True)
    ser.write(b"\x03")
    ser.flush()
    ser.close()
    time.sleep(4.0)


def main() -> int:
    print(f"Opening {PORT} @ {BAUD}...", flush=True)
    try:
        ser = open_port()
    except serial.SerialException as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    reboot_device(ser)
    ser = open_port()
    read_for(ser, 1.5)
    goto_wifi_menu(ser)

    results: list[TestResult] = []
    for name, steps in WIFI_MENU_TESTS:
        per_step = 5.0 if "AP" not in name else 4.0
        results.append(run_test(ser, name, steps, per_step))
        time.sleep(0.5)

    ser.close()

    print(f"\n{'=' * 60}\nWIFI TEST SUMMARY\n{'=' * 60}")
    for r in results:
        print(f"\n{r.name}: {r.status}")
        if r.steps_ok:
            print(f"  OK steps: {', '.join(r.steps_ok)}")
        if r.steps_fail:
            print(f"  FAIL steps: {', '.join(r.steps_fail)}")
        for note in r.notes:
            print(f"  note: {note}")

    failed = [r for r in results if r.status != "CONFIRMED RUNNING"]
    print(f"\nTotal: {len(results) - len(failed)}/{len(results)} confirmed running")
    return 0 if not failed else 2


if __name__ == "__main__":
    raise SystemExit(main())
