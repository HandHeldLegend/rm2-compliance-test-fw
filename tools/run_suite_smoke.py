#!/usr/bin/env python3
"""Flash-free serial check of WiFi/MFG or BT/stock menus on COM port."""
from __future__ import annotations

import argparse
import sys
import time

import serial
from serial.tools import list_ports


def find_pico() -> str:
    for p in list_ports.comports():
        if "2E8A" in (p.hwid or "").upper():
            return p.device
    raise SystemExit("No Pico USB serial (VID 2E8A) found")


def require_board(text: str, board_substr: str) -> None:
    """Optional --board filter, e.g. 'gcu2' or 'GC Ultimate' / 'picow'."""
    if not board_substr:
        return
    key = board_substr.lower()
    blob = text.lower()
    if key in blob or key in blob.replace(" ", ""):
        return
    # also match pinout id line
    if f"pinout id: {key}" in blob:
        return
    raise SystemExit(
        f"Connected board banner does not match --board {board_substr!r}.\n"
        f"Flash RM2_COMPLIANCE_{key}_*.uf2 for this hardware."
    )


def open_p(port: str, retries: int = 25) -> serial.Serial:
    last = None
    for _ in range(retries):
        try:
            s = serial.Serial(port, 115200, timeout=0.15)
            s.dtr = True
            s.rts = True
            time.sleep(1.0)
            return s
        except Exception as e:
            last = e
            time.sleep(0.4)
    raise SystemExit(f"open fail {port}: {last}")


def drain(s: serial.Serial, sec: float) -> str:
    end = time.time() + sec
    chunks: list[bytes] = []
    while time.time() < end:
        d = s.read(8192)
        if d:
            chunks.append(d)
        else:
            time.sleep(0.02)
    return b"".join(chunks).decode("utf-8", "replace")


def send(s: serial.Serial, line: str) -> None:
    s.write((line + "\n").encode())
    s.flush()


def reboot_session(port: str) -> serial.Serial:
    """Ctrl+C then reopen so we always see the build banner."""
    try:
        s = open_p(port, retries=8)
        s.write(b"\x03")
        s.flush()
        s.close()
    except Exception:
        pass
    time.sleep(3.5)
    return open_p(port)


def run_wifi(port: str, board: str = "") -> int:
    s = reboot_session(port)
    send(s, "")
    t = drain(s, 2.0)
    print("--- banner ---")
    for ln in t.splitlines():
        if any(k in ln for k in ("Board", "Build", "Pinout", "Select", "802.11", "WLTEST")):
            print(ln)
    require_board(t, board)
    if "WiFi MFG" not in t:
        print(t[-1500:])
        s.close()
        print("ERROR: flash RM2_COMPLIANCE_*_wifi.uf2 first (gcu2_wifi for GCU2)")
        return 2

    results = {}
    for mode, name in [("1", "11b"), ("2", "11g"), ("3", "11n")]:
        print(f"=== {name} ===", flush=True)
        for line in (mode, "1", "1"):
            send(s, line)
            time.sleep(0.45)
        out = drain(s, 28.0)
        fails = [ln for ln in out.splitlines() if "[FAIL]" in ln]
        set_fails = [ln for ln in fails if "SET" in ln]
        if "CONFIRMED RUNNING" in out:
            status = "CONFIRMED"
        elif "FAILED TO START" in out:
            status = "FAILED"
        else:
            status = "UNKNOWN"
        results[name] = {"status": status, "fails": fails, "set_fails": set_fails}
        print("status", status, flush=True)
        for ln in fails:
            print(" FAIL", ln)
        # show a few OK lines for confidence
        oks = [ln for ln in out.splitlines() if "[ OK ]" in ln]
        print(f"  OK steps: {len(oks)}")
        send(s, "")
        drain(s, 1.5)

    s.close()
    print("SUMMARY")
    for k, v in results.items():
        print(k, v["status"], "set_fails", len(v["set_fails"]), "fails", len(v["fails"]))

    if any(r["set_fails"] for r in results.values()):
        return 3
    if any(r["status"] != "CONFIRMED" for r in results.values()):
        return 4
    if any(r["fails"] for r in results.values()):
        return 5
    return 0


def run_bt(port: str, board: str = "") -> int:
    s = reboot_session(port)
    send(s, "")
    t = drain(s, 2.0)
    print("--- banner ---")
    for ln in t.splitlines():
        if any(k in ln for k in ("Board", "Build", "Pinout", "Select", "BT")):
            print(ln)
    require_board(t, board)
    if "Bluetooth (stock" not in t:
        print(t[-1500:])
        s.close()
        print("ERROR: flash RM2_COMPLIANCE_*_bt.uf2 first (gcu2_bt for GCU2)")
        return 2

    # Smoke: enter BT Classic menu and back, then BLE menu and back.
    # Full TX needs lab params; here we only ensure menus open with no crash.
    send(s, "1")
    out = drain(s, 3.0)
    print("--- BT Classic submenu ---")
    print(out[-800:])
    ok_btc = "BT Classic" in out or "Classic" in out
    send(s, "0")
    drain(s, 1.5)

    send(s, "2")
    out = drain(s, 3.0)
    print("--- BTLE submenu ---")
    print(out[-800:])
    ok_ble = "BTLE" in out or "BLE" in out or "LE" in out
    send(s, "0")
    drain(s, 1.0)
    s.close()

    if not ok_btc or not ok_ble:
        print("SUMMARY BT menu smoke FAILED", ok_btc, ok_ble)
        return 4
    print("SUMMARY BT menu smoke OK")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="")
    ap.add_argument("--suite", choices=("wifi", "bt"), required=True)
    ap.add_argument(
        "--board",
        default="",
        help="Require banner match, e.g. gcu2 or picow (avoids wrong UF2 on wrong hardware)",
    )
    args = ap.parse_args()
    port = args.port or find_pico()
    print("Using", port)
    if args.suite == "wifi":
        return run_wifi(port, args.board)
    return run_bt(port, args.board)


if __name__ == "__main__":
    raise SystemExit(main())
