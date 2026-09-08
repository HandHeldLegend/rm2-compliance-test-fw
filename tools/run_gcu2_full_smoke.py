#!/usr/bin/env python3
"""Careful GCU2 end-to-end command smoke: BT then WiFi MFG."""
from __future__ import annotations

import shutil
import string
import sys
import time
from ctypes import windll

import serial
from serial.tools import list_ports

ROOT = r"C:\Users\Mitch\GitRepos\rm2-compliance-test-fw"
BT_UF2 = ROOT + r"\build\RM2_COMPLIANCE_gcu2_bt.uf2"
WIFI_UF2 = ROOT + r"\build\RM2_COMPLIANCE_gcu2_wifi.uf2"


def find_port() -> str:
    for p in list_ports.comports():
        if "2E8A" in (p.hwid or "").upper():
            return p.device
    raise SystemExit("No Pico USB serial found")


def rpi_drives() -> list[str]:
    out: list[str] = []
    mask = windll.kernel32.GetLogicalDrives()
    for i, letter in enumerate(string.ascii_uppercase):
        if mask & (1 << i) and os_path_exists(f"{letter}:/INFO_UF2.TXT"):
            out.append(letter)
    return out


def os_path_exists(path: str) -> bool:
    import os

    return os.path.exists(path)


def open_p(port: str, retries: int = 30) -> serial.Serial:
    last: Exception | None = None
    for _ in range(retries):
        try:
            s = serial.Serial(port, 115200, timeout=0.2)
            time.sleep(1.0)
            return s
        except Exception as e:  # noqa: BLE001
            last = e
            time.sleep(0.4)
    raise SystemExit(f"open fail {port}: {last}")


def drain(s: serial.Serial, sec: float) -> str:
    end = time.time() + sec
    chunks: list[bytes] = []
    while time.time() < end:
        try:
            d = s.read(8192)
        except Exception:
            break
        if d:
            chunks.append(d)
        else:
            time.sleep(0.02)
    return b"".join(chunks).decode("utf-8", "replace")


def send(s: serial.Serial, line: str) -> None:
    s.write((line + "\n").encode())
    s.flush()


def reboot_session(port: str) -> serial.Serial:
    try:
        s = open_p(port, retries=5)
        s.write(b"\x03")
        s.flush()
        time.sleep(0.3)
        s.close()
    except Exception:
        pass
    time.sleep(4.5)
    port = find_port()
    last_err: Exception | None = None
    for _ in range(15):
        try:
            s = open_p(port, retries=8)
            t = banner(s)
            if "Board:" in t or "Enter choice" in t:
                return s
            s.close()
        except Exception as e:  # noqa: BLE001
            last_err = e
            time.sleep(1.0)
            try:
                port = find_port()
            except Exception:
                pass
    raise SystemExit(f"reboot_session failed: {last_err}")


def banner(s: serial.Serial) -> str:
    """Drain until the main menu is visible (do not send empty lines, that is Invalid input)."""
    end = time.time() + 20.0
    chunks: list[str] = []
    while time.time() < end:
        chunk = drain(s, 0.5)
        if chunk:
            chunks.append(chunk)
        text = "".join(chunks)
        if "Board:" in text and "Enter choice" in text:
            return text
    return "".join(chunks)


def flash_uf2(port: str, uf2: str, bootsel_choice: str) -> str:
    # Always Ctrl+C back to a fresh main menu before selecting bootsel.
    s = reboot_session(port)
    t = banner(s)
    print("--- pre-flash banner ---")
    for ln in t.splitlines():
        if any(k in ln for k in ("Board", "Build", "Pinout", "Select")):
            print(ln)
    send(s, bootsel_choice)
    time.sleep(0.8)
    try:
        s.close()
    except Exception:
        pass
    for i in range(40):
        ds = rpi_drives()
        if ds:
            shutil.copyfile(uf2, f"{ds[0]}:/fw.uf2")
            print(f"FLASHED {uf2} -> {ds[0]}:")
            time.sleep(12)
            break
        time.sleep(0.5)
    else:
        raise SystemExit("BOOTSEL drive not found")
    # port may change; rediscover after USB re-enum
    time.sleep(2.0)
    port = find_port()
    # Fresh image: do not Ctrl+C, just open and wait for menu
    for _ in range(20):
        try:
            s = open_p(port, retries=8)
            t = banner(s)
            if "Enter choice" in t or "Board:" in t or "Select test" in t:
                s.close()
                return port
            s.close()
        except Exception:
            pass
        time.sleep(1.0)
    return find_port()


def expect_ok(name: str, text: str, must_have: list[str], must_not: list[str]) -> bool:
    bad = [m for m in must_not if m in text]
    missing = [m for m in must_have if m not in text]
    if bad or missing:
        print(f"FAIL {name}: missing={missing} bad={bad}")
        # show relevant lines
        for ln in text.splitlines():
            if any(
                k in ln
                for k in (
                    "FAIL",
                    "ERROR",
                    "STATUS",
                    "HCI",
                    "BD_ADDR",
                    "CONFIRMED",
                    "Failed",
                    "[ OK ]",
                    "[FAIL]",
                )
            ):
                print(" ", ln)
        return False
    print(f"PASS {name}")
    return True


def run_btc(s: serial.Serial) -> bool:
    print("=== BTC TX ===", flush=True)
    # single-freq (1), 2441 MHz, PRBS9, ACL Basic, DM1, len 17, +3 dBm (max for 0xFC51)
    seq = ["1", "1", "1", "2441", "4", "1", "3", "17", "3"]
    for line in seq:
        send(s, line)
        time.sleep(0.5)
    out = drain(s, 16.0)
    send(s, "")
    drain(s, 1.5)
    return expect_ok(
        "BTC TX",
        out,
        must_have=["STATUS: CONFIRMED RUNNING", "Device BD_ADDR"],
        must_not=["Failed to start CYW43", "Timed out waiting", "STATUS: FAILED"],
    )


def run_ble(s: serial.Serial) -> bool:
    print("=== BLE TX ===", flush=True)
    for line in ("2", "1", "1", "0", "37", "0"):
        send(s, line)
        time.sleep(0.45)
    out = drain(s, 12.0)
    send(s, "")
    drain(s, 1.5)
    return expect_ok(
        "BLE TX",
        out,
        must_have=["STATUS: CONFIRMED RUNNING", "HCI complete"],
        must_not=["Failed to start CYW43", "Timed out waiting", "STATUS: FAILED"],
    )


def run_wifi_modes(port: str, s: serial.Serial) -> tuple[bool, serial.Serial]:
    """Run 11b/g/n with a clean reboot between modes so ratespec cannot stick."""
    ok_all = True
    for mode, name in (("1", "11b"), ("2", "11g"), ("3", "11n")):
        print(f"=== WiFi {name} ===", flush=True)
        for line in (mode, "1", "1"):
            send(s, line)
            time.sleep(0.45)
        out = drain(s, 32.0)
        set_fails = [ln for ln in out.splitlines() if "[FAIL]" in ln and "SET" in ln]
        fails = [ln for ln in out.splitlines() if "[FAIL]" in ln]
        confirmed = "STATUS: CONFIRMED RUNNING" in out
        print(f"  confirmed={confirmed} set_fails={len(set_fails)} fails={len(fails)}")
        for ln in fails:
            print(" ", ln)
        if set_fails or not confirmed:
            ok_all = False
            print(f"FAIL WiFi {name}")
            for ln in out.splitlines():
                if any(k in ln for k in ("Preparing", "MFG bus", "2g_rate", "STATUS", "[ OK ]", "[FAIL]")):
                    print(" ", ln)
        else:
            print(f"PASS WiFi {name}")
        # Reboot for clean radio state before next modulation
        s.write(b"\x03")
        s.flush()
        s.close()
        time.sleep(4.0)
        s = open_p(port)
        banner(s)
    return ok_all, s


def main() -> int:
    port = find_port()
    print("Using", port, flush=True)

    # Ensure BT/stock gcu2 image
    s = reboot_session(port)
    t = banner(s)
    print("--- banner ---")
    for ln in t.splitlines():
        if any(k in ln for k in ("Board", "Build", "Pinout", "Select")):
            print(ln)
    if "GC Ultimate" not in t or "Bluetooth (stock" not in t:
        s.close()
        print("Flashing gcu2_bt.uf2 ...", flush=True)
        # WiFi MFG bootsel is 4; BT bootsel is 3.
        choice = "4" if ("WiFi MFG" in t or "802.11b" in t) else "3"
        port = flash_uf2(port, BT_UF2, choice)
        s = open_p(port)
        t = banner(s)
        print("--- banner after BT flash ---")
        for ln in t.splitlines():
            if any(k in ln for k in ("Board", "Build", "Pinout", "Select")):
                print(ln)
        if "GC Ultimate" not in t or "Bluetooth (stock" not in t:
            print(t[-1200:])
            s.close()
            return 2

    results: dict[str, bool] = {}
    # Classic first, then BLE (vendor Classic RF after LE test is flaky without reset)
    results["BTC"] = run_btc(s)
    s.close()
    s = reboot_session(port)
    banner(s)
    results["BLE"] = run_ble(s)
    s.close()

    print("Flashing gcu2_wifi.uf2 ...", flush=True)
    # reboot_session inside flash_uf2 returns to BT main menu, then option 3 = bootsel
    port = flash_uf2(port, WIFI_UF2, "3")
    s = open_p(port)
    t = banner(s)
    print("--- WiFi banner ---")
    for ln in t.splitlines():
        if any(k in ln for k in ("Board", "Build", "Pinout", "Select")):
            print(ln)
    if "GC Ultimate" not in t or "WiFi MFG" not in t:
        print(t[-1200:])
        s.close()
        return 3

    results["WIFI"], s = run_wifi_modes(port, s)
    try:
        s.close()
    except Exception:
        pass

    print("\n==== SUMMARY ====")
    for k, v in results.items():
        print(k, "PASS" if v else "FAIL")
    return 0 if all(results.values()) else 4


if __name__ == "__main__":
    raise SystemExit(main())
