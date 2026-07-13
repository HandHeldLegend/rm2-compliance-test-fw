# RM2 Compliance Test Firmware — Lab Guide

This guide explains how to flash the firmware, connect with **PuTTY** (or another serial terminal), and run the regulatory test menus on a **Raspberry Pi Pico W** (or GC Ultimate 2 hardware with the same CYW43439 radio).

---

## 1. Build and flash

### Prerequisites

- [Pico SDK](https://github.com/raspberrypi/pico-sdk) 2.2.0 (or compatible)
- CMake and Ninja
- `picotool` (included with the Pico SDK)
- For WiFi UF2s: local MFG firmware extract (NDA) — see [`firmware/README.md`](firmware/README.md)

### Build (two UF2s per board)

One `cmake --build` produces **Bluetooth (stock)** and **WiFi (MFG)** images for every
pinout under [`board_pinouts/`](board_pinouts/):

```bash
cd rm2-compliance-test-fw
# NDA holders — once:
python tools/extract_mfg_fw_from_uf2.py path/to/picow-wifi-mfg-tester.uf2

cmake -B build
cmake --build build
```

| Board | Bluetooth / BLE (stock FW) | WiFi pkteng (MFG FW) |
|-------|----------------------------|----------------------|
| Raspberry Pi Pico W | `build/RM2_COMPLIANCE_picow_bt.uf2` | `build/RM2_COMPLIANCE_picow_wifi.uf2` |
| GC Ultimate 2 | `build/RM2_COMPLIANCE_gcu2_bt.uf2` | `build/RM2_COMPLIANCE_gcu2_wifi.uf2` |

Each image only exposes its own tests (no mixed BT+WiFi menu on one UF2).

Add another board by dropping `board_pinouts/<name>.cmake` (see that folder’s README).

### Flash

**Option A — USB bootloader (drag-and-drop)**

1. Hold **BOOTSEL** on the Pico, plug in USB, release BOOTSEL.
2. Copy the matching UF2 to the **RPI-RP2** drive
   (e.g. `RM2_COMPLIANCE_picow_bt.uf2` for Pico W Bluetooth tests).

**Option B — From the running firmware**

1. Connect serial (see below).
2. At the main menu, choose the **Reboot to USB bootloader** item
   (option **3** on BT builds, option **4** on WiFi builds).
3. Copy the new UF2 when the drive appears.

**Option C — picotool**

```bash
picotool load -f build/RM2_COMPLIANCE_picow_bt.elf
picotool reboot
```

### Board pinouts

CYW43439 GPIOs live under [`board_pinouts/`](board_pinouts/). Each `.cmake`
file sets `BOARD_NAME` (shown in the serial banner) plus the pin map.
Flash the UF2 that matches your hardware **and** test category — the banner
shows board name, pinout id, and build variant.

---

## 2. Find the COM port (Windows)

1. Plug the board in over USB.
2. Open **Device Manager** → **Ports (COM & LPT)**.
3. Look for **USB Serial Device (COMx)** — Raspberry Pi boards use vendor ID **2E8A**.
4. Note the COM number for PuTTY.

---

## 3. Connect with PuTTY

1. Open PuTTY.
2. **Connection type:** Serial.
3. **Serial line:** `COMx` (from Device Manager).
4. **Speed:** `115200`.
5. Click **Open**.
6. Press **Enter** once if the menu does not appear immediately.

You should see a banner with board name, pinout id, and build variant, then the
main menu for that UF2.

---

## 4. Using the menus

- Enter the number shown and press **Enter**.
- Invalid input is rejected with a retry prompt.
- After a test starts (or fails), press **Enter** to return to the menu when prompted.
- **Ctrl+C** or **Esc** reboots the application firmware (not bootloader).

### Bluetooth build (`*_bt.uf2`)

| Menu | Description |
|------|-------------|
| `1` | BT Classic TX |
| `2` | BTLE TX |
| `3` | Reboot to USB bootloader |

### WiFi build (`*_wifi.uf2`)

Official RPi Pico W 2 GHz script modulations only:

| Menu | Description | Script rate |
|------|-------------|-------------|
| `1` | 802.11b continuous TX | `2g_rate -r 1` |
| `2` | 802.11g continuous TX | `2g_rate -r 6` |
| `3` | 802.11n continuous TX (HT20 MCS0) | `2g_rate -h 0 -b 20` |
| `4` | Reboot to USB bootloader |

Selectable: channel (`-c`) and TX power Q (`-q`). Dest MAC is fixed to
`00:11:22:33:44:55`. Bandwidth is always 20 MHz.

#### Confirmation (WiFi)

Each script value SET is followed by a GET where possible. `[ OK ]` means
readback matched, or this firmware does not expose a readable value for that
iovar (SET was still accepted). Action-only commands note when there is no
value readback.

**Important:** Continuous TX RF must be verified with lab spectrum equipment.
Use the WiFi/MFG UF2 for regulatory TX ([`firmware/README.md`](firmware/README.md)).

---

## 5. Stopping a test

| Goal | Method |
|------|--------|
| Stop RF and return to menus | **Power off** the device, reconnect USB, reopen PuTTY |
| Restart firmware cleanly | **Ctrl+C** or **Esc** in the terminal |
| Flash new firmware | Main menu → bootloader item |

---

## 6. Optional: Python serial automation

```bash
python tools/rm2_serial.py --port COMx
python tools/verify_wifi_tests.py   # WiFi/MFG UF2
```

---

## 7. Quick reference

| UF2 | Tests |
|-----|-------|
| `RM2_COMPLIANCE_<board>_bt.uf2` | BT Classic + BLE (stock CYW43) |
| `RM2_COMPLIANCE_<board>_wifi.uf2` | 802.11b/g/n script TX (MFG CYW43) |
