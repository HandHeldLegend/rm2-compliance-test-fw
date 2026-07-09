# RM2 Compliance Test Firmware — Lab Guide

This guide explains how to flash the firmware, connect with **PuTTY** (or another serial terminal), and run the regulatory test menus on a **Raspberry Pi Pico W** (or GC Ultimate 2 hardware with the same CYW43439 radio).

---

## 1. Build and flash

### Prerequisites

- [Pico SDK](https://github.com/raspberrypi/pico-sdk) 2.2.0 (or compatible)
- CMake and Ninja
- `picotool` (included with the Pico SDK)

### Build

```bash
cd rm2-compliance-test-fw
mkdir build
cd build
cmake ..
cmake --build .
```

The UF2/ELF output is `build/RM2_COMPLIANCE.uf2` (or `.elf`).

### Flash

**Option A — USB bootloader (drag-and-drop)**

1. Hold **BOOTSEL** on the Pico, plug in USB, release BOOTSEL.
2. Copy `RM2_COMPLIANCE.uf2` to the **RPI-RP2** drive.

**Option B — From the running firmware**

1. Connect serial (see below).
2. At the main menu, choose **4) Reboot to USB bootloader**.
3. Copy the new UF2 when the drive appears.

**Option C — picotool**

```bash
picotool load -f build/RM2_COMPLIANCE.elf
picotool reboot
```

### Custom GPIO (production board)

If you are not using default Pico W pins, configure `EDIT_RM2_GPIO_PINS.cmake` before building. That file is included automatically by `CMakeLists.txt` when present.

---

## 2. Find the COM port (Windows)

1. Plug the board in over USB.
2. Open **Device Manager** → **Ports (COM & LPT)**.
3. Look for **USB Serial Device (COMx)** — Raspberry Pi boards use vendor ID **2E8A**.

Example: `USB Serial Device (COM6)`.

> **Note:** Only one program can use the port at a time. Close PuTTY, Arduino Serial Monitor, or other tools before using a different client.

---

## 3. Connect with PuTTY

### Download

Get PuTTY from [https://www.putty.org/](https://www.putty.org/) if it is not already installed.

### Session settings

| Setting | Value |
|--------|--------|
| Connection type | **Serial** |
| Serial line | Your COM port (e.g. `COM6`) |
| Speed (baud) | **115200** |

### Recommended terminal options

Open **Terminal** in the left tree:

- **Local echo**: *Force on* (optional — you will see what you type)
- **Local line editing**: *Force on* (optional — line is sent when you press Enter)

Open **Serial** in the left tree (if shown):

- **Data bits**: 8  
- **Stop bits**: 1  
- **Parity**: None  
- **Flow control**: None  

### Connect

1. Click **Open**.
2. A black terminal window appears.
3. If the menu does not show immediately, press **Enter** once or unplug/replug USB.

You should see:

```
================================================
  RM2 Compliance Test Firmware
  GC Ultimate 2 - Regulatory Testing
================================================

Select test category:
  1) BT Classic Tests
  2) BTLE Tests
  3) WiFi Tests
  4) Reboot to USB bootloader (flash new firmware)

Enter choice:
```

### Save the session (optional)

1. Enter the COM port and speed on the main screen.
2. Type a name under **Saved Sessions** → **Save**.
3. Next time, select the session and click **Load** before **Open**.

---

## 4. Using the menus

### Input

- Type a **number** for your choice and press **Enter**.
- Menus accept values in the shown range (e.g. `1`–`4` on the main menu).
- After a test finishes, read the **STATUS** line, then press **Enter** when prompted:
  ```
  Press Enter to return to the menu...
  ```

### Screen clears

The firmware clears the terminal between screens for readability. Scrollback in PuTTY (**Window** → increase **Lines of scrollback**) helps if you need to review earlier output.

### Active test banner

While a test is running, menus show a banner at the top:

```
************************************************
  ACTIVE TEST: 802.11b
  Ch 6 (2437 MHz), TX 16.75 dBm (Q=67)
  Status: CONFIRMED RUNNING
  Stop: power off the device
************************************************
```

### Keyboard shortcuts

| Key | Action |
|-----|--------|
| **Ctrl+C** or **Esc** | Reboot firmware (fresh start, stays in application) |
| **Main menu → 4** | Reboot to **USB bootloader** (for flashing only) |
| **Power off** | Stop RF transmission and return to menus after reconnect |

---

## 5. Test procedures

### 5.1 BT Classic TX

**Path:** Main menu → `1` → `1` (Start TX test)

You will be prompted for:

- Hopping mode (79-channel vs single frequency)
- Frequency (2402–2480 MHz)
- Modulation type, logical channel, packet type
- Packet length and TX power (0–8 dBm)

On success:

```
========================================
  BT Classic TX Test
  STATUS: CONFIRMED RUNNING
========================================
```

The controller must return HCI status `0x00` before this appears.

---

### 5.2 BLE TX

**Path:** Main menu → `2` → `1`

You will be prompted for:

- Channel index (0–39) or center frequency in MHz
- Packet length (0–37 bytes)
- Payload pattern (0–7)

Uses HCI LE Transmitter Test (`0x201E`). Success shows **STATUS: CONFIRMED RUNNING**.

---

### 5.3 WiFi tests

**Path:** Main menu → `3`

| Menu | Description |
|------|-------------|
| `1` | 802.11b continuous TX (pkteng) |
| `2` | 802.11g continuous TX |
| `3` | 802.11n HT20 continuous TX |
| `4` | 802.11n HT40 continuous TX |
| `5` | Open AP beacon (sanity check) |
| `0` | Back to main menu |

#### Channel selection

Each TX mode offers lab presets:

- **11b / 11g / 11n HT20:** channels 1, 6, 11 (2412 / 2437 / 2462 MHz)
- **11n HT40:** channels 3, 6, 9 (2422 / 2437 / 2452 MHz)

You can also enter a channel or frequency manually.

#### TX power

Power is set via the Broadcom `qtxpower` iovar in **quarter-dBm** units:

```
dBm = Q / 4
```

Example: **Q=67** → **16.75 dBm**

Options:

1. **Use recommended power** — values from the Raspberry Pi RP-002513-TE regulatory tables (varies by mode and channel)
2. **Enter Q-value** (0–127)
3. **Enter target power in dBm** (steps of 0.25, e.g. `16.75`)

#### Confirmation

During startup you will see driver steps marked `[ OK ]` or `[FAIL]`, then:

```
========================================
  802.11b
  STATUS: CONFIRMED RUNNING
========================================
```

**CONFIRMED RUNNING** means all driver commands succeeded. Verify actual RF output with your spectrum analyzer or power meter.

#### AP beacon test (recommended WiFi sanity check)

**Path:** WiFi menu → `5` → `1` (default SSID `RM2_COMPLIANCE_TEST`)

Look for that SSID on a phone, laptop, or WiFi scanner. No pkteng/MFG firmware is required for this test.

#### WiFi tips

- If you ran Bluetooth tests first, **power-cycle** before pkteng TX.
- Pkteng regulatory TX may require **MFG test firmware** on some setups; the AP beacon test is the simplest WiFi check.

---

## 6. Stopping a test

| Goal | Method |
|------|--------|
| Stop RF and return to menus | **Power off** the device, reconnect USB, reopen PuTTY |
| Restart firmware cleanly | **Ctrl+C** or **Esc** in the terminal |
| Flash new firmware | Main menu → **4** (USB bootloader) |

---

## 7. Optional: Python serial automation

For scripted testing (CI or repeated lab runs):

```bash
pip install -r tools/requirements.txt
```

```bash
# Read output for a few seconds
python tools/rm2_serial.py read --seconds 3 --port COM6

# Send one menu choice
python tools/rm2_serial.py send --line 3 --wait 2 --port COM6

# Run all WiFi tests (reboots first, then walks each mode)
python tools/verify_wifi_tests.py
```

Change `--port COM6` to match your system.

> Close PuTTY before running these scripts — only one client can own the COM port.

---

## 8. Troubleshooting

| Symptom | Likely cause | Fix |
|---------|----------------|-----|
| PuTTY says port unavailable | Another app has COM open | Close other serial monitors |
| `Invalid input` after every choice | Rare line-ending issue | Update firmware; use Enter only once per choice |
| Menu disappears instantly after test | Old firmware without “Press Enter” | Reflash latest build |
| WiFi pkteng `[FAIL]` | MFG firmware / bus contention | Try AP beacon test; power-cycle after BT |
| No menu until keypress | Normal USB CDC behavior | Press Enter after opening PuTTY |
| BT test times out on BD_ADDR | BTstack still starting | Wait and retry; reboot with Ctrl+C |

---

## 9. Quick reference — main menu

```
1  BT Classic Tests     → BTC vendor TX (HCI 0xFC51)
2  BTLE Tests           → BLE LE TX test (HCI 0x201E)
3  WiFi Tests           → 11b/g/n pkteng + AP beacon
4  USB bootloader       → Flash only (not a normal reboot)
```

---

## 10. Lab checklist

- [ ] Firmware built and flashed  
- [ ] COM port identified in Device Manager  
- [ ] PuTTY: Serial, 115200 baud, correct COM port  
- [ ] Main menu visible  
- [ ] Test started → **CONFIRMED RUNNING** seen  
- [ ] RF verified on bench equipment  
- [ ] Device powered off between major test changes when needed  
