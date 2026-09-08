# RM2 Compliance Test Firmware: Lab Guide

This guide explains how to flash the firmware, connect with **PuTTY** (or another serial terminal), and run the regulatory test menus on a **Raspberry Pi Pico W** (or GC Ultimate 2 hardware with the same CYW43439 radio).

---

## 1. Build and flash

### Prerequisites

- [Pico SDK](https://github.com/raspberrypi/pico-sdk) 2.2.0 (or compatible)
- CMake and Ninja
- `picotool` (included with the Pico SDK)
- For WiFi UF2s: local MFG firmware extract (NDA). See [`firmware/README.md`](firmware/README.md)

### Build (two UF2s per board)

One `cmake --build` produces **Bluetooth (stock)** and **WiFi (MFG)** images for every
pinout under [`board_pinouts/`](board_pinouts/):

```bash
cd rm2-compliance-test-fw
# NDA holders: once, before WiFi builds:
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

**Option A: USB bootloader (drag-and-drop)**

1. Hold **BOOTSEL** on the Pico, plug in USB, release BOOTSEL.
2. Copy the matching UF2 to the **RPI-RP2** drive
   (e.g. `RM2_COMPLIANCE_picow_bt.uf2` for Pico W Bluetooth tests).

**Option B: From the running firmware**

1. Connect serial (see below).
2. At the main menu, choose the **Reboot to USB bootloader** item
   (option **3** on BT builds, option **4** on WiFi builds).
3. Copy the new UF2 when the drive appears.

**Option C: picotool**

```bash
picotool load -f build/RM2_COMPLIANCE_picow_bt.elf
picotool reboot
```

### Board pinouts

CYW43439 GPIOs live under [`board_pinouts/`](board_pinouts/). Each `.cmake`
file sets `BOARD_NAME` (shown in the serial banner) plus the pin map.
Flash the UF2 that matches your hardware **and** test category. The banner
shows board name, pinout id, and build variant.

---

## 2. Find the COM port (Windows)

1. Plug the board in over USB.
2. Open **Device Manager** → **Ports (COM & LPT)**.
3. Look for **USB Serial Device (COMx)**. Raspberry Pi boards use vendor ID **2E8A**.
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
| `1` | BT Classic tests (BR/EDR) |
| `2` | BTLE tests (LE) |
| `3` | Reboot to USB bootloader |

#### BT Classic submenu

| Menu | Description |
|------|-------------|
| `1` | Transmit test, vendor `Tx_Test` (0xFC51) |
| `2` | Receiver test, vendor `Rx_Test` (0xFC52). Needs a second board transmitting |
| `3` | Stop test / HCI reset. Drops RF without a power cycle |

Command encodings are in [`docs/BREDR_VENDOR_HCI.md`](docs/BREDR_VENDOR_HCI.md).

**For regulatory TX, take the recommended option at each prompt:**

- **TX packet type.** Pick the named packet (`DH5`, `2-DH5`, `3-DH5`, and so on).
  The menu sets `Logical_Channel` and `Baseband_Packet_Type` together. Those two
  are not independent, and the same packet-type byte means a different packet on
  air depending on the logical channel.
- **Packet length.** Choose *Use controller maximum (0xFFFF)*. An explicit
  length below the packet maximum lowers the duty cycle and therefore the
  measured average power.
- **TX power.** Choose *Maximum output power*. This sends
  `Transmit_Power=0x09, dBm=0x00, Table_Index=0x00`, the combination Infineon
  documents for maximum output. The dBm and table-index modes are for
  characterisation, not for limits testing.

The transmit menu prints the exact bytes it puts on the wire, for example:

```
Tx_Test bytes: 51 FC 10 4C EF 16 C1 CD 28 01 28 04 00 0E FF FF 09 00 00
```

#### BTLE submenu

| Menu | Description |
|------|-------------|
| `1` | Transmit test, `HCI_LE_Transmitter_Test` (0x201E) |
| `2` | Receiver test, `HCI_LE_Receiver_Test` (0x201D). Needs a second board transmitting |
| `3` | Stop test / HCI reset |

Channel can be entered as a BLE channel index (0 to 39) or as a centre
frequency in MHz, which must land on a BLE channel (an even offset from 2402).

Both flows reset the controller before each run. Without that reset the
controller refuses a second `LE_Transmitter_Test` while one is still running and
returns `0x0C COMMAND_DISALLOWED`, which previously meant BLE channels could only
be changed by power cycling.

#### Verifying BLE TX with a second board

1. On the **RX** board: menu `2` then `2`. Enter the channel.
2. On the **TX** board: menu `2` then `1`. Enter **the same** channel, plus
   length and payload type.
3. Press Enter on the RX board to end the test. `HCI_LE_Test_End` (0x201F)
   returns the packet count.

Unlike the BR/EDR receiver there are no periodic statistics; the controller
reports its count once, when the test ends. Expect several thousand packets over
a few seconds.

To prove the transmitter is on the channel it was told, repeat with the RX board
on a different channel. The count should be exactly zero.

#### Verifying BR/EDR TX with a second board

`Tx_Test` returns `status=0x00` as long as the command parses, so a success
message alone does **not** prove RF is on the requested channel. To confirm it,
run the two-board procedure (Infineon BT Radio Test section 3.5):

1. On the **TX** board: menu `1` then `1`. Note the `Local BD_ADDR` it prints,
   then configure and start the transmit test.
2. On the **RX** board: menu `1` then `2`. Enter the TX board's BD_ADDR, then set
   **the same** frequency, modulation, packet type and packet length.
3. The RX board prints `Rx_Test` statistics once per report period.

Every parameter must match or the receiver will not synchronise. Expect roughly
267 packets per second for 2-DH5 at maximum length.

To prove the transmitter is on a *fixed* channel rather than hopping, repeat
step 2 with the RX board on a different frequency. A correctly fixed
transmitter yields no statistics events at all.

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
| Stop BR/EDR RF and return to menus | BT Classic submenu, **Stop test / HCI reset** |
| Stop RF and return to menus (any test) | **Power off** the device, reconnect USB, reopen PuTTY |
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
