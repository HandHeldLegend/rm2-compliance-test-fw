# RM2 Compliance Test Firmware

USB-serial test firmware for **regulatory / compliance RF checks** on products using the Infineon **CYW43439** (Raspberry Pi Pico W–class radio), including **GC Ultimate 2** and Pico W pinouts.

It drives the same class of WiFi continuous-TX sequences as the official Raspberry Pi Pico W 2.4 GHz WiFi test script, plus Bluetooth Classic and BLE HCI test menus. Host interaction is a simple menu over USB CDC (e.g. PuTTY).

## Two images per board

Bluetooth and WiFi **cannot** share one UF2. Each board pinout builds two firmwares:

| UF2 | Radio firmware | What it runs |
|-----|----------------|--------------|
| `RM2_COMPLIANCE_<board>_bt.uf2` | Stock Pico SDK CYW43 | BT Classic + BLE tests |
| `RM2_COMPLIANCE_<board>_wifi.uf2` | **MFG / WLTEST** (NDA) | WiFi 11b / 11g / 11n continuous TX + packet-engine RX |

Flash the UF2 that matches your hardware **and** the test you need. The serial banner shows board name, pinout id, and build variant.

**Stock WiFi firmware is not enough for these continuous-TX tests.** The same application code with the SDK’s normal CYW43 blob does not produce useful `pkteng` TX. Manufacturing / `WLTEST` firmware is required for the WiFi image. See [`firmware/README.md`](firmware/README.md).

## Quick start

### Prerequisites

- [Pico SDK](https://github.com/raspberrypi/pico-sdk) 2.2.0 (or compatible), CMake, and a build toolchain
- For WiFi builds only: local extract of Raspberry Pi `picow-wifi-mfg-tester` MFG firmware (NDA). **Do not commit these files.**

### Build

```bash
# NDA holders: once, before WiFi builds:
python tools/extract_mfg_fw_from_uf2.py path/to/picow-wifi-mfg-tester.uf2

cmake -B build
cmake --build build
```

| Board | Bluetooth | WiFi (MFG) |
|-------|-----------|------------|
| Pico W | `build/RM2_COMPLIANCE_picow_bt.uf2` | `build/RM2_COMPLIANCE_picow_wifi.uf2` |
| GC Ultimate 2 | `build/RM2_COMPLIANCE_gcu2_bt.uf2` | `build/RM2_COMPLIANCE_gcu2_wifi.uf2` |

BT targets build without NDA files. WiFi targets fail configure if the MFG header is missing.

### Flash and run

1. Hold **BOOTSEL**, plug USB, copy the matching UF2 to **RPI-RP2**.
2. Open the board’s COM port in a serial terminal (115200 is typical for Pico USB CDC; follow [`TEST_GUIDE.md`](TEST_GUIDE.md)).
3. Use the on-device menus. **Ctrl+C** / **Esc** reboots the app; the menu can also reboot into the USB bootloader for the next flash.

Full lab steps (PuTTY, COM port, stop/restart, automation): **[`TEST_GUIDE.md`](TEST_GUIDE.md)**.

## What the menus cover

**Bluetooth UF2**: Classic and BLE regulatory-style HCI tests (power, channels, etc. as implemented in-menu).

**WiFi UF2**: Aligned with the official Pico W 2 GHz script (`chanspec`, `2g_rate`, `txpwr1 -o -q`, continuous `pkteng` TX). Also includes packet-engine **RX** with `pkteng_stats` counters for a second-board sanity check (rate/MCS buckets + RSSI). Continuous TX must still be confirmed with spectrum / lab equipment.

Default script TX power is **Q=70 → 17.5 dBm**. Dest MAC is fixed to `00:11:22:33:44:55`.

## Boards and pinouts

CYW43439 GPIOs are defined under [`board_pinouts/`](board_pinouts/). Add a new `.cmake` file to produce `bt` and `wifi` UF2s for another product.

## Repository layout

| Path | Role |
|------|------|
| `main.c`, `platform.c`, `test_*.c` | Menus and test implementations |
| `board_pinouts/` | Per-board CYW43 pin maps |
| `firmware/` | Local MFG header only (gitignored blobs + NDA notes) |
| `tools/` | MFG extract helper and optional serial scripts |
| `TEST_GUIDE.md` | Lab operator guide |

## Important notes

- **NDA / proprietary:** Do not publish Infineon/Cypress MFG WiFi firmware, `picow-wifi-mfg-tester.uf2`, or extracts. They stay gitignored; see [`firmware/README.md`](firmware/README.md).
- **Not a WiFi sniffer:** `pkteng` RX reports counters and RSSI, not packet payloads.
- **Not a networking stack:** The WiFi/MFG image is for ioctl / packet-engine lab use (no STA association requirement for the script path).

## License

MIT. See [`LICENSE`](LICENSE). Third-party radio firmware remains under its own proprietary terms and is not part of this license.
