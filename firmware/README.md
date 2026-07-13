# Proprietary CYW43439 MFG Wi-Fi firmware (NDA)

> **Do not commit or publish** the manufacturing Wi-Fi firmware blob,
> the Raspberry Pi `picow-wifi-mfg-tester.uf2`, or any file derived from them.
> Those artifacts are Infineon/Cypress proprietary and are provided by
> Raspberry Pi only under NDA / restricted lab terms.
>
> They are listed in `.gitignore`. This directory is intentionally empty
> in the public tree aside from this README.

## Two UF2s per board

| Image | Radio firmware | Tests enabled |
|-------|----------------|---------------|
| `RM2_COMPLIANCE_<board>_bt.uf2` | Stock Pico SDK CYW43 | BT Classic + BLE only (HCI at boot, same model as ee8e331) |
| `RM2_COMPLIANCE_<board>_wifi.uf2` | MFG / `WLTEST` (this folder) | WiFi 11b/g/n script TX only (ioctl/pkteng — **no STA**) |

Bluetooth and WiFi are **separate flashes**. Do not expect one UF2 to run both.

## Extract (NDA holders)

```bash
python tools/extract_mfg_fw_from_uf2.py path/to/picow-wifi-mfg-tester.uf2
```

Writes gitignored files such as:

```text
firmware/mfg_cyw43_fw.h
firmware/mfg_cyw43_fw_picow_7_95_49.h   # if produced / renamed for Pico W
firmware/43439A0_mfg_combined.bin
```

CMake prefers `mfg_cyw43_fw.h` (known-good under cyw43-driver: **7.95.39**),
then falls back to `mfg_cyw43_fw_picow_7_95_49.h`.

Expected identity: **CYW43439A0** / **mfgtest / WLTEST**.
On device you should see `cyw43 loaded ok` and a `Version: 7.95.x …` line when STA comes up.
If STA stays down with stalls, try the 7.95.39 extract rather than 7.95.49.

## Build

```bash
cmake -B build
cmake --build build
```

WiFi targets fail configure/build with a clear error if no MFG header is present.
BT targets always build without NDA files.

## Verify

1. Flash `*_wifi.uf2` → WiFi menus 1–3 → spectrum check.
2. Flash `*_bt.uf2` → BT Classic / BLE menus.
