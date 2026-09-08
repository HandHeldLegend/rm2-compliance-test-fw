# Board pinouts

Each `*.cmake` file in this folder defines CYW43439 SPI/control GPIOs for one
hardware variant. CMake builds **two** firmwares per file:

| File | Bluetooth (stock) | WiFi (MFG) |
|------|-------------------|------------|
| `picow.cmake` | `RM2_COMPLIANCE_picow_bt.uf2` | `RM2_COMPLIANCE_picow_wifi.uf2` |
| `gcu2.cmake` | `RM2_COMPLIANCE_gcu2_bt.uf2` | `RM2_COMPLIANCE_gcu2_wifi.uf2` |

## Adding a board

1. Copy an existing file, e.g. `myboard.cmake`.
2. Set a display name. Either:

**A. Stock Pico W-style pins (SDK defaults)**

```cmake
set(BOARD_NAME "My Pico W Clone")
set(RM2_USE_SDK_CYW43_PINS 1)
```

**B. Custom CYW43439 GPIOs** (required for GC Ultimate 2, etc.)

```cmake
set(BOARD_NAME "My Board Display Name")
set(BT_ON_PIN ...)
set(BT_DATA_OUT_PIN ...)
set(BT_DATA_IN_PIN ...)
set(BT_WAKE_PIN ...)
set(BT_CLOCK_PIN ...)
set(BT_CS_PIN ...)
```

`BOARD_NAME` appears in the serial UI banner. The filename stem (`myboard`)
is the pinout id used in the UF2 names.

3. Reconfigure and build (MFG header required for `*_wifi` targets, see
   `firmware/README.md`):

```bash
cmake -B build
cmake --build build
```

Flash the UF2 that matches the board **and** test type (BT vs WiFi).
