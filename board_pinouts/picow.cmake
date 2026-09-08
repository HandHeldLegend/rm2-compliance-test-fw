# Raspberry Pi Pico W: use Pico SDK board defaults for CYW43 pins
# (same as ee8e331 which did not override pins on stock Pico W).
set(BOARD_NAME "Raspberry Pi Pico W")
set(RM2_USE_SDK_CYW43_PINS 1)
# Pin values kept for documentation / tools; not applied when RM2_USE_SDK_CYW43_PINS=1
set(BT_ON_PIN 23)
set(BT_DATA_OUT_PIN 24)
set(BT_DATA_IN_PIN  24)
set(BT_WAKE_PIN     24)
set(BT_CLOCK_PIN    29)
set(BT_CS_PIN       25)
