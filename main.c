#include <stdio.h>

#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include "platform.h"
#include "test_ui.h"
#include "test_session.h"

#if RM2_ENABLE_BT
#include "test_ble.h"
#include "test_btc.h"
#endif

#if RM2_ENABLE_WIFI
#include "test_wifi.h"
#endif

#ifndef RM2_ENABLE_BT
#define RM2_ENABLE_BT 0
#endif
#ifndef RM2_ENABLE_WIFI
#define RM2_ENABLE_WIFI 0
#endif

static bool serial_session_active = false;

static void show_main_menu(void) {
    ui_clear_screen();
    test_session_print_banner();
    ui_print_banner();
    ui_print_reboot_notice();

    printf("Select test:\n");
#if RM2_ENABLE_BT
    printf("  1) BT Classic Tests\n");
    printf("  2) BTLE Tests\n");
    printf("  3) Reboot to USB bootloader (flash new firmware)\n");
#elif RM2_ENABLE_WIFI
    printf("  1) 802.11b continuous TX (script)\n");
    printf("  2) 802.11g continuous TX (script)\n");
    printf("  3) 802.11n continuous TX (script)\n");
    printf("  4) Reboot to USB bootloader (flash new firmware)\n");
#else
#error "Build must enable RM2_ENABLE_BT or RM2_ENABLE_WIFI"
#endif
    printf("\n");
}

static void run_main_menu(void) {
    while (ui_serial_connected()) {
        int choice = 0;

        show_main_menu();
#if RM2_ENABLE_BT
        if (!ui_read_choice("Enter choice: ", 1, 3, &choice)) {
            continue;
        }
        switch (choice) {
            case 1:
                test_btc_menu();
                break;
            case 2:
                test_ble_menu();
                break;
            case 3:
                platform_reboot_to_bootloader();
                break;
            default:
                break;
        }
#elif RM2_ENABLE_WIFI
        if (!ui_read_choice("Enter choice: ", 1, 4, &choice)) {
            continue;
        }
        switch (choice) {
            case 1:
                test_wifi_run_mode(WIFI_MODE_MENU_11B);
                break;
            case 2:
                test_wifi_run_mode(WIFI_MODE_MENU_11G);
                break;
            case 3:
                test_wifi_run_mode(WIFI_MODE_MENU_11N);
                break;
            case 4:
                platform_reboot_to_bootloader();
                break;
            default:
                break;
        }
#endif
    }
}

int main(void) {
    stdio_init_all();

    /* Wait for USB before CYW43/HCI init (more reliable on Windows re-enum). */
    while (!stdio_usb_connected()) {
        sleep_ms(50);
    }
    sleep_ms(200);

    if (!platform_init_cyw43()) {
        return -1;
    }

    while (true) {
        if (!stdio_usb_connected()) {
            serial_session_active = false;
            sleep_ms(100);
            continue;
        }

        if (!serial_session_active) {
            serial_session_active = true;
            run_main_menu();
        }

        ui_poll_escape();
        sleep_ms(10);
    }

    return 0;
}
