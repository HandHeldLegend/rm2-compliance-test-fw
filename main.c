#include <stdio.h>

#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include "platform.h"
#include "test_ble.h"
#include "test_btc.h"
#include "test_ui.h"
#include "test_wifi.h"
#include "test_session.h"

static bool serial_session_active = false;

static void show_main_menu(void) {
    ui_clear_screen();
    test_session_print_banner();
    ui_print_banner();
    ui_print_reboot_notice();

    printf("Select test category:\n");
    printf("  1) BT Classic Tests\n");
    printf("  2) BTLE Tests\n");
    printf("  3) WiFi Tests\n");
    printf("  4) Reboot to USB bootloader (flash new firmware)\n");
    printf("\n");
}

static void run_main_menu(void) {
    while (ui_serial_connected()) {
        int choice = 0;

        show_main_menu();
        if (!ui_read_choice("Enter choice: ", 1, 4, &choice)) {
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
                test_wifi_menu();
                break;
            case 4:
                platform_reboot_to_bootloader();
                break;
            default:
                break;
        }
    }
}

int main(void) {
    stdio_init_all();

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
