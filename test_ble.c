#include "test_ble.h"

#include <stdio.h>

#include "hci_common.h"
#include "platform.h"
#include "test_ui.h"
#include "test_session.h"

static uint8_t ble_mhz_to_channel(int freq_mhz) {
    int k = (freq_mhz - 2402) / 2;
    if (k < 0) {
        k = 0;
    }
    if (k > 39) {
        k = 39;
    }
    return (uint8_t)k;
}

static void print_payload_help(void) {
    printf("  Payload types:\n");
    printf("    0 = PRBS9\n");
    printf("    1 = 11110000\n");
    printf("    2 = 10101010\n");
    printf("    3 = PRBS15\n");
    printf("    4 = 11111111\n");
    printf("    5 = 00000000\n");
    printf("    6 = 00001111\n");
    printf("    7 = 01010101\n");
}

static void run_ble_tx_flow(void) {
    int input_mode = 0;
    int channel = 0;
    int frequency_mhz = 0;
    int length = 37;
    int payload = 0;

    ui_clear_screen();
    ui_print_reboot_notice();

    printf("\n--- BLE Transmit Test ---\n");
    printf("This test uses HCI LE Transmitter Test (0x201E).\n\n");

    printf("Select channel input mode:\n");
    printf("  1) Enter BLE channel index (0-39)\n");
    printf("  2) Enter center frequency in MHz (2402-2480)\n");
    if (!ui_read_choice("Enter choice: ", 1, 2, &input_mode)) {
        return;
    }

    if (input_mode == 1) {
        if (!ui_read_int_in_range("Enter BLE channel (0-39): ", 0, 39, &channel)) {
            return;
        }
        frequency_mhz = 2402 + (channel * 2);
    } else {
        if (!ui_read_int_in_range("Enter frequency in MHz (2402-2480): ", 2402, 2480, &frequency_mhz)) {
            return;
        }
        if ((frequency_mhz - 2402) % 2 != 0) {
            printf("Frequency must be on a BLE channel (even MHz offset from 2402).\n");
            return;
        }
        channel = ble_mhz_to_channel(frequency_mhz);
    }

    if (!ui_read_int_in_range("Enter packet length in bytes (0-37): ", 0, 37, &length)) {
        return;
    }

    print_payload_help();
    if (!ui_read_int_in_range("Enter payload type (0-7): ", 0, 7, &payload)) {
        return;
    }

    printf("\nStarting BLE TX test with:\n");
    printf("  Channel: %d (%d MHz)\n", channel, frequency_mhz);
    printf("  Length: %d bytes\n", length);
    printf("  Payload type: %d\n", payload);
    printf("\n");

    printf("\nSending BLE TX command...\n");

    bool started = hci_start_ble_tx((uint8_t)channel, (uint8_t)length, (uint8_t)payload);

    char details[64];
    snprintf(details, sizeof(details), "Ch %d (%d MHz), %d bytes, payload %d", channel, frequency_mhz, length, payload);
    ui_print_test_result(started, "BLE TX Test", details);
}

void test_ble_menu(void) {
    if (!platform_init_btstack()) {
        return;
    }

    while (ui_serial_connected()) {
        int choice = 0;

        ui_clear_screen();
        test_session_print_banner();
        printf("--- BTLE Tests ---\n");
        printf("  1) Start TX test (configure all parameters)\n");
        printf("  0) Back to main menu\n");

        if (!ui_read_choice("Enter choice: ", 0, 1, &choice)) {
            continue;
        }

        if (choice == 0) {
            return;
        }

        run_ble_tx_flow();
    }

    platform_shutdown_btstack();
}
