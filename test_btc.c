#include "test_btc.h"

#include <stdio.h>

#include "hci_common.h"
#include "platform.h"
#include "pico/stdlib.h"
#include "test_ui.h"
#include "test_session.h"

static void print_modulation_help(void) {
    printf("  Modulation types:\n");
    printf("    1 = 0x00 8-bit pattern\n");
    printf("    2 = 0xFF 8-bit pattern\n");
    printf("    3 = 0xAA 8-bit pattern\n");
    printf("    4 = PRBS9\n");
    printf("    9 = 0xF0 8-bit pattern\n");
}

static void print_logical_channel_help(void) {
    printf("  Logical channels:\n");
    printf("    0 = ACL EDR\n");
    printf("    1 = ACL Basic\n");
    printf("    2 = eSCO EDR\n");
    printf("    3 = eSCO Basic\n");
    printf("    4 = SCO Basic\n");
}

static void print_packet_type_help(void) {
    printf("  Common packet types:\n");
    printf("     3 = DM1\n");
    printf("     4 = DH1 / 2-DH1\n");
    printf("    10 = DM3 / 2-DH3\n");
    printf("    11 = DH3 / 3-DH3\n");
    printf("    14 = DM5 / 2-DH5\n");
    printf("    15 = DH5 / 3-DH5\n");
}

static bool ensure_bdaddr(void) {
    if (hci_has_bdaddr()) {
        return true;
    }

    printf("Reading device BD_ADDR...\n");
    if (hci_read_bdaddr_wait(5000)) {
        return true;
    }

    printf("ERROR: Timed out reading BD_ADDR.\n");
    return false;
}

static void run_btc_tx_flow(void) {
    int hopping_mode = 0;
    int frequency_mhz = 0;
    int modulation_type = 0;
    int logical_channel = 0;
    int packet_type = 0;
    int packet_length = 0;
    int tx_power_dbm = 0;

    ui_clear_screen();
    ui_print_reboot_notice();

    printf("\n--- BT Classic Transmit Test ---\n");
    printf("This test uses the vendor BTC TX HCI command (0xFC51).\n\n");

    if (!ensure_bdaddr()) {
        return;
    }

    printf("Select hopping mode:\n");
    printf("  0) 79-channel hopping\n");
    printf("  1) Single frequency\n");
    if (!ui_read_choice("Enter choice: ", 0, 1, &hopping_mode)) {
        return;
    }

    if (!ui_read_int_in_range("Enter frequency in MHz (2402-2480): ", 2402, 2480, &frequency_mhz)) {
        return;
    }

    print_modulation_help();
    if (!ui_read_int_in_range("Enter modulation type: ", 1, 9, &modulation_type)) {
        return;
    }
    if (modulation_type != 1 && modulation_type != 2 && modulation_type != 3 &&
        modulation_type != 4 && modulation_type != 9) {
        printf("Invalid modulation type for this test.\n");
        return;
    }

    print_logical_channel_help();
    if (!ui_read_int_in_range("Enter logical channel: ", 0, 4, &logical_channel)) {
        return;
    }

    print_packet_type_help();
    if (!ui_read_int_in_range("Enter packet type (0-15): ", 0, 15, &packet_type)) {
        return;
    }

    if (!ui_read_int_in_range("Enter packet length (0-339): ", 0, 339, &packet_length)) {
        return;
    }

    if (!ui_read_int_in_range("Enter TX power in dBm (0-8): ", 0, 8, &tx_power_dbm)) {
        return;
    }

    printf("\nStarting BTC TX test with:\n");
    printf("  Hopping mode: %d\n", hopping_mode);
    printf("  Frequency: %d MHz\n", frequency_mhz);
    printf("  Modulation: %d\n", modulation_type);
    printf("  Logical channel: %d\n", logical_channel);
    printf("  Packet type: %d\n", packet_type);
    printf("  Packet length: %d\n", packet_length);
    printf("  TX power: %d dBm\n", tx_power_dbm);
    printf("\n");

    printf("\nSending BTC TX command...\n");

    bool started = hci_start_btc_tx((uint8_t)hopping_mode, (uint32_t)frequency_mhz, (uint8_t)modulation_type,
                                    (uint8_t)logical_channel, (uint8_t)packet_type, (uint16_t)packet_length,
                                    (uint8_t)tx_power_dbm);

    char details[64];
    snprintf(details, sizeof(details), "Freq %d MHz, power %d dBm", frequency_mhz, tx_power_dbm);
    ui_print_test_result(started, "BT Classic TX Test", details);
}

void test_btc_menu(void) {
    if (!platform_init_btstack()) {
        return;
    }

    while (ui_serial_connected()) {
        int choice = 0;

        ui_clear_screen();
        test_session_print_banner();
        printf("--- BT Classic Tests ---\n");
        printf("  1) Start TX test (configure all parameters)\n");
        printf("  0) Back to main menu\n");

        if (!ui_read_choice("Enter choice: ", 0, 1, &choice)) {
            continue;
        }

        if (choice == 0) {
            return;
        }

        run_btc_tx_flow();
    }

    platform_shutdown_btstack();
}
