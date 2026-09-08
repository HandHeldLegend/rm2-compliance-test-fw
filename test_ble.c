#include "test_ble.h"

#include <stdio.h>

#include "hci_common.h"
#include "pico/stdlib.h"
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

/* Both test roles pick a channel the same way. Returns false if the user
 * backed out or entered a frequency that is not a BLE channel. */
static bool read_ble_channel(int *channel_out, int *frequency_out) {
    int input_mode = 0;
    int channel = 0;
    int frequency_mhz = 0;

    printf("Select channel input mode:\n");
    printf("  1) Enter BLE channel index (0-39)\n");
    printf("  2) Enter center frequency in MHz (2402-2480)\n");
    if (!ui_read_choice("Enter choice: ", 1, 2, &input_mode)) {
        return false;
    }

    if (input_mode == 1) {
        if (!ui_read_int_in_range("Enter BLE channel (0-39): ", 0, 39, &channel)) {
            return false;
        }
        frequency_mhz = 2402 + (channel * 2);
    } else {
        if (!ui_read_int_in_range("Enter frequency in MHz (2402-2480): ", 2402, 2480,
                                  &frequency_mhz)) {
            return false;
        }
        if ((frequency_mhz - 2402) % 2 != 0) {
            printf("Frequency must be on a BLE channel (even MHz offset from 2402).\n");
            return false;
        }
        channel = ble_mhz_to_channel(frequency_mhz);
    }

    *channel_out = channel;
    *frequency_out = frequency_mhz;
    return true;
}

static void run_ble_rx_flow(void) {
    int channel = 0;
    int frequency_mhz = 0;
    uint16_t packets = 0;

    ui_clear_screen();
    ui_print_reboot_notice();

    printf("\n--- BLE Receiver Test ---\n");
    printf("This test uses HCI LE Receiver Test (0x201D).\n");
    printf("The channel must match the transmitting board.\n\n");

    printf("HCI reset before BLE RX...\n");
    if (!hci_do_reset_wait(3000)) {
        printf("ERROR: HCI reset failed.\n");
        return;
    }
    sleep_ms(100);

    if (!read_ble_channel(&channel, &frequency_mhz)) {
        return;
    }

    printf("\nStarting BLE RX test on channel %d (%d MHz)...\n", channel, frequency_mhz);

    if (!hci_start_ble_rx((uint8_t)channel)) {
        char details[64];
        snprintf(details, sizeof(details), "Ch %d (%d MHz)", channel, frequency_mhz);
        ui_print_test_result(false, "BLE RX Test", details);
        return;
    }

    printf("\nReceiving. Start the TX board now.\n");
    printf("The controller reports its packet count when the test ends.\n");
    printf("Press Enter to stop.\n\n");

    test_session_set_running("BLE RX Test", "LE Receiver Test running");

    while (ui_serial_connected()) {
        int c = getchar_timeout_us(50000);
        if (c == PICO_ERROR_TIMEOUT) {
            continue;
        }
        if (c == 0x03 || c == 0x1B) {
            platform_reboot_device();
        }
        if (c == '\r' || c == '\n') {
            break;
        }
    }

    printf("Ending test...\n");
    bool ended = hci_ble_test_end(&packets, 3000);

    test_session_clear();

    printf("\n=== Final BLE RX result ===\n");
    if (!ended) {
        printf("LE Test End failed; no packet count available.\n");
    } else {
        printf("  Channel: %d (%d MHz)\n", channel, frequency_mhz);
        printf("  Packets received: %u\n", (unsigned)packets);
        if (packets > 0) {
            printf("\nRECEIVED %u packets. The transmitter's RF was demodulated\n",
                   (unsigned)packets);
            printf("on this exact channel.\n");
        } else {
            printf("\nNo packets received. Check that the TX board is transmitting\n");
            printf("on this channel.\n");
        }
    }

    printf("\n");
    ui_wait_for_ack("Press Enter to return to the menu...");
}

static void run_ble_tx_flow(void) {
    int channel = 0;
    int frequency_mhz = 0;
    int length = 37;
    int payload = 0;

    ui_clear_screen();
    ui_print_reboot_notice();

    printf("\n--- BLE Transmit Test ---\n");
    printf("This test uses HCI LE Transmitter Test (0x201E).\n\n");

    /* The controller refuses a second LE_Transmitter_Test while one is still
     * running (status 0x0C), so clear any prior test mode first. */
    printf("HCI reset before BLE TX...\n");
    if (!hci_do_reset_wait(3000)) {
        printf("ERROR: HCI reset failed.\n");
        return;
    }
    sleep_ms(100);

    if (!read_ble_channel(&channel, &frequency_mhz)) {
        return;
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
        printf("  1) Transmit test (LE Transmitter Test)\n");
        printf("  2) Receiver test (LE Receiver Test, needs a second board transmitting)\n");
        printf("  3) Stop test / HCI reset\n");
        printf("  0) Back to main menu\n");

        if (!ui_read_choice("Enter choice: ", 0, 3, &choice)) {
            continue;
        }

        switch (choice) {
            case 0:
                return;
            case 1:
                run_ble_tx_flow();
                break;
            case 2:
                run_ble_rx_flow();
                break;
            case 3:
                printf("\nSending HCI reset...\n");
                if (hci_do_reset_wait(3000)) {
                    printf("Controller reset. RF output stopped.\n");
                    test_session_clear();
                } else {
                    printf("ERROR: HCI reset failed.\n");
                }
                ui_wait_for_ack("Press Enter to return to the menu...");
                break;
            default:
                break;
        }
    }
}
