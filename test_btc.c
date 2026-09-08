#include "test_btc.h"

#include <stdio.h>
#include <string.h>

#include "hci_common.h"
#include "platform.h"
#include "pico/stdlib.h"
#include "test_ui.h"
#include "test_session.h"

/* Logical_Channel and Baseband_Packet_Type are not independent: the same
 * packet-type byte means a different packet on air depending on the logical
 * channel. Offer the documented pairs directly so they cannot be mismatched
 * (Infineon BT Radio Test, sections 3.4 and 3.5). */
typedef struct {
    const char *name;
    uint8_t logical_channel;
    uint8_t packet_type;
    uint16_t max_length;
} btc_packet_preset_t;

static const btc_packet_preset_t k_packet_presets[] = {
    { "DH1",   0x01, 0x04, 27 },
    { "DH3",   0x01, 0x0B, 183 },
    { "DH5",   0x01, 0x0F, 339 },
    { "2-DH1", 0x00, 0x04, 54 },
    { "2-DH3", 0x00, 0x0A, 367 },
    { "2-DH5", 0x00, 0x0E, 679 },
    { "3-DH1", 0x00, 0x08, 83 },
    { "3-DH3", 0x00, 0x0B, 552 },
    { "3-DH5", 0x00, 0x0F, 1021 },
};

#define PACKET_PRESET_COUNT ((int)(sizeof(k_packet_presets) / sizeof(k_packet_presets[0])))

static void print_modulation_help(void) {
    printf("  Modulation types:\n");
    printf("    1 = repeated 00000000\n");
    printf("    2 = repeated 11111111\n");
    printf("    3 = repeated 10101010\n");
    printf("    4 = PRBS9 (standard for regulatory TX)\n");
    printf("    9 = repeated 11110000\n");
}

static bool read_modulation(int *out) {
    print_modulation_help();
    while (true) {
        if (!ui_read_int_in_range("Enter modulation type: ", 1, 9, out)) {
            return false;
        }
        if (*out == 1 || *out == 2 || *out == 3 || *out == 4 || *out == 9) {
            return true;
        }
        printf("Invalid modulation type. Choose 1, 2, 3, 4 or 9.\n");
    }
}

/* Returns the chosen preset, or NULL for a custom logical channel / packet
 * type pair (written through custom_lc / custom_pt). */
static const btc_packet_preset_t *read_packet_preset(uint8_t *custom_lc, uint8_t *custom_pt,
                                                     uint16_t *max_length, bool *ok) {
    int choice = 0;

    *ok = false;

    printf("  TX packet type:\n");
    for (int i = 0; i < PACKET_PRESET_COUNT; i++) {
        printf("    %d) %-6s (logical channel 0x%02X, packet type 0x%02X, max %u bytes)\n",
               i + 1, k_packet_presets[i].name, k_packet_presets[i].logical_channel,
               k_packet_presets[i].packet_type, (unsigned)k_packet_presets[i].max_length);
    }
    printf("   10) Custom (enter logical channel and packet type directly)\n");

    if (!ui_read_choice("Enter choice: ", 1, 10, &choice)) {
        return NULL;
    }

    if (choice <= PACKET_PRESET_COUNT) {
        const btc_packet_preset_t *preset = &k_packet_presets[choice - 1];
        *max_length = preset->max_length;
        *ok = true;
        return preset;
    }

    int lc = 0;
    int pt = 0;
    printf("  Logical channels:\n");
    printf("    0 = ACL EDR\n");
    printf("    1 = ACL Basic\n");
    printf("    2 = eSCO EDR\n");
    printf("    3 = eSCO Basic\n");
    printf("    4 = SCO Basic\n");
    if (!ui_read_int_in_range("Enter logical channel: ", 0, 4, &lc)) {
        return NULL;
    }
    if (!ui_read_int_in_range("Enter baseband packet type (0-15): ", 0, 15, &pt)) {
        return NULL;
    }

    *custom_lc = (uint8_t)lc;
    *custom_pt = (uint8_t)pt;
    *max_length = 1021;
    *ok = true;
    return NULL;
}

static bool read_packet_length(uint16_t preset_max, uint16_t *out) {
    int use_max = 0;

    printf("Packet length controls duty cycle. 0xFFFF lets the controller use\n");
    printf("the maximum for the selected packet type (%u bytes).\n", (unsigned)preset_max);
    printf("  1) Use controller maximum (0xFFFF) - recommended for regulatory TX\n");
    printf("  2) Enter an explicit length\n");

    if (!ui_read_choice("Enter choice: ", 1, 2, &use_max)) {
        return false;
    }

    if (use_max == 1) {
        *out = BTC_PACKET_LENGTH_MAX;
        return true;
    }

    int length = 0;
    if (!ui_read_int_in_range("Enter packet length in bytes: ", 0, 1021, &length)) {
        return false;
    }
    if (length > preset_max) {
        printf("NOTE: %d exceeds the %u-byte maximum for this packet type;\n", length,
               (unsigned)preset_max);
        printf("      the controller will clamp it.\n");
    }
    *out = (uint16_t)length;
    return true;
}

static bool read_tx_power(uint8_t *select, int8_t *dbm, uint8_t *table_index) {
    int mode = 0;

    printf("TX power selection:\n");
    printf("  1) Maximum output power (0x09/0x00/0x00) - required for regulatory limits\n");
    printf("  2) Specify power in dBm\n");
    printf("  3) Specify power table index\n");
    printf("  4) Fixed step (0 = 0 dBm, 1 = -4 dBm ... 7 = -28 dBm)\n");

    if (!ui_read_choice("Enter choice: ", 1, 4, &mode)) {
        return false;
    }

    *select = BTC_TX_POWER_SELECT_TABLE_INDEX;
    *dbm = 0;
    *table_index = 0;

    if (mode == 1) {
        return true;
    }

    if (mode == 2) {
        int value = 0;
        if (!ui_read_int_in_range("Enter TX power in dBm (-127 to 127): ", -127, 127, &value)) {
            return false;
        }
        *select = BTC_TX_POWER_SELECT_DBM;
        *dbm = (int8_t)value;
        return true;
    }

    if (mode == 3) {
        int value = 0;
        if (!ui_read_int_in_range("Enter power table index (0-255): ", 0, 255, &value)) {
            return false;
        }
        *select = BTC_TX_POWER_SELECT_TABLE_INDEX;
        *table_index = (uint8_t)value;
        return true;
    }

    int step = 0;
    if (!ui_read_int_in_range("Enter power step (0-7): ", 0, 7, &step)) {
        return false;
    }
    *select = (uint8_t)step;
    return true;
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

/* Both test roles begin from a clean controller state. */
static bool reset_and_load_bdaddr(void) {
    printf("HCI reset...\n");
    if (!hci_do_reset_wait(3000)) {
        printf("ERROR: HCI reset failed.\n");
        return false;
    }
    sleep_ms(100);
    if (!hci_read_bdaddr_wait(5000)) {
        printf("ERROR: Timed out reading BD_ADDR after reset.\n");
        return false;
    }
    return true;
}

static void print_bdaddr(const char *prefix, const uint8_t *addr) {
    printf("%s%02X %02X %02X %02X %02X %02X\n", prefix, addr[0], addr[1], addr[2], addr[3],
           addr[4], addr[5]);
}

static int hex_nibble(char c) {
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    return -1;
}

/* Accepts the exact form the TX board prints, with or without separators. */
static bool parse_bdaddr(const char *text, uint8_t out[6]) {
    int nibbles[12];
    int count = 0;

    for (const char *p = text; *p != '\0'; p++) {
        if (*p == ' ' || *p == ':' || *p == '-') {
            continue;
        }
        int n = hex_nibble(*p);
        if (n < 0) {
            return false;
        }
        if (count >= 12) {
            return false;
        }
        nibbles[count++] = n;
    }

    if (count != 12) {
        return false;
    }

    for (int i = 0; i < 6; i++) {
        out[i] = (uint8_t)((nibbles[i * 2] << 4) | nibbles[i * 2 + 1]);
    }
    return true;
}

static void run_btc_tx_flow(void) {
    int hopping_mode = 0;
    int frequency_mhz = 0;
    int modulation_type = 0;
    uint16_t packet_length = 0;
    uint8_t logical_channel = 0;
    uint8_t packet_type = 0;
    uint16_t preset_max = 1021;
    uint8_t power_select = 0;
    int8_t power_dbm = 0;
    uint8_t power_table_index = 0;
    bool preset_ok = false;

    ui_clear_screen();
    ui_print_reboot_notice();

    printf("\n--- BT Classic Transmit Test (Tx_Test 0xFC51) ---\n\n");

    if (!ensure_bdaddr()) {
        return;
    }

    /* Leave any prior LE/vendor test mode before Classic RF test. */
    if (!reset_and_load_bdaddr()) {
        return;
    }

    print_bdaddr("Local BD_ADDR (give this to the RX board): ", hci_get_bdaddr());
    printf("\n");

    printf("Select hopping mode:\n");
    printf("  0) All channels (79-channel hopping)\n");
    printf("  1) Single channel (fixed frequency)\n");
    if (!ui_read_choice("Enter choice: ", 0, 1, &hopping_mode)) {
        return;
    }

    if (!ui_read_int_in_range("Enter frequency in MHz (2402-2480): ", 2402, 2480, &frequency_mhz)) {
        return;
    }
    if (hopping_mode == 0) {
        printf("NOTE: In all-channel mode the controller hops and ignores the frequency.\n");
    }

    if (!read_modulation(&modulation_type)) {
        return;
    }

    const btc_packet_preset_t *preset =
        read_packet_preset(&logical_channel, &packet_type, &preset_max, &preset_ok);
    if (!preset_ok) {
        return;
    }
    if (preset != NULL) {
        logical_channel = preset->logical_channel;
        packet_type = preset->packet_type;
    }

    if (!read_packet_length(preset_max, &packet_length)) {
        return;
    }

    if (!read_tx_power(&power_select, &power_dbm, &power_table_index)) {
        return;
    }

    printf("\nStarting BTC TX test with:\n");
    printf("  Hopping mode: %d (%s)\n", hopping_mode,
           hopping_mode == 1 ? "single channel" : "all channels");
    printf("  Frequency: %d MHz (TX_Channel %d)\n", frequency_mhz, frequency_mhz - 2402);
    printf("  Modulation: %d\n", modulation_type);
    printf("  TX packet: %s\n", preset != NULL ? preset->name : "custom");
    printf("  Logical channel: 0x%02X\n", logical_channel);
    printf("  Packet type: 0x%02X\n", packet_type);
    if (packet_length == BTC_PACKET_LENGTH_MAX) {
        printf("  Packet length: 0xFFFF (controller maximum)\n");
    } else {
        printf("  Packet length: %u\n", (unsigned)packet_length);
    }
    printf("  Power select: 0x%02X", power_select);
    if (power_select == BTC_TX_POWER_SELECT_DBM) {
        printf(" (%d dBm)", power_dbm);
    } else if (power_select == BTC_TX_POWER_SELECT_TABLE_INDEX) {
        printf(" (table index %u%s)", (unsigned)power_table_index,
               power_table_index == 0 ? " = maximum output" : "");
    }
    printf("\n\n");

    printf("Sending Tx_Test command...\n");

    bool started = hci_start_btc_tx((uint8_t)hopping_mode, (uint32_t)frequency_mhz,
                                    (uint8_t)modulation_type, logical_channel, packet_type,
                                    packet_length, power_select, power_dbm, power_table_index);

    char details[96];
    snprintf(details, sizeof(details), "%s @ %d MHz, %s, power sel 0x%02X",
             preset != NULL ? preset->name : "custom", frequency_mhz,
             hopping_mode == 1 ? "single channel" : "hopping", power_select);
    ui_print_test_result(started, "BT Classic TX Test", details);
}

static void print_rx_stats(const btc_rx_stats_t *stats, uint32_t reports) {
    printf("--- Rx_Test statistics (report %lu) ---\n", (unsigned long)reports);
    printf("  Sync timeouts:    %lu\n", (unsigned long)stats->sync_timeout_count);
    printf("  HEC errors:       %lu\n", (unsigned long)stats->hec_error_count);
    printf("  Total packets:    %lu\n", (unsigned long)stats->total_received_packets);
    printf("  Good packets:     %lu\n", (unsigned long)stats->good_packets);
    printf("  CRC error packets:%lu\n", (unsigned long)stats->crc_error_packets);
    printf("  Total bits:       %lu\n", (unsigned long)stats->total_received_bits);
    printf("  Good bits:        %lu\n", (unsigned long)stats->good_bits);
    printf("  Error bits:       %lu\n", (unsigned long)stats->error_bits);
    fflush(stdout);
}

static void run_btc_rx_flow(void) {
    char line[64];
    uint8_t tx_bdaddr[6];
    int frequency_mhz = 0;
    int modulation_type = 0;
    int report_period = 1000;
    uint16_t packet_length = 0;
    uint8_t logical_channel = 0;
    uint8_t packet_type = 0;
    uint16_t preset_max = 1021;
    bool preset_ok = false;

    ui_clear_screen();
    ui_print_reboot_notice();

    printf("\n--- BT Classic Receiver Test (Rx_Test 0xFC52) ---\n");
    printf("This board becomes the test sink. Every parameter below must match\n");
    printf("the transmitting board exactly, or it will not synchronise.\n\n");

    if (!reset_and_load_bdaddr()) {
        return;
    }

    print_bdaddr("This board's BD_ADDR: ", hci_get_bdaddr());
    printf("\n");

    printf("Enter the TRANSMITTER's BD_ADDR, exactly as its menu printed it\n");
    printf("(12 hex digits; spaces, colons or dashes are ignored).\n");
    while (true) {
        ui_prompt("TX BD_ADDR: ");
        if (!ui_read_line(line, sizeof(line))) {
            printf("Invalid input. Please try again.\n");
            continue;
        }
        if (parse_bdaddr(line, tx_bdaddr)) {
            break;
        }
        printf("Could not parse that as 6 hex octets. Try again.\n");
    }
    print_bdaddr("Using TX BD_ADDR: ", tx_bdaddr);

    if (!ui_read_int_in_range("Enter frequency in MHz (2402-2480): ", 2402, 2480, &frequency_mhz)) {
        return;
    }

    if (!read_modulation(&modulation_type)) {
        return;
    }

    const btc_packet_preset_t *preset =
        read_packet_preset(&logical_channel, &packet_type, &preset_max, &preset_ok);
    if (!preset_ok) {
        return;
    }
    if (preset != NULL) {
        logical_channel = preset->logical_channel;
        packet_type = preset->packet_type;
    }

    if (!read_packet_length(preset_max, &packet_length)) {
        return;
    }

    if (!ui_read_int_in_range("Enter report period in ms (100-10000): ", 100, 10000,
                              &report_period)) {
        return;
    }

    printf("\nStarting Rx_Test with:\n");
    printf("  Frequency: %d MHz (RX_Channel %d)\n", frequency_mhz, frequency_mhz - 2402);
    printf("  Modulation: %d\n", modulation_type);
    printf("  TX packet: %s\n", preset != NULL ? preset->name : "custom");
    printf("  Logical channel: 0x%02X\n", logical_channel);
    printf("  Packet type: 0x%02X\n", packet_type);
    if (packet_length == BTC_PACKET_LENGTH_MAX) {
        printf("  Packet length: 0xFFFF (controller maximum)\n");
    } else {
        printf("  Packet length: %u\n", (unsigned)packet_length);
    }
    printf("  Report period: %d ms\n\n", report_period);

    printf("Sending Rx_Test command...\n");

    bool started = hci_start_btc_rx(tx_bdaddr, (uint16_t)report_period,
                                    (uint8_t)(frequency_mhz - 2402), (uint8_t)modulation_type,
                                    logical_channel, packet_type, packet_length);

    if (!started) {
        char details[96];
        snprintf(details, sizeof(details), "%s @ %d MHz", preset != NULL ? preset->name : "custom",
                 frequency_mhz);
        ui_print_test_result(false, "BT Classic RX Test", details);
        return;
    }

    printf("\nReceiving. Start the TX board now.\n");
    printf("Statistics print once per report period. Press Enter to stop.\n\n");

    test_session_set_running("BT Classic RX Test", "Rx_Test running");

    uint32_t last_report = 0;
    while (ui_serial_connected()) {
        int c = getchar_timeout_us(50000);
        if (c != PICO_ERROR_TIMEOUT) {
            if (c == 0x03 || c == 0x1B) {
                platform_reboot_device();
            }
            if (c == '\r' || c == '\n') {
                break;
            }
        }

        uint32_t reports = hci_rx_stats_report_count();
        if (reports != last_report) {
            btc_rx_stats_t stats;
            hci_rx_stats_get(&stats);
            print_rx_stats(&stats, reports);
            last_report = reports;
        }
    }

    printf("\nStopping receiver (HCI reset)...\n");
    hci_do_reset_wait(3000);

    btc_rx_stats_t final_stats;
    hci_rx_stats_get(&final_stats);
    uint32_t reports = hci_rx_stats_report_count();

    printf("\n=== Final Rx_Test result ===\n");
    if (reports == 0) {
        printf("No statistics events were received.\n");
        printf("The controller accepted Rx_Test but never reported.\n");
    } else {
        print_rx_stats(&final_stats, reports);
        if (final_stats.total_received_packets > 0) {
            printf("\nRECEIVED %lu packets - the transmitter's RF was demodulated\n",
                   (unsigned long)final_stats.total_received_packets);
            printf("on this exact channel.\n");
        } else {
            printf("\nNo packets received. Check that every TX parameter matches,\n");
            printf("and that the TX board is transmitting on this channel.\n");
        }
    }

    test_session_clear();
    printf("\n");
    ui_wait_for_ack("Press Enter to return to the menu...");
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
        printf("  1) Transmit test (Tx_Test)\n");
        printf("  2) Receiver test (Rx_Test, needs a second board transmitting)\n");
        printf("  3) Stop test / HCI reset\n");
        printf("  0) Back to main menu\n");

        if (!ui_read_choice("Enter choice: ", 0, 3, &choice)) {
            continue;
        }

        switch (choice) {
            case 0:
                return;
            case 1:
                run_btc_tx_flow();
                break;
            case 2:
                run_btc_rx_flow();
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
