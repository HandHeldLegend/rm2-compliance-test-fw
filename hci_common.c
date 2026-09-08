#include "hci_common.h"

#include <stdio.h>
#include <string.h>

#include "btstack.h"
#include "pico/stdlib.h"

#define HCI_CMD_TIMEOUT_MS 3000u

static bool got_bdaddr = false;
static uint8_t bdaddr[6];
static bool ble_test_active = false;
static uint16_t s_ble_rx_packets = 0;
static bool btc_test_active = false;
static bool btc_rx_active = false;

static btc_rx_stats_t s_rx_stats;
static uint32_t s_rx_report_count = 0;

static struct {
    bool waiting;
    uint16_t opcode;
    bool completed;
    uint8_t status;
} s_pending_cmd;

static void hci_begin_wait(uint16_t opcode) {
    s_pending_cmd.waiting = true;
    s_pending_cmd.opcode = opcode;
    s_pending_cmd.completed = false;
    s_pending_cmd.status = 0xFF;
}

static bool hci_wait_complete(uint32_t timeout_ms) {
    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);

    while (!s_pending_cmd.completed) {
        if (absolute_time_diff_us(get_absolute_time(), deadline) <= 0) {
            printf("ERROR: Timed out waiting for HCI response (opcode=0x%04X).\n",
                   s_pending_cmd.opcode);
            s_pending_cmd.waiting = false;
            return false;
        }
        sleep_ms(10);
    }

    s_pending_cmd.waiting = false;
    return s_pending_cmd.status == 0;
}

static void hci_complete_pending(uint16_t opcode, uint8_t status) {
    if (s_pending_cmd.waiting && opcode == s_pending_cmd.opcode) {
        s_pending_cmd.completed = true;
        s_pending_cmd.status = status;
    }
}

const char *hci_status_string(uint8_t status) {
    switch (status) {
        case 0x00: return "SUCCESS";
        case 0x01: return "UNKNOWN_HCI_COMMAND";
        case 0x03: return "HARDWARE_FAILURE";
        case 0x07: return "MEMORY_CAPACITY_EXCEEDED";
        case 0x0C: return "COMMAND_DISALLOWED";
        case 0x11: return "UNSUPPORTED_FEATURE_OR_PARAMETER_VALUE";
        case 0x12: return "INVALID_HCI_COMMAND_PARAMETERS";
        case 0x1F: return "UNSPECIFIED_ERROR";
        case 0x30: return "PARAMETER_OUT_OF_MANDATORY_RANGE";
        default: return "UNKNOWN_ERROR";
    }
}

static void print_hex_buffer(const char *prefix, const uint8_t *buffer, int length) {
    printf("%s", prefix);
    for (int i = 0; i < length; i++) {
        printf("%02X ", buffer[i]);
    }
    printf("\n");
}

static bool send_hci_command_bytes(uint8_t *command_buffer, size_t command_len) {
    if (command_len < 3) {
        printf("ERROR: Invalid HCI command length\n");
        return false;
    }
    uint8_t status = hci_send_cmd_packet(command_buffer, (int)command_len);
    if (status != 0) {
        printf("ERROR: HCI transport rejected command (status 0x%02X)\n", status);
        return false;
    }
    return true;
}

static void handle_read_bdaddr(uint8_t *data) {
    memcpy(bdaddr, data, 6);
    printf("Device BD_ADDR: ");
    print_hex_buffer("", bdaddr, 6);
    got_bdaddr = true;
}

/* Vendor event 0xFF sub-code 0x07: connectionless Rx_Test statistics.
 * Payload is the sub-code followed by eight little-endian uint32 counters
 * (Infineon BT Radio Test, section 2.2.4). */
static void parse_vendor_event(const uint8_t *params, uint16_t param_len) {
    if (param_len < 1) {
        return;
    }

    if (params[0] != BTC_RX_STATS_SUBCODE) {
        printf("Vendor event: sub-code=0x%02X len=%u\n", params[0], param_len);
        return;
    }

    if (param_len < 33) {
        printf("Vendor event 0x07: short payload (%u octets, expected 33)\n", param_len);
        return;
    }

    s_rx_stats.sync_timeout_count = little_endian_read_32(params, 1);
    s_rx_stats.hec_error_count = little_endian_read_32(params, 5);
    s_rx_stats.total_received_packets = little_endian_read_32(params, 9);
    s_rx_stats.good_packets = little_endian_read_32(params, 13);
    s_rx_stats.crc_error_packets = little_endian_read_32(params, 17);
    s_rx_stats.total_received_bits = little_endian_read_32(params, 21);
    s_rx_stats.good_bits = little_endian_read_32(params, 25);
    s_rx_stats.error_bits = little_endian_read_32(params, 29);
    s_rx_report_count++;
}

static void parse_hci_response(uint8_t *packet, uint16_t size) {
    if (size < 3) {
        return;
    }

    uint8_t event_code = packet[0];
    uint8_t param_len = packet[1];

    if (event_code == HCI_EVENT_COMMAND_COMPLETE && size >= 6) {
        uint16_t opcode = little_endian_read_16(packet, 3);
        uint8_t status = packet[5];
        uint8_t *data = &packet[6];
        uint16_t data_len = param_len - 3;

        printf("HCI complete: opcode=0x%04X status=0x%02X data_len=%u\n", opcode, status, data_len);
        hci_complete_pending(opcode, status);

        if (opcode == OPCODE_READBDADDR && status == 0 && data_len >= 6) {
            handle_read_bdaddr(data);
        } else if (opcode == OPCODE_BTC_TX_TEST && status == 0) {
            btc_test_active = true;
            printf("BTC TX command accepted by controller.\n");
        } else if (opcode == OPCODE_BTC_RX_TEST && status == 0) {
            btc_rx_active = true;
            printf("BTC RX command accepted by controller.\n");
        } else if (opcode == hci_le_transmitter_test.opcode && status == 0) {
            ble_test_active = true;
            printf("BLE TX command accepted by controller.\n");
        } else if (opcode == hci_le_receiver_test.opcode && status == 0) {
            ble_test_active = true;
            printf("BLE RX command accepted by controller.\n");
        } else if (opcode == hci_le_test_end.opcode && status == 0 && data_len >= 2) {
            s_ble_rx_packets = little_endian_read_16(data, 0);
            ble_test_active = false;
        } else if (opcode == hci_reset.opcode && status == 0) {
            ble_test_active = false;
            btc_test_active = false;
            btc_rx_active = false;
            printf("HCI reset complete.\n");
        } else if (status != 0) {
            printf("HCI command failed with status 0x%02X\n", status);
        }
    } else if (event_code == HCI_EVENT_COMMAND_STATUS && size >= 6) {
        uint16_t opcode = little_endian_read_16(packet, 4);
        uint8_t status = packet[2];
        printf("HCI status: opcode=0x%04X status=0x%02X (%s)\n", opcode, status,
               hci_status_string(status));
        if (status != 0) {
            hci_complete_pending(opcode, status);
        }
    } else if (event_code == HCI_EVENT_VENDOR_SPECIFIC && size >= 3) {
        parse_vendor_event(&packet[2], param_len);
    }
}

void hci_common_init(void) {
    got_bdaddr = false;
    ble_test_active = false;
    s_ble_rx_packets = 0;
    btc_test_active = false;
    btc_rx_active = false;
    hci_rx_stats_reset();
    memset(bdaddr, 0, sizeof(bdaddr));
    memset(&s_pending_cmd, 0, sizeof(s_pending_cmd));
}

void hci_common_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    (void)channel;

    if (packet_type == HCI_EVENT_PACKET) {
        parse_hci_response(packet, size);
    }
}

bool hci_test_is_idle(void) {
    return !ble_test_active && !btc_test_active && !btc_rx_active;
}

bool hci_rx_stats_available(void) {
    return s_rx_report_count > 0;
}

uint32_t hci_rx_stats_report_count(void) {
    return s_rx_report_count;
}

void hci_rx_stats_get(btc_rx_stats_t *out) {
    if (out != NULL) {
        *out = s_rx_stats;
    }
}

void hci_rx_stats_reset(void) {
    memset(&s_rx_stats, 0, sizeof(s_rx_stats));
    s_rx_report_count = 0;
}

bool hci_has_bdaddr(void) {
    return got_bdaddr;
}

const uint8_t *hci_get_bdaddr(void) {
    return bdaddr;
}

void hci_do_reset(void) {
    ble_test_active = false;
    btc_test_active = false;
    btc_rx_active = false;
    hci_send_cmd(&hci_reset);
}

bool hci_do_reset_wait(uint32_t timeout_ms) {
    ble_test_active = false;
    btc_test_active = false;
    btc_rx_active = false;
    got_bdaddr = false;
    hci_begin_wait(hci_reset.opcode);
    hci_send_cmd(&hci_reset);
    return hci_wait_complete(timeout_ms);
}

void hci_read_bdaddr(void) {
    got_bdaddr = false;

    uint8_t cmd[3] = {
        (uint8_t)(OPCODE_READBDADDR & 0xFF),
        (uint8_t)((OPCODE_READBDADDR >> 8) & 0xFF),
        0x00
    };
    send_hci_command_bytes(cmd, sizeof(cmd));
}

bool hci_read_bdaddr_wait(uint32_t timeout_ms) {
    got_bdaddr = false;
    hci_begin_wait(OPCODE_READBDADDR);
    hci_read_bdaddr();
    if (!hci_wait_complete(timeout_ms)) {
        return false;
    }
    return got_bdaddr;
}

bool hci_start_btc_tx(uint8_t hopping_mode, uint32_t frequency_mhz, uint8_t modulation_type,
                      uint8_t logical_channel, uint8_t packet_type, uint16_t packet_length,
                      uint8_t transmit_power_select, int8_t transmit_power_dbm,
                      uint8_t transmit_power_table_index) {
    if (!got_bdaddr) {
        printf("ERROR: BD_ADDR not loaded. Run device info read first.\n");
        return false;
    }

    if (frequency_mhz < 2402 || frequency_mhz > 2480) {
        printf("ERROR: Frequency must be between 2402 and 2480 MHz.\n");
        return false;
    }

    /* Tx_Test (0xFC51), Infineon BT Radio Test section 2.2.3:
     * BD_ADDR[6], Hopping_Mode, TX_Channel, Modulation_Mode, Logical_Channel,
     * Baseband_Packet_Type, Baseband_Packet_Length(le16), Transmit_Power,
     * Transmit_Power_dBm, Transmit_Power_Table_Index. */
    uint8_t channel = (uint8_t)(frequency_mhz - 2402);
    uint8_t cmd[19] = {
        (uint8_t)(OPCODE_BTC_TX_TEST & 0xFF),
        (uint8_t)((OPCODE_BTC_TX_TEST >> 8) & 0xFF),
        16,
        bdaddr[0], bdaddr[1], bdaddr[2], bdaddr[3], bdaddr[4], bdaddr[5],
        hopping_mode,
        channel,
        modulation_type,
        logical_channel,
        packet_type,
        (uint8_t)(packet_length & 0xFF),
        (uint8_t)((packet_length >> 8) & 0xFF),
        transmit_power_select,
        (uint8_t)transmit_power_dbm,
        transmit_power_table_index
    };

    print_hex_buffer("Tx_Test bytes: ", cmd, sizeof(cmd));

    btc_test_active = false;
    hci_begin_wait(OPCODE_BTC_TX_TEST);
    if (!send_hci_command_bytes(cmd, sizeof(cmd))) {
        return false;
    }
    return hci_wait_complete(HCI_CMD_TIMEOUT_MS);
}

bool hci_start_btc_rx(const uint8_t tx_bdaddr[6], uint16_t report_period_ms, uint8_t rx_channel,
                      uint8_t modulation_type, uint8_t logical_channel, uint8_t packet_type,
                      uint16_t packet_length) {
    if (tx_bdaddr == NULL) {
        printf("ERROR: Transmitter BD_ADDR required for Rx_Test.\n");
        return false;
    }

    /* Rx_Test (0xFC52), Infineon BT Radio Test section 2.2.4:
     * BD_ADDR[6] of the transmitter, Report_Period(le16), RX_Channel,
     * Modulation_Mode, Logical_Channel, Baseband_Packet_Type,
     * Baseband_Packet_Length(le16). */
    uint8_t cmd[17] = {
        (uint8_t)(OPCODE_BTC_RX_TEST & 0xFF),
        (uint8_t)((OPCODE_BTC_RX_TEST >> 8) & 0xFF),
        14,
        tx_bdaddr[0], tx_bdaddr[1], tx_bdaddr[2], tx_bdaddr[3], tx_bdaddr[4], tx_bdaddr[5],
        (uint8_t)(report_period_ms & 0xFF),
        (uint8_t)((report_period_ms >> 8) & 0xFF),
        rx_channel,
        modulation_type,
        logical_channel,
        packet_type,
        (uint8_t)(packet_length & 0xFF),
        (uint8_t)((packet_length >> 8) & 0xFF)
    };

    print_hex_buffer("Rx_Test bytes: ", cmd, sizeof(cmd));

    btc_rx_active = false;
    hci_rx_stats_reset();
    hci_begin_wait(OPCODE_BTC_RX_TEST);
    if (!send_hci_command_bytes(cmd, sizeof(cmd))) {
        return false;
    }
    return hci_wait_complete(HCI_CMD_TIMEOUT_MS);
}

bool hci_start_ble_rx(uint8_t channel) {
    if (channel > 0x27) {
        printf("ERROR: BLE channel must be 0-39.\n");
        return false;
    }

    ble_test_active = false;
    s_ble_rx_packets = 0;
    hci_begin_wait(hci_le_receiver_test.opcode);
    hci_send_cmd(&hci_le_receiver_test, channel);
    return hci_wait_complete(HCI_CMD_TIMEOUT_MS);
}

bool hci_ble_test_end(uint16_t *packets_out, uint32_t timeout_ms) {
    s_ble_rx_packets = 0;
    hci_begin_wait(hci_le_test_end.opcode);
    hci_send_cmd(&hci_le_test_end);
    if (!hci_wait_complete(timeout_ms)) {
        return false;
    }
    if (packets_out != NULL) {
        *packets_out = s_ble_rx_packets;
    }
    return true;
}

bool hci_start_ble_tx(uint8_t channel, uint8_t length, uint8_t payload) {
    if (channel > 0x27) {
        printf("ERROR: BLE channel must be 0-39.\n");
        return false;
    }
    if (length > 0x25) {
        printf("ERROR: BLE length must be 0-37.\n");
        return false;
    }
    if (payload > 0x07) {
        printf("ERROR: BLE payload type must be 0-7.\n");
        return false;
    }

    ble_test_active = false;
    hci_begin_wait(hci_le_transmitter_test.opcode);
    hci_send_cmd(&hci_le_transmitter_test, channel, length, payload);
    return hci_wait_complete(HCI_CMD_TIMEOUT_MS);
}
