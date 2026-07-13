#include "hci_common.h"

#include <stdio.h>
#include <string.h>

#include "btstack.h"
#include "pico/stdlib.h"

#define HCI_CMD_TIMEOUT_MS 3000u

static bool got_bdaddr = false;
static uint8_t bdaddr[6];
static bool ble_test_active = false;
static bool btc_test_active = false;

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

static void print_hex_buffer(const char *prefix, const uint8_t *buffer, int length) {
    printf("%s", prefix);
    for (int i = 0; i < length; i++) {
        printf("%02X ", buffer[i]);
    }
    printf("\n");
}

static void send_hci_command_bytes(uint8_t *command_buffer, size_t command_len) {
    if (command_len < 3) {
        printf("ERROR: Invalid HCI command length\n");
        return;
    }
    hci_send_cmd_packet(command_buffer, command_len);
}

static void handle_read_bdaddr(uint8_t *data) {
    memcpy(bdaddr, data, 6);
    printf("Device BD_ADDR: ");
    print_hex_buffer("", bdaddr, 6);
    got_bdaddr = true;
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
        } else if (opcode == hci_le_transmitter_test.opcode && status == 0) {
            ble_test_active = true;
            printf("BLE TX command accepted by controller.\n");
        } else if (opcode == hci_reset.opcode && status == 0) {
            ble_test_active = false;
            btc_test_active = false;
            printf("HCI reset complete.\n");
        } else if (status != 0) {
            printf("HCI command failed with status 0x%02X\n", status);
        }
    } else if (event_code == HCI_EVENT_COMMAND_STATUS && size >= 6) {
        uint16_t opcode = little_endian_read_16(packet, 4);
        uint8_t status = packet[2];
        printf("HCI status: opcode=0x%04X status=0x%02X\n", opcode, status);
        if (status != 0) {
            hci_complete_pending(opcode, status);
        }
    }
}

void hci_common_init(void) {
    got_bdaddr = false;
    ble_test_active = false;
    btc_test_active = false;
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
    return !ble_test_active && !btc_test_active;
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
    hci_send_cmd(&hci_reset);
}

bool hci_do_reset_wait(uint32_t timeout_ms) {
    ble_test_active = false;
    btc_test_active = false;
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
                      uint8_t transmit_power_dbm) {
    if (!got_bdaddr) {
        printf("ERROR: BD_ADDR not loaded. Run device info read first.\n");
        return false;
    }

    if (frequency_mhz < 2402 || frequency_mhz > 2480) {
        printf("ERROR: Frequency must be between 2402 and 2480 MHz.\n");
        return false;
    }

    /* Infineon/Broadcom Tx_Test (0xFC51) param order:
     * BD_ADDR, Hopping_Mode, Frequency, Modulation, Logical_Channel,
     * BB_Packet_Type, BB_Packet_Length(le16), Tx_Power_Level, Power, TableIndex.
     * Hopping_Mode=1 + Frequency=(mhz-2402) is single-frequency lab TX. */
    uint32_t freq_idx = frequency_mhz - 2402;
    uint8_t cmd[19] = {
        (uint8_t)(OPCODE_BTC_TX_TEST & 0xFF),
        (uint8_t)((OPCODE_BTC_TX_TEST >> 8) & 0xFF),
        16,
        bdaddr[0], bdaddr[1], bdaddr[2], bdaddr[3], bdaddr[4], bdaddr[5],
        hopping_mode,
        (uint8_t)freq_idx,
        modulation_type,
        logical_channel,
        packet_type,
        (uint8_t)(packet_length & 0xFF),
        (uint8_t)((packet_length >> 8) & 0xFF),
        0x08, /* specify power in dBm */
        transmit_power_dbm,
        0x00
    };

    btc_test_active = false;
    hci_begin_wait(OPCODE_BTC_TX_TEST);
    send_hci_command_bytes(cmd, sizeof(cmd));
    return hci_wait_complete(HCI_CMD_TIMEOUT_MS);
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
