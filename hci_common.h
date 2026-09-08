#ifndef HCI_COMMON_H
#define HCI_COMMON_H

#include <stdbool.h>
#include <stdint.h>

#define OPCODE_READBDADDR 0x1009
#define OPCODE_BTC_TX_TEST 0xFC51
#define OPCODE_BTC_RX_TEST 0xFC52

/* Tx_Test Transmit_Power selector (Infineon BT Radio Test §2.2.3). */
#define BTC_TX_POWER_SELECT_DBM 0x08
#define BTC_TX_POWER_SELECT_TABLE_INDEX 0x09

/* Firmware picks the maximum length for the chosen packet type. */
#define BTC_PACKET_LENGTH_MAX 0xFFFF

/* Vendor event 0xFF sub-code carrying connectionless Rx_Test statistics. */
#define BTC_RX_STATS_SUBCODE 0x07

typedef struct {
    uint32_t sync_timeout_count;
    uint32_t hec_error_count;
    uint32_t total_received_packets;
    uint32_t good_packets;
    uint32_t crc_error_packets;
    uint32_t total_received_bits;
    uint32_t good_bits;
    uint32_t error_bits;
} btc_rx_stats_t;

void hci_common_init(void);
void hci_common_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);

bool hci_test_is_idle(void);
bool hci_has_bdaddr(void);
const uint8_t *hci_get_bdaddr(void);

const char *hci_status_string(uint8_t status);

void hci_do_reset(void);
bool hci_do_reset_wait(uint32_t timeout_ms);
void hci_read_bdaddr(void);
bool hci_read_bdaddr_wait(uint32_t timeout_ms);

/* Tx_Test (0xFC51). transmit_power_select is BTC_TX_POWER_SELECT_* or a
 * 0x00..0x07 fixed step; power_dbm applies to 0x08, table_index to 0x09. */
bool hci_start_btc_tx(uint8_t hopping_mode, uint32_t frequency_mhz, uint8_t modulation_type,
                      uint8_t logical_channel, uint8_t packet_type, uint16_t packet_length,
                      uint8_t transmit_power_select, int8_t transmit_power_dbm,
                      uint8_t transmit_power_table_index);

/* Rx_Test (0xFC52). tx_bdaddr is the transmitter's address in wire order. */
bool hci_start_btc_rx(const uint8_t tx_bdaddr[6], uint16_t report_period_ms, uint8_t rx_channel,
                      uint8_t modulation_type, uint8_t logical_channel, uint8_t packet_type,
                      uint16_t packet_length);

/* Connectionless Rx test statistics, refreshed by vendor event 0xFF/0x07. */
bool hci_rx_stats_available(void);
uint32_t hci_rx_stats_report_count(void);
void hci_rx_stats_get(btc_rx_stats_t *out);
void hci_rx_stats_reset(void);

bool hci_start_ble_tx(uint8_t channel, uint8_t length, uint8_t payload);

/* LE Receiver Test (0x201D). The controller counts packets internally and
 * reports the total only when the test ends. */
bool hci_start_ble_rx(uint8_t channel);
bool hci_ble_test_end(uint16_t *packets_out, uint32_t timeout_ms);

#endif
