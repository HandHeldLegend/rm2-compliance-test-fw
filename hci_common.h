#ifndef HCI_COMMON_H
#define HCI_COMMON_H

#include <stdbool.h>
#include <stdint.h>

#define OPCODE_READBDADDR 0x1009
#define OPCODE_BTC_TX_TEST 0xFC51

void hci_common_init(void);
void hci_common_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);

bool hci_test_is_idle(void);
bool hci_has_bdaddr(void);
const uint8_t *hci_get_bdaddr(void);

void hci_do_reset(void);
bool hci_do_reset_wait(uint32_t timeout_ms);
void hci_read_bdaddr(void);
bool hci_read_bdaddr_wait(uint32_t timeout_ms);

bool hci_start_btc_tx(uint8_t hopping_mode, uint32_t frequency_mhz, uint8_t modulation_type,
                      uint8_t logical_channel, uint8_t packet_type, uint16_t packet_length,
                      uint8_t transmit_power_dbm);

bool hci_start_ble_tx(uint8_t channel, uint8_t length, uint8_t payload);

#endif
