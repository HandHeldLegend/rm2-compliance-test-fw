# BR/EDR vendor HCI reference (CYW43439)

Source: Infineon, *Bluetooth Radio Tests & Provisioning Commands*, 05 May 2020
(`BT_Radio_Test_and_Provision_05.05.20.pdf`, §2.2.3 Tx_Test, §2.2.4 Rx_Test,
§3.4 Connectionless Transmitter Test, §3.5 Fixed Frequency Receiver Test).

Both commands are `OGF 0x3F`. Parameters are little endian.

## Tx_Test (opcode 0xFC51, 16 parameter octets)

| Off | Size | Field | Values |
|-----|------|-------|--------|
| 0 | 6 | `BD_ADDR` | local address, little endian (wire order) |
| 6 | 1 | `Hopping_Mode` | `0x00` all channels, `0x01` single channel, `0x02` fixed pattern |
| 7 | 1 | `TX_Channel` | `N = MHz - 2402`, `0x00`..`0x4E` (2402..2480 MHz) |
| 8 | 1 | `Modulation_Mode` | `1`=00000000, `2`=11111111, `3`=10101010, `4`=PRBS9, `9`=11110000 |
| 9 | 1 | `Logical_Channel` | `0`=ACL EDR, `1`=ACL Basic, `2`=eSCO EDR, `3`=eSCO Basic, `4`=SCO Basic |
| 10 | 1 | `Baseband_Packet_Type` | see pairing table below |
| 11 | 2 | `Baseband_Packet_Length` | LE; `0xFFFF` = firmware max for the type |
| 13 | 1 | `Transmit_Power` | `0x00`..`0x07` = 0/-4/-8/-12/-16/-20/-24/-28 dBm; `0x08` = use dBm field; `0x09` = use table index |
| 14 | 1 | `Transmit_Power_dBm` | signed, when `Transmit_Power == 0x08` |
| 15 | 1 | `Transmit_Power_Table_Index` | when `Transmit_Power == 0x09` |

**Maximum output power is `Transmit_Power=0x09, dBm=0x00, Table_Index=0x00`.**

Documented example (single channel, 2402 MHz, PRBS9, ACL Basic, DH5, max len, max power):

    [51 FC 10]: 66 55 44 33 22 11 01 00 04 01 0F FF FF 09 00 00

## Rx_Test (opcode 0xFC52, 14 parameter octets)

| Off | Size | Field | Values |
|-----|------|-------|--------|
| 0 | 6 | `BD_ADDR` | **the transmitter's** address, little endian |
| 6 | 2 | `Report_Period` | LE milliseconds, e.g. `0x03E8` = 1000 ms |
| 8 | 1 | `RX_Channel` | `N = MHz - 2402`; `0xF0` when peer uses fixed pattern |
| 9 | 1 | `Modulation_Mode` | must match the transmitter |
| 10 | 1 | `Logical_Channel` | must match the transmitter |
| 11 | 1 | `Baseband_Packet_Type` | must match the transmitter |
| 12 | 2 | `Baseband_Packet_Length` | LE; `0xFFFF` = firmware max |

Documented example (2-DH5 @ 2442 MHz):

    [52 FC 0E]: 66 55 44 33 22 11 E8 03 28 04 00 0E FF FF

### Statistics: vendor event 0xFF, sub-code 0x07

`Connectionless Rx Test Statistics`, emitted every `Report_Period`.
Parameter length `0x21` = 33 octets: 1 sub-code + 8 x uint32 (LE).

| Off | Field |
|-----|-------|
| 0 | `Event_Sub_Code` = `0x07` |
| 1 | `Sync_Timeout_Count` |
| 5 | `HEC_Error_Count` |
| 9 | `Total_Received_Packets` |
| 13 | `Good_Packets` |
| 17 | `CRC_Error_Packets` |
| 21 | `Total_Received_Bits` |
| 25 | `Good_Bits` |
| 29 | `Error_Bits` |

Example (267 packets received, all good):

    [FF 21]: 07 00000000 00000000 0B010000 0B010000 00000000 00000000 00000000 00000000

## Logical_Channel x Baseband_Packet_Type pairs

`Baseband_Packet_Type` alone is ambiguous. `Logical_Channel` selects basic vs
enhanced data rate, so the same type byte means a different packet on air.

| TX packet | `Logical_Channel` | `Baseband_Packet_Type` | Max length |
|-----------|-------------------|------------------------|-----------|
| DH1    | `0x01` ACL Basic | `0x04` | 27 |
| DH3    | `0x01` ACL Basic | `0x0B` | 183 |
| DH5    | `0x01` ACL Basic | `0x0F` | 339 |
| 2-DH1  | `0x00` ACL EDR   | `0x04` | 54 |
| 2-DH3  | `0x00` ACL EDR   | `0x0A` | 367 |
| 2-DH5  | `0x00` ACL EDR   | `0x0E` | 679 |
| 3-DH1  | `0x00` ACL EDR   | `0x08` | 83 |
| 3-DH3  | `0x00` ACL EDR   | `0x0B` | 552 |
| 3-DH5  | `0x00` ACL EDR   | `0x0F` | 1021 |

## Two-board fixed-frequency procedure (§3.5)

1. `HCI_Reset` on both boards.
2. `Rx_Test` on the sink, with the **transmitter's** BD_ADDR.
3. `Tx_Test` on the source. Modulation, logical channel, packet type, packet
   length and channel **must match the Rx_Test parameters exactly**.
4. Sink emits `0xFF`/`0x07` statistics every `Report_Period`.
