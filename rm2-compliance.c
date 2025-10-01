#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/btstack_chipset_cyw43.h"
#include "btstack.h"
#include "pico/bootrom.h"

// Buffer sizes
#define MAX_COMMAND_LEN 256
#define MAX_HCI_PACKET_LEN 255
#define UART_BUFFER_SIZE 512

#define OPCODE_READBDADDR 0x1009
#define OPCODE_BTC_TX_TEST 0xFC51

// State variables
static char uart_buffer[UART_BUFFER_SIZE];
static int uart_buffer_pos = 0;
static bool waiting_for_response = false;
static uint16_t pending_opcode = 0;
static bool gotinfo = false;
static uint8_t addr[6] = {0};
static bool ble_test_active = false;
static bool btc_test_active = false;
static char last_command_name[64] = "";
static bool hci_reset_done = false;

// Function prototypes
void process_uart_input(void);
void send_hci_command_bytes(uint8_t *command_buffer, size_t command_len);
void print_hex_buffer(const char* prefix, const uint8_t* buffer, int length);
int hex_string_to_bytes(const char* hex_string, uint8_t* output, int max_length);
void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
void parse_and_execute_command(const char* command);

void cmd_help(void);
void cmd_read_bdaddr(void);

int parse_frequency_to_channel(int freq_mhz);
const char* get_hci_status_string(uint8_t status);
const char* get_opcode_name(uint16_t opcode);
void set_pending_command(uint16_t opcode, const char* command_name);

void uint8_array_to_hex_string(const uint8_t *data, size_t len, char *output, size_t output_size) {
    size_t pos = 0;
    for (size_t i = 0; i < len; i++) {
        // Ensure we don’t overflow output buffer
        if (pos + 3 >= output_size) break; // 2 hex digits + space + null
        pos += snprintf(output + pos, output_size - pos, "%02X", data[i]);
        if (i < len - 1) {
            output[pos++] = ' '; // add space between bytes
            output[pos] = '\0';  // null terminate after each addition
        }
    }
}

bool hci_test_check()
{
    if(ble_test_active) {
        printf("ERROR: \n");
        printf(" Must stop BLE TX with ble_stop first!\n");
        return false;
    }

    if(btc_test_active) {
        printf("ERROR: \n");
        printf(" Must stop BTC TX with ble_stop first!\n");
        return false;
    }

    return true;
}

// HCI Reset
void cmd_hci_reset() 
{
    printf("> HCI RESET:\n");

    uint8_t cmd[] = {0x03, 0x0C, 0x00};

    hci_reset_done = true;
    ble_test_active = false;
    btc_test_active = false;
    send_hci_command_bytes(cmd, 3); 
}

// Stop BTC TX Test
void cmd_stop_btc_tx()
{
    printf("> STOP BTC TX (hci reset):\n");

    btc_test_active = false;
    cmd_hci_reset();
}

// Stop BLE TX Test
void cmd_stop_ble_tx()
{
    printf("> STOP BLE TX (hci reset):\n");

    ble_test_active = false;
    cmd_hci_reset();
}

// Start BTC TX Test
void cmd_start_btc_tx(uint32_t frequency_mhz, uint8_t transmit_power_dbm)
{
    if(!hci_test_check()) return;

    if(!gotinfo) {
        printf("ERROR: \n");
        printf(" Must do loadinfo command to ensure you have the BDADDR ready.\n");
        return;
    }

    if(frequency_mhz < 2402 || frequency_mhz > 2480) {
        printf("ERROR: \n");
        printf(" Frequency must be between 2402 and 2480\n");
        return;
    }

    printf("> START BTC TX:\n");

    uint8_t opcode_lo =  OPCODE_BTC_TX_TEST & 0xFF;
    uint8_t opcode_hi = (OPCODE_BTC_TX_TEST & 0xFF00) >> 8;
    uint8_t param_len = 16;

    uint32_t freq_idx = frequency_mhz - 2402;
    uint8_t hopping_mode = 0x01; // Single frequency
    uint8_t modulation_type = 0x04; // PRBS9 PATTERN
    uint8_t logical_channel = 0x00; // ACL EDR
    uint8_t bb_packet_type = 0x0F; // DH5/3DH5
    uint16_t bb_packet_len = 339; // BB PACKET LENGTH
    uint8_t bb_packet_len_lo = (bb_packet_len & 0xFF);
    uint8_t bb_packet_len_hi = (bb_packet_len & 0xFF00) >> 8;

    uint8_t tx_power_level_sel = 0x08; // Specify power in dBm
    uint8_t tx_power_dbm = transmit_power_dbm;

    uint8_t transmit_power_table_index = 0x00;

    uint8_t cmd[19] = { opcode_lo, opcode_hi, param_len,
        addr[0], addr[1], addr[2], addr[3], addr[4], addr[5],
        (uint8_t) freq_idx, hopping_mode, modulation_type, logical_channel, bb_packet_type,
        bb_packet_len_lo, bb_packet_len_hi, tx_power_level_sel, tx_power_dbm, transmit_power_table_index
        };

    btc_test_active = true;
    send_hci_command_bytes(cmd, 19); 
}

void cmd_reset_bootloader()
{
    reset_usb_boot(0, 0);
}

uint8_t _ble_mhz_to_tx_channel(int freq_mhz) {
    int k = (freq_mhz - 2402) / 2;

    if (k < 0) k = 0;
    if (k > 39) k = 39;  // clamp to valid BLE channel range

    return (uint8_t)k;
}

// Start BLE TX Test
void cmd_start_ble_tx(uint32_t frequency_mhz)
{
    if(!hci_reset_done) {
        printf("ERROR: \n");
        printf(" Must do reset command first!\n");
        return;
    }

    if(ble_test_active) {
        printf("ERROR: \n");
        printf(" BLE test already running!\n");
        return;
    }

    if(btc_test_active) {
        printf("ERROR: \n");
        printf(" BTC test already running!\n");
        return;
    }

    if(frequency_mhz < 2402 || frequency_mhz > 2480) {
        printf("ERROR: \n");
        printf(" Frequency must be between 2402 and 2480\n");
        return;
    }

    printf("> START BLE TX:\n");

    uint8_t ch = _ble_mhz_to_tx_channel((int) frequency_mhz);

    uint8_t cmd[6] = {0x1E, 0x20, 0x03, ch, 0x25, 0x00};
    ble_test_active = true;
    send_hci_command_bytes(cmd, 6);
}

// Read the BDADDR
void cmd_read_bdaddr()
{
    // Reset gotinfo flag
    gotinfo = false;

    uint8_t opcode_lo =  OPCODE_READBDADDR & 0xFF;
    uint8_t opcode_hi = (OPCODE_READBDADDR & 0xFF00) >> 8;
    uint8_t param_len = 0;

    uint8_t cmd[3] = {opcode_lo, opcode_hi, 0x00};
    send_hci_command_bytes(cmd, 3);
}

void handle_read_bdaddr(uint8_t *data)
{
    memcpy(addr, data, 6);
    printf("> GOT BDADDR:\n");
    print_hex_buffer("  BDADDR: ", addr, 6);
    gotinfo = true;
}

// Get human-readable HCI status string
const char* get_hci_status_string(uint8_t status) {
    switch (status) {
        case 0x00: return "SUCCESS";
        case 0x01: return "UNKNOWN_HCI_COMMAND";
        case 0x02: return "UNKNOWN_CONNECTION_IDENTIFIER";
        case 0x03: return "HARDWARE_FAILURE";
        case 0x04: return "PAGE_TIMEOUT";
        case 0x05: return "AUTHENTICATION_FAILURE";
        case 0x06: return "PIN_OR_KEY_MISSING";
        case 0x07: return "MEMORY_CAPACITY_EXCEEDED";
        case 0x08: return "CONNECTION_TIMEOUT";
        case 0x09: return "CONNECTION_LIMIT_EXCEEDED";
        case 0x0A: return "SYNCHRONOUS_CONNECTION_LIMIT_EXCEEDED";
        case 0x0B: return "CONNECTION_ALREADY_EXISTS";
        case 0x0C: return "COMMAND_DISALLOWED";
        case 0x0D: return "CONNECTION_REJECTED_LIMITED_RESOURCES";
        case 0x0E: return "CONNECTION_REJECTED_SECURITY_REASONS";
        case 0x0F: return "CONNECTION_REJECTED_UNACCEPTABLE_BD_ADDR";
        case 0x10: return "CONNECTION_ACCEPT_TIMEOUT_EXCEEDED";
        case 0x11: return "UNSUPPORTED_FEATURE_OR_PARAMETER_VALUE";
        case 0x12: return "INVALID_HCI_COMMAND_PARAMETERS";
        case 0x13: return "REMOTE_USER_TERMINATED_CONNECTION";
        case 0x14: return "REMOTE_DEVICE_TERMINATED_CONNECTION_LOW_RESOURCES";
        case 0x15: return "REMOTE_DEVICE_TERMINATED_CONNECTION_POWER_OFF";
        case 0x16: return "CONNECTION_TERMINATED_BY_LOCAL_HOST";
        case 0x17: return "REPEATED_ATTEMPTS";
        case 0x18: return "PAIRING_NOT_ALLOWED";
        case 0x19: return "UNKNOWN_LMP_PDU";
        case 0x1A: return "UNSUPPORTED_REMOTE_FEATURE";
        case 0x1B: return "SCO_OFFSET_REJECTED";
        case 0x1C: return "SCO_INTERVAL_REJECTED";
        case 0x1D: return "SCO_AIR_MODE_REJECTED";
        case 0x1E: return "INVALID_LMP_PARAMETERS";
        case 0x1F: return "UNSPECIFIED_ERROR";
        case 0x20: return "UNSUPPORTED_LMP_PARAMETER_VALUE";
        case 0x21: return "ROLE_CHANGE_NOT_ALLOWED";
        case 0x22: return "LMP_RESPONSE_TIMEOUT";
        case 0x23: return "LMP_ERROR_TRANSACTION_COLLISION";
        case 0x24: return "LMP_PDU_NOT_ALLOWED";
        case 0x25: return "ENCRYPTION_MODE_NOT_ACCEPTABLE";
        case 0x26: return "LINK_KEY_CANNOT_BE_CHANGED";
        case 0x27: return "REQUESTED_QOS_NOT_SUPPORTED";
        case 0x28: return "INSTANT_PASSED";
        case 0x29: return "PAIRING_WITH_UNIT_KEY_NOT_SUPPORTED";
        case 0x2A: return "DIFFERENT_TRANSACTION_COLLISION";
        case 0x2C: return "QOS_UNACCEPTABLE_PARAMETER";
        case 0x2D: return "QOS_REJECTED";
        case 0x2E: return "CHANNEL_CLASSIFICATION_NOT_SUPPORTED";
        case 0x2F: return "INSUFFICIENT_SECURITY";
        case 0x30: return "PARAMETER_OUT_OF_MANDATORY_RANGE";
        case 0x32: return "ROLE_SWITCH_PENDING";
        case 0x34: return "RESERVED_SLOT_VIOLATION";
        case 0x35: return "ROLE_SWITCH_FAILED";
        case 0x36: return "EXTENDED_INQUIRY_RESPONSE_TOO_LARGE";
        case 0x37: return "SECURE_SIMPLE_PAIRING_NOT_SUPPORTED";
        case 0x38: return "HOST_BUSY_PAIRING";
        case 0x39: return "CONNECTION_REJECTED_NO_SUITABLE_CHANNEL_FOUND";
        case 0x3A: return "CONTROLLER_BUSY";
        case 0x3B: return "UNACCEPTABLE_CONNECTION_PARAMETERS";
        case 0x3C: return "DIRECTED_ADVERTISING_TIMEOUT";
        case 0x3D: return "CONNECTION_TERMINATED_MIC_FAILURE";
        case 0x3E: return "CONNECTION_FAILED_TO_BE_ESTABLISHED";
        case 0x3F: return "MAC_CONNECTION_FAILED";
        case 0x40: return "COARSE_CLOCK_ADJUSTMENT_REJECTED";
        default: return "UNKNOWN_ERROR";
    }
}

// Get human-readable command name from opcode
const char* get_opcode_name(uint16_t opcode) {
    switch (opcode) {
        case 0x1002: return "HCI_Inquiry_Cancel";
        case 0x1004: return "HCI_Exit_Periodic_Inquiry_Mode";
        case 0x1005: return "HCI_Create_Connection";
        case 0x1006: return "HCI_Disconnect";
        case 0x1008: return "HCI_Accept_Connection_Request";
        case 0x1009: return "HCI_Reject_Connection_Request";
        case 0x100A: return "HCI_Link_Key_Request_Reply";
        case 0x100B: return "HCI_Link_Key_Request_Negative_Reply";
        case 0x100C: return "HCI_PIN_Code_Request_Reply";
        case 0x100D: return "HCI_PIN_Code_Request_Negative_Reply";
        case 0x1001: return "HCI_Reset";
        case 0x1803: return "HCI_Enable_Device_Under_Test_Mode";
        case 0x201D: return "HCI_LE_Receiver_Test";
        case 0x201E: return "HCI_LE_Transmitter_Test";
        case 0x201F: return "HCI_LE_Test_End";
        case 0xFC51: return "BTC_TX_Test (Vendor)";
        case 0xFC52: return "BTC_RX_Test (Vendor)";
        case 0xFC53: return "BTC_Test_End (Vendor)";
        default: return "Unknown_Command";
    }
}

// Set pending command with name tracking
void set_pending_command(uint16_t opcode, const char* command_name) {
    pending_opcode = opcode;
    waiting_for_response = true;
    strncpy(last_command_name, command_name, sizeof(last_command_name) - 1);
    last_command_name[sizeof(last_command_name) - 1] = '\0';
}

// Convert hex string to byte array
int hex_string_to_bytes(const char* hex_string, uint8_t* output, int max_length) {
    int len = strlen(hex_string);
    if (len % 2 != 0) return -1; // Must be even number of hex chars
    
    int byte_count = len / 2;
    if (byte_count > max_length) return -1;
    
    for (int i = 0; i < byte_count; i++) {
        char byte_str[3] = {hex_string[i*2], hex_string[i*2+1], '\0'};
        output[i] = (uint8_t)strtol(byte_str, NULL, 16);
    }
    
    return byte_count;
}

// Print buffer as hex
void print_hex_buffer(const char* prefix, const uint8_t* buffer, int length) {
    printf("%s: ", prefix);
    for (int i = 0; i < length; i++) {
        printf("%02X ", buffer[i]);
    }
    printf("\n");
}

// Send HCI command from a byte array
void send_hci_command_bytes(uint8_t *command_buffer, size_t command_len) {
    if (command_len < 3) {
        printf("ERROR: Invalid command format. Need at least opcode (2 bytes) + length (1 byte)\n");
        return;
    }
    
    // Extract opcode and parameter length
    uint16_t opcode = command_buffer[0] | (command_buffer[1] << 8);
    uint8_t param_len = command_buffer[2];
    
    if (command_len != param_len + 3) {
        printf("ERROR: Command length mismatch. Expected %d bytes, got %zu\n", param_len + 3, command_len);
        return;
    }
    
    printf("Sending HCI Command:\n");
    printf("  Opcode: 0x%04X\n", opcode);
    printf("  Param Length: %d\n", param_len);
    print_hex_buffer("  Raw", command_buffer, command_len);
    
    // Track pending command
    set_pending_command(opcode, "Raw HCI Command");
    
    // Send via BTStack
    hci_send_cmd_packet(command_buffer, command_len);
}

// Help command
void cmd_help(void) {
    printf("\n=== HCI Bridge Help ===\n");
    printf("General Commands:\n");
    printf("  help                    - Show this help\n");
    printf("  loadinfo                - Load device info, required to perform tests\n");
    printf("  reset                   - Perform HCI reset\n");
    printf("  bootloader              - Reset to USB bootloader\n");
    printf("\nBluetooth Test Commands:\n");
    printf("  btc_tx <freq>           - Start BTC transmit test\n");
    printf("                            freq: Frequency in MHz\n");
    printf("                            Power level is set to 4dBm always\n");
    printf("  btc_stop                - Stop BTC transmit test\n");
    printf("  ble_tx <freq>           - Start BLE transmit test\n");
    printf("                            freq: Frequency in MHz\n");
    printf("                            MUST REBOOT TO STOP TEST\n");
    //printf("  ble_stop                - Stop BLE transmit test\n");
    printf("\nRaw HCI Commands:\n");
    printf("  <hex_string>            - Send raw HCI command\n");
    printf("  Format: OPCODE_LOW OPCODE_HIGH PARAM_LEN [PARAM_BYTES...]\n");
    printf("  Example: 01 10 00 (HCI_Reset)\n");
    printf("====================\n\n");
}

// Parse and execute command
void parse_and_execute_command(const char* command) {
    if (strncmp(command, "help", 4) == 0) {
        cmd_help();
    } else if (strncmp(command, "btc_tx", 6) == 0) {
        // Skip past "btc_tx" and any following spaces
        const char* params = command + 6;
        while (*params == ' ') params++; // Skip whitespace
 
        int param1;
        int parsed = sscanf(params, "%d %d", &param1);
        
        if (parsed == 1) {
            // Successfully parsed both parameters
            printf("BTC TX command: freq_mhz=%d\n", param1);
            // Call your function with the parameters
            cmd_start_btc_tx((uint32_t) param1, 4);
        } else {
            printf("Error: btc_tx requires frequency parameter\n");
            printf("Usage: btc_tx <freq>\n");
        }
    } else if (strncmp(command, "btc_stop", 8) == 0) {
        cmd_stop_btc_tx();
    } else if (strncmp(command, "reset", 5) == 0) {
        cmd_hci_reset();
    } else if (strncmp(command, "bootloader", 6) == 0) {
        cmd_reset_bootloader();
    } else if (strncmp(command, "ble_tx", 6) == 0) {
        const char* params = command + 6;
        while (*params == ' ') params++; // Skip whitespace
 
        int param1;
        int parsed = sscanf(params, "%d %d", &param1);
        
        if (parsed == 1) {
            // Successfully parsed both parameters
            printf("BLE TX command: freq_mhz=%d\n", param1);
            // Call your function with the parameters
            cmd_start_ble_tx((uint32_t) param1);
        } else {
            printf("Error: btc_tx requires frequency parameter\n");
            printf("Usage: btc_tx <freq>\n");
        }
    } else if (strncmp(command, "ble_stop", 8) == 0) {
        cmd_stop_ble_tx();
    } else if (strncmp(command, "loadinfo", 8) == 0) {
        cmd_read_bdaddr();
    } else if (strlen(command) > 0) {
        // Convert hex string to uint8_t array
        uint8_t command_buffer[MAX_HCI_PACKET_LEN];
        char clean_hex[MAX_COMMAND_LEN];
        int clean_pos = 0;

        // Remove spaces and convert to uppercase
        for (int i = 0; command[i] && clean_pos < MAX_COMMAND_LEN-1; i++) {
            if (command[i] != ' ' && command[i] != '\t') {
                clean_hex[clean_pos++] = toupper(command[i]);
            }
        }
        clean_hex[clean_pos] = '\0';

        int command_len = hex_string_to_bytes(clean_hex, command_buffer, MAX_HCI_PACKET_LEN);
        if (command_len <= 0) {
            printf("ERROR: Failed to parse hex command\n");
            return;
        }

        // Call the new byte-array version
        send_hci_command_bytes(command_buffer, command_len);
    }
}

// Minimal response parser for HCI events
void parse_hci_response(uint8_t *packet, uint16_t size) {
    if (size < 3) return; // Not enough data

    uint8_t event_code = packet[0];
    uint8_t param_len = packet[1];

    if (event_code == HCI_EVENT_COMMAND_COMPLETE && size >= 6) {
        uint8_t num_hci_cmd_packets = packet[2];
        uint16_t opcode = little_endian_read_16(packet, 3);
        uint8_t status = packet[5];
        uint8_t *data = &packet[6];
        uint16_t data_len = param_len - 3; // subtract Num_HCI_Command_Packets + Opcode

        printf("Command Complete:\n");
        printf("  Opcode: 0x%04X\n", opcode);
        printf("  Status: 0x%02X\n", status);
        printf("  Data Length: %d\n", data_len);
        print_hex_buffer("  Data", data, data_len);
        printf("\n");

        switch(opcode)
        {
            case OPCODE_READBDADDR:
            handle_read_bdaddr(data);
            break;
        }
        
    }
    else if (event_code == HCI_EVENT_COMMAND_STATUS && size >= 6) {
        uint8_t status = packet[2];
        uint8_t num_hci_cmd_packets = packet[1]; // not usually needed
        uint16_t opcode = little_endian_read_16(packet, 4);

        printf("Command Status:\n");
        printf("  Opcode: 0x%04X\n", opcode);
        printf("  Status: 0x%02X\n", status);
    }
}

// BTStack packet handler
void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    UNUSED(channel);
    
    switch (packet_type) {
        case HCI_EVENT_PACKET:
            parse_hci_response(packet, size);
            break;
            
        case HCI_ACL_DATA_PACKET:
            printf("\n=== HCI ACL Data ===\n");
            print_hex_buffer("ACL Data", packet, size);
            printf("==================\n\n");
            break;
            
        default:
            printf("\n=== Unknown Packet Type 0x%02X ===\n", packet_type);
            print_hex_buffer("Unknown", packet, size);
            printf("==============================\n\n");
            break;
    }
}

// Process UART input
void process_uart_input(void) {
    int c = getchar_timeout_us(0);
    if (c == PICO_ERROR_TIMEOUT) return;
    
    if (c == '\r' || c == '\n') {
        if (uart_buffer_pos > 0) {
            uart_buffer[uart_buffer_pos] = '\0';
            parse_and_execute_command(uart_buffer);
            uart_buffer_pos = 0;
            printf("\n> ");
        }
    } else if (c == '\b' || c == 127) { // Backspace
        if (uart_buffer_pos > 0) {
            uart_buffer_pos--;
            printf("\b \b");
        }
    } else if (uart_buffer_pos < UART_BUFFER_SIZE - 1 && c >= 32 && c < 127) {
        uart_buffer[uart_buffer_pos++] = c;
        putchar(c);
    }
}

int main() {
    // Initialize stdio
    stdio_init_all();
    
    printf("\n\n=== Raspberry Pi Pico W HCI Bridge ===\n");
    printf("Initializing...\n");
    
    // Initialize CYW43 (WiFi/Bluetooth chip)
    if (cyw43_arch_init()) {
        printf("ERROR: Failed to initialize CYW43\n");
        return -1;
    }
    
    // Initialize BTStack
    hci_set_chipset(btstack_chipset_cyw43_instance());
    
    // Register packet handler
    btstack_packet_callback_registration_t hci_event_callback_registration;
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);
    
    // Power on Bluetooth
    hci_power_control(HCI_POWER_ON);
    
    printf("BTStack initialized. Type 'help' for commands.\n");
    printf("> ");
    
    // Main loop
    while (true) {
        // Process UART input
        process_uart_input();
        
        // Small delay to prevent busy waiting
        sleep_ms(1);
    }
    
    return 0;
}