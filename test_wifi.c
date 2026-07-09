#include "test_wifi.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "btstack.h"
#include "cyw43.h"
#include "cyw43_config.h"
#include "cyw43_country.h"
#include "cyw43_ll.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "test_ui.h"
#include "test_session.h"

#define WWD_STA_INTERFACE 0
#define WLC_UP 2
#define WL_TXPWR_OVERRIDE 0x80

#define WL_CHANSPEC_BW_20 0x1000u
#define WL_CHANSPEC_BW_40 0x1800u
#define WL_CHANSPEC_CTL_SB_NONE 0x0000u
#define WL_CHANSPEC_CTL_SB_LOWER 0x0000u
#define WL_CHANSPEC_BAND_2G 0x0000u

#define WL_PKTENG_PER_TX 0x0001u
#define WL_PKTENG_ASYNC 0x0010u

#define WIFI_AP_SSID_DEFAULT "RM2_COMPLIANCE_TEST"

typedef enum {
    WIFI_MODE_11B,
    WIFI_MODE_11G,
    WIFI_MODE_11N20,
    WIFI_MODE_11N40
} wifi_mode_t;

typedef struct {
    uint32_t flags;
    uint32_t delay;
    uint32_t length;
    uint32_t nframes;
    uint8_t dest[6];
    uint8_t src[6];
} wl_pkteng_t;

static void put_le32(uint8_t *buf, uint32_t val) {
    buf[0] = (uint8_t)(val & 0xFF);
    buf[1] = (uint8_t)((val >> 8) & 0xFF);
    buf[2] = (uint8_t)((val >> 16) & 0xFF);
    buf[3] = (uint8_t)((val >> 24) & 0xFF);
}


static int wifi_ioctl(uint32_t cmd, size_t len, uint8_t *buf) {
    CYW43_THREAD_ENTER;
    int ret = cyw43_ll_ioctl(&cyw43_state.cyw43_ll, cmd, len, buf, WWD_STA_INTERFACE);
    CYW43_THREAD_EXIT;
    return ret;
}

static int wifi_set_iovar_u32(const char *var, uint32_t val) {
    uint8_t buf[128];
    size_t varlen = strlen(var) + 1;
    if (varlen + 4 > sizeof(buf)) {
        return -1;
    }
    memcpy(buf, var, varlen);
    put_le32(buf + varlen, val);
    return wifi_ioctl(CYW43_IOCTL_SET_VAR, varlen + 4, buf);
}

static int wifi_set_iovar_data(const char *var, const void *data, size_t datalen) {
    uint8_t buf[256];
    size_t varlen = strlen(var) + 1;
    if (varlen + datalen > sizeof(buf)) {
        return -1;
    }
    memcpy(buf, var, varlen);
    memcpy(buf + varlen, data, datalen);
    return wifi_ioctl(CYW43_IOCTL_SET_VAR, varlen + datalen, buf);
}

static int wifi_default_q_value(wifi_mode_t mode, int channel) {
    switch (mode) {
        case WIFI_MODE_11B:
            return 67;
        case WIFI_MODE_11G:
            return (channel >= 11) ? 62 : 61;
        case WIFI_MODE_11N20:
            if (channel == 11) {
                return 62;
            }
            if (channel >= 12) {
                return 63;
            }
            return 64;
        case WIFI_MODE_11N40:
            if (channel == 11) {
                return 62;
            }
            if (channel >= 12) {
                return 63;
            }
            return 64;
        default:
            return 70;
    }
}

static const char *wifi_mode_name(wifi_mode_t mode) {
    switch (mode) {
        case WIFI_MODE_11B: return "802.11b";
        case WIFI_MODE_11G: return "802.11g";
        case WIFI_MODE_11N20: return "802.11n HT20";
        case WIFI_MODE_11N40: return "802.11n HT40";
        default: return "unknown";
    }
}

static void wifi_format_q_value(int q_value, char *buf, size_t buflen) {
    int whole_dbm = q_value / 4;
    int quarter = q_value % 4;
    snprintf(buf, buflen, "%d.%02d dBm (Q=%d)", whole_dbm, quarter * 25, q_value);
}

static bool wifi_read_q_value_from_dbm(int *q_value_out) {
    char line[32];
    char power_text[48];

    ui_prompt("Enter TX power in dBm (0.00-31.75, steps of 0.25): ");
    if (!ui_read_line(line, sizeof(line))) {
        return false;
    }

    char *end = NULL;
    double dbm = strtod(line, &end);
    while (end && *end && (*end == ' ' || *end == '\t')) {
        end++;
    }
    if (end == NULL || *end != '\0') {
        printf("Invalid power. Enter a number like 16 or 16.75\n");
        return false;
    }

    if (dbm < 0.0 || dbm > 31.75) {
        printf("Power must be between 0.00 and 31.75 dBm.\n");
        return false;
    }

    int q_value = (int)(dbm * 4.0 + 0.5);
    if (q_value < 0 || q_value > 127) {
        printf("Power must map to a Q-value between 0 and 127.\n");
        return false;
    }

    double q_exact = dbm * 4.0;
    if (q_exact < (double)q_value - 0.01 || q_exact > (double)q_value + 0.01) {
        printf("Use quarter-dBm steps only (e.g. 16.00, 16.25, 16.50, 16.75).\n");
        return false;
    }

    *q_value_out = q_value;
    wifi_format_q_value(q_value, power_text, sizeof(power_text));
    printf("  -> %s\n", power_text);
    return true;
}

static bool wifi_pick_tx_power(wifi_mode_t mode, int channel, int *q_value_out) {
    int choice = 0;
    int q_value = 0;
    char power_text[48];

    q_value = wifi_default_q_value(mode, channel);
    wifi_format_q_value(q_value, power_text, sizeof(power_text));

    printf("\nTX power setup:\n");
    printf("  The CYW43 driver uses qtxpower with quarter-dBm (Q) units.\n");
    printf("  Formula: dBm = Q / 4   (example: Q=67 -> 16.75 dBm)\n");
    printf("  Values come from the Raspberry Pi RP-002513-TE regulatory tables.\n");
    printf("\n");
    printf("  Recommended for %s channel %d: %s\n",
           wifi_mode_name(mode), channel, power_text);
    printf("\n");
    printf("  1) Use recommended power\n");
    printf("  2) Enter Q-value directly (0-127)\n");
    printf("  3) Enter target power in dBm\n");

    if (!ui_read_choice("Enter choice: ", 1, 3, &choice)) {
        return false;
    }

    switch (choice) {
        case 1:
            *q_value_out = q_value;
            break;
        case 2:
            if (!ui_read_int_in_range("Enter Q-value (0-127): ", 0, 127, &q_value)) {
                return false;
            }
            wifi_format_q_value(q_value, power_text, sizeof(power_text));
            printf("  -> %s\n", power_text);
            *q_value_out = q_value;
            break;
        case 3:
            if (!wifi_read_q_value_from_dbm(&q_value)) {
                return false;
            }
            *q_value_out = q_value;
            break;
        default:
            return false;
    }

    return true;
}

static uint16_t wifi_build_chanspec(int channel, bool ht40) {
    if (ht40) {
        return (uint16_t)(channel | WL_CHANSPEC_BW_40 | WL_CHANSPEC_CTL_SB_LOWER | WL_CHANSPEC_BAND_2G);
    }
    return (uint16_t)(channel | WL_CHANSPEC_BW_20 | WL_CHANSPEC_CTL_SB_NONE | WL_CHANSPEC_BAND_2G);
}

static void wifi_report_step(const char *step, int err) {
    printf("  [%s] %s", err == 0 ? " OK " : "FAIL", step);
    if (err != 0) {
        printf(" (error %d)", err);
    }
    printf("\n");
}

static bool wifi_prepare_interface(void) {
    printf("Preparing WiFi interface...\n");
    cyw43_wifi_set_up(&cyw43_state, CYW43_ITF_STA, true, CYW43_COUNTRY_WORLDWIDE);
    sleep_ms(250);
    bool sta_up = (cyw43_state.itf_state & (1u << CYW43_ITF_STA)) != 0;
    wifi_report_step("STA interface up", sta_up ? 0 : -1);
    return sta_up;
}

static void wifi_print_ioctl_help(void) {
    printf("\nNote on CYW43 debug messages:\n");
    printf("  'got unexpected packet 0' means an ioctl response (type 0)\n");
    printf("  arrived while the driver was busy. This is usually bus\n");
    printf("  contention between Bluetooth and WiFi, not a success code.\n");
    printf("  Regulatory packet-engine TX also requires MFG test firmware\n");
    printf("  (the Raspberry Pi wlarm_le flow), not just standard firmware.\n\n");
}

static bool wifi_apply_common_setup(void) {
    int err = 0;
    int failed = 0;

    err = wifi_set_iovar_u32("frameburst", 1);
    wifi_report_step("frameburst", err);
    failed += (err != 0);

    err = wifi_set_iovar_u32("ampdu", 1);
    wifi_report_step("ampdu", err);
    failed += (err != 0);

    err = wifi_set_iovar_u32("bi", 65000);
    wifi_report_step("beacon interval", err);
    failed += (err != 0);

    err = wifi_set_iovar_u32("phy_watchdog", 0);
    wifi_report_step("phy_watchdog", err);
    failed += (err != 0);

    err = wifi_set_iovar_u32("mpc", 0);
    wifi_report_step("mpc", err);
    failed += (err != 0);

    err = wifi_set_iovar_u32("txchain", 1);
    wifi_report_step("txchain", err);
    failed += (err != 0);

    err = wifi_set_iovar_u32("mimo_bw_cap", 1);
    wifi_report_step("mimo_bw_cap", err);
    failed += (err != 0);

    err = wifi_set_iovar_u32("band", 2);
    wifi_report_step("2.4 GHz band", err);
    failed += (err != 0);

    err = wifi_set_iovar_u32("scansuppress", 1);
    wifi_report_step("scansuppress", err);
    failed += (err != 0);

    if (failed != 0) {
        wifi_print_ioctl_help();
    }

    return failed == 0;
}

static bool wifi_set_rate(wifi_mode_t mode) {
    switch (mode) {
        case WIFI_MODE_11B:
            return wifi_set_iovar_u32("2g_rate", 2) == 0;
        case WIFI_MODE_11G:
            return wifi_set_iovar_u32("2g_rate", 12) == 0;
        case WIFI_MODE_11N20:
        case WIFI_MODE_11N40:
            return wifi_set_iovar_u32("2g_rate", 0x80u) == 0;
        default:
            return false;
    }
}

static bool wifi_start_pkteng(int q_value) {
    wl_pkteng_t pkteng;
    char power_text[48];
    memset(&pkteng, 0, sizeof(pkteng));

    pkteng.flags = WL_PKTENG_PER_TX | WL_PKTENG_ASYNC;
    pkteng.delay = 20;
    pkteng.length = 1500;
    pkteng.nframes = 0;
    pkteng.dest[0] = 0x00;
    pkteng.dest[1] = 0x11;
    pkteng.dest[2] = 0x22;
    pkteng.dest[3] = 0x33;
    pkteng.dest[4] = 0x44;
    pkteng.dest[5] = 0x55;

    int err = wifi_set_iovar_u32("qtxpower", ((uint32_t)q_value & 0xFFu) | WL_TXPWR_OVERRIDE);
    wifi_format_q_value(q_value, power_text, sizeof(power_text));
    printf("  Setting qtxpower to %s\n", power_text);
    wifi_report_step("qtxpower", err);

    err = wifi_set_iovar_u32("phy_forcecal", 1);
    wifi_report_step("phy_forcecal", err);

    err = wifi_set_iovar_data("pkteng", &pkteng, sizeof(pkteng));
    wifi_report_step("pkteng start", err);
    if (err != 0) {
        wifi_print_ioctl_help();
        return false;
    }

    return true;
}

static bool wifi_run_tx_test(wifi_mode_t mode, int channel, int q_value) {
    uint16_t chanspec = wifi_build_chanspec(channel, mode == WIFI_MODE_11N40);
    int mhz = ui_wifi_channel_to_mhz(channel);
    uint8_t up_buf[4] = {0};
    bool success = true;
    int err;
    char power_text[48];

    wifi_format_q_value(q_value, power_text, sizeof(power_text));

    printf("\nStarting %s TX test:\n", wifi_mode_name(mode));
    printf("  Channel: %d (%d MHz)\n", channel, mhz);
    printf("  TX power: %s\n", power_text);
    printf("  Chanspec: 0x%04X\n", chanspec);
    printf("\nVerifying driver commands:\n");

    if (!wifi_prepare_interface()) {
        success = false;
    }

    if (!wifi_apply_common_setup()) {
        success = false;
    }

    err = wifi_set_iovar_u32("chanspec", chanspec);
    wifi_report_step("chanspec", err);
    if (err != 0) {
        success = false;
    }

    if (!wifi_set_rate(mode)) {
        wifi_report_step("data rate", -1);
        success = false;
    } else {
        wifi_report_step("data rate", 0);
    }

    err = wifi_ioctl((WLC_UP << 1) | 1, 0, up_buf);
    wifi_report_step("WLC_UP", err);
    if (err != 0) {
        success = false;
    }

    sleep_ms(50);

    err = wifi_set_iovar_u32("disassoc", 0);
    wifi_report_step("disassoc", err);

    if (!wifi_start_pkteng(q_value)) {
        success = false;
    }

    char details[96];
    snprintf(details, sizeof(details), "Ch %d (%d MHz), TX %s", channel, mhz, power_text);
    ui_print_test_result(success, wifi_mode_name(mode), details);
    return success;
}

static bool wifi_pick_channel(wifi_mode_t mode, int *channel_out) {
    int choice = 0;
    int channel = 0;
    int mhz = 0;

    printf("\nSelect test channel:\n");
    if (mode == WIFI_MODE_11N40) {
        printf("  Lab preset channels for 11n HT40:\n");
        printf("    1) Channel 3  (2422 MHz)\n");
        printf("    2) Channel 6  (2437 MHz)\n");
        printf("    3) Channel 9  (2452 MHz)\n");
    } else {
        printf("  Lab preset channels for 11b/11g/11n HT20:\n");
        printf("    1) Channel 1  (2412 MHz)\n");
        printf("    2) Channel 6  (2437 MHz)\n");
        printf("    3) Channel 11 (2462 MHz)\n");
    }
    printf("    4) Enter channel manually (1-13)\n");
    printf("    5) Enter center frequency in MHz\n");

    if (!ui_read_choice("Enter choice: ", 1, 5, &choice)) {
        return false;
    }

    switch (choice) {
        case 1:
            channel = (mode == WIFI_MODE_11N40) ? 3 : 1;
            break;
        case 2:
            channel = 6;
            break;
        case 3:
            channel = (mode == WIFI_MODE_11N40) ? 9 : 11;
            break;
        case 4:
            if (!ui_read_int_in_range("Enter WiFi channel (1-13): ", 1, 13, &channel)) {
                return false;
            }
            break;
        case 5:
            if (!ui_read_int_in_range("Enter center frequency in MHz (2412-2484): ", 2412, 2484, &mhz)) {
                return false;
            }
            channel = ui_mhz_to_wifi_channel(mhz);
            if (channel < 1 || channel > 13) {
                printf("Frequency does not map to a valid 2.4 GHz WiFi channel (1-13).\n");
                return false;
            }
            break;
        default:
            return false;
    }

    *channel_out = channel;
    return true;
}

static void run_wifi_mode_flow(wifi_mode_t mode) {
    int channel = 0;
    int q_value = 0;

    ui_clear_screen();
    ui_print_reboot_notice();

    printf("\n--- %s Transmit Test ---\n", wifi_mode_name(mode));
    printf("Continuous packet engine TX per Raspberry Pi RP-002513-TE scripts.\n");
    printf("If you ran Bluetooth tests first, power-cycle before this test.\n\n");

    if (!wifi_pick_channel(mode, &channel)) {
        return;
    }

    if (!wifi_pick_tx_power(mode, channel, &q_value)) {
        return;
    }

    wifi_run_tx_test(mode, channel, q_value);
}

static void run_wifi_ap_beacon_test(void) {
    char ssid[33];
    int use_default = 0;

    ui_clear_screen();
    ui_print_reboot_notice();

    printf("\n--- WiFi AP Beacon Sanity Check ---\n");
    printf("Starts an open access point so you can verify WiFi with\n");
    printf("a phone, laptop, or RF survey tool. Bluetooth may remain on.\n\n");

    printf("  1) Use default SSID: %s\n", WIFI_AP_SSID_DEFAULT);
    printf("  2) Enter custom SSID (1-32 characters)\n");
    if (!ui_read_choice("Enter choice: ", 1, 2, &use_default)) {
        return;
    }

    if (use_default == 1) {
        strncpy(ssid, WIFI_AP_SSID_DEFAULT, sizeof(ssid) - 1);
        ssid[sizeof(ssid) - 1] = '\0';
    } else {
        char line[64];
        ui_prompt("Enter SSID: ");
        if (!ui_read_line(line, sizeof(line))) {
            printf("Invalid SSID.\n");
            return;
        }
        size_t len = strlen(line);
        if (len == 0 || len > 32) {
            printf("SSID must be 1-32 characters.\n");
            return;
        }
        strncpy(ssid, line, sizeof(ssid) - 1);
        ssid[sizeof(ssid) - 1] = '\0';
    }

    printf("\nStarting open AP...\n");
    printf("  SSID: %s\n", ssid);
    printf("\nVerifying driver state:\n");

    cyw43_arch_enable_ap_mode(ssid, NULL, CYW43_AUTH_OPEN);
    sleep_ms(500);

    bool ap_up = (cyw43_state.itf_state & (1u << CYW43_ITF_AP)) != 0;
    wifi_report_step("AP interface up", ap_up ? 0 : -1);

    uint8_t mac[6];
    bool mac_ok = cyw43_wifi_get_mac(&cyw43_state, CYW43_ITF_AP, mac) == 0;
    if (mac_ok) {
        printf("  MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
               mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        wifi_report_step("AP MAC assigned", 0);
    } else {
        wifi_report_step("AP MAC assigned", -1);
    }

    char details[64];
    snprintf(details, sizeof(details), "SSID: %s", ssid);
    ui_print_test_result(ap_up && mac_ok, "WiFi AP Beacon",
                         ap_up && mac_ok ? details : "AP failed to start");
}

void test_wifi_menu(void) {
    while (ui_serial_connected()) {
        int choice = 0;

        ui_clear_screen();
        test_session_print_banner();
        printf("--- WiFi Tests (2.4 GHz) ---\n");
        printf("Supported: 802.11b / 802.11g / 802.11n HT20 / 802.11n HT40\n");
        printf("  1) 802.11b continuous TX (regulatory / pkteng)\n");
        printf("  2) 802.11g continuous TX (regulatory / pkteng)\n");
        printf("  3) 802.11n HT20 continuous TX (regulatory / pkteng)\n");
        printf("  4) 802.11n HT40 continuous TX (regulatory / pkteng)\n");
        printf("  5) Start open AP beacon (WiFi sanity check)\n");
        printf("  0) Back to main menu\n");

        if (!ui_read_choice("Enter choice: ", 0, 5, &choice)) {
            continue;
        }

        switch (choice) {
            case 0:
                return;
            case 1:
                run_wifi_mode_flow(WIFI_MODE_11B);
                break;
            case 2:
                run_wifi_mode_flow(WIFI_MODE_11G);
                break;
            case 3:
                run_wifi_mode_flow(WIFI_MODE_11N20);
                break;
            case 4:
                run_wifi_mode_flow(WIFI_MODE_11N40);
                break;
            case 5:
                run_wifi_ap_beacon_test();
                break;
            default:
                break;
        }
    }
}
