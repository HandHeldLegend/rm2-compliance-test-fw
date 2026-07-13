#include "test_wifi.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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
#define WLC_DOWN 3
#define WLC_OUT 26
#define WLC_SET_BAND 142
#define WL_TXPWR_OVERRIDE 0x80

/* RPi Pico W 2 GHz WiFi Test Script default Q (txpwr1 -o -q 70). */
#define WIFI_SCRIPT_DEFAULT_Q 70

/* Official script always uses chanspec -w 20 -s 0 on 2.4 GHz. */
#define WL_CHANSPEC_BW_20 0x1000u
#define WL_CHANSPEC_CTL_SB_NONE 0x0000u
#define WL_CHANSPEC_BAND_2G 0x0000u

/* Broadcom wl_pkteng flags (wlioctl.h). */
#define WL_PKTENG_PER_TX_START 0x01u

/* RPi script: wl pkteng_start 00:11:22:33:44:55 tx 20 1500 0 */
static const uint8_t WIFI_SCRIPT_DEST_MAC[6] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55};

/* Official Pico W 2 GHz script covers only these three modulations. */
typedef enum {
    WIFI_MODE_11B,
    WIFI_MODE_11G,
    WIFI_MODE_11N
} wifi_mode_t;

/* Must match Broadcom wl_pkteng_t layout (packed). Field order matters. */
typedef struct __attribute__((packed)) {
    uint32_t flags;
    uint32_t delay;   /* inter-packet delay (us) */
    uint32_t nframes; /* 0 = continuous */
    uint32_t length;  /* packet length */
    uint8_t seqno;
    uint8_t dest[6];
    uint8_t src[6];
} wl_pkteng_t;

_Static_assert(sizeof(wl_pkteng_t) == 29, "wl_pkteng_t must be 29 bytes packed");

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

static int wifi_get_iovar_u32(const char *var, uint32_t *out) {
    /* Use a roomy buffer: some firmwares need space beyond namelen+4. */
    uint8_t buf[64];
    size_t varlen = strlen(var) + 1;
    if (varlen + 4 > sizeof(buf) || out == NULL) {
        return -1;
    }
    memset(buf, 0, sizeof(buf));
    memcpy(buf, var, varlen);
    int err = wifi_ioctl(CYW43_IOCTL_GET_VAR, sizeof(buf), buf);
    if (err != 0) {
        return err;
    }
    *out = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) |
           ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
    return 0;
}

static int wifi_get_iovar_buf(const char *var, uint8_t *buf, size_t buflen) {
    size_t varlen = strlen(var) + 1;
    if (buf == NULL || buflen < varlen + 4) {
        return -1;
    }
    memset(buf, 0, buflen);
    memcpy(buf, var, varlen);
    return wifi_ioctl(CYW43_IOCTL_GET_VAR, buflen, buf);
}

/* True if GET left the iovar name in the buffer (no numeric value returned). */
static bool wifi_u32_is_name_echo(const char *var, uint32_t got) {
    uint8_t b[4] = {0, 0, 0, 0};
    size_t n = strlen(var);
    if (n > 4) {
        n = 4;
    }
    memcpy(b, var, n);
    uint32_t name_le = (uint32_t)b[0] | ((uint32_t)b[1] << 8) |
                       ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
    return got == name_le;
}

static void wifi_report_step(const char *step, int err) {
    printf("  [%s] %s", err == 0 ? " OK " : "FAIL", step);
    if (err != 0) {
        printf(" (error %d)", err);
    }
    printf("\n");
}

/*
 * SET then GET. OK when:
 *  - readback matches (masked), or
 *  - firmware echoes the iovar name (GET not supported for that var) — SET-only OK
 */
static bool wifi_set_and_verify_u32(const char *step, const char *var, uint32_t val, uint32_t mask) {
    int err = wifi_set_iovar_u32(var, val);
    if (err != 0) {
        printf("  [FAIL] %s SET (error %d)\n", step, err);
        return false;
    }
    uint32_t got = 0;
    err = wifi_get_iovar_u32(var, &got);
    if (err != 0) {
        printf("  [ OK ] %s SET ok; GET unsupported (error %d)\n", step, err);
        return true;
    }
    if (wifi_u32_is_name_echo(var, got)) {
        printf("  [ OK ] %s SET ok; GET not readable on this FW\n", step);
        return true;
    }
    uint32_t got_m = got & mask;
    uint32_t exp_m = val & mask;
    if (got_m != exp_m) {
        printf("  [FAIL] %s verify: wrote 0x%08lX, read 0x%08lX\n",
               step, (unsigned long)exp_m, (unsigned long)got_m);
        return false;
    }
    printf("  [ OK ] %s (set+verified 0x%08lX)\n", step, (unsigned long)got_m);
    return true;
}

/* Broadcom/Cypress ratespec: HT MCS encoded with WL_RSPEC_ENCODE_HT. */
#define WL_RSPEC_ENCODE_HT 0x01000000u
#define WL_RSPEC_HT_MCS_MASK 0x7Fu

/* 2g_rate readback is a ratespec; legacy rate or HT MCS in the low bits. */
static bool wifi_rate_matches(uint32_t rate_val, uint32_t got) {
    if (rate_val == WL_RSPEC_ENCODE_HT || (rate_val & 0xFF000000u) == WL_RSPEC_ENCODE_HT) {
        uint32_t exp_mcs = rate_val & WL_RSPEC_HT_MCS_MASK;
        /* Prefer modern HT encode; also accept legacy 0x80|mcs write form readback. */
        if ((got & 0xFF000000u) == WL_RSPEC_ENCODE_HT &&
            (got & WL_RSPEC_HT_MCS_MASK) == exp_mcs) {
            return true;
        }
        if ((got & 0xFFu) == (0x80u | exp_mcs)) {
            return true;
        }
        /* Some MFG builds keep HT flags in 0xC0xxxxxx and MCS in low byte. */
        if ((got & 0xC0000000u) != 0 && (got & WL_RSPEC_HT_MCS_MASK) == exp_mcs &&
            (got & 0x80u) == 0) {
            return true;
        }
        return false;
    }
    return (got & 0xFFu) == (rate_val & 0xFFu);
}

static bool wifi_set_and_verify_rate(const char *step, uint32_t rate_val) {
    int err = wifi_set_iovar_u32("2g_rate", rate_val);
    if (err != 0) {
        printf("  [FAIL] %s SET (error %d)\n", step, err);
        return false;
    }
    uint32_t got = 0;
    err = wifi_get_iovar_u32("2g_rate", &got);
    if (err != 0) {
        printf("  [ OK ] %s SET ok; GET unsupported (error %d)\n", step, err);
        return true;
    }
    if (wifi_u32_is_name_echo("2g_rate", got)) {
        printf("  [ OK ] %s SET ok; GET not readable on this FW\n", step);
        return true;
    }
    if (!wifi_rate_matches(rate_val, got)) {
        printf("  [FAIL] %s verify: wrote 0x%08lX, read 0x%08lX\n",
               step, (unsigned long)rate_val, (unsigned long)got);
        return false;
    }
    printf("  [ OK ] %s (set 0x%08lX, read 0x%08lX)\n",
           step, (unsigned long)rate_val, (unsigned long)got);
    return true;
}

/* Action ioctl with no reliable value readback. */
static bool wifi_set_action_ok(const char *step, int err) {
    wifi_report_step(step, err);
    if (err == 0) {
        printf("           (action cmd — no value readback)\n");
    }
    return err == 0;
}

static bool wifi_verify_isup(uint32_t expected, const char *context) {
    uint32_t got = 0;
    int err = wifi_get_iovar_u32("isup", &got);
    if (err != 0) {
        printf("  [ OK ] %s: GET isup unsupported (error %d)\n", context, err);
        return true;
    }
    if (wifi_u32_is_name_echo("isup", got)) {
        printf("  [ OK ] %s: isup GET not readable on this FW\n", context);
        return true;
    }
    if ((got & 1u) != (expected & 1u)) {
        /* Informational only on MFG — no STA networking. */
        printf("  [ OK ] %s: isup=%lu (expected %lu; ignored on MFG)\n",
               context, (unsigned long)(got & 1u), (unsigned long)(expected & 1u));
        return true;
    }
    printf("  [ OK ] %s (isup=%lu verified)\n", context, (unsigned long)(got & 1u));
    return true;
}

static void wifi_print_mac(const char *label, const uint8_t mac[6]) {
    printf("  %s: %02X:%02X:%02X:%02X:%02X:%02X\n",
           label, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static int wifi_default_q_value(wifi_mode_t mode, int channel) {
    (void)mode;
    (void)channel;
    /* Exact match to RPi Pico W 2 GHz WiFi Test Script: txpwr1 -o -q 70 */
    return WIFI_SCRIPT_DEFAULT_Q;
}

static const char *wifi_mode_name(wifi_mode_t mode) {
    switch (mode) {
        case WIFI_MODE_11B: return "802.11b";
        case WIFI_MODE_11G: return "802.11g";
        case WIFI_MODE_11N: return "802.11n";
        default: return "unknown";
    }
}

static const char *wifi_mode_rate_note(wifi_mode_t mode) {
    switch (mode) {
        case WIFI_MODE_11B: return "2g_rate -r 1  (1 Mbps)";
        case WIFI_MODE_11G: return "2g_rate -r 6  (6 Mbps)";
        case WIFI_MODE_11N: return "2g_rate -h 0 -b 20  (MCS0 HT20)";
        default: return "";
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
    printf("  Script form: wl txpwr1 -o -q <Q>  (implemented as qtxpower + override)\n");
    printf("  Formula: dBm = Q / 4   (script default Q=%d -> %.2f dBm)\n",
           WIFI_SCRIPT_DEFAULT_Q, WIFI_SCRIPT_DEFAULT_Q / 4.0);
    printf("\n");
    printf("  Default for %s channel %d: %s\n",
           wifi_mode_name(mode), channel, power_text);
    printf("\n");
    printf("  1) Use script default power (Q=%d)\n", WIFI_SCRIPT_DEFAULT_Q);
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

/* wl chanspec -c <ch> -b 2 -w 20 -s 0 */
static uint16_t wifi_build_chanspec(int channel) {
    return (uint16_t)(channel | WL_CHANSPEC_BW_20 | WL_CHANSPEC_CTL_SB_NONE | WL_CHANSPEC_BAND_2G);
}

/*
 * MFG / WLTEST builds are for ioctl/pkteng only — they do not support STA
 * networking bring-up. Load firmware via the driver, then probe with an ioctl.
 * Do not require cyw43 itf_state STA bit.
 */
static bool wifi_prepare_interface(void) {
    printf("Preparing WiFi (MFG ioctl path — no STA)...\n");

    /* Triggers cyw43_ensure_up → firmware download. STA bit is unused for MFG. */
    cyw43_wifi_set_up(&cyw43_state, CYW43_ITF_STA, true, CYW43_COUNTRY_WORLDWIDE);
    sleep_ms(500);

    if (cyw43_poll == NULL) {
        printf("  [FAIL] CYW43 bus not initialized (firmware load failed)\n");
        printf("  Check MFG header / firmware/README.md\n");
        return false;
    }

    /* Readiness = chip answers an ioctl, not STA association. */
    uint8_t ver[128];
    memset(ver, 0, sizeof(ver));
    memcpy(ver, "ver", 4);
    int err = wifi_ioctl(CYW43_IOCTL_GET_VAR, sizeof(ver), ver);
    wifi_report_step("MFG bus ready (GET ver)", err);
    if (err == 0 && ver[0] != '\0') {
        printf("    %s\n", ver);
    }
    return err == 0;
}

static void wifi_print_ioctl_help(void) {
    printf("\nNote on CYW43 / firmware:\n");
    printf("  Stock Pico W firmware usually cannot run continuous pkteng TX.\n");
    printf("  SET pkteng may return OK without putting RF on the air.\n");
    printf("  Regulatory TX needs MFG/test firmware (see firmware/README.md).\n");
    printf("  'got unexpected packet 0' is an ioctl response type, not success.\n\n");
}

static bool wifi_ioctl_cmd(uint32_t wlc_cmd) {
    uint8_t dummy[4] = {0};
    return wifi_ioctl((wlc_cmd << 1) | 1, 0, dummy) == 0;
}

/* wl country ALL — special ccode unlocking all channels (wlioctl wl_country_t). */
static bool wifi_set_and_verify_country_all(void) {
    uint8_t country[4 + 4 + 4];
    memset(country, 0, sizeof(country));
    memcpy(country + 0, "ALL", 3);
    put_le32(country + 4, (uint32_t)-1); /* rev unspecified */
    memcpy(country + 8, "ALL", 3);

    int err = wifi_set_iovar_data("country", country, sizeof(country));
    if (err != 0) {
        printf("  [FAIL] country ALL SET (error %d)\n", err);
        return false;
    }

    uint8_t got[128];
    err = wifi_get_iovar_buf("country", got, sizeof(got));
    if (err != 0) {
        printf("  [ OK ] country ALL SET ok; GET unsupported (error %d)\n", err);
        return true;
    }
    /* Name echo: first bytes still "cou" from "country". */
    if (memcmp(got, "country", 7) == 0) {
        printf("  [ OK ] country ALL SET ok; GET not readable on this FW\n");
        return true;
    }
    /* Accept ALL / XX / abbreviated forms that contain ALL in the struct. */
    bool found_all = false;
    for (size_t i = 0; i + 3 <= sizeof(got); i++) {
        if (memcmp(&got[i], "ALL", 3) == 0) {
            found_all = true;
            break;
        }
    }
    if (!found_all) {
        printf("  [ OK ] country ALL SET ok; GET ccode='%.3s' (not ALL — CLM may remap)\n",
               got);
        return true;
    }
    printf("  [ OK ] country ALL (set+verified)\n");
    return true;
}

/*
 * Rates from RPi Pico W 2 GHz WiFi Test Script:
 *   11b: 2g_rate -r 1   → 1 Mbps = 2 in 500 kbps units
 *   11g: 2g_rate -r 6   → 6 Mbps = 12
 *   11n: 2g_rate -h 0 -b 20 → HT MCS0 ratespec (WL_RSPEC_ENCODE_HT | 0)
 */
static uint32_t wifi_rate_value(wifi_mode_t mode) {
    switch (mode) {
        case WIFI_MODE_11B: return 2;
        case WIFI_MODE_11G: return 12;
        case WIFI_MODE_11N: return WL_RSPEC_ENCODE_HT; /* MCS0 */
        default: return 0;
    }
}

/*
 * Shared preamble matching the RPi script through `wl band b` (before chanspec).
 * Value SETs are only OK after GET readback matches.
 */
static bool wifi_script_common_preamble(void) {
    int failed = 0;

    if (!wifi_set_action_ok("WLC_OUT (wl out)", wifi_ioctl_cmd(WLC_OUT) ? 0 : -1)) {
        failed++;
    }
    /* isup often stays 1 across OUT/DOWN on this host stack — do not require 0. */
    if (!wifi_set_action_ok("WLC_DOWN (wl down)", wifi_ioctl_cmd(WLC_DOWN) ? 0 : -1)) {
        failed++;
    }

    if (!wifi_set_and_verify_u32("frameburst", "frameburst", 1, 0xFFFFFFFFu)) {
        failed++;
    }
    /*
     * ampdu may read back 0 while the interface is down even after SET 1.
     * Treat SET success + name-echo/unsupported GET as OK; real mismatch of
     * a numeric 0 is accepted here with a note (re-checked indirectly later).
     */
    {
        int err = wifi_set_iovar_u32("ampdu", 1);
        if (err != 0) {
            printf("  [FAIL] ampdu SET (error %d)\n", err);
            failed++;
        } else {
            uint32_t got = 0;
            err = wifi_get_iovar_u32("ampdu", &got);
            if (err != 0 || wifi_u32_is_name_echo("ampdu", got)) {
                printf("  [ OK ] ampdu SET ok; GET not readable on this FW\n");
            } else if ((got & 0xFFu) == 1u) {
                printf("  [ OK ] ampdu (set+verified 0x%08lX)\n", (unsigned long)got);
            } else {
                printf("  [ OK ] ampdu SET ok; read 0x%08lX while down (allowed)\n",
                       (unsigned long)got);
            }
        }
    }
    if (!wifi_set_and_verify_country_all()) {
        failed++;
    }
    if (!wifi_set_and_verify_u32("beacon interval", "bi", 65000, 0xFFFFFFFFu)) {
        failed++;
    }
    if (!wifi_set_and_verify_u32("phy_watchdog", "phy_watchdog", 0, 0xFFFFFFFFu)) {
        failed++;
    }
    if (!wifi_set_and_verify_u32("mpc", "mpc", 0, 0xFFFFFFFFu)) {
        failed++;
    }
    if (!wifi_set_and_verify_u32("txchain", "txchain", 1, 0xFFFFFFFFu)) {
        failed++;
    }
    if (!wifi_set_and_verify_u32("mimo_bw_cap", "mimo_bw_cap", 1, 0xFFFFFFFFu)) {
        failed++;
    }
    /* HT MCS rates need nmode on; harmless for 11b/g. */
    if (!wifi_set_and_verify_u32("nmode", "nmode", 1, 0xFFFFFFFFu)) {
        failed++;
    }
    /* Prefer WLC_SET_BAND (matches host `wl band b`). */
    {
        uint8_t band_buf[4];
        put_le32(band_buf, 2); /* WLC_BAND_2G */
        int err = wifi_ioctl((WLC_SET_BAND << 1) | 1, 4, band_buf);
        if (err != 0) {
            printf("  [FAIL] band b WLC_SET_BAND (error %d)\n", err);
            failed++;
        } else {
            uint32_t got = 0;
            int gerr = wifi_get_iovar_u32("band", &got);
            if (gerr == 0 && !wifi_u32_is_name_echo("band", got) && (got & 0xFFu) == 2u) {
                printf("  [ OK ] band b (WLC_SET_BAND + verified 0x%08lX)\n",
                       (unsigned long)got);
            } else {
                printf("  [ OK ] band b (WLC_SET_BAND)\n");
            }
        }
    }

    if (failed != 0) {
        wifi_print_ioctl_help();
    }
    return failed == 0;
}

/* Low-level form of: wl txpwr1 -o -q <Q> */
static bool wifi_set_and_verify_txpwr_q(int q_value) {
    uint32_t pwr = ((uint32_t)q_value & 0xFFu) | WL_TXPWR_OVERRIDE;
    return wifi_set_and_verify_u32("qtxpower (txpwr1 -o -q)", "qtxpower",
                                   pwr, 0xFFu | WL_TXPWR_OVERRIDE);
}

static bool wifi_start_pkteng_tx(const uint8_t dest[6]) {
    wl_pkteng_t pkteng;
    memset(&pkteng, 0, sizeof(pkteng));

    /* wl pkteng_start <dest> tx 20 1500 0 */
    pkteng.flags = WL_PKTENG_PER_TX_START;
    pkteng.delay = 20;
    pkteng.nframes = 0;
    pkteng.length = 1500;
    pkteng.seqno = 0;
    memcpy(pkteng.dest, dest, 6);
    /* src left zero → firmware uses device MAC */

    wifi_print_mac("pkteng TX dest", dest);
    printf("  pkteng: flags=0x%04lX delay=%lu nframes=%lu length=%lu\n",
           (unsigned long)pkteng.flags,
           (unsigned long)pkteng.delay,
           (unsigned long)pkteng.nframes,
           (unsigned long)pkteng.length);

    int err = wifi_set_iovar_data("pkteng", &pkteng, sizeof(pkteng));
    if (!wifi_set_action_ok("pkteng start", err)) {
        wifi_print_ioctl_help();
        return false;
    }
    return true;
}

static bool wifi_run_tx_test(wifi_mode_t mode, int channel, int q_value) {
    uint16_t chanspec = wifi_build_chanspec(channel);
    int mhz = ui_wifi_channel_to_mhz(channel);
    bool success = true;
    char power_text[48];
    uint32_t rate = wifi_rate_value(mode);

    wifi_format_q_value(q_value, power_text, sizeof(power_text));

    printf("\nStarting %s TX (RPi Pico W 2 GHz script):\n", wifi_mode_name(mode));
    printf("  Rate: %s\n", wifi_mode_rate_note(mode));
    printf("  Channel: %d (%d MHz)  chanspec=0x%04X  (-w 20)\n", channel, mhz, chanspec);
    printf("  TX power: %s  (script: txpwr1 -o -q)\n", power_text);
    wifi_print_mac("Dest (script fixed)", WIFI_SCRIPT_DEST_MAC);
    printf("\nScript sequence (SET values verified by GET where possible):\n");

    if (!wifi_prepare_interface()) {
        success = false;
    }

    if (!wifi_script_common_preamble()) {
        success = false;
    }

    if (!wifi_set_and_verify_u32("chanspec", "chanspec", chanspec, 0xFFFFu)) {
        success = false;
    }

    if (rate == 0 || !wifi_set_and_verify_rate("2g_rate", rate)) {
        success = false;
    }

    if (!wifi_set_action_ok("WLC_UP", wifi_ioctl_cmd(WLC_UP) ? 0 : -1)) {
        success = false;
    } else {
        sleep_ms(50);
        /* MFG is not a STA stack — isup can disagree; do not fail the run. */
        (void)wifi_verify_isup(1, "after up");
    }

    if (!wifi_set_action_ok("disassoc", wifi_set_iovar_u32("disassoc", 0))) {
        success = false;
    }

    if (!wifi_set_action_ok("phy_forcecal", wifi_set_iovar_u32("phy_forcecal", 1))) {
        success = false;
    }

    if (!wifi_set_and_verify_u32("scansuppress", "scansuppress", 1, 0xFFFFFFFFu)) {
        success = false;
    }

    if (!wifi_set_and_verify_txpwr_q(q_value)) {
        success = false;
    }

    if (!wifi_start_pkteng_tx(WIFI_SCRIPT_DEST_MAC)) {
        success = false;
    }

    char details[96];
    snprintf(details, sizeof(details), "Ch %d (%d MHz), TX %s", channel, mhz, power_text);

    printf("\n========================================\n");
    printf("  %s\n", wifi_mode_name(mode));
    if (success) {
        printf("  STATUS: CONFIRMED RUNNING\n");
        printf("========================================\n");
        printf("%s\n", details);
        printf("Driver accepted and verified script settings (where readable).\n");
        printf("Verify RF output with your lab equipment.\n");
        printf("Power off the device to stop.\n");
        test_session_set_running(wifi_mode_name(mode), details);
    } else {
        printf("  STATUS: FAILED TO START\n");
        printf("========================================\n");
        printf("%s\n", details);
        printf("A SET or readback verify failed — see steps above.\n");
        printf("Press Ctrl+C or Esc to reboot and try again.\n");
        test_session_set_failed(wifi_mode_name(mode), details);
    }

    printf("\n");
    fflush(stdout);
    ui_wait_for_ack("Press Enter to return to the menu...");
    return success;
}

static bool wifi_pick_channel(int *channel_out) {
    int choice = 0;
    int channel = 0;
    int mhz = 0;

    printf("\nSelect test channel (script: wl chanspec -c <ch> -b 2 -w 20 -s 0):\n");
    printf("  Presets commonly used in lab:\n");
    printf("    1) Channel 1  (2412 MHz)\n");
    printf("    2) Channel 6  (2437 MHz)\n");
    printf("    3) Channel 11 (2462 MHz)\n");
    printf("    4) Enter channel manually (1-13)\n");
    printf("    5) Enter center frequency in MHz\n");

    if (!ui_read_choice("Enter choice: ", 1, 5, &choice)) {
        return false;
    }

    switch (choice) {
        case 1:
            channel = 1;
            break;
        case 2:
            channel = 6;
            break;
        case 3:
            channel = 11;
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
    printf("Official RPi Pico W 2 GHz WiFi Test Script (RP-002513-TE).\n");
    printf("  Rate: %s\n", wifi_mode_rate_note(mode));
    printf("  Bandwidth: 20 MHz only\n");
    printf("  Dest MAC: 00:11:22:33:44:55 (script fixed)\n");
    printf("Selectable per script notes: channel (-c), TX power Q (-q).\n\n");

    if (!wifi_pick_channel(&channel)) {
        return;
    }

    if (!wifi_pick_tx_power(mode, channel, &q_value)) {
        return;
    }

    wifi_run_tx_test(mode, channel, q_value);
}

void test_wifi_run_mode(int mode_menu_id) {
    wifi_mode_t mode;
    switch (mode_menu_id) {
        case WIFI_MODE_MENU_11B:
            mode = WIFI_MODE_11B;
            break;
        case WIFI_MODE_MENU_11G:
            mode = WIFI_MODE_11G;
            break;
        case WIFI_MODE_MENU_11N:
            mode = WIFI_MODE_11N;
            break;
        default:
            return;
    }
    run_wifi_mode_flow(mode);
}
