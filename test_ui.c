#include "test_ui.h"

#include <ctype.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include "platform.h"
#include "test_session.h"

#define UI_LINE_MAX 128

static int s_pushback_char = -1;

static bool ui_read_char(int *out);

bool ui_serial_connected(void) {
    return stdio_usb_connected();
}

void ui_print_banner(void) {
    printf("\n");
    printf("================================================\n");
    printf("  RM2 Compliance Test Firmware\n");
    printf("  GC Ultimate 2 - Regulatory Testing\n");
    printf("================================================\n");
    printf("\n");
}

void ui_print_reboot_notice(void) {
    printf("\n");
    printf("*** IMPORTANT ***\n");
    printf("Press Ctrl+C or Esc at any prompt to reboot and start fresh.\n");
    printf("To stop a running test without rebooting, power off the device,\n");
    printf("then reconnect USB serial.\n");
    ui_print_escape_help();
}

void ui_print_escape_help(void) {
    printf("Use main menu option 4 only to enter the USB bootloader.\n");
    printf("\n");
}

void ui_clear_screen(void) {
    printf("\033[2J\033[H");
    fflush(stdout);
}

void ui_print_test_result(bool success, const char *test_name, const char *details) {
    printf("\n");
    printf("========================================\n");
    printf("  %s\n", test_name);
    if (success) {
        printf("  STATUS: CONFIRMED RUNNING\n");
        printf("========================================\n");
        if (details != NULL && details[0] != '\0') {
            printf("%s\n", details);
        }
        printf("The controller accepted the start command.\n");
        printf("Verify RF output with your lab equipment.\n");
        printf("Power off the device to stop.\n");
        test_session_set_running(test_name, details);
    } else {
        printf("  STATUS: FAILED TO START\n");
        printf("========================================\n");
        if (details != NULL && details[0] != '\0') {
            printf("%s\n", details);
        }
        printf("The start command was not confirmed.\n");
        printf("Review the messages above for details.\n");
        printf("Press Ctrl+C or Esc to reboot and try again.\n");
        test_session_set_failed(test_name, details);
    }
    printf("\n");
    fflush(stdout);
    ui_wait_for_ack("Press Enter to return to the menu...");
}

void ui_wait_for_ack(const char *prompt) {
    ui_prompt(prompt);
    while (ui_serial_connected()) {
        int c;
        if (!ui_read_char(&c)) {
            return;
        }
        if (c == '\r' || c == '\n') {
            if (c == '\r') {
                int next = getchar_timeout_us(100000);
                if (next != PICO_ERROR_TIMEOUT && next != '\n') {
                    s_pushback_char = next;
                }
            }
            printf("\n");
            fflush(stdout);
            return;
        }
    }
}

static bool ui_is_escape_key(int c) {
    return c == 0x03 || c == 0x1B;
}

static void ui_handle_escape_key(int c) {
    if (ui_is_escape_key(c)) {
        platform_reboot_device();
    }
}

void ui_poll_escape(void) {
    int c = getchar_timeout_us(0);
    if (c != PICO_ERROR_TIMEOUT) {
        ui_handle_escape_key(c);
    }
}

void ui_prompt(const char *text) {
    printf("%s", text);
    fflush(stdout);
}

void ui_printf(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    fflush(stdout);
}

static bool ui_read_char(int *out) {
    if (s_pushback_char >= 0) {
        *out = s_pushback_char;
        s_pushback_char = -1;
        return true;
    }

    while (ui_serial_connected()) {
        int c = getchar_timeout_us(100000);
        if (c == PICO_ERROR_TIMEOUT) {
            continue;
        }
        ui_handle_escape_key(c);
        *out = c;
        return true;
    }
    return false;
}

bool ui_read_line(char *buf, size_t buflen) {
    size_t pos = 0;

    if (buflen == 0) {
        return false;
    }

    while (true) {
        int c;
        if (!ui_read_char(&c)) {
            return false;
        }

        if (c == '\r' || c == '\n') {
            buf[pos] = '\0';
            printf("\n");
            fflush(stdout);
            if (c == '\r') {
                int next = getchar_timeout_us(100000);
                if (next != PICO_ERROR_TIMEOUT && next != '\n') {
                    s_pushback_char = next;
                }
            }
            return pos > 0;
        }

        if (c == '\b' || c == 127) {
            if (pos > 0) {
                pos--;
                printf("\b \b");
                fflush(stdout);
            }
            continue;
        }

        if (c >= 32 && c < 127 && pos < buflen - 1) {
            buf[pos++] = (char)c;
            putchar(c);
            fflush(stdout);
        }
    }
}

bool ui_read_int_in_range(const char *prompt, int min_val, int max_val, int *out) {
    char line[UI_LINE_MAX];

    while (true) {
        ui_prompt(prompt);
        if (!ui_read_line(line, sizeof(line))) {
            printf("Invalid input. Please try again.\n");
            continue;
        }

        char *end = NULL;
        long value = strtol(line, &end, 10);
        while (end && *end && isspace((unsigned char)*end)) {
            end++;
        }
        if (end == NULL || *end != '\0') {
            printf("Invalid number. Enter a value between %d and %d.\n", min_val, max_val);
            continue;
        }

        if (value < min_val || value > max_val) {
            printf("Out of range. Enter a value between %d and %d.\n", min_val, max_val);
            continue;
        }

        *out = (int)value;
        return true;
    }
}

bool ui_read_choice(const char *prompt, int min_val, int max_val, int *out) {
    return ui_read_int_in_range(prompt, min_val, max_val, out);
}

int ui_mhz_to_wifi_channel(int mhz) {
    if (mhz < 2412 || mhz > 2484) {
        return -1;
    }
    if (mhz == 2484) {
        return 14;
    }
    if ((mhz - 2407) % 5 != 0) {
        return -1;
    }
    return (mhz - 2407) / 5;
}

int ui_wifi_channel_to_mhz(int channel) {
    if (channel < 1 || channel > 14) {
        return -1;
    }
    if (channel == 14) {
        return 2484;
    }
    return 2407 + (channel * 5);
}
