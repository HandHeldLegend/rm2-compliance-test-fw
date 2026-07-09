#ifndef TEST_UI_H
#define TEST_UI_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

void ui_print_banner(void);
void ui_print_reboot_notice(void);
void ui_print_escape_help(void);
bool ui_serial_connected(void);

void ui_clear_screen(void);
void ui_print_test_result(bool success, const char *test_name, const char *details);
void ui_wait_for_ack(const char *prompt);

void ui_prompt(const char *text);
void ui_printf(const char *fmt, ...);

bool ui_read_line(char *buf, size_t buflen);
bool ui_read_int_in_range(const char *prompt, int min_val, int max_val, int *out);
bool ui_read_choice(const char *prompt, int min_val, int max_val, int *out);

void ui_poll_escape(void);

int ui_mhz_to_wifi_channel(int mhz);
int ui_wifi_channel_to_mhz(int channel);

#endif
