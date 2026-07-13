#ifndef TEST_WIFI_H
#define TEST_WIFI_H

/* Menu selectors for the WiFi-only firmware build (main menu entries). */
#define WIFI_MODE_MENU_11B  0
#define WIFI_MODE_MENU_11G  1
#define WIFI_MODE_MENU_11N  2

void test_wifi_run_mode(int mode_menu_id);
void test_wifi_run_rx(void);

#endif
