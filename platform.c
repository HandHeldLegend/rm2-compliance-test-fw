#include "platform.h"

#include "btstack.h"
#include "hci_common.h"
#include "test_session.h"
#include "pico/async_context_threadsafe_background.h"
#include "pico/btstack_chipset_cyw43.h"
#include "pico/btstack_cyw43.h"
#include "pico/cyw43_arch.h"
#include "pico/cyw43_driver.h"
#include "pico/bootrom.h"
#include "hardware/watchdog.h"
#include <stdio.h>

static async_context_threadsafe_background_t s_async_ctx;
static bool s_cyw43_ready = false;
static bool s_btstack_ready = false;

bool platform_init_cyw43(void) {
    if (s_cyw43_ready) {
        return true;
    }

    async_context_threadsafe_background_config_t config =
        async_context_threadsafe_background_default_config();

    if (!async_context_threadsafe_background_init(&s_async_ctx, &config)) {
        printf("ERROR: Failed to initialize async context.\n");
        return false;
    }

    async_context_t *context = &s_async_ctx.core;
    cyw43_arch_set_async_context(context);

    if (!cyw43_driver_init(context)) {
        printf("ERROR: Failed to initialize CYW43 driver.\n");
        return false;
    }

    s_cyw43_ready = true;
    return true;
}

bool platform_init_btstack(void) {
    if (!s_cyw43_ready) {
        printf("ERROR: CYW43 must be initialized before Bluetooth.\n");
        return false;
    }

    if (s_btstack_ready) {
        hci_power_control(HCI_POWER_ON);
        return true;
    }

    async_context_t *context = cyw43_arch_async_context();
    if (!btstack_cyw43_init(context)) {
        printf("ERROR: Failed to initialize BTstack.\n");
        return false;
    }

    hci_set_chipset(btstack_chipset_cyw43_instance());

    static btstack_packet_callback_registration_t hci_event_callback_registration;
    hci_event_callback_registration.callback = &hci_common_packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    hci_power_control(HCI_POWER_ON);
    hci_common_init();

    s_btstack_ready = true;
    return true;
}

void platform_shutdown_btstack(void) {
    if (!s_btstack_ready) {
        return;
    }

    async_context_t *context = cyw43_arch_async_context();
    btstack_cyw43_deinit(context);
    s_btstack_ready = false;
}

bool platform_btstack_is_active(void) {
    return s_btstack_ready;
}

void platform_reboot_device(void) {
    printf("\nRebooting device...\n");
    fflush(stdout);
    test_session_clear();
    platform_shutdown_btstack();
    sleep_ms(200);
    watchdog_reboot(0, 0, 0);
}

void platform_reboot_to_bootloader(void) {
    printf("\nRebooting to USB bootloader...\n");
    fflush(stdout);
    platform_shutdown_btstack();
    sleep_ms(500);
    reset_usb_boot(0, 0);
}
