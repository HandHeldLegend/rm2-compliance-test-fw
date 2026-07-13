#include "platform.h"

#include "test_session.h"
#include "pico/bootrom.h"
#include "pico/stdlib.h"
#include "hardware/watchdog.h"
#include <stdio.h>

#if RM2_ENABLE_BT
#include "btstack.h"
#include "hci_common.h"
#include "pico/btstack_chipset_cyw43.h"
#include "pico/cyw43_arch.h"
#else
#include "pico/async_context_threadsafe_background.h"
#include "pico/cyw43_arch.h"
#include "pico/cyw43_driver.h"
#endif

#ifndef RM2_ENABLE_BT
#define RM2_ENABLE_BT 0
#endif

#if RM2_ENABLE_BT
static bool s_ready = false;
#else
static async_context_threadsafe_background_t s_async_ctx;
static bool s_cyw43_ready = false;
#endif

bool platform_init_cyw43(void) {
#if RM2_ENABLE_BT
    /*
     * Match the working ee8e331 HCI bridge: cyw43_arch_init() brings up the
     * CYW43 driver and BTstack together, then power HCI on at boot.
     */
    if (s_ready) {
        return true;
    }

    if (cyw43_arch_init()) {
        printf("ERROR: Failed to initialize CYW43 / BTstack.\n");
        return false;
    }

    hci_set_chipset(btstack_chipset_cyw43_instance());

    static btstack_packet_callback_registration_t hci_event_callback_registration;
    hci_event_callback_registration.callback = &hci_common_packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    hci_power_control(HCI_POWER_ON);
    /* Allow CYW43 + BT firmware download before first HCI command. */
    sleep_ms(1500);
    hci_common_init();

    s_ready = true;
    return true;
#else
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
#endif
}

bool platform_init_btstack(void) {
#if !RM2_ENABLE_BT
    printf("ERROR: This firmware build does not include Bluetooth tests.\n");
    printf("Flash the *_bt.uf2 (stock) image for BT Classic / BLE.\n");
    return false;
#else
    /* BTstack is powered on during platform_init_cyw43() (ee8e331 style). */
    if (!s_ready) {
        return platform_init_cyw43();
    }
    return true;
#endif
}

void platform_shutdown_btstack(void) {
    /* Keep HCI up for the life of a BT/stock image (same as ee8e331). */
}

bool platform_btstack_is_active(void) {
#if RM2_ENABLE_BT
    return s_ready;
#else
    return false;
#endif
}

void platform_reboot_device(void) {
    printf("\nRebooting device...\n");
    fflush(stdout);
    test_session_clear();
    sleep_ms(200);
    watchdog_reboot(0, 0, 0);
}

void platform_reboot_to_bootloader(void) {
    printf("\nRebooting to USB bootloader...\n");
    fflush(stdout);
    sleep_ms(500);
    reset_usb_boot(0, 0);
}
