#ifndef PLATFORM_H
#define PLATFORM_H

#include <stdbool.h>

bool platform_init_cyw43(void);
bool platform_init_btstack(void);
void platform_shutdown_btstack(void);
bool platform_btstack_is_active(void);
void platform_reboot_device(void);
void platform_reboot_to_bootloader(void);

#endif
