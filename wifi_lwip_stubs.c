#include "cyw43.h"

#if !CYW43_LWIP

void cyw43_cb_tcpip_set_link_up(__unused cyw43_t *self, __unused int itf) {
}

void cyw43_cb_tcpip_set_link_down(__unused cyw43_t *self, __unused int itf) {
}

void cyw43_cb_process_ethernet(__unused void *cb_data, __unused int itf, __unused size_t len,
                               __unused const uint8_t *buf) {
}

#endif
