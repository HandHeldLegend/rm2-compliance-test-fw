#include "test_session.h"

#include <stdio.h>
#include <string.h>

typedef struct {
    bool running;
    bool failed;
    char name[48];
    char details[96];
} test_session_t;

static test_session_t s_session;

static void copy_field(char *dst, size_t dstlen, const char *src) {
    if (src == NULL) {
        dst[0] = '\0';
        return;
    }
    strncpy(dst, src, dstlen - 1);
    dst[dstlen - 1] = '\0';
}

void test_session_clear(void) {
    memset(&s_session, 0, sizeof(s_session));
}

void test_session_set_running(const char *name, const char *details) {
    s_session.running = true;
    s_session.failed = false;
    copy_field(s_session.name, sizeof(s_session.name), name);
    copy_field(s_session.details, sizeof(s_session.details), details);
}

void test_session_set_failed(const char *name, const char *details) {
    s_session.running = false;
    s_session.failed = true;
    copy_field(s_session.name, sizeof(s_session.name), name);
    copy_field(s_session.details, sizeof(s_session.details), details);
}

bool test_session_is_running(void) {
    return s_session.running;
}

bool test_session_last_failed(void) {
    return s_session.failed;
}

void test_session_print_banner(void) {
    if (s_session.running) {
        printf("************************************************\n");
        printf("  ACTIVE TEST: %s\n", s_session.name);
        if (s_session.details[0] != '\0') {
            printf("  %s\n", s_session.details);
        }
        printf("  Status: CONFIRMED RUNNING\n");
        printf("  Stop: power off the device\n");
        printf("************************************************\n\n");
        return;
    }

    if (s_session.failed) {
        printf("************************************************\n");
        printf("  LAST TEST FAILED: %s\n", s_session.name);
        if (s_session.details[0] != '\0') {
            printf("  %s\n", s_session.details);
        }
        printf("  Review the output above, then start another test\n");
        printf("  or press Ctrl+C / Esc to reboot.\n");
        printf("************************************************\n\n");
    }
}
