#ifndef TEST_SESSION_H
#define TEST_SESSION_H

#include <stdbool.h>

void test_session_set_running(const char *name, const char *details);
void test_session_set_failed(const char *name, const char *details);
void test_session_clear(void);

bool test_session_is_running(void);
bool test_session_last_failed(void);

void test_session_print_banner(void);

#endif
