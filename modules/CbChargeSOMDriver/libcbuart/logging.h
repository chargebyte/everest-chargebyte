/*
 * Copyright © 2024 chargebyte GmbH
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdarg.h>

void error(const char *format, ...);
void debug(const char *format, ...);

typedef void (*libcbuart_msg_cb)(const char *format, va_list args);

void libcbuart_set_error_msg_cb(libcbuart_msg_cb cb);
void libcbuart_set_debug_msg_cb(libcbuart_msg_cb cb);

#ifdef __cplusplus
}
#endif
