/*
 * Copyright © 2024 chargebyte GmbH
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stddef.h>
#include <termios.h>

struct uart_ctx {
    /* pointer to uart device */
    const char *device;

    /* file descriptor */
    int fd;

    /* currently successfully configured baudrate */
    int current_baudrate;

    /* saved termios settings during usage */
    struct termios oldtio;

    /* termios settings to use */
    struct termios newtio;
};

int uart_open(struct uart_ctx *ctx, const char *port, int baudrate);
int uart_close(struct uart_ctx *ctx);
int uart_reconfigure_baudrate(struct uart_ctx *ctx, int baudrate);

int uart_dump_frame(bool is_sending, uint8_t *buffer, size_t len);

int uart_flush_input(struct uart_ctx *ctx);

/* wait for the fd to have data to read available, timeout in ms */
int uart_wait_frame(struct uart_ctx *ctx, int timeout_ms);

/* write bytes (looped) with call to tcdrain followed */
ssize_t uart_write_drain(struct uart_ctx *ctx, const uint8_t *buf, size_t count);

/* read bytes (looped), with overall timeout in ms */
ssize_t uart_read_with_timeout(struct uart_ctx *ctx, uint8_t *buf, size_t count, int timeout_ms);

#ifdef __cplusplus
}
#endif
