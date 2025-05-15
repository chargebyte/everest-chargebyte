/*
 * Copyright © 2025 chargebyte GmbH
 * SPDX-License-Identifier: Apache-2.0
 */
#include <endian.h>
#include <errno.h>
#include <inttypes.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include "uart.h"
#include "logging.h"
#include "crc8_j1850.h"
#include "cb_uart.h"

/* frame start/end markers */
#define CB_SOF 0xA5
#define CB_EOF 0x03

/* UART frame */
struct uart_frame {
    uint8_t sof;
    uint8_t com;
    uint64_t data;
    uint8_t crc;
    uint8_t eof;
} __attribute__((packed));

const char *cb_uart_com_to_str(enum cb_uart_com com)
{
    switch (com) {
    case COM_CHARGE_CONTROL:
        return "COM_CHARGE_CONTROL";
    case COM_CHARGE_STATE:
        return "COM_CHARGE_STATE";
    case COM_PT1000_STATE:
        return "COM_PT1000_STATE";
    case COM_FW_VERSION:
        return "COM_FW_VERSION";
    case COM_GIT_HASH:
        return "COM_GIT_HASH";
    case COM_INQUIRY:
        return "COM_INQUIRY";
    default:
        return "UNKNOWN";
    }
}

int cb_uart_send(struct uart_ctx *uart, enum cb_uart_com com, uint64_t data)
{
    struct uart_frame frame;
    ssize_t c;

    /* prepare packet */
    memset(&frame, 0, sizeof(frame));
    frame.sof = CB_SOF;
    frame.com = com;
    frame.data = htobe64(data);
    frame.crc = crc8_j1850(&frame.com, sizeof(frame.com) + sizeof(frame.data));
    frame.eof = CB_EOF;

    if (uart->trace)
        uart_dump_frame(true, (uint8_t *)&frame, sizeof(frame));

    c = uart_write_drain(uart, (const uint8_t *)&frame, sizeof(frame));
    if (c < 0)
        return c;

    return 0;
}

int cb_uart_recv(struct uart_ctx *uart, enum cb_uart_com *com, uint64_t *data)
{
    struct uart_frame frame;
    uint8_t crc;
    ssize_t c;

    /* Regarding the timeout: when this function is called, it it usually async to the interval/periodicity
     * of the safety controller. Thus we expect at least after the CB_UART_RECV_INTERVAL a fully UART frame.
     * We add half of the period as safety margin.
     */
    c = uart_read_with_timeout(uart, (uint8_t *)&frame, sizeof(frame), CB_UART_RECV_INTERVAL + CB_UART_RECV_INTERVAL / 2);
    if (c < 0)
        return c;

    if (uart->trace)
        uart_dump_frame(false, (uint8_t *)&frame, sizeof(frame));

    /* check field patterns */
    if (frame.sof != CB_SOF) {
        error("SOF pattern mismatch: expected 0x%02x, got 0x%02" PRIx8, CB_SOF, frame.sof);
        errno = EBADMSG;
        return -1;
    }
    if (frame.eof != CB_EOF) {
        error("EOF pattern mismatch: expected 0x%02x, got 0x%02" PRIx8, CB_EOF, frame.eof);
        errno = EBADMSG;
        return -1;
    }

    /* check crc */
    crc = crc8_j1850(&frame.com, sizeof(frame.com) + sizeof(frame.data));
    if (crc != frame.crc) {
        error("CRC pattern mismatch: expected 0x%02x, got 0x%02" PRIx8, crc, frame.crc);
        errno = EBADMSG;
        return -1;
    }

    debug("received frame looks valid (SOF, EOF, CRC)");

    if (com)
        *com = frame.com;
    if (data)
        *data = be64toh(frame.data);

    return 0;
}

int cb_uart_recv_and_sync(struct uart_ctx *uart, enum cb_uart_com *com, uint64_t *data)
{
    unsigned int trial = CB_UART_MAX_SYNC_TRIALS;
    int rv;

retry:
    trial--;

    rv = cb_uart_recv(uart, com, data);
    if (rv < 0) {
        /* in case we are async to the safety controller, we might have looked at
         * the wrong positions; then discard all data in receive buffers, wait for next
         * frame and try again, but at max CB_UART_MAX_SYNC_TRIALS times.
         */
        if (errno == EBADMSG) {
            rv = uart_flush_input(uart);
            if (rv < 0)
                return rv;

            if (trial)
                goto retry;
        }

        return rv;
    }

    return 0;
}
