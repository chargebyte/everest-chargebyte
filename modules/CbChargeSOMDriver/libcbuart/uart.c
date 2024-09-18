/*
 * Copyright © 2024 chargebyte GmbH
 * SPDX-License-Identifier: Apache-2.0
 */
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include "uart.h"
#include "tools.h"
#include "logging.h"

static speed_t baudrate_to_speed(int baudrate)
{
    switch (baudrate) {
    case 1200:   return B1200;
    case 2400:   return B2400;
    case 4800:   return B4800;
    case 9600:   return B9600;
    case 19200:  return B19200;
    case 38400:  return B38400;
    case 57600:  return B57600;
    case 115200: return B115200;
    default:     return B0;
    }
}

static int uart_prepare_new_settings(struct uart_ctx *ctx, int baudrate)
{
    speed_t speed;
    int rv;

    /* prepare new settings based upon current settings */
    memcpy(&ctx->newtio, &ctx->oldtio, sizeof(ctx->newtio));

    /* apply baudrate */
    speed = baudrate_to_speed(baudrate);
    rv = cfsetispeed(&ctx->newtio, speed);
    if (rv)
        return -1;
    rv = cfsetospeed(&ctx->newtio, speed);
    if (rv)
        return -1;

    /* setting: 8N1 */
    ctx->newtio.c_cflag &= ~PARENB;
    ctx->newtio.c_cflag &= ~CSTOPB;
    ctx->newtio.c_cflag &= ~CSIZE;
    ctx->newtio.c_cflag &= ~CMSPAR;
    ctx->newtio.c_cflag |= CS8;
    ctx->newtio.c_cflag |= CREAD;
    ctx->newtio.c_cflag |= CLOCAL;

    /* ignore framing errors and parity errors */
    ctx->newtio.c_iflag |= IGNPAR;

    /* disable hardware flow control */
    ctx->newtio.c_cflag &= ~CRTSCTS;

    /* disable software flow control */
    ctx->newtio.c_iflag &= ~(IXON | IXOFF | IXANY);

    /* local flags */
    ctx->newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    /* input flags */
    ctx->newtio.c_iflag |= IGNBRK;
    ctx->newtio.c_iflag &= ~(INPCK | INLCR | ICRNL | IGNCR | ISTRIP);

    /* output flags */
    ctx->newtio.c_oflag &= ~OPOST;

    /* timeout handling */
    ctx->newtio.c_cc[VTIME] = 5; /* timeout 5 deci-seconds */
    ctx->newtio.c_cc[VMIN] = 0;

    return 0;
}

int uart_open(struct uart_ctx *ctx, const char *device, int baudrate)
{
    int saved_errno, rv;
    char *error_cause;

    ctx->device = device;
    ctx->fd = -1;

    /* check permissions first */
    rv = access(device, W_OK);
    if (rv) {
        error("unable to access '%s': %m", device);
        return -1;
    }

    /* open device */
    ctx->fd = open(device, O_RDWR | O_NOCTTY);
    if (ctx->fd == -1) {
        if (errno == EBUSY) {
            error("the port '%s' is locked by another program", device);
            return -1;
        }

        /* in all other error cases bail out */
        error("could not open '%s': %m", device);
        return -1;
    }

    /* get exclusive access - part 1 */
    rv = flock(ctx->fd, LOCK_EX | LOCK_NB);
    if (rv) {
        saved_errno = errno;
        error_cause = "flock";
        goto close_out;
    }

    /* get exclusive access - part 2 */
    rv = ioctl(ctx->fd, TIOCEXCL);
    if (rv) {
        saved_errno = errno;
        error_cause = "ioctl(TIOCEXCL)";
        goto close_out;
    }

    /* save current port settings */
    rv = tcgetattr(ctx->fd, &ctx->oldtio);
    if (rv) {
        saved_errno = errno;
        error_cause = "tcgetattr";
        goto unlock_out;
    }

    /* prepare the UART for raw access and 8N1 with desired baudrate */
    rv = uart_prepare_new_settings(ctx, baudrate);
    if (rv) {
        saved_errno = errno;
        error_cause = "cfsetispeed/cfsetospeed";
        goto unlock_out;
    }

    /* apply our desired port settings */
    rv = tcsetattr(ctx->fd, TCSAFLUSH, &ctx->newtio);
    if (rv) {
        saved_errno = errno;
        error_cause = "tcsetattr";
        goto unlock_out;
    }

    /* remember successfully set baudrate */
    ctx->current_baudrate = baudrate;

    return 0;

    /* we ignore error in the following since we are already in error path */

unlock_out:
    /* release exclusive lock */
    ioctl(ctx->fd, TIOCNXCL);

close_out:
    close(ctx->fd);
    ctx->fd = -1;

    /* restore errno to be available in higher levels */
    errno = saved_errno;
    error("%s failed for '%s': %s", error_cause, device, strerror(saved_errno));

    return rv;
}

int uart_close(struct uart_ctx *ctx)
{
    int rv = 0;

    /* restore saved settings */
    rv |= tcsetattr(ctx->fd, TCSAFLUSH, &ctx->oldtio);

    /* release lock */
    rv |= ioctl(ctx->fd, TIOCNXCL);

    /* finally close fd */
    rv |= close(ctx->fd);
    ctx->fd = -1;

    return rv;
}

int uart_reconfigure_baudrate(struct uart_ctx *ctx, int baudrate)
{
    int saved_errno, rv;
    char *error_cause;

    rv = uart_prepare_new_settings(ctx, baudrate);
    if (rv) {
        saved_errno = errno;
        error_cause = "cfsetispeed/cfsetospeed";
        goto err_out;
    }

    /* apply our desired port settings */
    rv = tcsetattr(ctx->fd, TCSAFLUSH, &ctx->newtio);
    if (rv) {
        saved_errno = errno;
        error_cause = "tcsetattr";
        goto err_out;
    }

    /* remember successfully set baudrate */
    ctx->current_baudrate = baudrate;

    return 0;

err_out:
    /* restore errno to be available in higher levels */
    errno = saved_errno;
    error("%s failed for '%s': %s", error_cause, ctx->device, strerror(saved_errno));

    return rv;
}

/*
 * Print for each character in buffer [XX] or <XX> with log level trace.
 */
int uart_dump_frame(bool is_sending, uint8_t *buffer, size_t len)
{
    const char *fmt = is_sending ? "[%02hhx]" : "<%02hhx>";
    char *msg = NULL;
    size_t i;

    msg = malloc(4 * len + 1);
    if (!msg)
        return -1;

    for (i = 0; i < len; ++i)
        sprintf(&msg[4 * i], fmt, buffer[i]);

    debug("%s: %s", is_sending ? "Sending" : "Received", msg);

    free(msg);
    return 0;
}

int uart_flush_input(struct uart_ctx *ctx)
{
    return tcflush(ctx->fd, TCIFLUSH);
}

int uart_wait_frame(struct uart_ctx *ctx, int timeout_ms)
{
    struct pollfd pollfd = { ctx->fd, POLLIN, 0 };
    int rv;

    /* use poll to handle the general response timeout */
    rv = poll(&pollfd, 1, timeout_ms);
    if (rv < 0) {
        debug("poll() failed: %m");
        return -1;
    }
    if (rv == 0) {
        debug("poll() timeout");
        errno = ETIMEDOUT;
        return -1;
    }

    return 0;
}

ssize_t uart_write_drain(struct uart_ctx *ctx, const uint8_t *buf, size_t count)
{
    size_t bytes_written = 0;
    int rv;

    while (bytes_written < count) {
        ssize_t c = write(ctx->fd, &buf[bytes_written], count - bytes_written);
        if (c < 0)
            return c;

        bytes_written += c;
    }

    rv = tcdrain(ctx->fd);
    if (rv)
        return rv;

    return bytes_written;
}

ssize_t uart_read_with_timeout(struct uart_ctx *ctx, uint8_t *buf, size_t count, int timeout_ms)
{
    struct timespec ts_timeout, ts_now;
    size_t bytes_read = 0;
    int rv;

    /* get current timestamp and calculate the timeout one */
    rv = clock_gettime(CLOCK_MONOTONIC, &ts_timeout);
    if (rv)
        return rv;

    timespec_add_ms(&ts_timeout, timeout_ms);

    while (bytes_read < count) {
        int remaining_timeout;
        ssize_t c;

        rv = clock_gettime(CLOCK_MONOTONIC, &ts_now);
        if (rv)
            return rv;

        remaining_timeout = timespec_to_ms(timespec_sub(ts_timeout, ts_now));
        if (remaining_timeout <= 0) {
            errno = ETIMEDOUT;
            return -1;
        }

        rv = uart_wait_frame(ctx, remaining_timeout);
        if (rv)
            return rv;

        c = read(ctx->fd, &buf[bytes_read], count - bytes_read);
        if (c < 0)
            return c;

        bytes_read += c;
    }

    return bytes_read;
}
