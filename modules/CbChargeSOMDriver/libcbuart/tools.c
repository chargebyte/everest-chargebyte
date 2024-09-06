/*
 * Copyright © 2024 chargebyte GmbH
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdbool.h>
#include <stdint.h>
#include <time.h>
#include "tools.h"

#define NSEC_PER_SEC 1000000000L

bool timespec_is_set(const struct timespec *ts)
{
    return ts->tv_sec || ts->tv_nsec;
}

void set_normalized_timespec(struct timespec *ts, time_t sec, int64_t nsec)
{
    while (nsec >= NSEC_PER_SEC) {
        nsec -= NSEC_PER_SEC;
        ++sec;
    }
    while (nsec < 0) {
        nsec += NSEC_PER_SEC;
        --sec;
    }
    ts->tv_sec = sec;
    ts->tv_nsec = nsec;
}

struct timespec timespec_add(struct timespec lhs, struct timespec rhs)
{
    struct timespec ts_delta;

    set_normalized_timespec(&ts_delta, lhs.tv_sec + rhs.tv_sec, lhs.tv_nsec + rhs.tv_nsec);

    return ts_delta;
}

struct timespec timespec_sub(struct timespec lhs, struct timespec rhs)
{
    struct timespec ts_delta;

    set_normalized_timespec(&ts_delta, lhs.tv_sec - rhs.tv_sec, lhs.tv_nsec - rhs.tv_nsec);

    return ts_delta;
}

void timespec_add_ms(struct timespec *ts, long long msec)
{
    long long sec = msec / 1000;

    set_normalized_timespec(ts, ts->tv_sec + sec, ts->tv_nsec + (msec - sec * 1000) * 1000 * 1000);
}

/*
 * lhs < rhs:  return < 0
 * lhs == rhs: return 0
 * lhs > rhs:  return > 0
 */
int timespec_compare(const struct timespec *lhs, const struct timespec *rhs)
{
    if (lhs->tv_sec < rhs->tv_sec)
        return -1;
    if (lhs->tv_sec > rhs->tv_sec)
        return 1;
    return lhs->tv_nsec - rhs->tv_nsec;
}

long long timespec_to_ms(struct timespec ts)
{
    return ((long long)ts.tv_sec * 1000) + (ts.tv_nsec / 1000000);
}

long long timespec_to_us(struct timespec ts)
{
    return ((long long)ts.tv_sec * 1000000) + (ts.tv_nsec / 1000);
}
