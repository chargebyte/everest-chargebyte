/*
 * Copyright © 2024 chargebyte GmbH
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdio.h>
#include <termios.h>
#include <time.h>

#define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x)          (sizeof(x) / sizeof((x)[0]))
#endif

#ifndef ROUND_UP
#define ROUND_UP(N, S) ((((N) + (S) - 1) / (S)) * (S))
#endif

bool timespec_is_set(const struct timespec *ts);
void set_normalized_timespec(struct timespec *ts, time_t sec, int64_t nsec);
struct timespec timespec_add(struct timespec lhs, struct timespec rhs);
struct timespec timespec_sub(struct timespec lhs, struct timespec rhs);
void timespec_add_ms(struct timespec *ts, long long msec);
int timespec_compare(const struct timespec *lhs, const struct timespec *rhs);
long long timespec_to_ms(struct timespec ts);
long long timespec_to_us(struct timespec ts);

#ifdef __cplusplus
}
#endif
