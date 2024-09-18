/*
 * Copyright © 2024 chargebyte GmbH
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

uint8_t crc8_j1850(uint8_t *p, size_t len);

#ifdef __cplusplus
}
#endif
