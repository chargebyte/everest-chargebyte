// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest
#pragma once

extern "C" {

#include <inttypes.h>

float extract_float(unsigned char* p);
uint16_t extract_uint16(unsigned char* p);
uint32_t extract_uint32(unsigned char* p);
void put_float(unsigned char* p, float f);
void put_uint32(unsigned char* p, uint32_t u);

}
