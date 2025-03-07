// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest

#include <endian.h>
#include <inttypes.h>

float extract_float(unsigned char* p) {
    union {
        uint32_t u32;
        float f;
    } u;

    u.u32 = be32toh(*(uint32_t*)p);
    return u.f;
}

uint16_t extract_uint16(unsigned char* p) {
    return be16toh(*(uint16_t*)p);
}
