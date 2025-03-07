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

uint32_t extract_uint32(unsigned char* p) {
    return be32toh(*(uint32_t*)p);
}

void put_float(unsigned char* p, float f) {
    union {
        float f;
        uint32_t u32;
    } u;

    u.f = f;
    *(uint32_t *)p = htobe32(u.u32);
}

void put_uint32(unsigned char* p, uint32_t u) {
    *(uint32_t *)p = htobe32(u);
}
