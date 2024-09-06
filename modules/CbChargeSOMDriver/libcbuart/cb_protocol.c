/*
 * Copyright © 2024 chargebyte GmbH
 * SPDX-License-Identifier: Apache-2.0
 */
#include <endian.h>
#include <errno.h>
#include <inttypes.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include "uart.h"
#include "tools.h"
#include "logging.h"
#include "crc8_j1850.h"
#include "cb_protocol.h"

/* frame start/end markers */
#define CB_SOF 0xA5
#define CB_EOF 0x03

/* values for COM fields */
#define COM_DIGITAL_OUTPUT_01 0x00
#define COM_DIGITAL_INPUT_01 0x01
#define COM_ANALOG_INPUT_01 0x02
#define COM_ANALOG_INPUT_02 0x03
#define COM_ANALOG_INPUT_03 0x04
#define COM_ANALOG_INPUT_04 0x05

/* data packets have fixed size */
struct inquiry_pkt {
    uint8_t sof;
    uint8_t com;
    uint8_t crc;
    uint8_t eof;
} __attribute__((packed));

#define DATA_PKT_PAYLOAD_LENGTH 8

struct data_pkt {
    uint8_t sof;
    uint8_t com;
    union {
        uint8_t data8[DATA_PKT_PAYLOAD_LENGTH / sizeof(uint8_t)];
        uint16_t data16[DATA_PKT_PAYLOAD_LENGTH / sizeof(uint16_t)];
    } __attribute__((packed));
    uint8_t crc;
    uint8_t eof;
} __attribute__((packed));

static int cb_request_data(struct uart_ctx *uart, uint8_t com, struct data_pkt *response)
{
    struct inquiry_pkt request;
    uint8_t crc;
    ssize_t c;

    c = uart_flush_input(uart);
    if (c < 0)
        return c;

    /* prepare inquiry packet */
    memset(&request, 0, sizeof(request));
    request.sof = CB_SOF;
    request.com = com;
    request.crc = crc8_j1850(&request.com, 1);
    request.eof = CB_EOF;

    debug("sending inquiry packet");

    c = uart_write_drain(uart, (const uint8_t *)&request, sizeof(request));
    if (c < 0)
        return c;

    debug("waiting for response");

    c = uart_read_with_timeout(uart, (uint8_t *)response, sizeof(*response), RESPONSE_TIMEOUT_MS);
    if (c < 0)
        return c;

    debug("received response");
    //uart_dump_frame(false, (uint8_t *)response, sizeof(*response));

    /* check field patterns */
    if (response->sof != CB_SOF) {
        error("SOF pattern mismatch: expected 0x%02x, got 0x%02" PRIx8, CB_SOF, response->sof);
        errno = EBADMSG;
        return -1;
    }
    if (response->eof != CB_EOF) {
        error("EOF pattern mismatch: expected 0x%02x, got 0x%02" PRIx8, CB_EOF, response->eof);
        errno = EBADMSG;
        return -1;
    }

    /* check crc */
    crc = crc8_j1850(&response->com, sizeof(response->com) + sizeof(response->data8));
    if (crc != response->crc) {
        error("CRC pattern mismatch: expected 0x%02x, got 0x%02" PRIx8, crc, response->crc);
        errno = EBADMSG;
        return -1;
    }

    debug("received response looks valid (SOF, EOF, CRC)");

    return 0;
}

static int cb_wait_throttle_timeout(struct safety_ctx *ctx)
{
    int rv;

    /* only wait when the timestamp was set at all */
    if (timespec_is_set(&ctx->ts_next_query)) {
        /* sleep until the MCU accepts the next request */
        rv = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ctx->ts_next_query, NULL);
        if (rv)
            return rv;
    }

    /* calculate the next timestamp... */
    rv = clock_gettime(CLOCK_MONOTONIC, &ctx->ts_next_query);
    if (rv)
        return rv;

    /* ...we must only add half of the timeout since this is our query interval */
    timespec_add_ms(&ctx->ts_next_query, RESPONSE_TIMEOUT_MS / 2);

    return 0;
}

#define DATA_PKT_SET_BIT(offset, bit, datasource) \
    request.data8[(offset)] |= (datasource) << (bit)

static int cb_send_do(struct uart_ctx *uart, struct safety_controller *data)
{
    struct data_pkt request;
    ssize_t c;
    int i;

    /* prepare packet */
    memset(&request, 0, sizeof(request));
    request.sof = CB_SOF;
    request.com = COM_DIGITAL_OUTPUT_01;

    for (i = 0; i < MAX_PT1000_CHANNELS; i++)
        DATA_PKT_SET_BIT(0, i, data->ptx_en[i]);

    DATA_PKT_SET_BIT(0, 4, data->hvsw3_precharge);
    DATA_PKT_SET_BIT(0, 5, data->sc_alive);
    DATA_PKT_SET_BIT(0, 6, data->hvsw1_hs);
    DATA_PKT_SET_BIT(0, 7, data->hvsw2_hs);

    DATA_PKT_SET_BIT(1, 0, data->motor_drv_out2);
    DATA_PKT_SET_BIT(1, 1, data->motor_drv_out1);
    DATA_PKT_SET_BIT(1, 2, data->cp_rst_pos_peak_det);
    DATA_PKT_SET_BIT(1, 3, data->cp_rst_neg_peak_det);
    DATA_PKT_SET_BIT(1, 4, data->cp_state_c);
    DATA_PKT_SET_BIT(1, 5, data->pp_sae_iec);

    request.data8[2] = data->duty_cycle;

    request.crc = crc8_j1850(&request.com, sizeof(request.com) + sizeof(request.data8));
    request.eof = CB_EOF;

    debug("sending packet");

    c = uart_write_drain(uart, (const uint8_t *)&request, sizeof(request));
    if (c < 0)
        return c;

    return 0;
}

#define DATA_PKT_GET_BIT(offset, bit) \
    ((response.data8[(offset)] >> (bit)) & 1)

int cb_single_run(struct safety_ctx *ctx)
{
    struct safety_controller *data = &ctx->data;
    struct uart_ctx *uart = &ctx->uart;
    struct data_pkt response;
    int i, rv;

    rv = cb_wait_throttle_timeout(ctx);
    if (rv)
        return rv;

    debug("sending COM_DIGITAL_OUTPUT_01");

    rv = cb_send_do(uart, data);
    if (rv)
        return rv;

    rv = cb_wait_throttle_timeout(ctx);
    if (rv)
        return rv;

    debug("sending COM_DIGITAL_INPUT_01");

    rv = cb_request_data(uart, COM_DIGITAL_INPUT_01, &response);
    if (rv)
        return rv;

    data->pp_sae_iec =      DATA_PKT_GET_BIT(0, 0);
    data->p403 =            DATA_PKT_GET_BIT(0, 1);
    for (i = 0; i < MAX_ESTOP_CHANNELS; i++)
        data->estopx[i] =   DATA_PKT_GET_BIT(0, 2 + i);
    data->p408 =            DATA_PKT_GET_BIT(0, 5);
    data->p407 =            DATA_PKT_GET_BIT(0, 6);
    data->ptx_en[0] =       DATA_PKT_GET_BIT(0, 7);
    data->ptx_en[1] =       DATA_PKT_GET_BIT(1, 0);
    data->ptx_en[2] =       DATA_PKT_GET_BIT(1, 1);
    data->ptx_en[3] =       DATA_PKT_GET_BIT(1, 2);
    data->hvsw3_precharge = DATA_PKT_GET_BIT(1, 3);
    data->sc_alive =        DATA_PKT_GET_BIT(1, 4);
    data->hvsw1_hs =        DATA_PKT_GET_BIT(1, 5);
    data->hvsw2_hs =        DATA_PKT_GET_BIT(1, 6);
    data->motor_drv_fault = DATA_PKT_GET_BIT(1, 7);

    data->motor_drv_out2 =  DATA_PKT_GET_BIT(2, 0);
    data->motor_drv_out1 =  DATA_PKT_GET_BIT(2, 1);
    data->p107 =            DATA_PKT_GET_BIT(2, 2);
    data->p106 =            DATA_PKT_GET_BIT(2, 3);
    data->p104 =            DATA_PKT_GET_BIT(2, 4);
    data->sc_dev_2 =        DATA_PKT_GET_BIT(2, 5);
    data->sc_dev_1 =        DATA_PKT_GET_BIT(2, 6);
    data->cp_state_c =      DATA_PKT_GET_BIT(2, 7);

    rv = cb_wait_throttle_timeout(ctx);
    if (rv)
        return rv;

    debug("sending COM_ANALOG_INPUT_01");

    rv = cb_request_data(uart, COM_ANALOG_INPUT_01, &response);
    if (rv)
        return rv;

    for (i = 0; i < MAX_PT1000_CHANNELS; i++)
        data->ptx_vfb[i] = le16toh(response.data16[i]);

    rv = cb_wait_throttle_timeout(ctx);
    if (rv)
        return rv;

    debug("sending COM_ANALOG_INPUT_02");

    rv = cb_request_data(uart, COM_ANALOG_INPUT_02, &response);
    if (rv)
        return rv;

    data->cp_neg_peak_det = le16toh(response.data16[0]);
    data->cp_pos_peak_det = le16toh(response.data16[1]);
    data->pp_value = le16toh(response.data16[2]);
    data->u_in = le16toh(response.data16[3]);

    rv = cb_wait_throttle_timeout(ctx);
    if (rv)
        return rv;

    debug("sending COM_ANALOG_INPUT_03");

    rv = cb_request_data(uart, COM_ANALOG_INPUT_03, &response);
    if (rv)
        return rv;

    data->precharge_cfb = le16toh(response.data16[0]);
    data->hs1_cfb = le16toh(response.data16[1]);
    data->hs2_cfb = le16toh(response.data16[2]);
    data->pt_1_2_cfb = le16toh(response.data16[3]);

    rv = cb_wait_throttle_timeout(ctx);
    if (rv)
        return rv;

    debug("sending COM_ANALOG_INPUT_04");

    rv = cb_request_data(uart, COM_ANALOG_INPUT_04, &response);
    if (rv)
        return rv;

    data->pt_3_4_cfb = le16toh(response.data16[0]);
    data->int_temp = le16toh(response.data16[1]);
    data->int_refvolt = le16toh(response.data16[2]);
    // response.data16[3] is reserved

    return 0;
}

void cb_dump_data(struct safety_controller *data)
{
    float neg_cp, pos_cp;
    int i;

    printf("COM_DIGITAL_INPUT_01:\n");
    printf("\tDuty Cycle: %u\n", data->duty_cycle);
    printf("\tCP Reset Peak Detector (Neg/Pos): %u/%u\n", data->cp_rst_neg_peak_det, data->cp_rst_pos_peak_det);
    printf("\tPP SAE IEC: %u\n", data->pp_sae_iec);
    printf("\tESTOP1: %u    ESTOP1: %u    ESTOP1: %u\n", data->estopx[0], data->estopx[1], data->estopx[2]);
    printf("\tPT1_EN: %u    PT2_EN: %u    PT3_EN: %u    PT3_EN: %u\n", data->ptx_en[0], data->ptx_en[1], data->ptx_en[2], data->ptx_en[3]);
    printf("\tP104: %u    P106: %u    P107: %u\n", data->p104, data->p106, data->p107);
    printf("\tP403: %u    P407: %u    P407: %u\n", data->p403, data->p407, data->p408);
    printf("\tHVSW1_HS:        %u    HVSW2_HS:       %u    SC_ALIVE:       %u\n", data->hvsw1_hs, data->hvsw2_hs, data->sc_alive);
    printf("\tMOTOR_DRV_FAULT: %u    MOTOR_DRV_OUT1: %u    MOTOR_DRV_OUT1: %u\n", data->motor_drv_fault, data->motor_drv_out1, data->motor_drv_out2);
    printf("\tSC_DEV_1:        %u    SC_DEV_2:       %u    CP_STATE_C:     %u\n", data->sc_dev_1, data->sc_dev_2, data->cp_state_c);

    printf("COM_ANALOG_INPUT_01:\n");
    for (i = 0; i < MAX_PT1000_CHANNELS; i++)
        printf("\tPT%d_VFB: %u\n", i + 1, data->ptx_vfb[i]);

    printf("COM_ANALOG_INPUT_02:\n");
    neg_cp = (3.3 * data->cp_neg_peak_det * 150.0) / (4096 * -36.0);
    pos_cp = (3.3 * data->cp_pos_peak_det * (47.0 + 150.0)) / (4096 * 47.0);
    printf("\tCP_NEG_PEAK_DET: %-5u    %+8.3f V\n", data->cp_neg_peak_det, neg_cp);
    printf("\tCP_POS_PEAK_DET: %-5u    %+8.3f V\n", data->cp_pos_peak_det, pos_cp);
    printf("\tPP_VALUE: %u\n", data->pp_value);
    printf("\tU_in: %u\n", data->u_in);

    printf("COM_ANALOG_INPUT_03:\n");
    printf("\tPRECHARGE_CFB: %u\n", data->precharge_cfb);
    printf("\tHS1_CFB: %u\n", data->hs1_cfb);
    printf("\tHS2_CFB: %u\n", data->hs2_cfb);
    printf("\tPT_1_2_CFB: %u\n", data->pt_1_2_cfb);

    printf("COM_ANALOG_INPUT_04:\n");
    printf("\tPT_3_4_CFB: %u\n", data->pt_3_4_cfb);
    printf("\tINT_TEMP: %u\n", data->int_temp);
    printf("\tINT_REFVOLT: %u\n", data->int_refvolt);
}
