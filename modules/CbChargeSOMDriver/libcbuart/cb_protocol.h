/*
 * Copyright © 2024 chargebyte GmbH
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <time.h>
#include "uart.h"

/* the MCU is expected to answer requests within this time, in ms */
#define RESPONSE_TIMEOUT_MS 20

/* maximum count of PT1000 channels */
#define MAX_PT1000_CHANNELS 4

/* maximum emergency stop channels */
#define MAX_ESTOP_CHANNELS 3

/* holds the current MCU state */
struct safety_controller {
    /* data for CMD_DIGITAL_OUTPUT_01 */
    unsigned int duty_cycle;
    bool cp_rst_neg_peak_det;
    bool cp_rst_pos_peak_det;

    /* data from CMD_DIGITAL_INPUT_01 and/or CMD_DIGITAL_OUTPUT_01 */
    bool pp_sae_iec;
    bool p403;
    bool estopx[MAX_ESTOP_CHANNELS];
    bool p408;
    bool p407;
    bool ptx_en[MAX_PT1000_CHANNELS];
    bool hvsw3_precharge;
    bool sc_alive;
    bool hvsw1_hs;
    bool hvsw2_hs;
    bool motor_drv_fault;
    bool motor_drv_out2;
    bool motor_drv_out1;
    bool p107;
    bool p106;
    bool p104;
    bool sc_dev_2;
    bool sc_dev_1;
    bool cp_state_c;

    /* data from CMD_ANALOG_INPUT_01 */
    unsigned int ptx_vfb[MAX_PT1000_CHANNELS];

    /* data from CMD_ANALOG_INPUT_02 */
    unsigned int cp_neg_peak_det;
    unsigned int cp_pos_peak_det;
    unsigned int pp_value;
    unsigned int u_in;

    /* data from CMD_ANALOG_INPUT_03 */
    unsigned int precharge_cfb;
    unsigned int hs1_cfb;
    unsigned int hs2_cfb;
    unsigned int pt_1_2_cfb;

    /* data from CMD_ANALOG_INPUT_04 */
    unsigned int pt_3_4_cfb;
    unsigned int int_temp;
    unsigned int int_refvolt;
};

struct safety_ctx {
    /* the UART context */
    struct uart_ctx uart;

    /* the current/desired MCU state */
    struct safety_controller data;

    /* timestamp when next query is scheduled (CLOCK_MONOTONIC) */
    struct timespec ts_next_query;
};

int cb_single_run(struct safety_ctx *ctx);
void cb_dump_data(struct safety_controller *data);

#ifdef __cplusplus
}
#endif
