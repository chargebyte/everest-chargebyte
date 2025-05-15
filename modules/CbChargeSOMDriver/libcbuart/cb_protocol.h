/*
 * Copyright © 2024 chargebyte GmbH
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "uart.h"
#include "cb_uart.h"

/* the MCU is expected to answer requests via inquiry messages within this time, in ms */
#define RESPONSE_TIMEOUT_MS 20

/* the MCU expects Charge Control messages with this periodicity, in ms */
#define CHARGE_CONTROL_INTERVAL 100

/* the MCU sends Charge State messages with this periodicity, in ms */
#define CHARGE_STATE_INTERVAL 100

/* maximum count of PT1000 channels */
#define MAX_PT1000_CHANNELS 4

/* maximum emergency stop channels */
#define MAX_ESTOP_CHANNELS 3

/* possible CP states */
enum cp_state {
    CP_STATE_UNKNOWN = 0x0,
    CP_STATE_A,
    CP_STATE_B,
    CP_STATE_C,
    CP_STATE_D,
    CP_STATE_E,
    CP_STATE_F,
    CP_STATE_INVALID
};

/* CP related bit flags */
#define CP_SHORT_CIRCUIT 0x1
#define CP_DIODE_FAULT 0x2

/* possible PP states */
enum pp_state {
    PP_STATE_NO_CABLE,
    PP_STATE_13A,
    PP_STATE_20A,
    PP_STATE_32A,
    PP_STATE_63_70A,
    PP_STATE_TYPE1_CONNECTED,
    PP_STATE_TYPE1_CONNECTED_BUTTON_PRESSED,
    PP_STATE_INVALID,
};

/* PT1000 related bit flags */
#define PT1000_CHARGING_STOPPED 0x1
#define PT1000_SELFTEST_FAILED 0x2

/* magic value to indicate that this channel is not used */
#define PT1000_TEMPERATURE_UNUSED 0x1fff

/* IMD/RCM related bits */
#define IMD_RCM_STATE_TEST_FAILED 0x1
#define IMD_RCM_STATE_CHARGING_ABORT 0x2

/* buffer size for timestamp */
#define TS_STR_RECV_COM_BUFSIZE 32

/* holds the current MCU state */
struct safety_controller {
    /* latest message to send */
    uint64_t charge_control;

    /* the latest received messages */
    uint64_t charge_state;
    uint64_t pt1000;
    uint64_t fw_version;

    /* Git hash is special: usually handled as byte stream, so here stored
     * in wrong (host) byte order -> this is handled in `cb_proto_set_git_hash_str`
     */
    uint64_t git_hash;

    /* parsed fw version as string: major.minor.build;
     * maximum length: 3 * 3 chars + 2 dots + NUL = 12 byte; padded to 64bit => 16 byte
     */
    char fw_version_str[16];

    /* string representation of git hash;
     * 2 char a 8 byte + NUL = 17 byte; padded to 64 bit => 24 byte
     */
    char git_hash_str[24];

    /* plain text receive timestampes for each packet type */
    char ts_str_recv_com[COM_MAX][TS_STR_RECV_COM_BUFSIZE];
};

bool cb_proto_get_actual_pwm_active(struct safety_controller *ctx);
bool cb_proto_get_target_pwm_active(struct safety_controller *ctx);
void cb_proto_set_pwm_active(struct safety_controller *ctx, bool active);

unsigned int cb_proto_get_actual_duty_cycle(struct safety_controller *ctx);
unsigned int cb_proto_get_target_duty_cycle(struct safety_controller *ctx);
void cb_proto_set_duty_cycle(struct safety_controller *ctx, unsigned int duty_cycle);

bool cb_proto_get_actual_contactor_state(struct safety_controller *ctx, unsigned int contactor);
bool cb_proto_get_target_contactor_state(struct safety_controller *ctx, unsigned int contactor);
void cb_proto_set_contactor_state(struct safety_controller *ctx, unsigned int contactor, bool active);

bool cb_proto_has_contactor_errors(struct safety_controller *ctx);
bool cb_proto_has_contactorN_error(struct safety_controller *ctx, unsigned int contactor);

enum cp_state cb_proto_get_cp_state(struct safety_controller *ctx);
unsigned int cb_proto_get_cp_errors(struct safety_controller *ctx);
bool cb_proto_is_cp_short_circuit(struct safety_controller *ctx);
bool cb_proto_is_diode_fault(struct safety_controller *ctx);

enum pp_state cb_proto_get_pp_state(struct safety_controller *ctx);

bool cb_proto_has_estop_tripped(struct safety_controller *ctx);
bool cb_proto_has_estopX_tripped(struct safety_controller *ctx, unsigned int estop);

unsigned int cb_proto_get_imd_rcm_errors(struct safety_controller *ctx);

bool cb_proto_pt1000_is_active(struct safety_controller *ctx, unsigned int channel);
double cb_proto_pt1000_get_temp(struct safety_controller *ctx, unsigned int channel);
unsigned int cb_proto_pt1000_get_errors(struct safety_controller *ctx, unsigned int channel);

/* possible firmware platform types */
enum fw_platform_type {
    FW_PLATFORM_TYPE_UNSPECIFIED = 0xff,
    FW_PLATFORM_TYPE_UNKNOWN = 0x00,
    FW_PLATFORM_TYPE_CHARGESOM = 0x81,
    FW_PLATFORM_TYPE_CCY = 0x82,
};

enum fw_application_type {
    FW_APPLICATION_TYPE_FW = 0x3,
    FW_APPLICATION_TYPE_EOL = 0x4,
    FW_APPLICATION_TYPE_QUALIFICATION = 0x5,
};

unsigned int cb_proto_fw_get_major(struct safety_controller *ctx);
unsigned int cb_proto_fw_get_minor(struct safety_controller *ctx);
unsigned int cb_proto_fw_get_build(struct safety_controller *ctx);
enum fw_platform_type cb_proto_fw_get_platform_type(struct safety_controller *ctx);
enum fw_application_type cb_proto_fw_get_application_type(struct safety_controller *ctx);

void cb_proto_set_fw_version_str(struct safety_controller *ctx);
void cb_proto_set_git_hash_str(struct safety_controller *ctx);

/* helpers */
const char *cb_proto_cp_state_to_str(enum cp_state state);
const char *cb_proto_pp_state_to_str(enum cp_state state);
const char *cb_proto_fw_platform_type_to_str(enum fw_platform_type type);
const char *cb_proto_fw_application_type_to_str(enum fw_application_type type);

int cb_proto_set_ts_str(struct safety_controller *ctx, uint8_t com);

void cb_proto_dump(struct safety_controller *ctx);

/* low-level helpers */
int cb_send_uart_inquiry(struct uart_ctx *uart, uint8_t com);


#ifdef __cplusplus
}
#endif
