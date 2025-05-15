/*
 * Copyright © 2024 chargebyte GmbH
 * SPDX-License-Identifier: Apache-2.0
 */
#include <sys/time.h>
#include <endian.h>
#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include "uart.h"
#include "cb_uart.h"
#include "cb_protocol.h"
#include "logging.h"

#define BITMASK(len) \
    ((1 << (len)) - 1)

#define DATA_SET_BITS(dst, bit, len, datasource) \
    (dst) &= ~((uint64_t)BITMASK(len) << (bit)); \
    (dst) |= ((uint64_t)(datasource) & BITMASK(len)) << (bit)

#define DATA_GET_BITS(src, bit, len) \
    (((src) >> (bit)) & BITMASK(len))

int cb_send_uart_inquiry(struct uart_ctx *uart, uint8_t com)
{
    uint64_t data = 0;

    DATA_SET_BITS(data, 56, 8, com);

    return cb_uart_send(uart, COM_INQUIRY, data);
}

bool cb_proto_get_actual_pwm_active(struct safety_controller *ctx)
{
    return DATA_GET_BITS(ctx->charge_state, 63, 1);
}

bool cb_proto_get_target_pwm_active(struct safety_controller *ctx)
{
    return DATA_GET_BITS(ctx->charge_control, 63, 1);
}

void cb_proto_set_pwm_active(struct safety_controller *ctx, bool active)
{
    DATA_SET_BITS(ctx->charge_control, 63, 1, active);
}

unsigned int cb_proto_get_actual_duty_cycle(struct safety_controller *ctx)
{
    return DATA_GET_BITS(ctx->charge_state, 48, 10);
}

unsigned int cb_proto_get_target_duty_cycle(struct safety_controller *ctx)
{
    return DATA_GET_BITS(ctx->charge_control, 48, 10);
}

void cb_proto_set_duty_cycle(struct safety_controller *ctx, unsigned int duty_cycle)
{
    DATA_SET_BITS(ctx->charge_control, 48, 10, duty_cycle);
}

bool cb_proto_get_actual_contactor_state(struct safety_controller *ctx, unsigned int contactor)
{
    return DATA_GET_BITS(ctx->charge_state, 24 + contactor, 1);
}

bool cb_proto_get_target_contactor_state(struct safety_controller *ctx, unsigned int contactor)
{
    return DATA_GET_BITS(ctx->charge_control, 40 + contactor, 1);
}

void cb_proto_set_contactor_state(struct safety_controller *ctx, unsigned int contactor, bool active)
{
    DATA_SET_BITS(ctx->charge_control, 40 + contactor, 1, active);
}

bool cb_proto_has_contactor_errors(struct safety_controller *ctx)
{
    return DATA_GET_BITS(ctx->charge_state, 26, 2);
}

bool cb_proto_has_contactorN_error(struct safety_controller *ctx, unsigned int contactor)
{
    return DATA_GET_BITS(ctx->charge_state, 26 + contactor, 1);
}

enum cp_state cb_proto_get_cp_state(struct safety_controller *ctx)
{
    return DATA_GET_BITS(ctx->charge_state, 40, 3);
}

unsigned int cb_proto_get_cp_errors(struct safety_controller *ctx)
{
    return DATA_GET_BITS(ctx->charge_state, 43, 2);
}

bool cb_proto_is_cp_short_circuit(struct safety_controller *ctx)
{
    return cb_proto_get_cp_errors(ctx) & CP_SHORT_CIRCUIT;
}

bool cb_proto_is_diode_fault(struct safety_controller *ctx)
{
    return cb_proto_get_cp_errors(ctx) & CP_DIODE_FAULT;
}

enum pp_state cb_proto_get_pp_state(struct safety_controller *ctx)
{
    return DATA_GET_BITS(ctx->charge_state, 32, 3);
}

bool cb_proto_has_estop_tripped(struct safety_controller *ctx)
{
    return DATA_GET_BITS(ctx->charge_state, 16, 3);
}

bool cb_proto_has_estopX_tripped(struct safety_controller *ctx, unsigned int estop)
{
    return DATA_GET_BITS(ctx->charge_state, 16, 3) & (1 << estop);
}

unsigned int cb_proto_get_imd_rcm_errors(struct safety_controller *ctx)
{
    return DATA_GET_BITS(ctx->charge_state, 19, 2);
}

bool cb_proto_pt1000_is_active(struct safety_controller *ctx, unsigned int channel)
{
    return DATA_GET_BITS(ctx->pt1000, 16 * (MAX_PT1000_CHANNELS - 1 - channel) + 2, 14) != PT1000_TEMPERATURE_UNUSED;
}

double cb_proto_pt1000_get_temp(struct safety_controller *ctx, unsigned int channel)
{
    /* we access the whole 16 bit so that we shift correctly */
    int16_t d = DATA_GET_BITS(ctx->pt1000, 16 * (MAX_PT1000_CHANNELS - 1 - channel), 16);

    return (double)(d >> 2) / 10.0;
}

unsigned int cb_proto_pt1000_get_errors(struct safety_controller *ctx, unsigned int channel)
{
    return DATA_GET_BITS(ctx->pt1000, 16 * (MAX_PT1000_CHANNELS - 1 - channel), 2);
}

unsigned int cb_proto_fw_get_major(struct safety_controller *ctx)
{
    return DATA_GET_BITS(ctx->fw_version, 56, 8);
}

unsigned int cb_proto_fw_get_minor(struct safety_controller *ctx)
{
    return DATA_GET_BITS(ctx->fw_version, 48, 8);
}

unsigned int cb_proto_fw_get_build(struct safety_controller *ctx)
{
    return DATA_GET_BITS(ctx->fw_version, 40, 8);
}

enum fw_platform_type cb_proto_fw_get_platform_type(struct safety_controller *ctx)
{
    return DATA_GET_BITS(ctx->fw_version, 32, 8);
}

enum fw_application_type cb_proto_fw_get_application_type(struct safety_controller *ctx)
{
    return DATA_GET_BITS(ctx->fw_version, 24, 8);
}

void cb_proto_set_fw_version_str(struct safety_controller *ctx)
{
    snprintf(ctx->fw_version_str, sizeof(ctx->fw_version_str), "%u.%u.%u",
             cb_proto_fw_get_major(ctx),
             cb_proto_fw_get_minor(ctx),
             cb_proto_fw_get_build(ctx));
}

void cb_proto_set_git_hash_str(struct safety_controller *ctx)
{
    uint8_t *p = (uint8_t *)&ctx->git_hash + sizeof(ctx->git_hash) - 1;
    char *s = &ctx->git_hash_str[0];
    unsigned int i;

    for (i = 0; i < sizeof(ctx->git_hash); ++i) {
        sprintf(s, "%02" PRIx8, *p);
        p--;
        s += 2;
    }

    *s = '\0';
}


const char *cb_proto_cp_state_to_str(enum cp_state state)
{
    switch (state) {
    case CP_STATE_UNKNOWN:
        return "unknown";
    case CP_STATE_A:
        return "A";
    case CP_STATE_B:
        return "B";
    case CP_STATE_C:
        return "C";
    case CP_STATE_D:
        return "D";
    case CP_STATE_E:
        return "E";
    case CP_STATE_F:
        return "F";
    case CP_STATE_INVALID:
        return "invalid";
    default:
        return "undefined";
    }
}

const char *cb_proto_pp_state_to_str(enum cp_state state)
{
    switch (state) {
    case PP_STATE_NO_CABLE:
        return "no cable detected";
    case PP_STATE_13A:
        return "13 A";
    case PP_STATE_20A:
        return "20 A";
    case PP_STATE_32A:
        return "32 A";
    case PP_STATE_63_70A:
        return "63/70 A";
    case PP_STATE_TYPE1_CONNECTED:
        return "connected";
    case PP_STATE_TYPE1_CONNECTED_BUTTON_PRESSED:
        return "connected, button pressed";
    case PP_STATE_INVALID:
        return "invalid";
    default:
        return "undefined";
    }
}

const char *cb_proto_fw_platform_type_to_str(enum fw_platform_type type)
{
    switch (type) {
    case FW_PLATFORM_TYPE_UNSPECIFIED:
        return "unspecified";
    case FW_PLATFORM_TYPE_UNKNOWN:
        return "unknown";
    case FW_PLATFORM_TYPE_CHARGESOM:
        return "Charge SOM";
    case FW_PLATFORM_TYPE_CCY:
        return "CC Y";
    default:
        return "unknown value";
    }
}

const char *cb_proto_fw_application_type_to_str(enum fw_application_type type)
{
    switch (type) {
    case FW_APPLICATION_TYPE_FW:
        return "firmware";
    case FW_APPLICATION_TYPE_EOL:
        return "eol";
    case FW_APPLICATION_TYPE_QUALIFICATION:
        return "qualification";
    default:
        return "unknown";
    }
}

int cb_proto_set_ts_str(struct safety_controller *ctx, uint8_t com)
{
    char *buffer = ctx->ts_str_recv_com[com];
    struct timeval tv;
    struct tm tm;
    size_t offset;
    int rv;

    if (gettimeofday(&tv, NULL) < 0) {
         error("gettimeofday() failed: %m");
         return -1;
    }

    if (localtime_r(&tv.tv_sec, &tm) == NULL) {
        error("localtime_r() failed: %m");
        return -1;
    }

    offset = strftime(buffer, TS_STR_RECV_COM_BUFSIZE, "%Y-%m-%d %H:%M:%S", &tm);
    if (offset < 1) {
        error("strftime() failed");
        return -1;
    }

    rv = snprintf(&buffer[offset], TS_STR_RECV_COM_BUFSIZE - offset, ".%03ld", tv.tv_usec / 1000);
    if (rv < 0) {
        error("strftime() failed");
        return -1;
    }

    return rv;
}

#define printfnl(fmt, ...) \
        do { \
            printf(fmt "\r\n", ##__VA_ARGS__); \
        } while (0)

#define THIS_BIT_AND_ANY_OF_THE_LOWER(src, bit) \
    (((src) & (bit)) && ((src) & ((bit) - 1)))

void cb_proto_dump(struct safety_controller *ctx)
{
    unsigned int i;

    printfnl("== Various ==");
    printfnl("Control Pilot:   %s (%s%s%s%s)", cb_proto_cp_state_to_str(cb_proto_get_cp_state(ctx)),
             cb_proto_get_cp_errors(ctx) ? "" : "-no flags set-",
             (cb_proto_get_cp_errors(ctx) & CP_DIODE_FAULT) ? "diode fault" : "",
             THIS_BIT_AND_ANY_OF_THE_LOWER(cb_proto_get_cp_errors(ctx), CP_DIODE_FAULT) ? "," : "",
             (cb_proto_get_cp_errors(ctx) & CP_SHORT_CIRCUIT) ? "short circuit" : "");

    printfnl("Proximity Pilot: %s", cb_proto_pp_state_to_str(cb_proto_get_pp_state(ctx)));

    printfnl("Emergency Stop Tripped: ESTOP1=%-3s  ESTOP2=%-3s  ESTOP2=%-3s",
             cb_proto_has_estopX_tripped(ctx, 0) ? "yes" : "no",
             cb_proto_has_estopX_tripped(ctx, 1) ? "yes" : "no",
             cb_proto_has_estopX_tripped(ctx, 2) ? "yes" : "no");

    printfnl("IMD/RCM State: 0x%02x (%s%s%s%s)", cb_proto_get_imd_rcm_errors(ctx),
             cb_proto_get_imd_rcm_errors(ctx) ? "" : "-no flags set-",
             (cb_proto_get_imd_rcm_errors(ctx) & IMD_RCM_STATE_CHARGING_ABORT) ? "charging abort" : "",
             THIS_BIT_AND_ANY_OF_THE_LOWER(cb_proto_get_imd_rcm_errors(ctx), IMD_RCM_STATE_CHARGING_ABORT) ? "," : "",
             (cb_proto_get_imd_rcm_errors(ctx) & IMD_RCM_STATE_TEST_FAILED) ? "test failed" : "");

    printfnl("");
    printfnl("== PWM ==");
    printfnl("Enable:               %-3s      Is Enabled:         %-3s",
             cb_proto_get_target_pwm_active(ctx)? "yes" : "no",
             cb_proto_get_actual_pwm_active(ctx) ? "yes" : "no");
    printfnl("Requested Duty Cycle: %5.1f%%   Current Duty Cycle: %5.1f%%",
             cb_proto_get_target_duty_cycle(ctx) / 10.0,
             cb_proto_get_actual_duty_cycle(ctx) / 10.0);

    printfnl("");
    printfnl("== Contactor ==");
    printfnl("Contactor 1: requested=%-5s   actual=%-6s   %s",
             cb_proto_get_target_contactor_state(ctx, 0) ? "CLOSE" : "open",
             cb_proto_get_actual_contactor_state(ctx, 0) ? "CLOSED" : "open",
             cb_proto_has_contactorN_error(ctx, 0) ? "ERROR" : "no error");
    printfnl("Contactor 2: requested=%-5s   actual=%-6s   %s",
              cb_proto_get_target_contactor_state(ctx, 1) ? "CLOSE" : "open",
              cb_proto_get_actual_contactor_state(ctx, 1) ? "CLOSED" : "open",
              cb_proto_has_contactorN_error(ctx, 1) ? "ERROR" : "no error");

    printfnl("");
    printfnl("== Temperatures ==");
    for (i = 0; i < MAX_PT1000_CHANNELS; ++i) {
        bool is_enabled = cb_proto_pt1000_is_active(ctx, i);

        printf("Channel %d: enabled=%-3s temperature=", i + 1, is_enabled ? "yes" : "no");
        if (is_enabled)
            printf("%5.1f °C", cb_proto_pt1000_get_temp(ctx, i));
        else
            printf("-n/a- °C");
        printfnl(" (%s%s%s%s)",
                 cb_proto_pt1000_get_errors(ctx, i) ? "" : "-no flags set-",
                 (cb_proto_pt1000_get_errors(ctx, i) & PT1000_SELFTEST_FAILED) ? "selftest failed" : "",
                 THIS_BIT_AND_ANY_OF_THE_LOWER(cb_proto_pt1000_get_errors(ctx, i), PT1000_SELFTEST_FAILED) ? "," : "",
                 (cb_proto_pt1000_get_errors(ctx, i) & PT1000_CHARGING_STOPPED) ? "charging stop cause" : "");
    }

    printfnl("");
    printfnl("== Firmware Info ==");
    printfnl("Version: %s (%s, %s)",
           ctx->fw_version ? ctx->fw_version_str : "unknown",
           cb_proto_fw_platform_type_to_str(cb_proto_fw_get_platform_type(ctx)),
           cb_proto_fw_application_type_to_str(cb_proto_fw_get_application_type(ctx)));
    printfnl("Git Hash: %s", ctx->git_hash ? ctx->git_hash_str : "unknown");

    printfnl("");
    printfnl("== Timestamps ==");
    for (i = 0; i < COM_MAX; ++i) {
        if (strlen(ctx->ts_str_recv_com[i]))
            printfnl("%-20s: %s", cb_uart_com_to_str(i), ctx->ts_str_recv_com[i]);
    }
}
