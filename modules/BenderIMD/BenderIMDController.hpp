// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest
#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <sigslot/signal.hpp>
#include <generated/interfaces/serial_communication_hub/Interface.hpp>
#include "BenderMeasurement.hpp"
#include "BenderIMDController.hpp"

using namespace std::chrono_literals;

///
/// A class for abstracting the Bender IMD's Modbus interface
///
class BenderIMDController {

public:
    /// @brief Default ctor.
    BenderIMDController();

    void init();

    void start_measurement();

    void stop_measurement();

    void trigger_self_test(double& test_voltage_V);

    void trigger_reset();

    std::function<types::serial_comm_hub_requests::Result(unsigned int addr, unsigned int count)>
        mb_read_holding_registers;
    std::function<types::serial_comm_hub_requests::StatusCodeEnum(unsigned int addr, int data)>
        mb_write_single_register;

    sigslot::signal<const float&, const float&> on_update;
    sigslot::signal<bool> on_selftest_finished;

private:
    std::string device_name;
    std::string sw_ident_number;
    std::string sw_date;
    std::string sw_version;
    std::string modbus_drv_version;

    /// @brief Remember the expected voltage in V which should be measured during selftest.
    double selftest_voltage;

    /// @brief Various Modbus register addresses and length, and some
    ///        magic constants needed to trigger actions.
    static constexpr unsigned int REG_ADDR_PREWARNING_R1 = 3005;
    static constexpr unsigned int REG_ADDR_ALARM_R2 = 3007;
    static constexpr unsigned int REG_ADDR_OPMODE_K1 = 3013;
    static constexpr unsigned int REG_ADDR_OPMODE_K2 = 3014;
    static constexpr unsigned int REG_ADDR_STARTUP_DELAY = 3018;
    static constexpr unsigned int REG_ADDR_T_ON_DELAY = 3019;
    static constexpr unsigned int REG_ADDR_T_OFF_DELAY = 3020;
    static constexpr unsigned int REG_ADDR_AUTO_SELFTEST_TIME = 3021;
    static constexpr unsigned int REG_ADDR_MONITORING_MODE = 3023;
    static constexpr unsigned int REG_ADDR_BOOTUP_SELFTEST = 3025;
    static constexpr unsigned int REG_ADDR_STOP_REQUEST = 3026;
    static constexpr unsigned int REG_ADDR_K1_ALARMS = 3027;
    static constexpr unsigned int REG_ADDR_K2_ALARMS = 3028;
    static constexpr unsigned int REG_ADDR_START_SELFTEST = 8005;
    static constexpr unsigned int MAGIC_START_SELFTEST = 0x5445; // = "TE"
    static constexpr unsigned int REG_ADDR_RESET = 8005;
    static constexpr unsigned int MAGIC_RESET = 0x433C; // = "CL"
    static constexpr unsigned int REG_ADDR_DEVICE_NAME = 9800;
    static constexpr unsigned int REG_COUNT_DEVICE_NAME = 10;
    static constexpr unsigned int REG_ADDR_SW_IDENT_NUMBER = 9820;
    static constexpr unsigned int REG_ADDR_SW_VERSION_NUMBER = 9821;
    static constexpr unsigned int REG_ADDR_SW_VERSION_YEAR = 9822;
    static constexpr unsigned int REG_ADDR_SW_VERSION_MONTH = 9823;
    static constexpr unsigned int REG_ADDR_SW_VERSION_DAY = 9824;
    static constexpr unsigned int REG_ADDR_MODBUS_DRV_VERSION = 9825;

    /// @brief List of start addresses of the valid measurement slots.
    ///        Each slots consists of 4 registers, i.e. 4 x 2 = 8 bytes
    inline static const std::vector<unsigned int> REG_ADDR_MEASUREMENT_SLOT = {1000, 1004, 1008, 1012, 1016,
                                                                               1020, 1024, 1028, 1032};
    static constexpr unsigned int REG_COUNT_MEASUREMENT_SLOT = 4;
};
