// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest

#include <cstdint>
#include <iosfwd>
#include <iostream>
#include <string>
#include <stdexcept>
#include <utility>
#include <everest/logging.hpp>
#include "BenderMeasurement.hpp"
#include "BenderIMDController.hpp"

BenderIMDController::BenderIMDController() {
}

void BenderIMDController::init() {
    types::serial_comm_hub_requests::Result r;

    r = this->mb_read_holding_registers()
}

void BenderIMDController::start_measurement() {
}

void BenderIMDController::stop_measurement() {
}

void BenderIMDController::trigger_self_test(double& test_voltage_V) {
    this->selftest_voltage = test_voltage_V;
}

void BenderIMDController::trigger_reset() {
}
