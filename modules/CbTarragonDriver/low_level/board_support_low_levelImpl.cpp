// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest

#include "board_support_low_levelImpl.hpp"

namespace module {
namespace low_level {

void board_support_low_levelImpl::init() {
}

void board_support_low_levelImpl::ready() {
}

double board_support_low_levelImpl::handle_get_grid_current_limit() {
    // your code for cmd get_grid_current_limit goes here
    return 3.14;
}

int board_support_low_levelImpl::handle_get_max_phase_count() {
    // your code for cmd get_max_phase_count goes here
    return 42;
}

void board_support_low_levelImpl::handle_enable_CP(bool& value) {
    // your code for cmd enable_CP goes here
}

void board_support_low_levelImpl::handle_set_duty_cycle(double& value) {
    // your code for cmd set_duty_cycle goes here
}

void board_support_low_levelImpl::handle_close_contactor(bool& value) {
    // your code for cmd close_contactor goes here
}

} // namespace low_level
} // namespace module
