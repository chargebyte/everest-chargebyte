// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest

#include "cb_chargesom_mcsImpl.hpp"

namespace module {
namespace mcs {

void cb_chargesom_mcsImpl::init() {
}

void cb_chargesom_mcsImpl::ready() {
}

void cb_chargesom_mcsImpl::handle_mcs_hlc_enable(bool& value) {
    mod->controller.set_mcs_hlc_enable(value);
}

} // namespace mcs
} // namespace module
