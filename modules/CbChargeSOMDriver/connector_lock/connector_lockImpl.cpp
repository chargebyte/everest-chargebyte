// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH, Pionix GmbH and Contributors to EVerest

#include "connector_lockImpl.hpp"
#include <string_view>
#include <ra-utils/cb_protocol.h>

namespace module {
namespace connector_lock {

// taken from safety firmware
typedef enum
{
    ERR_CODES_INLET_DEFAULT = 0,

    ERR_CODES_INLET_FBK_AT_INIT,         ///< Invalid feedback during initialization [feedback, -]
    ERR_CODES_INLET_FBK_AT_OPEN,         ///< Feedback indicates invalid position while open [feedback, -]
    ERR_CODES_INLET_EXCEEDED_CLOSE_TIME, ///< Inlet did not reach closed position in time [-, -]
    ERR_CODES_INLET_FBK_AT_CLOSED,       ///< Feedback indicates invalid position while closed [feedback, -]
    ERR_CODES_INLET_EXCEEDED_OPEN_TIME,  ///< Inlet did not reach open position in time [-, -]

    ERR_CODES_INLET_INVALID_PARAMETER,  ///< Invalid inlet parameter configuration [parameter, -]
    ERR_CODES_INLET_MOTOR_DRIVER_FAULT, ///< Motor driver is signaling a fault [-, -]
    ERR_CODES_INLET_CODE_ERROR,         ///< Internal software error [code_line, -]

    ERR_CODES_INLET_NUM_OF_CODES

} err_codes_inlet_t;

void connector_lockImpl::init() {
    this->mod->controller.on_reset.connect([&]() {
        this->clear_all_errors_of_impl();
        this->error_reported = false;
    });

    this->mod->controller.on_inlet_state_change.connect([&](const enum inlet_state& new_inlet_state) {
        switch (new_inlet_state) {
        case inlet_state::INLET_STATE_UNDEFINED:
            // do nothing
            break;

        case inlet_state::INLET_STATE_NOT_CONFIGURED:
            // do nothing
            break;

        case inlet_state::INLET_STATE_ERROR:
            if (!this->error_reported.exchange(true)) {
                Everest::error::Error error_object = this->error_factory->create_error(
                    "connector_lock/MREC1ConnectorLockFailure", "", "", Everest::error::Severity::High);
                this->raise_error(error_object);
            }
            break;

        default:
            // clear any previously raised error
            if (this->error_reported.exchange(false)) {
                this->clear_error("connector_lock/MREC1ConnectorLockFailure");
            }
            break;
        }
    });

    this->mod->controller.on_errmsg.connect([&](bool is_active, unsigned int module, const std::string_view& module_str,
                                                unsigned int reason, const std::string_view& reason_str,
                                                unsigned int additional_data1, unsigned int additional_data2) {
        static_cast<void>(additional_data1);
        static_cast<void>(additional_data2);

        // filter for pluglock (aka inlet) related errors only
        if (module != ERRMSG_MODULE_APP_INLET)
            return;

        err_codes_inlet_t err_reason = static_cast<err_codes_inlet_t>(reason);
        const std::string error_sub_type {module_str};
        const std::string error_message {reason_str};

        if (is_active) {
            Everest::error::Error e;

            switch (err_reason) {
            case ERR_CODES_INLET_FBK_AT_OPEN:
                EVLOG_error << fmt::format("Safety Controller reported error: {}", reason_str);
                e = this->error_factory->create_error("connector_lock/ConnectorLockUnexpectedClose", error_sub_type,
                                                      error_message, Everest::error::Severity::High);
                break;
            case ERR_CODES_INLET_EXCEEDED_CLOSE_TIME:
                EVLOG_error << fmt::format("Safety Controller reported error: {}", reason_str);
                e = this->error_factory->create_error("connector_lock/ConnectorLockFailedLock", error_sub_type,
                                                      error_message, Everest::error::Severity::High);
                break;
            case ERR_CODES_INLET_FBK_AT_CLOSED:
                EVLOG_error << fmt::format("Safety Controller reported error: {}", reason_str);
                e = this->error_factory->create_error("connector_lock/ConnectorLockUnexpectedOpen", error_sub_type,
                                                      error_message, Everest::error::Severity::High);
                break;
            case ERR_CODES_INLET_EXCEEDED_OPEN_TIME:
                EVLOG_error << fmt::format("Safety Controller reported error: {}", reason_str);
                e = this->error_factory->create_error("connector_lock/ConnectorLockFailedUnlock", error_sub_type,
                                                      error_message, Everest::error::Severity::High);
                break;
            case ERR_CODES_INLET_MOTOR_DRIVER_FAULT:
                EVLOG_warning << fmt::format("Safety Controller reported error: {}", reason_str);
                e = this->error_factory->create_error("connector_lock/VendorWarning", error_sub_type, error_message,
                                                      Everest::error::Severity::High);
                break;
            default:
                EVLOG_error << fmt::format("Safety Controller reported error: {}", reason_str);
                e = this->error_factory->create_error("connector_lock/VendorError", error_sub_type, error_message,
                                                      Everest::error::Severity::High);
                break;
            }

            this->raise_error(e);
        } else {
            EVLOG_info << fmt::format("Safety Controller cleared error: {}", reason_str);

            switch (err_reason) {
            case ERR_CODES_INLET_FBK_AT_OPEN:
                this->clear_error("connector_lock/ConnectorLockUnexpectedClose");
                break;
            case ERR_CODES_INLET_EXCEEDED_CLOSE_TIME:
                this->clear_error("connector_lock/ConnectorLockFailedLock");
                break;
            case ERR_CODES_INLET_FBK_AT_CLOSED:
                this->clear_error("connector_lock/ConnectorLockUnexpectedOpen");
                break;
            case ERR_CODES_INLET_EXCEEDED_OPEN_TIME:
                this->clear_error("connector_lock/ConnectorLockFailedUnlock");
                break;
            case ERR_CODES_INLET_MOTOR_DRIVER_FAULT:
                this->clear_error("connector_lock/VendorWarning");
                break;
            default:
                this->clear_error("connector_lock/VendorError");
                break;
            }
        }
    });
}

void connector_lockImpl::ready() {
}

void connector_lockImpl::handle_lock() {
    try {
        EVLOG_info << "handle_lock: forwarding lock request to safety controller";
        this->mod->controller.inlet_lock();
    } catch (std::exception& e) {
        EVLOG_error << e.what();
    }
}

void connector_lockImpl::handle_unlock() {
    // clear all previous errors
    this->clear_all_errors_of_impl();

    try {
        EVLOG_info << "handle_unlock: forwarding unlock request to safety controller";
        this->mod->controller.inlet_unlock();
    } catch (std::exception& e) {
        EVLOG_error << e.what();
    }
}

} // namespace connector_lock
} // namespace module
