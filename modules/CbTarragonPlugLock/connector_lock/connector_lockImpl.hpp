// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest
#ifndef CONNECTOR_LOCK_CONNECTOR_LOCK_IMPL_HPP
#define CONNECTOR_LOCK_CONNECTOR_LOCK_IMPL_HPP

//
// AUTO GENERATED - MARKED REGIONS WILL BE KEPT
// template version 3
//

#include <generated/interfaces/connector_lock/Implementation.hpp>

#include "../CbTarragonPlugLock.hpp"

// ev@75ac1216-19eb-4182-a85c-820f1fc2c091:v1
#include "CbLockActuator.hpp"
#include "CbLockSense.hpp"
#include "CbCapSense.hpp"

#include <atomic>
#include <mutex>
#include <utils/thread.hpp>
// ev@75ac1216-19eb-4182-a85c-820f1fc2c091:v1

namespace module {
namespace connector_lock {

struct Conf {};

class connector_lockImpl : public connector_lockImplBase {
public:
    connector_lockImpl() = delete;
    connector_lockImpl(Everest::ModuleAdapter* ev, const Everest::PtrContainer<CbTarragonPlugLock>& mod, Conf& config) :
        connector_lockImplBase(ev, "connector_lock"), mod(mod), config(config) {};

    // ev@8ea32d28-373f-4c90-ae5e-b4fcc74e2a61:v1
    // insert your public definitions here
    ~connector_lockImpl();
    // ev@8ea32d28-373f-4c90-ae5e-b4fcc74e2a61:v1

protected:
    // command handler functions (virtual)
    virtual void handle_lock() override;
    virtual void handle_unlock() override;

    // ev@d2d1847a-7b88-41dd-ad07-92785f06f5c4:v1
    // insert your protected definitions here
    // ev@d2d1847a-7b88-41dd-ad07-92785f06f5c4:v1

private:
    const Everest::PtrContainer<CbTarragonPlugLock>& mod;
    const Conf& config;

    virtual void init() override;
    virtual void ready() override;

    // ev@3370e4dd-95f4-47a9-aaec-ea76f34a66c9:v1
    bool wait_for_charged(std::chrono::seconds timeout);

    CbLockActuator lock_actuator;
    CbLockSense lock_sense;
    CbCapSense cap_sense;
    static constexpr std::chrono::seconds CHARGED_TIMEOUT_WORK {5};
    static constexpr std::chrono::seconds CHARGED_TIMEOUT_INITIAL {400};
    static constexpr std::chrono::milliseconds FEEDBACK_CHECK_INTERVAL {1000};
    std::atomic_bool is_connectorLockCapNotCharged_raised {false};
    std::atomic_bool is_connectorLockFailedLock_raised {false};
    std::atomic_bool is_connectorLockFailedUnlock_raised {false};

    /// @brief plug lock observation thread handle
    Everest::Thread lock_observation_thread;
    /// @brief plug lock observation worker method
    void lock_observation_worker();
    /// @brief mutex to protect feedback error evaluation during driving
    std::mutex observation_mtx;
    /// @brief current assumed lock state, true if locked
    bool assumed_is_locked {false};
    // ev@3370e4dd-95f4-47a9-aaec-ea76f34a66c9:v1
};

// ev@3d7da0ad-02c2-493d-9920-0bbbd56b9876:v1
// insert other definitions here
// ev@3d7da0ad-02c2-493d-9920-0bbbd56b9876:v1

} // namespace connector_lock
} // namespace module

#endif // CONNECTOR_LOCK_CONNECTOR_LOCK_IMPL_HPP
