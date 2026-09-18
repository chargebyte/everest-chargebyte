// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest
#ifndef CB_CAN_LOG_MARK_HPP
#define CB_CAN_LOG_MARK_HPP

//
// AUTO GENERATED - MARKED REGIONS WILL BE KEPT
// template version 2
//

#include "ld-ev.hpp"

// ev@4bf81b14-a215-475c-a1d3-0a484ae48918:v1
// insert your custom include headers here
#include <atomic>
#include <string>
#include <thread>
// ev@4bf81b14-a215-475c-a1d3-0a484ae48918:v1

namespace module {

struct Conf {
    std::string device;
    int bitrate;
    int error_can_id;
    int warning_can_id;
    int info_can_id;
    int debug_can_id;
};

class CbCANLogMark : public Everest::ModuleBase {
public:
    CbCANLogMark() = delete;
    CbCANLogMark(const ModuleInfo& info, Conf& config) : ModuleBase(info), config(config) {
    }
    ~CbCANLogMark();

    const Conf& config;

    // ev@1fce4c5e-0ab8-41bb-90f7-14277703d2ac:v1
    // insert your public definitions here
    // ev@1fce4c5e-0ab8-41bb-90f7-14277703d2ac:v1

protected:
    // ev@4714b2ab-a24f-4b95-ab81-36439e1478de:v1
    // insert your protected definitions here
    // ev@4714b2ab-a24f-4b95-ab81-36439e1478de:v1

private:
    friend class LdEverest;
    void init();
    void ready();

    // ev@211cfdbe-f69a-4cd6-a4ec-f8aaa3d1b6c8:v1
    // insert your private definitions here
    void validate_config() const;
    void configure_and_open_can();
    void receive_worker();
    void stop();

    std::atomic_bool termination_requested {false};
    std::thread receive_thread;
    int can_fd {-1};
    // ev@211cfdbe-f69a-4cd6-a4ec-f8aaa3d1b6c8:v1
};

// ev@087e516b-124c-48df-94fb-109508c7cda9:v1
// insert other definitions here
// ev@087e516b-124c-48df-94fb-109508c7cda9:v1

} // namespace module

#endif // CB_CAN_LOG_MARK_HPP
