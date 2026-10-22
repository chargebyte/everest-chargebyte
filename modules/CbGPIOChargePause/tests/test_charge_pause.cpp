// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 chargebyte GmbH
#include "CbGPIOChargePause.hpp"

#include <chrono>
#include <cstdint>
#include <future>
#include <stdexcept>
#include <sys/eventfd.h>
#include <unistd.h>
#include <gtest/gtest.h>

using namespace std::chrono_literals;

namespace module {

class CbGPIOChargePauseTestAccess {
public:
    static void init(CbGPIOChargePause& mod) {
        mod.init();
    }
    static void ready(CbGPIOChargePause& mod) {
        mod.ready();
    }
    static void apply(CbGPIOChargePause& mod, bool paused) {
        mod.apply_limit(paused);
    }
    static bool wait(CbGPIOChargePause& mod, int fd, int timeout_ms) {
        return mod.wait_for_activity(fd, timeout_ms);
    }
    static void stop(CbGPIOChargePause& mod) {
        mod.shutdown();
    }
    static void monitor(CbGPIOChargePause& mod) {
        mod.monitor_gpio();
    }
};

class ChargePauseTest : public testing::Test {
protected:
    void SetUp() override {
        m_config.pause_gpio_line_name = "__cb_gpio_charge_pause_test_missing_line__";
        m_config.pause_active_low = false;
        m_config.pause_direction = "both";
        m_info.name = "CbGPIOChargePause";
        m_info.id = "pause_test";
        m_adapter.get_error_manager_req = [](const auto&) { return nullptr; };
        m_adapter.get_error_state_monitor_req = [](const auto&) { return nullptr; };
        m_adapter.call = [this](const Requirement&, const std::string& cmd, const Parameters& params) -> Result {
            EXPECT_EQ(cmd, "set_external_limits");
            if (m_fail_delivery) {
                throw std::runtime_error("Simulated command failure");
            }
            m_limits.push_back(params.at("value").get<types::energy::ExternalLimits>());
            return std::nullopt;
        };
        auto sink = std::make_unique<external_energy_limitsIntf>(&m_adapter, Requirement {}, m_info.id, std::nullopt);
        m_module = std::make_unique<CbGPIOChargePause>(m_info, std::move(sink), m_config);
        CbGPIOChargePauseTestAccess::init(*m_module);
    }

    static void expect_released(const types::energy::ExternalLimits& limits) {
        EXPECT_TRUE(limits.schedule_import.empty());
        EXPECT_TRUE(limits.schedule_export.empty());
        EXPECT_TRUE(limits.schedule_setpoints.empty());
    }

    ModuleInfo m_info;
    Conf m_config {};
    Everest::ModuleAdapter m_adapter;
    std::vector<types::energy::ExternalLimits> m_limits;
    bool m_fail_delivery {false};
    std::unique_ptr<CbGPIOChargePause> m_module;
};

TEST_F(ChargePauseTest, SendsInitialStateAndOnlyChangedStates) {
    CbGPIOChargePauseTestAccess::apply(*m_module, false);
    CbGPIOChargePauseTestAccess::apply(*m_module, false);
    CbGPIOChargePauseTestAccess::apply(*m_module, true);
    CbGPIOChargePauseTestAccess::apply(*m_module, true);
    CbGPIOChargePauseTestAccess::apply(*m_module, false);
    ASSERT_EQ(m_limits.size(), 3);
    expect_released(m_limits[0]);
    ASSERT_EQ(m_limits[1].schedule_import.size(), 1);
    EXPECT_FLOAT_EQ(m_limits[1].schedule_import[0].limits_to_root.ac_max_current_A->value, 0.f);
    expect_released(m_limits[2]);
}

TEST_F(ChargePauseTest, SendsSelectedDirectionsAndEmptySetpoints) {
    for (const auto& direction : {"import", "export", "both"}) {
        m_config.pause_direction = direction;
        for (const bool paused : {true, false}) {
            CbGPIOChargePauseTestAccess::apply(*m_module, paused);
            const auto& limits = m_limits.back();
            EXPECT_EQ(limits.schedule_import.size(), paused && m_config.pause_direction != "export" ? 1 : 0);
            EXPECT_EQ(limits.schedule_export.size(), paused && m_config.pause_direction != "import" ? 1 : 0);
            EXPECT_TRUE(limits.schedule_setpoints.empty());
            if (!paused) {
                expect_released(limits);
                continue;
            }
            for (const auto* schedule : {&limits.schedule_import, &limits.schedule_export}) {
                for (const auto& entry : *schedule) {
                    EXPECT_FALSE(entry.timestamp.empty());
                    ASSERT_TRUE(entry.limits_to_root.ac_max_current_A);
                    EXPECT_EQ(entry.limits_to_root.ac_max_current_A->source, "CbGPIOChargePause");
                    EXPECT_FLOAT_EQ(entry.limits_to_root.ac_max_current_A->value, 0.f);
                    EXPECT_FALSE(entry.limits_to_root.total_power_W);
                    EXPECT_FALSE(entry.limits_to_leaves.ac_max_current_A);
                }
            }
            if (m_config.pause_direction == "both" && limits.schedule_import.size() == 1 &&
                limits.schedule_export.size() == 1) {
                EXPECT_EQ(limits.schedule_import[0].timestamp, limits.schedule_export[0].timestamp);
            }
        }
    }
}

TEST_F(ChargePauseTest, FailedDeliveryDoesNotReplaceLastAppliedState) {
    CbGPIOChargePauseTestAccess::apply(*m_module, false);
    m_fail_delivery = true;
    EXPECT_THROW(CbGPIOChargePauseTestAccess::apply(*m_module, true), std::runtime_error);
    m_fail_delivery = false;
    CbGPIOChargePauseTestAccess::apply(*m_module, true);
    ASSERT_EQ(m_limits.size(), 2);
    EXPECT_FLOAT_EQ(m_limits.back().schedule_import[0].limits_to_root.ac_max_current_A->value, 0.f);
}

TEST_F(ChargePauseTest, RetriesFailedReleaseCommand) {
    CbGPIOChargePauseTestAccess::apply(*m_module, true);
    m_fail_delivery = true;
    EXPECT_THROW(CbGPIOChargePauseTestAccess::apply(*m_module, false), std::runtime_error);
    m_fail_delivery = false;
    CbGPIOChargePauseTestAccess::apply(*m_module, false);
    ASSERT_EQ(m_limits.size(), 2);
    expect_released(m_limits.back());
}

TEST_F(ChargePauseTest, ReassertsCurrentStateAfterAmbiguousDeliveryFailure) {
    CbGPIOChargePauseTestAccess::apply(*m_module, false);
    m_adapter.call = [this](const Requirement&, const std::string&, const Parameters& params) -> Result {
        m_limits.push_back(params.at("value").get<types::energy::ExternalLimits>());
        if (m_fail_delivery) {
            throw std::runtime_error("Reply lost after applying limit");
        }
        return std::nullopt;
    };
    m_fail_delivery = true;
    EXPECT_THROW(CbGPIOChargePauseTestAccess::apply(*m_module, true), std::runtime_error);
    m_fail_delivery = false;
    CbGPIOChargePauseTestAccess::apply(*m_module, false);
    ASSERT_EQ(m_limits.size(), 3);
    expect_released(m_limits.back());
}

TEST_F(ChargePauseTest, RetriesReleaseAfterAmbiguousDeliveryFailure) {
    CbGPIOChargePauseTestAccess::apply(*m_module, true);
    m_adapter.call = [this](const Requirement&, const std::string&, const Parameters& params) -> Result {
        m_limits.push_back(params.at("value").get<types::energy::ExternalLimits>());
        if (m_fail_delivery) {
            throw std::runtime_error("Reply lost after clearing limit");
        }
        return std::nullopt;
    };
    m_fail_delivery = true;
    EXPECT_THROW(CbGPIOChargePauseTestAccess::apply(*m_module, false), std::runtime_error);
    m_fail_delivery = false;
    CbGPIOChargePauseTestAccess::apply(*m_module, false);
    ASSERT_EQ(m_limits.size(), 3);
    expect_released(m_limits[1]);
    expect_released(m_limits[2]);
}

TEST_F(ChargePauseTest, GpioDescriptorWakesIndefiniteWait) {
    const int fd = eventfd(1, EFD_CLOEXEC);
    ASSERT_GE(fd, 0);
    EXPECT_TRUE(CbGPIOChargePauseTestAccess::wait(*m_module, fd, -1));
    close(fd);
}

TEST_F(ChargePauseTest, ShutdownWakesIndefiniteWait) {
    auto waiting = std::async(std::launch::async, [&] { return CbGPIOChargePauseTestAccess::wait(*m_module, -1, -1); });
    CbGPIOChargePauseTestAccess::stop(*m_module);
    EXPECT_EQ(waiting.wait_for(500ms), std::future_status::ready);
    EXPECT_FALSE(waiting.get());
    CbGPIOChargePauseTestAccess::stop(*m_module);
}

TEST_F(ChargePauseTest, ShutdownInterruptsRecoveryWait) {
    auto waiting =
        std::async(std::launch::async, [&] { return CbGPIOChargePauseTestAccess::wait(*m_module, -1, 1000); });
    CbGPIOChargePauseTestAccess::stop(*m_module);
    EXPECT_EQ(waiting.wait_for(500ms), std::future_status::ready);
    EXPECT_FALSE(waiting.get());
}

TEST_F(ChargePauseTest, MonitoringFailureRetainsExistingLimit) {
    CbGPIOChargePauseTestAccess::apply(*m_module, true);
    auto monitoring = std::async(std::launch::async, [&] { CbGPIOChargePauseTestAccess::monitor(*m_module); });
    EXPECT_EQ(monitoring.wait_for(50ms), std::future_status::timeout);
    CbGPIOChargePauseTestAccess::stop(*m_module);
    EXPECT_EQ(monitoring.wait_for(500ms), std::future_status::ready);
    monitoring.get();
    ASSERT_EQ(m_limits.size(), 1);
    EXPECT_FLOAT_EQ(m_limits[0].schedule_import[0].limits_to_root.ac_max_current_A->value, 0.f);
}

TEST_F(ChargePauseTest, UnreadableInitialGpioDoesNotSendLimit) {
    auto monitoring = std::async(std::launch::async, [&] { CbGPIOChargePauseTestAccess::monitor(*m_module); });
    EXPECT_EQ(monitoring.wait_for(50ms), std::future_status::timeout);
    CbGPIOChargePauseTestAccess::stop(*m_module);
    monitoring.get();
    EXPECT_TRUE(m_limits.empty());
}

TEST_F(ChargePauseTest, ShutdownJoinsWorkerAndDoesNotReleaseLimit) {
    CbGPIOChargePauseTestAccess::apply(*m_module, true);
    CbGPIOChargePauseTestAccess::ready(*m_module);
    CbGPIOChargePauseTestAccess::stop(*m_module);
    CbGPIOChargePauseTestAccess::stop(*m_module);
    ASSERT_EQ(m_limits.size(), 1);
    EXPECT_FLOAT_EQ(m_limits[0].schedule_import[0].limits_to_root.ac_max_current_A->value, 0.f);
}

} // namespace module
