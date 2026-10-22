// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 chargebyte GmbH, Pionix GmbH and Contributors to EVerest
#include "CbGPIOChargePause.hpp"

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <poll.h>
#include <stdexcept>
#include <sys/eventfd.h>
#include <system_error>
#include <unistd.h>
#include <chargebyte/gpiodUtils.hpp>
#include <utils/date.hpp>

using namespace std::chrono_literals;

namespace module {

CbGPIOChargePause::~CbGPIOChargePause() {
    stop();
    if (m_stop_fd >= 0) {
        close(m_stop_fd);
    }
}

void CbGPIOChargePause::init() {
    if (config.pause_direction != "import" && config.pause_direction != "export" && config.pause_direction != "both") {
        throw std::invalid_argument("Invalid pause_direction: " + config.pause_direction);
    }
    if (!std::isfinite(config.pause_release_current_A) || config.pause_release_current_A < 0 ||
        config.pause_release_current_A > std::numeric_limits<float>::max()) {
        throw std::invalid_argument("pause_release_current_A must be non-negative and representable as a finite float");
    }
    m_stop_fd = eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
    if (m_stop_fd < 0) {
        throw std::system_error(errno, std::generic_category(), "Cannot create GPIO shutdown event");
    }
}

void CbGPIOChargePause::ready() {
    std::lock_guard<std::mutex> lock(m_stop_mutex);
    if (!m_stopping && !m_worker.joinable()) {
        m_worker = std::thread(&CbGPIOChargePause::monitor_gpio, this);
    }
}

void CbGPIOChargePause::shutdown() {
    stop();
}

void CbGPIOChargePause::stop() {
    std::lock_guard<std::mutex> lock(m_stop_mutex);
    m_stopping = true;
    if (m_stop_fd >= 0) {
        const std::uint64_t value = 1;
        ssize_t result;
        do {
            result = write(m_stop_fd, &value, sizeof(value));
        } while (result < 0 && errno == EINTR);
    }
    if (m_worker.joinable()) {
        m_worker.join();
    }
}

bool CbGPIOChargePause::wait_for_activity(int gpio_fd, int timeout_ms) {
    pollfd fds[] = {{m_stop_fd, POLLIN, 0}, {gpio_fd, POLLIN, 0}};
    int result;
    do {
        result = poll(fds, 2, timeout_ms);
    } while (result < 0 && errno == EINTR && !m_stopping);
    if (m_stopping || (fds[0].revents & POLLIN)) {
        return false;
    }
    if (result < 0) {
        throw std::system_error(errno, std::generic_category(), "Cannot wait for GPIO events");
    }
    if ((fds[0].revents | fds[1].revents) & (POLLERR | POLLHUP | POLLNVAL)) {
        throw std::runtime_error("GPIO event descriptor became unavailable");
    }
    return (fds[1].revents & POLLIN) != 0;
}

void CbGPIOChargePause::monitor_gpio() {
    EVLOG_info << "Watching GPIO '" << config.pause_gpio_line_name << "' (active_low=" << config.pause_active_low
               << ", energy direction=" << config.pause_direction << ")";

    while (!m_stopping) {
        try {
            gpiod::line_settings settings;
            settings.set_direction(gpiod::line::direction::INPUT)
                .set_active_low(config.pause_active_low)
                .set_edge_detection(gpiod::line::edge::BOTH)
                .set_debounce_period(50ms);
            auto request = get_gpioline_by_name(config.pause_gpio_line_name, "CbGPIOChargePause", settings);
            gpiod::edge_event_buffer events;

            while (!m_stopping) {
                const bool paused = request.get_value(request.offsets()[0]) == gpiod::line::value::ACTIVE;
                try {
                    if (!m_stopping) {
                        apply_limit(paused);
                    }
                } catch (const std::exception& e) {
                    EVLOG_error << "Cannot apply GPIO charge limit: " << e.what();
                    wait_for_activity(-1, 1000);
                    continue;
                }

                if (!m_stopping && wait_for_activity(request.fd(), -1)) {
                    request.read_edge_events(events);
                }
            }
        } catch (const std::exception& e) {
            EVLOG_error << "Cannot monitor GPIO '" << config.pause_gpio_line_name << "': " << e.what()
                        << "; retaining the last limit and retrying";
            try {
                wait_for_activity(-1, 1000);
            } catch (const std::exception& wait_error) {
                EVLOG_error << "Cannot wait for GPIO recovery: " << wait_error.what();
                return;
            }
        }
    }
}

void CbGPIOChargePause::apply_limit(bool paused) {
    if (m_last_applied_state == paused) {
        return;
    }

    types::energy::ScheduleReqEntry entry {};
    entry.timestamp = Everest::Date::to_rfc3339(date::utc_clock::now());
    const auto current = paused ? 0.0f : static_cast<float>(config.pause_release_current_A);
    entry.limits_to_root.ac_max_current_A = types::energy::NumberWithSource {current, "CbGPIOChargePause"};

    types::energy::ExternalLimits limits {};
    if (config.pause_direction == "import" || config.pause_direction == "both") {
        limits.schedule_import.push_back(entry);
    }
    if (config.pause_direction == "export" || config.pause_direction == "both") {
        limits.schedule_export.push_back(entry);
    }
    try {
        r_energy_sink->call_set_external_limits(limits);
    } catch (...) {
        // Delivery can fail after the sink has already applied the command.
        m_last_applied_state.reset();
        throw;
    }
    m_last_applied_state = paused;
    EVLOG_info << (paused ? "Charging PAUSED (release input active)" : "Charging released") << " ("
               << config.pause_direction << ": " << current << " A)";
}

} // namespace module
