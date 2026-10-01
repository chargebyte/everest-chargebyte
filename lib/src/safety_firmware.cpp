// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest

#include <array>
#include <filesystem>
#include <string>

#include <everest/logging.hpp>
#include <ra-utils/fw_file.h>

#include <chargebyte/safety_firmware.hpp>

namespace chargebyte::safety_firmware {

CheckResult check_version(const std::string_view firmware_prefix, const std::string_view running_version,
                          const std::filesystem::path& firmware_directory) {
    std::filesystem::path firmware_file;
    const std::string filename_prefix = std::string(firmware_prefix) + "_fw_";

    try {
        for (const auto& entry : std::filesystem::directory_iterator(firmware_directory)) {
            const auto filename = entry.path().filename().string();
            if (entry.is_regular_file() && entry.path().extension() == ".bin" &&
                filename.rfind(filename_prefix, 0) == 0) {
                firmware_file = entry.path();
                break;
            }
        }
    } catch (const std::filesystem::filesystem_error& e) {
        EVLOG_error << "Could not inspect the safety controller firmware directory " << firmware_directory << ": "
                    << e.what();
        return {CheckStatus::CheckFailed, {}};
    }

    if (firmware_file.empty()) {
        EVLOG_error << "Could not find a safety controller firmware image in " << firmware_directory;
        return {CheckStatus::CheckFailed, {}};
    }

    std::array<char, 128> firmware_version {};
    if (fw_get_version_from_file(firmware_file.c_str(), firmware_version.data(), firmware_version.size()) != 0) {
        EVLOG_error << "Could not read the safety controller firmware version from " << firmware_file;
        return {CheckStatus::CheckFailed, {}};
    }

    const std::string expected_version = firmware_version.data();
    return {running_version == expected_version ? CheckStatus::Match : CheckStatus::Mismatch, expected_version};
}

} // namespace chargebyte::safety_firmware
