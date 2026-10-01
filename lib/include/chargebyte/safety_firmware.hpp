// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest
#pragma once

#include <filesystem>
#include <string>
#include <string_view>

namespace chargebyte::safety_firmware {

enum class CheckStatus
{
    Match,
    Mismatch,
    CheckFailed,
};

struct CheckResult {
    CheckStatus status;
    std::string expected_version;
};

CheckResult check_version(std::string_view firmware_prefix, std::string_view running_version,
                          const std::filesystem::path& firmware_directory);

} // namespace chargebyte::safety_firmware
