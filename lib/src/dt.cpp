// Copyright © 2026 chargebyte GmbH
// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iosfwd>
#include <stdexcept>
#include <chargebyte/dt.hpp>

namespace chargebyte::dt {

std::vector<std::string> read_dt_compatibles() {
    std::vector<std::string> rv;

    // the compatible file is list of strings, separated by NUL bytes

    std::string path = "/proc/device-tree/compatible";
    std::ifstream f(path, std::ios::binary);
    if (!f)
        throw std::runtime_error("Failed to open '" + path + "'");

    std::vector<char> buf((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());

    std::size_t start = 0;
    while (start < buf.size()) {
        std::size_t end = start;
        while (end < buf.size() && buf[end] != '\0')
            ++end;

        if (end > start) {
            rv.emplace_back(&buf[start], end - start);
        }

        start = end + 1;
    }

    return rv;
}

bool is_dt_compatible(const std::vector<std::string>& dt_compatibles, const std::string& value) {
    return std::find(dt_compatibles.begin(), dt_compatibles.end(), value) != dt_compatibles.end();
}

bool is_dt_compatible(const std::vector<std::string>& dt_compatibles, std::initializer_list<std::string_view> values) {

    for (std::string_view s : values) {
        if (std::find(dt_compatibles.begin(), dt_compatibles.end(), s) != dt_compatibles.end()) {
            return true;
        }
    }

    return false;
}

} // namespace chargebyte::dt
