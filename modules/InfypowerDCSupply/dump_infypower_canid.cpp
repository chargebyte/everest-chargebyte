// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest

#include <cstdlib>
#include <iostream>
#include <string>
#include "InfypowerCANID.hpp"

int main(int argc, char* argv[]) {
    unsigned int can_id;

    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <can_id>" << std::endl;
        std::exit(1);
    }

    try {
        can_id = std::stoul(argv[1], nullptr, 16);
    } catch (const std::invalid_argument& e) {
        std::cerr << "Error: Invalid CAN ID given." << std::endl;
        return 1;
    } catch (const std::out_of_range& e) {
        std::cerr << "Error: CAN ID is out of range." << std::endl;
        return 1;
    }

    InfypowerCANID infypwr_can_id(can_id);

    std::cout << infypwr_can_id << std::endl;

    return 0;
}
