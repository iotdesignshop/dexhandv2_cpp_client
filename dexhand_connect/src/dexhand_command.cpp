/***
 * @file dexhand_command.cpp
 * @brief This file contains the definition of commands that can be sent to a DexHand device
 * @copyright Copyright (c) 2024 IoT Design Shop Inc.
 */


#include "dexhand_command.hpp"

namespace dexhand_connect {

    std::string SetServoPositionsCommand::serialize() const {
        std::string out;
        msg.SerializeToString(&out);
        return out;
    }

} // namespace dexhand_connect