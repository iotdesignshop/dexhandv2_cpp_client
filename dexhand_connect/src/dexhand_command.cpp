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

    std::string SetServoVarsCommand::serialize() const {
        std::string out;
        msg.SerializeToString(&out);
        return out;
    }

    std::string SetHandParameterCommand::serialize() const {
        std::string out;
        msg.SerializeToString(&out);
        return out;
    }

    std::string SetServoRegisterCommand::serialize() const {
        std::string out;
        msg.SerializeToString(&out);
        return out;
    }

} // namespace dexhand_connect