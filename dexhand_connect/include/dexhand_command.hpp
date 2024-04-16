/***
 * @file dexhand_command.hpp
 * @brief This file contains the definition of commands that can be sent to a DexHand device
 * @copyright Copyright (c) 2024 IoT Design Shop Inc.
 */

#pragma once
#include <string>

#include "dexhand_msg_types.hpp"
#include "dexhand_msg.pb.h"

namespace dexhand_connect {

class DexhandCommand {
    public:
        DexhandCommand(dexhand::DexhandMsgID id) : msgId(id) {}
        virtual ~DexhandCommand() {}

        virtual std::string serialize() const = 0;

        dexhand::DexhandMsgID getMessageID() const { return msgId; }

    private:
        dexhand::DexhandMsgID msgId;
};


/// @brief Command to set the position of one or more servos on the DexHand
class SetServoPositionsCommand : public DexhandCommand {
    public:
        SetServoPositionsCommand() : DexhandCommand(dexhand::SERVO_POSITION_UPDATE_MSG) {}
        std::string serialize() const override;

        /// @brief Adds a servo position to the message
        void setServoPosition(uint8_t servoID, uint16_t position) {
            dexhand::ServoStatus* servo = msg.add_servos();
            servo->set_servoid(servoID);
            servo->set_position(position);
        }

    private:
        dexhand::ServoStatusList msg;
};

}