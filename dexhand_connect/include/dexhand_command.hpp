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
        virtual ~DexhandCommand() = default;

        virtual std::string serialize() const = 0;

        dexhand::DexhandMsgID getMessageID() const { return msgId; }

    private:
        dexhand::DexhandMsgID msgId;
};

/// @brief Command to reset the hand to a known state. This called at the start of a DexHand session to initialize
/// the hand to a known state and to enable the hand to start moving. It can also be called at any time to reset 
/// the hand to the default pose if needed.
class ResetHandCommand : public DexhandCommand {
    public:
        ResetHandCommand() : DexhandCommand(dexhand::RESET_HAND_CMD) {}
        virtual std::string serialize() const override { return ""; }
};


/// @brief Command to set the position of one or more servos on the DexHand
class SetServoPositionsCommand : public DexhandCommand {
    public:
        SetServoPositionsCommand() : DexhandCommand(dexhand::SERVO_POSITION_UPDATE_CMD) {}
        virtual std::string serialize() const override;

        /// @brief Adds a servo position to the message
        void setServoPosition(uint8_t servoID, uint16_t position) {
            dexhand::ServoStatus* servo = findOrAddServo(servoID);
            servo->set_position(position);
        }

    private:
        dexhand::ServoStatus* findOrAddServo(uint8_t servoID) {
            for (int i = 0; i < msg.servos_size(); i++) {
                if (msg.servos(i).servoid() == servoID) {
                    return msg.mutable_servos(i);
                }
            }

            // Not found, add a new one
            dexhand::ServoStatus* servo =  msg.add_servos();
            servo->set_servoid(servoID);
            return servo;
        }

        dexhand::ServoStatusList msg;
};

/// @brief Command to set the variables for a servo on the DexHand. Typically
/// these changes are written to the flash memory of the servos or the controller
/// and persist once they are changed.
class SetServoVarsCommand : public DexhandCommand {
    public:
        SetServoVarsCommand() : DexhandCommand(dexhand::SERVO_VARS_UPDATE_CMD) {}
        virtual std::string serialize() const override;

        void setID(uint8_t id) { msg.set_servoid(id); }
        void setSWMinPosition(uint16_t pos) { msg.set_swminposition(pos); }
        void setSWMaxPosition(uint16_t pos) { msg.set_swmaxposition(pos); }
        void setHomePosition(uint16_t pos) { msg.set_homeposition(pos); }
        void setMaxLoadPct(uint8_t load) { msg.set_maxloadpct(load); }
        void setMaxTemp(uint8_t temp) { msg.set_maxtemperature(temp); }
        void setTorqueEnable(bool enable) { msg.set_torqueenable(enable); }

        /// @brief Sets the hardware minimum travel limit for the servo
        /// @note Do not adjust this value unless you are sure of the hardware limits as it can cause damage
        /// Use the software limits instead for tuning and to adjust the range of motion unless you really
        /// need to limit the physical travel of the servo.
        /// @param pos New minimum travel limit
        void setHWMinPosition(uint16_t pos) { msg.set_hwminposition(pos); }

        /// @brief Sets the hardware maximum travel limit for the servo
        /// @note Do not adjust this value unless you are sure of the hardware limits as it can cause damage
        /// Use the software limits instead for tuning and to adjust the range of motion unless you really
        /// need to limit the physical travel of the servo.
        /// @param pos New maximum travel limit
        void setHWMaxPosition(uint16_t pos) { msg.set_hwmaxposition(pos); }
        

    private:
        dexhand::ServoVars msg;
};

} // namespace dexhand_connect

