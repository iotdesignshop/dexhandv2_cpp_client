/***
 * @file dexhand_message.hpp
 * @brief This file contains the message definitions for the Dexhand Connect library
 * @copyright Copyright (c) 2024 IoT Design Shop Inc.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include "dexhand_msg_types.hpp"
#include "dexhand_msg.pb.h"


namespace dexhand_connect {

class DexhandMessage {
    public:
        
        DexhandMessage(dexhand::DexhandMsgID id) : msgId(id) {}
        virtual ~DexhandMessage() {}

        virtual void parseMessage(const uint8_t* data, size_t size) = 0;

        dexhand::DexhandMsgID getMessageID() const { return msgId; }

    private:
        dexhand::DexhandMsgID msgId;
};

class ServoFullStatusMessage : public DexhandMessage {
    public:
        ServoFullStatusMessage() : DexhandMessage(dexhand::SERVO_FULL_STATUS_MSG) {}
        ~ServoFullStatusMessage() override {}

        void parseMessage(const uint8_t* data, size_t size) override;

        uint8_t getServoID() const { return msg.servoid(); }
        uint8_t getStatus() const { return msg.status(); }
        uint16_t getPosition() const { return msg.position(); }
        int16_t getSpeed() const { return msg.speed(); }
        int16_t getLoad() const { return msg.load(); }
        uint8_t getVoltage() const { return msg.voltage(); }
        uint8_t getTemperature() const { return msg.temperature(); }

    private:
        dexhand::ServoStatus msg;
};

class ServoDynamicsMessage : public DexhandMessage {
    public:
        ServoDynamicsMessage() : DexhandMessage(dexhand::SERVO_DYNAMICS_LIST_MSG) {}
        ~ServoDynamicsMessage() override {}

        void parseMessage(const uint8_t* data, size_t size) override;

        size_t getNumServos() const { return msg.servos_size(); }

        class ServoStatus {
            public:
                ServoStatus(const dexhand::ServoStatus& status) : msg(status) {}

                uint8_t getServoID() const { return msg.servoid(); }
                uint8_t getStatus() const { return msg.status(); }
                uint16_t getPosition() const { return msg.position(); }
                int16_t getSpeed() const { return msg.speed(); }
                int16_t getLoad() const { return msg.load(); }
                
            private:
                const dexhand::ServoStatus& msg;
        };

        const ServoStatus getServoStatus(size_t index) const { return ServoStatus(msg.servos(index)); }
        
    private:
        dexhand::ServoStatusList msg;
};



} // namespace dexhand_connect