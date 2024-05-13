/***
 * @file dexhand_message.hpp
 * @brief This file contains the message definitions for the Dexhand Connect library
 * @copyright Copyright (c) 2024 IoT Design Shop Inc.
 */

#pragma once

#include <string>
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

/// @brief Status flags for the servo
enum class ServoStatusFlags : uint8_t {
    PRESENT = 1,                ///< Servo is present and in nominal state
    TORQUE_OFF = 2,             ///< Torque is disabled on the servo
    VOLTAGE_FAULT = 4,          ///< Voltage fault - under or over voltage
    TEMPERATURE_FAULT = 8,      ///< Temperature fault - over temperature
    TORQUE_FAULT = 16,          ///< Torque fault - over current
    LOST_OFFLINE = 32           ///< Servo was previosly present, but disappeared from serial bus
};

/// @brief Full status message from a single servo. These messages are reported by the hand at a lower frequency
/// than dynamics messages and contain extended state information such as the voltage and temperature of the servo.
class ServoFullStatusMessage : public DexhandMessage {
    public:
        ServoFullStatusMessage() : DexhandMessage(dexhand::SERVO_FULL_STATUS_MSG) {}
        ~ServoFullStatusMessage() override {}

        void parseMessage(const uint8_t* data, size_t size) override;

        /// @brief Retrieve the ID of the servo
        /// @return Servo ID
        uint8_t getServoID() const { return msg.servoid(); }

        /// @brief Retrieve the status of the servo
        /// @return Servo status as a bitfield
        /// @see ServoStatusFlags for definitions of the status flags
        uint8_t getStatus() const { return msg.status(); }

        /// @brief Retieve the current position of the servo
        /// @return Value from 0-4095 representing the servo position
        uint16_t getPosition() const { return msg.position(); }

        /// @brief Retrieve the current speed of the servo
        /// @return Current speed
        int16_t getSpeed() const { return msg.speed(); }

        /// @brief Retrieve the current load on the servo
        /// @return Current load in mA
        int16_t getLoad() const { return msg.load(); }

        /// @brief Retrieve the current voltage of the servo
        /// @return Current voltage in volts*10, ie 60 = 6.0V
        uint8_t getVoltage() const { return msg.voltage(); }

        /// @brief Retrieve the current temperature of the servo
        /// @return Current temperature in degrees C
        uint8_t getTemperature() const { return msg.temperature(); }

    private:
        dexhand::ServoStatus msg;
};

/// @brief Dynamics message from multiple servos. These messages are reported by the hand at a higher frequency
/// than full status messages and contain only the servo position, speed, load, and status.
class ServoDynamicsMessage : public DexhandMessage {
    public:
        ServoDynamicsMessage() : DexhandMessage(dexhand::SERVO_DYNAMICS_LIST_MSG) {}
        ~ServoDynamicsMessage() override {}

        void parseMessage(const uint8_t* data, size_t size) override;

        /// @brief Retrieves the number of servos in the message
        /// @return Count of servos reported by the message
        size_t getNumServos() const { return servoStatus.size(); }

        /// @brief Class to represent the status of a single servo since multiple servos
        /// are reported in a single message
        class ServoStatus {
            public:
                ServoStatus(const dexhand::ServoStatus& status) : msg(status) {}

                /// @brief Retrieve the servo ID
                /// @return Servo ID
                uint8_t getServoID() const { return msg.servoid(); }

                /// @brief Retrieve the servo status
                /// @return Servo status as a bitfield
                /// @see ServoStatusFlags for definitions of the status flags
                uint8_t getStatus() const { return msg.status(); }

                /// @brief Retrieve the current position of the servo
                /// @return Value from 0-4095 representing the servo position
                uint16_t getPosition() const { return msg.position(); }

                /// @brief Retrieve the current speed of the servo
                /// @return Current speed
                int16_t getSpeed() const { return msg.speed(); }

                /// @brief Retrieve the current load on the servo
                /// @return Current load in mA
                int16_t getLoad() const { return msg.load(); }
                
            private:
                const dexhand::ServoStatus& msg;
        };

        std::map<uint8_t, ServoStatus> getServoStatus() const { return servoStatus; }

    private:
        dexhand::ServoStatusList msg;
        std::map<uint8_t, ServoStatus> servoStatus;
};

/// @brief Servo variables message from the hand, containing the hardware and software limits for each servo
/// as well as the home position, max load percentage, and max temperature. This table is typically broadcast
/// by the hand each time the hand is reset.
class ServoVarsListMessage : public DexhandMessage {
    public:
        ServoVarsListMessage() : DexhandMessage(dexhand::SERVO_VARS_LIST_MSG) {}
        ~ServoVarsListMessage() override {}

        void parseMessage(const uint8_t* data, size_t size) override;

        /// @brief Retrieve the number of servos reported in the message
        /// @return Servo count
        size_t getNumServos() const { return servoVars.size(); }

        /// @brief Class to represent the variables for a single servo since multiple servos
        /// are reported in a single message
        class ServoVars {
            public:
                ServoVars(const dexhand::ServoVars& vars) : msg(vars) {}
                ~ServoVars() {}

                /// @brief Retrieve Servo ID
                /// @return Servo ID
                uint8_t getServoID() const { return msg.servoid(); }

                /// @brief Retrieve the minimum hardware position for the servo
                /// @return Minimum hardware position
                uint16_t getHWMinPosition() const { return msg.hwminposition(); }

                /// @brief Retrieve the maximum hardware position for the servo
                /// @return Maximum hardware position
                uint16_t getHWMaxPosition() const { return msg.hwmaxposition(); }

                /// @brief Retrieve the minimum software position for the servo
                /// @return Minimum software position
                uint16_t getSWMinPosition() const { return msg.swminposition(); }

                /// @brief Retrieve the maximum software position for the servo
                /// @return Maximum software position
                uint16_t getSWMaxPosition() const { return msg.swmaxposition(); }

                /// @brief Retrieve the home position for the servo
                /// @return Home position
                uint16_t getHomePosition() const { return msg.homeposition(); }

                /// @brief Retrieve the maximum load percentage for the servo
                /// @return Maximum load percentage
                uint8_t getMaxLoadPct() const { return msg.maxloadpct(); }

                /// @brief Retrieve the maximum temperature for the servo
                /// @return Maximum temperature
                uint8_t getMaxTemp() const { return msg.maxtemperature(); }
                
            private:
                const dexhand::ServoVars& msg;
        };

        std::map<uint8_t, ServoVars> getServoVars() const { return servoVars; }
        
    private:
        dexhand::ServoVarsList msg;
        std::map<uint8_t, ServoVars> servoVars;
};

/// @brief Firmware version message from the hand, containing the major and minor version numbers
class FirmwareVersionMessage : public DexhandMessage {
    public:
        FirmwareVersionMessage() : DexhandMessage(dexhand::FIRMWARE_VERSION_MSG) {}
        ~FirmwareVersionMessage() override {}

        void parseMessage(const uint8_t* data, size_t size) override;

        uint8_t getMajorVersion() const { return msg.major(); }
        uint8_t getMinorVersion() const { return msg.minor(); }
        const std::string getVersionName() const { return msg.name(); }
        
    private:
        dexhand::FirmwareVersion msg;
};



} // namespace dexhand_connect