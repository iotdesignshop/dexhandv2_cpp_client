/***
 * @file dexhand_message.cpp
 * @brief This file contains the message definitions for the Dexhand Connect library
 * @copyright Copyright (c) 2024 IoT Design Shop Inc.
 */

#include <iostream>
#include "dexhand_message.hpp"

using namespace std;

namespace dexhand_connect {

    void ServoFullStatusMessage::parseMessage(const uint8_t* data, size_t size) {
        if (!msg.ParseFromArray(data, size)) {
            // Failed to parse message - this is unusual
            cerr << "Failed to parse ServoStatusMessage" << endl;
        }
    }

    void ServoDynamicsMessage::parseMessage(const uint8_t* data, size_t size) {
        if (!msg.ParseFromArray(data, size)) {
            // Failed to parse message - this is unusual
            cerr << "Failed to parse ServoStatusListMessage" << endl;
        }
        else {
            // Populate the map
            for (int i = 0; i < msg.servos_size(); i++) {
                const dexhand::ServoStatus& status = msg.servos(i);
                servoStatus.emplace(std::piecewise_construct,
                                        std::forward_as_tuple(status.servoid()),
                                        std::forward_as_tuple(status));
            }
        }
    }

    void ServoVarsListMessage::parseMessage(const uint8_t* data, size_t size) {
        if (!msg.ParseFromArray(data, size)) {
            // Failed to parse message - this is unusual
            cerr << "Failed to parse ServoVarsMessage" << endl;
        }
        else {
            // Populate the map
            for (int i = 0; i < msg.servos_size(); i++) {
                const dexhand::ServoVars& vars = msg.servos(i);
                servoVars.emplace(std::piecewise_construct,
                                  std::forward_as_tuple(vars.servoid()),
                                  std::forward_as_tuple(vars));
            }
        }
    }

    void FirmwareVersionMessage::parseMessage(const uint8_t* data, size_t size) {
        if (!msg.ParseFromArray(data, size)) {
            // Failed to parse message - this is unusual
            cerr << "Failed to parse FirmwareVersionMessage" << endl;
        }
    }


} // namespace dexhand_connect