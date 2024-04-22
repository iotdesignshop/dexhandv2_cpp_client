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
    }

    void ServoVarsListMessage::parseMessage(const uint8_t* data, size_t size) {
        if (!msg.ParseFromArray(data, size)) {
            // Failed to parse message - this is unusual
            cerr << "Failed to parse ServoVarsMessage" << endl;
        }
    }

    void FirmwareVersionMessage::parseMessage(const uint8_t* data, size_t size) {
        if (!msg.ParseFromArray(data, size)) {
            // Failed to parse message - this is unusual
            cerr << "Failed to parse FirmwareVersionMessage" << endl;
        }
    }


} // namespace dexhand_connect