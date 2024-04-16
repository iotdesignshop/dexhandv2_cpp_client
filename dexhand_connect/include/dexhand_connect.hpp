/****
 * @file dexhand_connect.hpp
 * @brief Main interface to the Dexhand Connect library
 * @copyright Copyright (c) 2024 IoT Design Shop Inc.
*/

#pragma once

#include <string>
#include <vector>


namespace dexhand_connect {

/// @brief Main interface to the Dexhand Connect library
class DexhandConnect {
public:
    DexhandConnect();
    ~DexhandConnect();

    /// @brief Structure to hold information about a connected Dexhand device
    struct DexhandUSBDevice {
        std::string port;
        std::string manufacturer;
        std::string product;
        std::string serial;
    };

    /// @brief Scans for connected Dexhand devices via USB and returns a list of devices
    /// @return List of connected Dexhand devices
    static std::vector<DexhandUSBDevice> enumerateDevices();

    /// @brief Opens a serial connection to a Dexhand device
    /// @param port Serial port to connect to
    /// @return true if connection was successful, false otherwise
    bool openSerial(const std::string& port);

    /// @brief Closes the serial connection
    void closeSerial();

    /// @brief Checks if the serial connection is open
    bool isSerialOpen() const { return serialFd != -1; }

    /// @brief Updates the connection to the Dexhand device. This should be called periodically
    /// to process incoming messages
    void update();

    
private:
    int serialFd;   // File descriptor for serial port

    size_t writeSerial(const char* data, size_t size);
    size_t readSerial(uint8_t* data, size_t size);
    size_t readBytesAvailable();

    // Messsage header format for Dexhand messages
    struct msgHeader {
        uint8_t msgStart;
        uint8_t msgId;
        uint16_t msgSize;
        uint8_t msgData;
    };

    // Message tail format for Dexhand messages
    struct msgTail {
        uint8_t checksum;
        uint8_t msgEnd;
    };

    static const size_t MESSAGE_HEADER_OVERHEAD = 4;    // Bytes of header/checksum data on messages
    static const size_t MESSAGE_TAIL_SIZE = 2;          // Bytes of checksum/msgEnd data on messages


    static const size_t MAX_MESSAGE_SIZE = 512;    // Maximum size of a message
    std::vector<uint8_t> receivedData;   // Buffered data from serial port

    // Polls the serial port for incoming data and accumulates it in receivedData
    void receiveUSBData();

    // Processes incoming messages in receivedData
    void processMessages();

    // Checks if a message header is valid
    bool isValidHeader(const DexhandConnect::msgHeader* header);

    // Calculates the checksum of a message
    uint8_t calculateChecksum(const uint8_t* data, size_t size);

    
    
};

} // namespace dexhand_connect