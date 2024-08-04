/****
 * @file dexhand_connect.hpp
 * @brief Main interface to the Dexhand Connect library
 * @copyright Copyright (c) 2024 IoT Design Shop Inc.
*/

#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <typeindex>
#include <typeinfo>
#include <mutex>

#include "dexhand_message.hpp"
#include "dexhand_command.hpp"

namespace dexhand_connect {

/// @brief  Interface for objects that wish to receive messages from the Dexhand device
/// @tparam T Message type to subscribe to
template<typename T>
class IDexhandMessageSubscriber {
    public:
        virtual ~IDexhandMessageSubscriber() {}
        virtual void messageReceived(const T& message) = 0;
};


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
    std::vector<DexhandUSBDevice> enumerateDevices();

    /// @brief Opens a serial connection to a Dexhand device
    /// @param port Serial port to connect to
    /// @return true if connection was successful, false otherwise
    bool openSerial(const std::string& port);

    /// @brief Closes the serial connection
    void closeSerial();

    /// @brief Checks if the serial connection is open
    bool isSerialOpen() const { return serialFd != -1; }

    /// @brief Resets the hand. This is required to initialize the hand to a known state at the start
    /// of a session, but can also be called at any time to reset the hand to the default pose.
    void resetHand();

    /// @brief Updates the connection to the Dexhand device. This should be called periodically
    /// to process incoming messages
    void update();

    /// @brief Enables or disables the automatic thumb exensor feature on the hand. This defaults to on, and means
    /// that the thumb extensor servo will automatically adjust its position to complement the thumb flexor servo.
    void enableAutoThumbExtensor(bool enable);

    /// @brief Commences thumb auto tuning sequence. See the Dexhand documentation for more information on how
    /// to properly use this feature to tune the thumb dynamics.
    void autoTuneThumb();


    /// @brief Subscribe to messages posted by the Dexhand device
    /// @tparam T Type of message to subscribe to
    /// @see dexhand_message.hpp for message types
    /// @param subscriber Object wishing to receive the messages
    template<class T>
    void subscribe(IDexhandMessageSubscriber<T>* subscriber) {
        subscribers[typeid(T)].push_back(static_cast<void*>(subscriber));
    }

    /// @brief Unsubscribe from messages posted by the Dexhand device
    /// @tparam T Type of message to unsubscribe from
    /// @see dexhand_message.hpp for message types
    /// @param subscriber Object wishing to stop receiving the messages
    template<class T>
    void unsubscribe(IDexhandMessageSubscriber<T>* subscriber) {
        auto& subs = subscribers[typeid(T)];
        subs.erase(std::remove(subs.begin(), subs.end(), subscriber), subs.end());
    }

    /// @brief Sends a command to the Dexhand device
    bool sendCommand(const DexhandCommand& command);

    
private:
    int serialFd;   // File descriptor for serial port

    size_t writeSerial(const uint8_t* data, size_t size);
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

    // Message subscribers
    std::unordered_map<std::type_index, std::vector<void*>> subscribers;

    // Notifies subscribers of a message
    template<typename T>
    void notifyMessageSubscribers(const T& message) {
        auto subs = subscribers.find(typeid(T));
        if (subs != subscribers.end()) {
            for (auto sub : subs->second) {
                static_cast<IDexhandMessageSubscriber<T>*>(sub)->messageReceived(message);
            }
        }
    }

    // Thread safety
    std::mutex updateMutex;
    std::mutex commandMutex;

    // Odometry
    size_t rxBytes;
    size_t txBytes;
    
};

} // namespace dexhand_connect