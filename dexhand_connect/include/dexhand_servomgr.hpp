/// @file dexhand_servomgr.hpp
/// @brief DexHand ServoManager class

#pragma once

#include <map>
#include <thread>
#include <memory>

#include "dexhand_servo.hpp"

namespace dexhand_connect {
class DexhandConnect;
class FullServoStatusSubscriber;
class DynamicsSubscriber;
class ServoVarsSubscriber;

/// @brief Class to manage the servos on the DexHand
class ServoManager {
    public:
        ServoManager(DexhandConnect& connect);
        virtual ~ServoManager();

        /// @brief Initializes the ServoManager
        /// @param connect Reference to a DexhandConnect object
        /// @param rxFrequency Frequency to receive servo status messages (Hz)
        /// @param txFrequency Frequency to send servo position updates (Hz)
        /// @return true if initialization was successful, false otherwise
        bool start(unsigned int rxFrequency = 100, unsigned int txFrequency = 20);

        /// @brief Checks if the ServoManager is ready to use
        /// @return true if the ServoManager is ready, false otherwise
        /// @note The ServoManager needs to receive the servo variables from the hand before it is ready to use.
        bool isReady() const { return ready; }

        /// @brief Shuts down the ServoManager - Call this to stop update threads and close communications to the hand
        void stop();

        /// @brief Retrieve active servos
        /// @return Map of servos
        const std::map<uint8_t, Servo>& getServos() const { return servos; }

    private:
        bool ready = false;  

        DexhandConnect& dc;
        std::unique_ptr<FullServoStatusSubscriber> fullStatusSubscriber;
        std::unique_ptr<DynamicsSubscriber> dynamicsSubscriber;
        std::unique_ptr<ServoVarsSubscriber> varsSubscriber;

        std::map<uint8_t, Servo> servos;

        bool run_threads = true;

        std::thread rxThread;
        std::thread txThread;

        unsigned int rxFrequency = 100;
        unsigned int txFrequency = 20;
};

} // namespace dexhand_connect