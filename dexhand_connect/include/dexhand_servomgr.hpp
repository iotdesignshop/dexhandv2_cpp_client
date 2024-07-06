/***
 * @file dexhand_servomgr.hpp
 * @brief The DexHand ServoManager class provides an abstraction interface to manage the servos on the DexHand. 
 * @copyright Copyright (c) 2024 IoT Design Shop Inc.
 */

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
        bool isReady() const { return (servos.size() > 0); }

        /// @brief Shuts down the ServoManager - Call this to stop update threads and close communications to the hand
        void stop();

        /// @brief Add a servo to the ServoManager
        void addServo(std::shared_ptr<Servo> servo) { servos[servo->getID()] = servo;}

        /// @brief Retrieve a servo by ID
        /// @param id Servo ID
        /// @return Pointer to the servo, or nullptr if not found
        std::shared_ptr<Servo> getServo(uint8_t id) const {
            auto it = servos.find(id);
            if (it != servos.end()) {
                return it->second;
            }
            return nullptr;
        }

        /// @brief Retrieve active servos
        /// @return Map of servos
        const std::map<uint8_t, std::shared_ptr<Servo>> getServos() const { return servos; }

    private:
        DexhandConnect& dc;
        std::shared_ptr<FullServoStatusSubscriber> fullStatusSubscriber;
        std::shared_ptr<DynamicsSubscriber> dynamicsSubscriber;
        std::shared_ptr<ServoVarsSubscriber> varsSubscriber;

        std::map<uint8_t, std::shared_ptr<Servo>> servos;
        std::map<uint8_t, std::shared_ptr<Servo>> lastServos;

        bool run_threads = true;

        std::thread rxThread;
        std::thread txThread;

        unsigned int rxFrequency = 100;
        unsigned int txFrequency = 20;

        void sendServoMessages();
};

} // namespace dexhand_connect