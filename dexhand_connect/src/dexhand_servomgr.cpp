/// @file dexhand_servomgr.cpp
/// @brief DexHand ServoManager class implementation

#include <thread>
#include <chrono>

#include "dexhand_servomgr.hpp"
#include "dexhand_connect.hpp"

namespace dexhand_connect {

ServoManager::ServoManager(DexhandConnect& connect) : dc(connect) {}

ServoManager::~ServoManager() {
    stop();
}

bool ServoManager::start(unsigned int rxFrequency, unsigned int txFrequency) {

    run_threads = true;

    // Start the receive thread
    std::thread rxThread([&](){
        while (run_threads) {
            if (dc.isSerialOpen()) {
                dc.update();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1000/rxFrequency));
        }
    });

    // Start the transmit thread
    std::thread txThread([&](){
        while (run_threads) {
            if (dc.isSerialOpen()) {
                dc.update();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1000/txFrequency));
        }
    });

    return true;
}

void ServoManager::stop() {
    // Stop the threads
    if (run_threads) {
        run_threads = false;
        rxThread.join();
        txThread.join();
    }
}



} // namespace dexhand_connect