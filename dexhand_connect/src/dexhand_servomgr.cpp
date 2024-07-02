/***
 * @file dexhand_servomgr.cpp
 * @brief DexHand Servo Manager class implementation
 * @copyright Copyright (c) 2024 IoT Design Shop Inc.
 */

#include <thread>
#include <chrono>
#include <iostream>
#include <memory>

#include "dexhand_servomgr.hpp"
#include "dexhand_connect.hpp"

using namespace std;

namespace dexhand_connect {



class FullServoStatusSubscriber : public IDexhandMessageSubscriber<ServoFullStatusMessage> {
    public:
        FullServoStatusSubscriber(ServoManager& mgr) : manager(mgr) {}
        virtual ~FullServoStatusSubscriber() {}

        void messageReceived(const ServoFullStatusMessage& message) override {
            if (!manager.isReady()) return;
            auto servo = manager.getServos().at(message.getServoID());
            if (servo)
                servo->setFullStatus(message.getPosition(), message.getSpeed(), message.getLoad(), message.getTemperature(), message.getVoltage(), message.getStatus());
            else
                cerr << "Servo ID " << (int)message.getServoID() << " not found" << endl;
        }

    private:
        ServoManager& manager;
};
    

class DynamicsSubscriber : public IDexhandMessageSubscriber<ServoDynamicsMessage> {
    public:
        DynamicsSubscriber(ServoManager& mgr) : manager(mgr) {}
        virtual ~DynamicsSubscriber() {}

        void messageReceived(const ServoDynamicsMessage& message) override {
            if (!manager.isReady()) return;
            for (const auto& status : message.getServoStatus()){
                auto servo = manager.getServos().at(status.first);
                if (servo)
                    servo->setDynamics(status.second.getPosition(), status.second.getSpeed(), status.second.getLoad(), status.second.getStatus());
                else
                    cerr << "Servo ID " << (int)status.first << " not found" << endl;   
            }
        }
    private:
        ServoManager& manager;
};
    
class ServoVarsSubscriber : public IDexhandMessageSubscriber<ServoVarsListMessage> {
    public:
        ServoVarsSubscriber(ServoManager& mgr) : manager(mgr) {}
        virtual ~ServoVarsSubscriber() {}

        void messageReceived(const ServoVarsListMessage& message) override {
            // Iterate over each servo and print out the vars
            for (const auto& vars : message.getServoVars()){
                manager.addServo(std::make_shared<Servo>(vars.first, vars.second.getHWMinPosition(), vars.second.getHWMaxPosition(), vars.second.getSWMinPosition(), vars.second.getSWMaxPosition(), vars.second.getHomePosition(), vars.second.getMaxLoadPct(), vars.second.getMaxTemp()));
            }
        }

    private:
        ServoManager& manager;
};
    

ServoManager::ServoManager(DexhandConnect& connect) : 
    dc(connect), 
    fullStatusSubscriber(new FullServoStatusSubscriber(*this)),
    dynamicsSubscriber(new DynamicsSubscriber(*this)),
    varsSubscriber(new ServoVarsSubscriber(*this)) 
    {}

ServoManager::~ServoManager() {
    stop();
}

bool ServoManager::start(unsigned int rxFreq, unsigned int txFreq) {

    run_threads = true;

    // Subscribe to messages
    dc.subscribe(fullStatusSubscriber.get());
    dc.subscribe(dynamicsSubscriber.get());
    dc.subscribe(varsSubscriber.get());

    rxFrequency = rxFreq;
    txFrequency = txFreq;

    // Start the receive thread
    rxThread = std::thread([&](){
        while (run_threads) {
            if (dc.isSerialOpen()) {
                dc.update();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1000/rxFrequency));
        }
    });

    // Start the transmit thread
    txThread = std::thread([&](){
        while (run_threads) {
            if (dc.isSerialOpen()) {
                sendServoMessages();
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

    // Unsubscribe from messages
    dc.unsubscribe(fullStatusSubscriber.get());
    dc.unsubscribe(dynamicsSubscriber.get());
    dc.unsubscribe(varsSubscriber.get());
}

void ServoManager::sendServoMessages() {
    // Do we have a copy of the servos from last iteration?
    if (lastServos.size() > 0) {
        // Prepare a position message - all servos get aggregated into one message
        SetServoPositionsCommand poscmd;
        bool sendPosCmd = false;

        for (const auto& servo : servos) {
            // Check if the servo target has changed
            auto last = lastServos.find(servo.first);
            if (last != lastServos.end()) {
                if (servo.second->getTarget() != last->second->getTarget()) {
                    poscmd.setServoPosition(servo.first, servo.second->getTarget());
                    sendPosCmd = true;
                }
            }

            // Check on tuning parameters
            if (servo.second->getHWMinPosition() != last->second->getHWMinPosition() ||
                servo.second->getHWMaxPosition() != last->second->getHWMaxPosition() ||
                servo.second->getSWMinPosition() != last->second->getSWMinPosition() ||
                servo.second->getSWMaxPosition() != last->second->getSWMaxPosition() ||
                servo.second->getHomePosition() != last->second->getHomePosition() ||
                servo.second->getMaxLoadPct() != last->second->getMaxLoadPct() ||
                servo.second->getMaxTemp() != last->second->getMaxTemp() ||
                servo.second->getTorqueEnableRequest() != last->second->getTorqueEnable()) {

                SetServoVarsCommand varscmd;
                varscmd.setID(servo.first);
                
                if (servo.second->getHWMinPosition() != last->second->getHWMinPosition()) {
                    varscmd.setHWMinPosition(servo.second->getHWMinPosition());
                }
                if (servo.second->getHWMaxPosition() != last->second->getHWMaxPosition()) {
                    varscmd.setHWMaxPosition(servo.second->getHWMaxPosition());
                }
                if (servo.second->getSWMinPosition() != last->second->getSWMinPosition()) {
                    varscmd.setSWMinPosition(servo.second->getSWMinPosition());
                }
                if (servo.second->getSWMaxPosition() != last->second->getSWMaxPosition()) {
                    varscmd.setSWMaxPosition(servo.second->getSWMaxPosition());
                }
                if (servo.second->getHomePosition() != last->second->getHomePosition()) {
                    varscmd.setHomePosition(servo.second->getHomePosition());
                }
                if (servo.second->getMaxLoadPct() != last->second->getMaxLoadPct()) {
                    varscmd.setMaxLoadPct(servo.second->getMaxLoadPct());
                }
                if (servo.second->getMaxTemp() != last->second->getMaxTemp()) {
                    varscmd.setMaxTemp(servo.second->getMaxTemp());
                }
                if (servo.second->getTorqueEnableRequest() != last->second->getTorqueEnable()) {
                    varscmd.setTorqueEnable(servo.second->getTorqueEnableRequest());
                }

                dc.sendCommand(varscmd);
            }
        }


        // Send the message
        if (sendPosCmd) {
            dc.sendCommand(poscmd);
        }
    }

    // Deep copy current servos into lastServos for comparison next iteration
    for (const auto& servo : servos) {
        lastServos[servo.first] = std::make_shared<Servo>(*servo.second);
    }
}


} // namespace dexhand_connect