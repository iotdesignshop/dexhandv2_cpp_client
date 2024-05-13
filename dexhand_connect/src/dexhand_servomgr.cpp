/// @file dexhand_servomgr.cpp
/// @brief DexHand ServoManager class implementation

#include <thread>
#include <chrono>
#include <iostream>

#include "dexhand_servomgr.hpp"
#include "dexhand_connect.hpp"

using namespace std;

namespace dexhand_connect {



class FullServoStatusSubscriber : public IDexhandMessageSubscriber<ServoFullStatusMessage> {
    public:
        FullServoStatusSubscriber(ServoManager& mgr) : manager(mgr) {}
        virtual ~FullServoStatusSubscriber() {}

        void messageReceived(const ServoFullStatusMessage& message) override {
            cout << "Full Status for Servo ID: " << (int)message.getServoID() << endl;
            cout << "--------------------------------" << endl;
            cout << "Status: " << (int)message.getStatus() << endl;
            cout << "Position: " << message.getPosition() << endl;
            cout << "Speed: " << message.getSpeed() << endl;
            cout << "Load: " << message.getLoad() << endl;
            cout << "Voltage: " << (int)message.getVoltage() << endl;
            cout << "Temperature: " << (int)message.getTemperature() << endl;
            cout << "--------------------------------" << endl << endl;
        }

    private:
        ServoManager& manager;
};
    

class DynamicsSubscriber : public IDexhandMessageSubscriber<ServoDynamicsMessage> {
    public:
        DynamicsSubscriber(ServoManager& mgr) : manager(mgr) {}
        virtual ~DynamicsSubscriber() {}

        void messageReceived(const ServoDynamicsMessage& message) override {
            cout << "Dynamics message received" << endl;
            cout << "Num servos: " << message.getNumServos() << endl;
            cout << "ID:\tStatus\tPos\tSpd\tLoad" << endl;
            cout << "------------------------------------" << endl;

            for (const auto& status : message.getServoStatus()){
                cout << (int)status.first << "\t";
                cout << (int)status.second.getStatus() << "\t";
                cout << status.second.getPosition() << "\t";
                cout << status.second.getSpeed() << "\t";
                cout << status.second.getLoad() << endl;
            }
            cout << "------------------------------------" << endl << endl;
        }
    private:
        ServoManager& manager;
};
    
class ServoVarsSubscriber : public IDexhandMessageSubscriber<ServoVarsListMessage> {
    public:
        ServoVarsSubscriber(ServoManager& mgr) : manager(mgr) {}
        virtual ~ServoVarsSubscriber() {}

        void messageReceived(const ServoVarsListMessage& message) override {
            cout << "Servo Vars message received" << endl;
            cout << "Num servos: " << message.getNumServos() << endl;
            cout << "ID\tHWMin\tHWMax\tSWMin\tSWMax\tHome\tMaxLoad\tMaxTemp" << endl;
            cout << "------------------------------------------------------------------" << endl;


            // Iterate over each servo and print out the vars
            for (const auto& vars : message.getServoVars()){
                cout << (int)vars.first << "\t";
                cout << vars.second.getHWMinPosition() << "\t";
                cout << vars.second.getHWMaxPosition() << "\t";
                cout << vars.second.getSWMinPosition() << "\t";
                cout << vars.second.getSWMaxPosition() << "\t";
                cout << vars.second.getHomePosition() << "\t";
                cout << (int)vars.second.getMaxLoadPct() << "\t";
                cout << (int)vars.second.getMaxTemp() << endl;
            }
            
            cout << "------------------------------------" << endl << endl;
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
                dc.update();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1000/txFrequency));
        }
    });

    ready = true;

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



} // namespace dexhand_connect