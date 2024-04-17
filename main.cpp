#include <iostream>
#include <chrono>
#include <thread>
#include <unistd.h>
#include "dexhand_connect.hpp"
#include "CLI11.hpp"


using namespace dexhand_connect;
using namespace std;



class FullServoStatusSubscriber : public IDexhandMessageSubscriber<ServoFullStatusMessage> {
    public:
        void messageRecieved(const ServoFullStatusMessage& message) override {
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
};

class DynamicsSubscriber : public IDexhandMessageSubscriber<ServoDynamicsMessage> {
    public:
        void messageRecieved(const ServoDynamicsMessage& message) override {
            cout << "Dynamics message received" << endl;
            cout << "Num servos: " << message.getNumServos() << endl;
            cout << "ID:\tStatus\tPos\tSpd\tLoad" << endl;
            cout << "------------------------------------" << endl;
            for (size_t i = 0; i < message.getNumServos(); i++){
                const ServoDynamicsMessage::ServoStatus& status = message.getServoStatus(i);
                cout << (int)status.getServoID() << "\t";
                cout << (int)status.getStatus() << "\t";
                cout << status.getPosition() << "\t";
                cout << status.getSpeed() << "\t";
                cout << status.getLoad() << endl;
            }
            cout << "------------------------------------" << endl << endl;
        }
};

class ServoVarsSubscriber : public IDexhandMessageSubscriber<ServoVarsListMessage> {
    public:
        void messageRecieved(const ServoVarsListMessage& message) override {
            cout << "Servo Vars message received" << endl;
            cout << "Num servos: " << message.getNumServos() << endl;
            cout << "ID\tHWMin\tHWMax\tSWMin\tSWMax\tHome\tMaxLoad\tMaxTemp" << endl;
            cout << "------------------------------------------------------------------" << endl;
            for (size_t i = 0; i < message.getNumServos(); i++){
                const ServoVarsListMessage::ServoVars& vars = message.getServoVars(i);
                cout << (int)vars.getServoID() << "\t";
                cout << vars.getHWMinPosition() << "\t";
                cout << vars.getHWMaxPosition() << "\t";
                cout << vars.getSWMinPosition() << "\t";
                cout << vars.getSWMaxPosition() << "\t";
                cout << vars.getHomePosition() << "\t";
                cout << (int)vars.getMaxLoadPct() << "\t";
                cout << (int)vars.getMaxTemp() << endl;
            }
            cout << "------------------------------------" << endl << endl;
        }
};


int main(int argc, char** argv){


    string port;

    // Default is to look for first Dexhand device
    vector<DexhandConnect::DexhandUSBDevice> devices = DexhandConnect::enumerateDevices();
    if (devices.size() > 0){
        cout << "Found " << devices.size() << " Dexhand devices" << endl;
        cout << "Manufacturer: " << devices[0].manufacturer << endl;
        cout << "Product: " << devices[0].product << endl;
        cout << "Serial: " << devices[0].serial << endl;
        cout << "Port: " << devices[0].port << endl;

        port = devices[0].port;
    }
    else{
        cerr << "No Dexhand devices found" << endl;
        return 1;
    }

    // Parse command line args - allows user to override port if multiple devices are found
    CLI::App app{"Dexhand Connect CLI"};
    app.add_option("-p,--port", port, "Serial port to connect to");
    CLI11_PARSE(app, argc, argv);

    // Ensure we have a port to try connect to
    if (port.empty()){
        cerr << "No port specified" << endl;
        return 1;
    }

    // Attempt serial connection
    DexhandConnect hand;
    if (hand.openSerial(port)){
        cout << "Connected to " << port << endl;
    }
    else{
        cerr << "Failed to connect to " << port << endl;
        return 1;
    }

    // Subscribe to messages
    FullServoStatusSubscriber fullStatusSubscriber;
    hand.subscribe(&fullStatusSubscriber);

    DynamicsSubscriber dynamicsSubscriber;
    hand.subscribe(&dynamicsSubscriber);

    ServoVarsSubscriber varsSubscriber;
    hand.subscribe(&varsSubscriber);

    #define MIN_POS 400
    #define MAX_POS 1300
    #define SERVO_MIN 111
    #define SERVO_MAX 114
    uint16_t testpos = 400;

    hand.update();

    // Tuning parameters
    /*SetServoVarsCommand varsCmd;
    varsCmd.setID(114);
    varsCmd.setHWMaxPosition(1600);
    varsCmd.setMaxLoadPct(50);
    hand.sendCommand(varsCmd);
    hand.update();*/

    


    
    while(true) {
        hand.update();
        
        // Send a command to set the position of a servo
        SetServoPositionsCommand cmd;
        for (uint8_t i = SERVO_MIN; i <= SERVO_MAX; i++){
            cmd.setServoPosition(i, testpos);
        }
        hand.sendCommand(cmd);
        testpos += 20;
        if (testpos > MAX_POS){
            testpos = MIN_POS;
        }

        // Don't hammer the device
        std::this_thread::sleep_for(std::chrono::milliseconds(20));     //50Hz

    }
    
}
