#include <iostream>
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

    while(true) {
        hand.update();
        usleep(1000);
    }
    
}
