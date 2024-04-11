#include <iostream>
#include <unistd.h>
#include "dexhand_connect.hpp"
#include "CLI11.hpp"
#include "dexhand_msg.pb.h"

using namespace dexhand_connect;
using namespace std;

struct msgHeader {
    uint8_t msgStart;
    uint8_t msgId;
    uint8_t msgSize;
    uint8_t msgData;
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

    // Parse command line args
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

    // Main loop 
    while(true) {
        if (hand.readBytesAvailable() > 0) {
            char data[256];
            size_t bytesRead = hand.readSerial(data, 256);
            cout << "Bytes Read: " << bytesRead << endl;
            msgHeader* header = reinterpret_cast<msgHeader*>(data);
            if (header->msgStart == 0xff && header->msgId == 0x01)
            {
                dexhand::ServoStatus statusMsg;
                statusMsg.ParseFromArray(&header->msgData, header->msgSize);

                cout << "Servo ID: " << statusMsg.servoid() << " Message Size:" << header->msgSize << endl;
                cout << "\tStatus: " << statusMsg.status() << endl;
                cout << "\tPosition: " << statusMsg.position() << endl;
                cout << "\tSpeed: " << statusMsg.speed() << endl;
                cout << "\tLoad: " << statusMsg.load() << endl;
                cout << "\tVoltage: " << statusMsg.voltage() << endl;
                cout << "\tTemperature: " << statusMsg.temperature() << endl;

            }

        }
    }
    
}
