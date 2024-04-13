#include <iostream>
#include <unistd.h>
#include "dexhand_connect.hpp"
#include "CLI11.hpp"
#include "dexhand_msg.pb.h"
#include "dexhand_msg_types.hpp"

using namespace dexhand_connect;
using namespace dexhand;
using namespace std;


struct msgHeader {
    uint8_t msgStart;
    uint8_t msgId;
    uint16_t msgSize;
    uint8_t msgData;
};

static const size_t MESSAGE_HEADER_OVERHEAD = 4;    // Bytes of header/checksum data on messages

bool isValidHeader(const msgHeader* header) {
    return (header->msgStart == 0xff && header->msgSize > 0 && header->msgSize < 512 && header->msgId < DexhandMsgID::NUM_MSGS);
}

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
    vector<uint8_t> receivedData;
    receivedData.reserve(512);

    while(true) {

        // Check for incoming data
        if (hand.readBytesAvailable() > 0) {
            uint8_t data[256];
            size_t bytesRead = hand.readSerial(data, 256);
            cout << "Bytes Read: " << bytesRead << endl;

            // Do we already have a partial message?
            if (receivedData.size() > 0) {
                cout << "Partial Message Bytes: " << receivedData.size() << endl;
            
                // Add the data to the received data buffer - we can't really know if it's valid yet,
                // we just know that we got a valid header, and more data afterward.
                receivedData.insert(receivedData.end(), data, data + bytesRead);
            }
            else {
                // Check if we have a valid message start
                for (size_t i = 0; i < bytesRead-sizeof(msgHeader); i++) {
                    if (isValidHeader(reinterpret_cast<msgHeader*>(data + i))) {
                        cout << "Valid Header Found" << endl;
                        receivedData.insert(receivedData.end(), data + i, data + bytesRead);
                        break;
                    }
                }
            }
        }

        // Parse any complete messages
        while(receivedData.size() > 0)
        {  
            // Do we have a complete message to parse?
            if (receivedData.size() < MESSAGE_HEADER_OVERHEAD) {
                // Not enough data to parse
                break;
            }
            msgHeader* header = reinterpret_cast<msgHeader*>(receivedData.data());
            if (receivedData.size() < MESSAGE_HEADER_OVERHEAD + header->msgSize) {
                // Incomplete message
                cout << "Incomplete message Bytes: " << receivedData.size() << " Expected: " << sizeof(msgHeader) + header->msgSize << endl;
                break;
            }
            
            // Parse the message
            if (header->msgStart == 0xff && header->msgId == 0x01)
            {
                // Individual servo status
                dexhand::ServoStatus statusMsg;
                statusMsg.ParseFromArray(&header->msgData, header->msgSize);

                cout << "Servo ID: " << statusMsg.servoid() << " Message Size:" << static_cast<unsigned int>(header->msgSize) << endl;
                cout << "\tStatus: " << statusMsg.status() << endl;
                cout << "\tPosition: " << statusMsg.position() << endl;
                cout << "\tSpeed: " << statusMsg.speed() << endl;
                cout << "\tLoad: " << statusMsg.load() << endl;
                cout << "\tVoltage: " << statusMsg.voltage() << endl;
                cout << "\tTemperature: " << statusMsg.temperature() << endl;
            }
            else if(header->msgStart == 0xff && header->msgId == 0x02)
            {
                // Servo status list
                dexhand::ServoStatusList statusListMsg;
                statusListMsg.ParseFromArray(&header->msgData, header->msgSize);

                cout << "Servo Status List Message Size:" << static_cast<unsigned int>(header->msgSize) << endl;
                
                cout << "Servo\tStatus\tPos\tLoad\n";
                cout << "--------------------------------\n";
                for (int i = 0; i < statusListMsg.servos_size(); i++)
                {
                    const dexhand::ServoStatus& statusMsg = statusListMsg.servos(i);
                    cout << statusMsg.servoid() << "\t" << statusMsg.status() << "\t" << statusMsg.position() << "\t" << statusMsg.load() << endl;
                }
            }
            else 
            {
                cout << "Unknown message Type: " << static_cast<unsigned int>(header->msgId) << endl;
            }

            // Remove the parsed message from the buffer
            cout << "Removing " << MESSAGE_HEADER_OVERHEAD + header->msgSize << " bytes from buffer" << endl;
            receivedData.erase(receivedData.begin(), receivedData.begin() + MESSAGE_HEADER_OVERHEAD + header->msgSize);
            cout << "Message Parsed. Bytes Remaining: " << receivedData.size() << endl;
        }
    }
    
}
