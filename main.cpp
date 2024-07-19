#include <iostream>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>

#include "dexhand_connect.hpp"
#include "dexhand_servomgr.hpp"
#include "CLI11.hpp"


using namespace dexhand_connect;
using namespace std;

int kbhit();


class FirmwareVersionSubscriber : public IDexhandMessageSubscriber<FirmwareVersionMessage> {
    public:
        void messageReceived(const FirmwareVersionMessage& message) override {
            cout << "Dexhand Firmware: " << message.getVersionName() << endl;
            cout << "Firmware version: " << (int)message.getMajorVersion() << "." << (int)message.getMinorVersion() << endl;
        }
};

void displayServoTable(const ServoManager& servoManager) {
    cout << "ID\tStatus\tPos\tSpeed\tLoad\tVoltage\tTemp\thwMin\thwMax\tswMin\tswMax\thome\tload%\tmaxTemp" << endl;
        cout << "--------------------------------------------------------------------------------------------------------------------" << endl;
        for (const auto& servo : servoManager.getServos()){
            cout << (int)servo.second->getID() << "\t" 
                << (int)servo.second->getStatus() << "\t" 
                << servo.second->getPosition() << "\t" 
                << servo.second->getSpeed() << "\t" 
                << servo.second->getLoad() << "\t" 
                << (int)servo.second->getVoltage() << "\t" 
                << (int)servo.second->getTemperature() << "\t" 
                << servo.second->getHWMinPosition() << "\t" 
                << servo.second->getHWMaxPosition() << "\t" 
                << servo.second->getSWMinPosition() << "\t" 
                << servo.second->getSWMaxPosition() << "\t" 
                << servo.second->getHomePosition() << "\t" 
                << (int)servo.second->getMaxLoadPct() << "\t" 
                << (int)servo.second->getMaxTemp() << endl;
        }

}


int main(int argc, char** argv){


    DexhandConnect hand;
    string port;

    // Default is to look for first Dexhand device
    vector<DexhandConnect::DexhandUSBDevice> devices = hand.enumerateDevices();
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
    if (hand.openSerial(port)){
        cout << "Connected to " << port << endl;
    }
    else{
        cerr << "Failed to connect to " << port << endl;
        return 1;
    }

    // Subscribe to messages
    FirmwareVersionSubscriber firmwareSubscriber;
    hand.subscribe(&firmwareSubscriber);

    // Set up the servo manager
    ServoManager servoManager(hand);
    servoManager.start(100,50);

    // Reset the hand to enable motion
    hand.resetHand();

    // Wait for the servo manager to be ready
    while (!servoManager.isReady()){
        cout << "Waiting for ServoManager to be ready..." << endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    
    // Run through the range of motion on fingers and thumb
    uint16_t servoIDs[5][3] = {
        {101,102,111},
        {103,104,112},
        {105,106,113},
        {107,108,114},
        {109,110,115}
    };
    
    // Run until key press
    uint16_t finger = 0;
    float v = 0;
    float d = 0.01;


    while(!kbhit()) {

        // Move our position
        v += d;
        if (v > 1){ // Top of range? Go back down
            v = 1;
            d = -d;
        }
        if (v < 0){ // Bottom of range? Next finger, and Go back up
            if (++finger > 4) {
                finger = 0;
            }
            v = 0;
            d = -d;
        }

        // Set the target position of the servo
        for (int i = 0; i < 3; i++){
            servoManager.getServo(servoIDs[finger][i])->setTargetNormalized(v);
        }

        // Draw table of servos
        displayServoTable(servoManager);

        // Limit updates to 50Hz
        std::this_thread::sleep_for(std::chrono::milliseconds(20));     //50Hz
    }
    


    // Stop the servo manager
     servoManager.stop();

    
}

int kbhit() {
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}
