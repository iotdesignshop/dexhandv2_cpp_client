/// @file joint_angles.cpp
/// @brief Example of using the DexHand Connect library to control the DexHand using the JointAngleController class

#include <iostream>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>


#include "dexhand_connect.hpp"
#include "CLI11.hpp"
#include "dexhand_joint_angle_controller.hpp"
#include "dexhand_servomgr.hpp"


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
    hand.subscribe<FirmwareVersionMessage>(std::make_shared<FirmwareVersionSubscriber>());

    // Reset hand to enable motion
    hand.resetHand();

    // Set up the servo manager
    ServoManager servoManager(hand);
    servoManager.start(100,50);

    // Wait for the servo manager to be ready
    while (!servoManager.isReady()){
        cout << "Waiting for ServoManager to be ready..." << endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Set up joint angle controller
    JointAngleController controller(servoManager);
    controller.start();

    while(!kbhit()) {

        // First - move through flexor on all fingers
        std::vector<JointAngleController::JointID> flexJoints = {
            controller.getJointID("index_flexor"), 
            controller.getJointID("middle_flexor"), 
            controller.getJointID("ring_flexor"), 
            controller.getJointID("pinky_flexor"), 
            controller.getJointID("thumb_flexor")};

        for (JointAngleController::JointID joint : flexJoints){
            cout << "Moving " << controller.jointNames[joint] << " through range of motion..." << endl;
            float min = controller.getJointMin(joint);
            float max = controller.getJointMax(joint);
            for (float angle = min; angle < max; angle++){
                controller.setJointAngle(joint, angle);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            controller.setJointAngle(joint, min);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    
        // Second - move through pitch on all fingers
        std::vector<JointAngleController::JointID> pitchJoints = {
            controller.getJointID("index_pitch"), 
            controller.getJointID("middle_pitch"), 
            controller.getJointID("ring_pitch"), 
            controller.getJointID("pinky_pitch"),
            controller.getJointID("thumb_pitch")};
        
        for (JointAngleController::JointID joint : pitchJoints){
            cout << "Moving " << controller.jointNames[joint] << " through range of motion..." << endl;
            float min = controller.getJointMin(joint);
            float max = controller.getJointMax(joint);
            for (float angle = min; angle < max; angle++){
                controller.setJointAngle(joint, angle);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            controller.setJointAngle(joint, min);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        // Third - move through yaw on all fingers
        std::vector<JointAngleController::JointID> yawJoints = {
            controller.getJointID("index_yaw"), 
            controller.getJointID("middle_yaw"), 
            controller.getJointID("ring_yaw"), 
            controller.getJointID("pinky_yaw"), 
            controller.getJointID("thumb_yaw")};
        
        for (JointAngleController::JointID joint : yawJoints){
            cout << "Moving " << controller.jointNames[joint] << " through range of motion..." << endl;
            float min = controller.getJointMin(joint);
            float max = controller.getJointMax(joint);

            // Move to min
            for (float angle = 0; angle > min; angle--){
                controller.setJointAngle(joint, angle);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            // Move from min to max
            for (float angle = min; angle < max; angle++){
                controller.setJointAngle(joint, angle);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            // Move from max to 0
            for (float angle = max; angle > 0; angle--){
                controller.setJointAngle(joint, angle);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

        }

        // Fourth move through pitch + flexor on all fingers
        for (int c = 0; c < 4; ++c) {
            cout << "Moving " << controller.jointNames[pitchJoints[c]] << " and " << controller.jointNames[flexJoints[c]] << " through range of motion..." << endl;
            for (float p = 0; p <= 100.0; p+=1.0) {
                float pa = controller.getJointMin(pitchJoints[c]) + (controller.getJointMax(pitchJoints[c]) - controller.getJointMin(pitchJoints[c])) * p / 100.0;
                float fa = controller.getJointMin(flexJoints[c]) + (controller.getJointMax(flexJoints[c]) - controller.getJointMin(flexJoints[c])) * p / 100.0;
                controller.setJointAngle(pitchJoints[c], pa);
                controller.setJointAngle(flexJoints[c], fa);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            for (float p = 100.0; p >= 0; p-=1.0) {
                float pa = controller.getJointMin(pitchJoints[c]) + (controller.getJointMax(pitchJoints[c]) - controller.getJointMin(pitchJoints[c])) * p / 100.0;
                float fa = controller.getJointMin(flexJoints[c]) + (controller.getJointMax(flexJoints[c]) - controller.getJointMin(flexJoints[c])) * p / 100.0;
                controller.setJointAngle(pitchJoints[c], pa);
                controller.setJointAngle(flexJoints[c], fa);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            
            }
        }


        
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
