#include <iostream>
#include <unistd.h>
#include "dexhand_connect.hpp"
#include "CLI11.hpp"

using namespace dexhand_connect;
using namespace std;

int main(int argc, char** argv){

    // Parse command line args
    CLI::App app{"Dexhand Connect CLI"};
    string port = "/dev/ttyACM1";
    app.add_option("-p,--port", port, "Serial port to connect to");
    CLI11_PARSE(app, argc, argv);

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
            data[bytesRead] = '\0'; // For printing
            cout << "Read: " << data << endl;
            }
    }
    
}
