#pragma once

#include <string>
#include <vector>

namespace dexhand_connect {

class DexhandConnect {
public:
    DexhandConnect();
    ~DexhandConnect();

    struct DexhandUSBDevice {
        std::string port;
        std::string manufacturer;
        std::string product;
        std::string serial;
    };

    static std::vector<DexhandUSBDevice> enumerateDevices();

    
    bool openSerial(const std::string& port);
    void closeSerial();
    bool isSerialOpen() const { return serialFd != -1; }

    size_t writeSerial(const char* data, size_t size);

    size_t readSerial(char* data, size_t size);

    size_t readBytesAvailable();
    
private:
    int serialFd;

};

} // namespace dexhand_connect