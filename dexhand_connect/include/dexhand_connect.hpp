#pragma once

#include <string>

namespace dexhand_connect {

class DexhandConnect {
public:
    DexhandConnect();
    ~DexhandConnect();
    
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