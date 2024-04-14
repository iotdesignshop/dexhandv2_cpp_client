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

    void update();

    
private:
    int serialFd;

    size_t writeSerial(const char* data, size_t size);
    size_t readSerial(uint8_t* data, size_t size);
    size_t readBytesAvailable();

    struct msgHeader {
        uint8_t msgStart;
        uint8_t msgId;
        uint16_t msgSize;
        uint8_t msgData;
    };

    struct msgTail {
        uint8_t checksum;
        uint8_t msgEnd;
    };

    static const size_t MESSAGE_HEADER_OVERHEAD = 4;    // Bytes of header/checksum data on messages
    static const size_t MESSAGE_TAIL_SIZE = 2;          // Bytes of checksum/msgEnd data on messages


    static const size_t MAX_MESSAGE_SIZE = 512;    // Maximum size of a message
    std::vector<uint8_t> receivedData;   // Buffered data from serial port
    void receiveUSBData();
    void processMessages();
    bool isValidHeader(const DexhandConnect::msgHeader* header);
    uint8_t calculateChecksum(const uint8_t* data, size_t size);

    
    
};

} // namespace dexhand_connect