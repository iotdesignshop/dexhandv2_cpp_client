#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <string.h>
#include <sys/ioctl.h>
#include <vector>
#include <libudev.h>

#include "dexhand_msg.pb.h"
#include "dexhand_msg_types.hpp"


#include "dexhand_connect.hpp"
#include "dexhand_message.hpp"

    

using namespace std;
using namespace dexhand;

namespace dexhand_connect {

static const string DEXHAND_MANUFACTURER = "DexHand";


std::vector<DexhandConnect::DexhandUSBDevice> DexhandConnect::enumerateDevices()
{
    // Use UDEV to enumerate USB devices
    struct udev* udev = udev_new();
    if (!udev) {
        cerr << "Failed to create udev context" << endl;
        return {};
    }

    struct udev_enumerate* enumerate = udev_enumerate_new(udev);
    if (!enumerate) {
        cerr << "Failed to create udev enumerate" << endl;
        udev_unref(udev);
        return {};
    }

    udev_enumerate_add_match_subsystem(enumerate, "tty");
    udev_enumerate_scan_devices(enumerate);

    struct udev_list_entry* devices = udev_enumerate_get_list_entry(enumerate);
    struct udev_list_entry* entry;
    std::vector<DexhandUSBDevice> deviceList;
    udev_list_entry_foreach(entry, devices) {
        const char* path = udev_list_entry_get_name(entry);
        struct udev_device* dev = udev_device_new_from_syspath(udev, path);
        if (dev) {

            struct udev_device* usb_dev = udev_device_get_parent_with_subsystem_devtype(dev, "usb", "usb_device");
            if (usb_dev) {
                const char* manufacturer = udev_device_get_sysattr_value(usb_dev, "manufacturer");
                const char* product = udev_device_get_sysattr_value(usb_dev, "product");
                const char* serial = udev_device_get_sysattr_value(usb_dev, "serial");
                const char* port = udev_device_get_devnode(dev);
                if (manufacturer && product && serial && port) {

                    // Skip if this isn't a dexhand device
                    if (DEXHAND_MANUFACTURER.compare(manufacturer) != 0) {
                        continue;
                    }

                    DexhandUSBDevice device;
                    device.manufacturer = manufacturer;
                    device.product = product;
                    device.serial = serial;
                    device.port = port;
                    deviceList.push_back(device);
                }
            }
            udev_device_unref(dev);
        }
    }   

    udev_enumerate_unref(enumerate);
    udev_unref(udev);

    return deviceList;

}


DexhandConnect::DexhandConnect() : serialFd(-1) {
    receivedData.reserve(MAX_MESSAGE_SIZE);
}

DexhandConnect::~DexhandConnect() {
    closeSerial();
}

bool DexhandConnect::openSerial(const string& port) {
    serialFd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serialFd < 0) {
        cerr << "Error " << errno << " opening " << port << ": " << strerror(errno) << endl;
        return false;
    }

    struct termios options;
    tcgetattr(serialFd, &options);
    
    // Dexhand uses 1MBPS baud rate
    cfsetispeed(&options, B1000000);
    cfsetospeed(&options, B1000000);

    // 8N1
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    // No flow control
    options.c_cflag &= ~CRTSCTS;

    // Enable receiver
    options.c_cflag |= CREAD;

    // Set local mode
    options.c_cflag |= CLOCAL;

    // Disable software flow control
    options.c_iflag &= ~(IXON | IXOFF | IXANY);

    // Prevent CR to LF conversion
    options.c_iflag &= ~(ICRNL | INLCR);
    options.c_oflag &= ~(ONLCR | OCRNL);

    // Raw input
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // Raw output
    options.c_oflag &= ~OPOST;

    // Set the new options for the port
    tcsetattr(serialFd, TCSANOW, &options);

    return true;
}

void DexhandConnect::closeSerial() {
    if (isSerialOpen()) {
        close(serialFd);
        serialFd = -1;
    }
}

size_t DexhandConnect::writeSerial(const uint8_t* data, size_t size) {
    if (!isSerialOpen()) {
        return 0;
    }

    // How much data is available in output stream?
    size_t result = write(serialFd, data, size);
    //tcdrain(serialFd);  // Flush

    return result;

}

size_t DexhandConnect::readSerial(uint8_t* data, size_t size) {
    if (!isSerialOpen()) {
        return 0;
    }
    return read(serialFd, data, size);
}

size_t DexhandConnect::readBytesAvailable() {
    if (!isSerialOpen()) {
        return 0;
    }
    int bytesAvailable;
    ioctl(serialFd, FIONREAD, &bytesAvailable);
    return bytesAvailable;
}

void DexhandConnect::update() {
    // Message processing
    receiveUSBData();
    processMessages();
}

bool DexhandConnect::isValidHeader(const msgHeader* header) {
    return (header->msgStart == 0xff && header->msgSize > 0 && header->msgSize < 512 && header->msgId < DexhandMsgID::NUM_MSGS);
}


void DexhandConnect::receiveUSBData() {
    if (!isSerialOpen()) {
        return;
    }

    // Check for incoming data
    if (readBytesAvailable() > 0) {

        uint8_t data[MAX_MESSAGE_SIZE];
        size_t bytesRead = readSerial(data, MAX_MESSAGE_SIZE);
        
        // Do we already have a partial message?
        if (receivedData.size() > 0) {
            // Add the data to the received data buffer - we can't really know if it's valid yet,
            // we just know that we got a valid header, and more data afterward.
            receivedData.insert(receivedData.end(), data, data + bytesRead);
        }
        else {
            // Check if we have a valid message start
            for (size_t i = 0; i < bytesRead-sizeof(msgHeader); i++) {
                if (isValidHeader(reinterpret_cast<msgHeader*>(data + i))) {
                    receivedData.insert(receivedData.end(), data + i, data + bytesRead);
                    break;
                }
            }
        }
    }

}

// Used for checking endianness on fields that are not protobuf generated 
bool isLittleEndian() {
    union {
        uint32_t i;
        char c[4];
    } bint = {0x01020304};

    return bint.c[0] == 4;  // Little endian if true
}


void DexhandConnect::processMessages() {
    
    // Parse any complete messages
    while(receivedData.size() > 0)
    {  
        // Do we have a complete message to parse?
        if (receivedData.size() < MESSAGE_HEADER_OVERHEAD) {
            // Not enough data to parse
            break;
        }

        assert(isLittleEndian() && "This code will need to be updated for endianness");
        msgHeader* header = reinterpret_cast<msgHeader*>(receivedData.data());
        if (receivedData.size() < MESSAGE_HEADER_OVERHEAD + header->msgSize) {
            // Incomplete message - wait for more data
            break;
        }

        // Check the checksum
        msgTail* tail = reinterpret_cast<msgTail*>(receivedData.data() + MESSAGE_HEADER_OVERHEAD + header->msgSize - MESSAGE_TAIL_SIZE);
        uint8_t checksum = calculateChecksum(&header->msgData, header->msgSize-MESSAGE_TAIL_SIZE);
        if (checksum != tail->checksum || tail->msgEnd != 0x7f) {

                cerr << "Message Size: " << static_cast<unsigned int>(header->msgSize) << endl;
                cerr << "Checksum: " << static_cast<unsigned int>(checksum) << endl;
                cerr << "Tail Checksum: " << static_cast<unsigned int>(tail->checksum) << endl;
                cerr << "Tail End: " << static_cast<unsigned int>(tail->msgEnd) << endl;

                for (int i = 0; i < header->msgSize; i++)
                {
                    cerr << "Data[" << i << "]: " << static_cast<unsigned int>(*(&header->msgData+i)) << endl;
                }    
    
            cerr << "Message Checksum Failed - Discarding" << endl;
            receivedData.erase(receivedData.begin(), receivedData.begin() + MESSAGE_HEADER_OVERHEAD + header->msgSize);
            continue;
        }
        
        // Parse the message
        if (header->msgStart == 0xff)
        {
            switch(header->msgId) {
                case SERVO_FULL_STATUS_MSG:
                {
                    // Individual servo status
                    ServoFullStatusMessage statusMsg;
                    statusMsg.parseMessage(&header->msgData, header->msgSize-MESSAGE_TAIL_SIZE);
                    notifyMessageSubscribers(statusMsg);
                }
                break;
            
                case SERVO_DYNAMICS_LIST_MSG:
                {
                    // Servo dynamics
                    ServoDynamicsMessage dynamicsMsg;
                    dynamicsMsg.parseMessage(&header->msgData, header->msgSize-MESSAGE_TAIL_SIZE);
                    notifyMessageSubscribers(dynamicsMsg);
                }
                break;

                case SERVO_VARS_LIST_MSG:
                {
                    // Servo variables
                    ServoVarsListMessage varsMsg;
                    varsMsg.parseMessage(&header->msgData, header->msgSize-MESSAGE_TAIL_SIZE);
                    notifyMessageSubscribers(varsMsg);
                }
                break;
        
                default:
                    cerr << "Unknown message Type: " << static_cast<unsigned int>(header->msgId) << endl;
                    break;
            }
        }

        // Remove the parsed message from the buffer
        receivedData.erase(receivedData.begin(), receivedData.begin() + MESSAGE_HEADER_OVERHEAD + header->msgSize);
        
    }

}


uint8_t DexhandConnect::calculateChecksum(const uint8_t* data, size_t size) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < size; i++) {
        checksum += data[i];
    }
    return checksum;
}


bool DexhandConnect::sendCommand(const DexhandCommand& command) {
    
    if (!isSerialOpen()) {
        return false;
    }

    // Serialize the command
    string data = command.serialize();

    // Create the message header - it is: 0xff, msgId, msgSize
    string headerData;
    headerData.push_back(0xff);
    headerData.push_back(static_cast<uint8_t>(command.getMessageID()));
    
    assert(isLittleEndian() && "This code will need to be updated for endianness");
    uint16_t msgSize = data.size()+MESSAGE_TAIL_SIZE;
    headerData.push_back(msgSize & 0xff);
    headerData.push_back((msgSize >> 8) & 0xff);

    
    // Calculate the checksum
    uint8_t checksum = calculateChecksum(reinterpret_cast<const uint8_t*>(data.c_str()), data.size());

    // Create the message tail - it is checksum, 0x7f
    string tailData;
    tailData.push_back(checksum);
    tailData.push_back(0x7f);

    // Compose the mesage
    string message = headerData + data + tailData;

    if (message.size() > MAX_MESSAGE_SIZE) {
        cerr << "Command too large to send" << endl;
        return false;
    }

    // Send the command
    if (writeSerial(reinterpret_cast<const uint8_t*>(message.c_str()), message.size()) != message.size()) {
        cerr << "Error sending command" << endl;
        return false;
    }
    
    return true;
}

} // namespace dexhand_connect