#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <string.h>
#include <sys/ioctl.h>

#include "dexhand_connect.hpp"

using namespace std;

namespace dexhand_connect {

DexhandConnect::DexhandConnect() : serialFd(-1) {
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

size_t DexhandConnect::writeSerial(const char* data, size_t size) {
    if (!isSerialOpen()) {
        return false;
    }
    return write(serialFd, data, size);
}

size_t DexhandConnect::readSerial(char* data, size_t size) {
    if (!isSerialOpen()) {
        return false;
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


} // namespace dexhand_connect