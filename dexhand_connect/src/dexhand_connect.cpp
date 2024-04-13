#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <string.h>
#include <sys/ioctl.h>
#include <vector>
#include <libudev.h>

#include "dexhand_connect.hpp"

using namespace std;

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
        return 0;
    }
    return write(serialFd, data, size);
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


} // namespace dexhand_connect