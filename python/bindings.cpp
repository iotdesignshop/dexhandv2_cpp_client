#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "dexhand_connect.hpp"

using namespace dexhand_connect;

namespace py = pybind11;

// Trampoline class for messages
template<typename T>
class PyIDexhandMessageSubscriber: public dexhand_connect::IDexhandMessageSubscriber<T> {
public:
    using dexhand_connect::IDexhandMessageSubscriber<T>::IDexhandMessageSubscriber; // Inherit constructors

    void messageReceived(const T& message) override {
        PYBIND11_OVERRIDE_PURE(
            void,                    // Return type
            dexhand_connect::IDexhandMessageSubscriber<T>, // Parent class
            messageReceived,         // Name of function in C++ (must match exactly!)
            message                  // Arguments
        );
    }
};


PYBIND11_MODULE(dexhand_connect_py, m) {
    m.doc() = "Dexhand Connect Python"; // optional module docstring}

    py::class_<DexhandConnect>(m, "DexhandConnect")
        .def(py::init<>())
        .def("enumerateDevices", &DexhandConnect::enumerateDevices)
        .def("openSerial", &DexhandConnect::openSerial)
        .def("closeSerial", &DexhandConnect::closeSerial)
        .def("update", &DexhandConnect::update)
        .def("subscribe", [](DexhandConnect& self, IDexhandMessageSubscriber<FirmwareVersionMessage>& subscriber){
            self.subscribe(&subscriber);
        })
        .def("unsubscribe", [](DexhandConnect& self, IDexhandMessageSubscriber<FirmwareVersionMessage>& subscriber){
            self.unsubscribe(&subscriber);
        });
    
    py::class_<DexhandConnect::DexhandUSBDevice>(m, "DexhandUSBDevice")
        .def_readonly("port", &DexhandConnect::DexhandUSBDevice::port)
        .def_readonly("manufacturer", &DexhandConnect::DexhandUSBDevice::manufacturer)
        .def_readonly("product", &DexhandConnect::DexhandUSBDevice::product)
        .def_readonly("serial", &DexhandConnect::DexhandUSBDevice::serial);

    py::class_<FirmwareVersionMessage>(m, "FirmwareVersionMessage")
        .def(py::init<>())
        .def("getMajorVersion", &FirmwareVersionMessage::getMajorVersion)
        .def("getMinorVersion", &FirmwareVersionMessage::getMinorVersion)
        .def("getVersionName", &FirmwareVersionMessage::getVersionName);

    py::class_<IDexhandMessageSubscriber<FirmwareVersionMessage>, PyIDexhandMessageSubscriber<FirmwareVersionMessage>>(m, "FirmwareVersionMessageSubscriber")
        .def(py::init<>())
        .def("messageReceived", &IDexhandMessageSubscriber<FirmwareVersionMessage>::messageReceived);

    

    
}
    
