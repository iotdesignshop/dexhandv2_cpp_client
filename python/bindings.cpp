#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
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

// Type casters for message data
namespace pybind11 { namespace detail {

template <> struct type_caster<FirmwareVersionMessage> {
public:
    PYBIND11_TYPE_CASTER(FirmwareVersionMessage, _("FirmwareVersionMessage"));

    bool load(handle src, bool) {
        assert(false); // Unimplemented
        return false;
    }

    static handle cast(const FirmwareVersionMessage& src, return_value_policy, handle) {
        py::dict d;
        d["majorVersion"] = src.getMajorVersion();
        d["minorVersion"] = src.getMinorVersion();
        d["versionName"] = src.getVersionName();
        return d.release();
        }
};

template <> struct type_caster<DexhandConnect::DexhandUSBDevice> {
public:
    PYBIND11_TYPE_CASTER(DexhandConnect::DexhandUSBDevice, _("DexhandUSBDevice"));

    bool load(handle src, bool) {
        assert(false); // Unimplemented
        return false;
    }

    static handle cast(const DexhandConnect::DexhandUSBDevice& src, return_value_policy, handle) {
        py::dict d;
        d["port"] = src.port;
        d["manufacturer"] = src.manufacturer;
        d["product"] = src.product;
        d["serial"] = src.serial;
        return d.release();
        }
};

template <> struct type_caster<ServoFullStatusMessage> {
public:
    PYBIND11_TYPE_CASTER(ServoFullStatusMessage, _("ServoFullStatusMessage"));

    bool load(handle src, bool) {
        assert(false); // Unimplemented
        return false;
    }

    static handle cast(const ServoFullStatusMessage& src, return_value_policy, handle) {
        py::dict d;
        d["status"] = src.getStatus();
        d["position"] = src.getPosition();
        d["speed"] = src.getSpeed();
        d["load"] = src.getLoad();
        d["voltage"] = src.getVoltage();
        d["temperature"] = src.getTemperature();
        
        py::dict p;
        p[py::int_(src.getServoID())] = d.release();
        return p.release();
    }
};

template <> struct type_caster<ServoVarsListMessage> {
public:
    PYBIND11_TYPE_CASTER(ServoVarsListMessage, _("ServoVarsListMessage"));

    bool load(handle src, bool) {
        assert(false); // Unimplemented
        return false;
    }

    static handle cast(const ServoVarsListMessage& src, return_value_policy, handle) {
        py::dict d;
        for (const auto& [id, vars] : src.getServoVars()) {
            d[py::int_(id)] = vars;
        }
        return d.release();
    }
};

template<> struct type_caster<ServoVarsListMessage::ServoVars> {
public:
    PYBIND11_TYPE_CASTER(ServoVarsListMessage::ServoVars, _("ServoVars"));

    bool load(handle src, bool) {
        assert(false); // Unimplemented
        return false;
    }

    static handle cast(const ServoVarsListMessage::ServoVars& src, return_value_policy, handle) {
        py::dict d;
        d["hwMinPosition"] = src.getHWMinPosition();
        d["hwMaxPosition"] = src.getHWMaxPosition();
        d["swMinPosition"] = src.getSWMinPosition();
        d["swMaxPosition"] = src.getSWMaxPosition();
        d["homePosition"] = src.getHomePosition();
        d["maxLoadPct"] = src.getMaxLoadPct();
        d["maxTemp"] = src.getMaxTemp();
        return d.release();
    }

    
};

template<> struct type_caster<ServoDynamicsMessage::ServoStatus> {
public:
    PYBIND11_TYPE_CASTER(ServoDynamicsMessage::ServoStatus, _("ServoStatus"));

    bool load(handle src, bool) {
        assert(false); // Unimplemented
        return false;
    }

    static handle cast(const ServoDynamicsMessage::ServoStatus& src, return_value_policy, handle) {
        py::dict d;
        d["servoID"] = src.getServoID();
        d["status"] = src.getStatus();
        d["position"] = src.getPosition();
        d["speed"] = src.getSpeed();
        d["load"] = src.getLoad();
        return d.release();
    }
};

template<> struct type_caster<ServoDynamicsMessage> {
public:
    PYBIND11_TYPE_CASTER(ServoDynamicsMessage, _("ServoDynamicsMessage"));

    bool load(handle src, bool) {
        assert(false); // Unimplemented
        return false;
    }

    static handle cast(const ServoDynamicsMessage& src, return_value_policy, handle) {
        py::dict d;
        for (const auto& [id, status] : src.getServoStatus()) {
            d[py::int_(id)] = status;
        }
        return d.release();
    }
};

}
}   


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
        })
        .def("subscribe", [](DexhandConnect& self, IDexhandMessageSubscriber<ServoFullStatusMessage>& subscriber){
            self.subscribe(&subscriber);
        })
        .def("unsubscribe", [](DexhandConnect& self, IDexhandMessageSubscriber<ServoFullStatusMessage>& subscriber){
            self.unsubscribe(&subscriber);
        })
        .def("subscribe", [](DexhandConnect& self, IDexhandMessageSubscriber<ServoDynamicsMessage>& subscriber){
            self.subscribe(&subscriber);
        })
        .def("unsubscribe", [](DexhandConnect& self, IDexhandMessageSubscriber<ServoDynamicsMessage>& subscriber){
            self.unsubscribe(&subscriber);
        })
        .def("subscribe", [](DexhandConnect& self, IDexhandMessageSubscriber<ServoVarsListMessage>& subscriber){
            self.subscribe(&subscriber);
        })
        .def("unsubscribe", [](DexhandConnect& self, IDexhandMessageSubscriber<ServoVarsListMessage>& subscriber){
            self.unsubscribe(&subscriber);
        });
    

    py::class_<IDexhandMessageSubscriber<FirmwareVersionMessage>, PyIDexhandMessageSubscriber<FirmwareVersionMessage>>(m, "FirmwareVersionMessageSubscriber")
        .def(py::init<>())
        .def("messageReceived", &IDexhandMessageSubscriber<FirmwareVersionMessage>::messageReceived);

    py::class_<IDexhandMessageSubscriber<ServoFullStatusMessage>, PyIDexhandMessageSubscriber<ServoFullStatusMessage>>(m, "ServoFullStatusMessageSubscriber")
        .def(py::init<>())
        .def("messageReceived", &IDexhandMessageSubscriber<ServoFullStatusMessage>::messageReceived);

    py::class_<IDexhandMessageSubscriber<ServoDynamicsMessage>, PyIDexhandMessageSubscriber<ServoDynamicsMessage>>(m, "ServoDynamicsMessageSubscriber")
        .def(py::init<>())
        .def("messageReceived", &IDexhandMessageSubscriber<ServoDynamicsMessage>::messageReceived);

    py::class_<IDexhandMessageSubscriber<ServoVarsListMessage>, PyIDexhandMessageSubscriber<ServoVarsListMessage>>(m, "ServoVarsListMessageSubscriber")
        .def(py::init<>())
        .def("messageReceived", &IDexhandMessageSubscriber<ServoVarsListMessage>::messageReceived);
    

    

    
}
    
