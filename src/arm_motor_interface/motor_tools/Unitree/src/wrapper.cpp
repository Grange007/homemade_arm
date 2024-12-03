#include <pybind11/pybind11.h>
#include "unitreeMotor/unitreeMotor.h"
#include "serialPort/SerialPort.h"

namespace py = pybind11;

PYBIND11_MODULE(UnitreeMotorSDK, m)
{
    m.doc() = "UnitreeMotorSDK python wrapper";

    py::enum_<MotorType>(m, "MotorType")
    .value("GO_M8010_6", MotorType::GO_M8010_6, "GO_M8010_6 Motor");

    py::class_<MotorCmd>(m, "MotorCmd")
    .def(pybind11::init<>())
    .def_readwrite("motorType", &MotorCmd::motorType)
    .def_readwrite("hex_len",   &MotorCmd::hex_len)
    .def_readwrite("id",        &MotorCmd::id)
    .def_readwrite("mode",      &MotorCmd::mode)
    .def_readwrite("T",         &MotorCmd::T)
    .def_readwrite("W",         &MotorCmd::W)
    .def_readwrite("Pos",       &MotorCmd::Pos)
    .def_readwrite("K_P",       &MotorCmd::K_P)
    .def_readwrite("K_W",       &MotorCmd::K_W);

    py::class_<MotorData>(m, "MotorData")
    .def(pybind11::init<>())
    .def_readwrite("motorType", &MotorData::motorType)
    .def_readwrite("hex_len",   &MotorData::hex_len)
    .def_readwrite("correct",   &MotorData::correct)
    .def_readwrite("motor_id",  &MotorData::motor_id)
    .def_readwrite("mode",      &MotorData::mode)
    .def_readwrite("Temp",      &MotorData::Temp)
    .def_readwrite("MError",    &MotorData::MError)
    .def_readwrite("T",         &MotorData::T)
    .def_readwrite("W",         &MotorData::W)
    .def_readwrite("Pos",       &MotorData::Pos)
    .def_readwrite("footForce", &MotorData::footForce);

    py::class_<SerialPort>(m, "SerialPort")
    .def(py::init<const std::string &>())
    .def("sendRecv", py::overload_cast<MotorCmd*, MotorData*>(&SerialPort::sendRecv));
}