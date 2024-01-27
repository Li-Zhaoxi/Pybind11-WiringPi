#include "submodules.h"

#include <drcNet.h>
#include <drcSerial.h>

void add_submodule_drc(py::module_ &m)
{
  auto subm = m.def_submodule(
    "drc", 
    R"pbdoc(
        Extend wiringPi with 
          DRC Network protocol (e.g. to another Pi)
          DRC Serial protocol (e.g. to Arduino)
        Copyright (c) 2016-2017 Gordon Henderson
    )pbdoc"
  );

  subm.def(
    "drcSetupNet", 
    [](int pinBase, int numPins, 
       const char *ipAddress, 
       const char *port, 
       const char *password) -> bool
    {
      return drcSetupNet(pinBase, numPins, ipAddress, port, password) > 0;
    }, 
    py::arg("pinBase"), py::arg("numPins"),
    py::arg("ipAddress"), py::arg("port"),
    py::arg("password"),
    R"pbdoc(
        Create a new instance of an DRC GPIO interface.
        Could be a variable nunber of pins here - we might not know in advance.
    )pbdoc"
  );
  
  subm.def(
    "drcSetupSerial", 
    [](int pinBase, int numPins, 
       const char *device, int baud) -> bool
    {
      return drcSetupSerial(pinBase, numPins, device, baud) > 0;
    }, 
    py::arg("pinBase"), py::arg("numPins"),
    py::arg("device"), py::arg("baud"),
    R"pbdoc(
      Create a new instance of an DRC GPIO interface.
      Could be a variable nunber of pins here - we might not know in advance
      if it's an ATmega with 14 pins, or something with less or more!
    )pbdoc"
  );
}