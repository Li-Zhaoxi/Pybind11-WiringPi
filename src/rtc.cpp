#include "submodules.h"
 
#include <ds1302.h>

void supplement_ds1302(py::module_ &subm);

void add_submodule_rtc(py::module_ &m)
{
  auto subm = m.def_submodule(
    "rtc", 
    R"pbdoc(
        Extend wiringPi with 
          the DS1302 Real Time clock
        Copyright (c) 2013 Gordon Henderson.
    )pbdoc"
  );
  
  supplement_ds1302(subm);

}


void supplement_ds1302(py::module_ &subm)
{
  subm.def(
    "ds1302rtcRead", &ds1302rtcRead, 
    py::arg("reg"),
    R"pbdoc(
        Reads the data to/from the RTC register
    )pbdoc"
  );

  subm.def(
    "ds1302rtcWrite", &ds1302rtcWrite, 
    py::arg("reg"), py::arg("data"),
    R"pbdoc(
        Writes the data to/from the RTC register
    )pbdoc"
  );

  subm.def(
    "ds1302ramRead", &ds1302ramRead, 
    py::arg("addr"),
    R"pbdoc(
        Reads the data to/from the RTC register
    )pbdoc"
  );

  subm.def(
    "ds1302ramWrite", &ds1302ramWrite, 
    py::arg("addr"), py::arg("data"),
    R"pbdoc(
        Writes the data to/from the RTC register
    )pbdoc"
  );

  subm.def(
    "ds1302clockRead", 
    []() -> py::array_t<int>
    {
      int clockData[8];
      ds1302clockRead(clockData);
      return ptr_to_array1d(clockData, 8);
    }, 
    R"pbdoc(
        Read all 8 bytes of the clock in a single operation
    )pbdoc"
  );

  subm.def(
    "ds1302clockWrite", 
    [](const py::array_t<int> &pydata)
    {
      int clockData[8];
      array1d_to_ptr(pydata, clockData, 8);
      ds1302clockWrite(clockData);
    }, 
    py::arg("clockData"),
    R"pbdoc(
        Write all 8 bytes of the clock in a single operation
    )pbdoc"
  );

  subm.def(
    "ds1302trickleCharge", &ds1302trickleCharge, 
    py::arg("diodes"), py::arg("resistors"),
    R"pbdoc(
        Set the bits on the trickle charger.
        Probably best left alone...
    )pbdoc"
  );
  
  subm.def(
    "ds1302setup", &ds1302setup, 
    py::arg("clockPin"), 
    py::arg("dataPin"), 
    py::arg("csPin"),
    R"pbdoc(
        Initialise the chip & remember the pins we're using
    )pbdoc"
  );
}