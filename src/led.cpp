#include "submodules.h"

#include <sn3218.h>

void supplement_sn3218(py::module_ &subm);


void add_submodule_led(py::module_ &m)
{
  auto subm = m.def_submodule(
    "led", 
    R"pbdoc(
      Extend wiringPi with the 
        SN3218 I2C LEd Driver

      Copyright (c) 2013 Gordon Henderson
    )pbdoc"
  );
  
  supplement_sn3218(subm);
}


void supplement_sn3218(py::module_ &subm)
{
  subm.def(
    "sn3218Setup", 
    [](int pinBase) -> bool
    {
      return sn3218Setup(pinBase) > 0;
    }, 
    py::arg("pinBase"),
    R"pbdoc(
        Create a new wiringPi device node for an sn3218 on the Pi's SPI interface.
    )pbdoc"
  );
}