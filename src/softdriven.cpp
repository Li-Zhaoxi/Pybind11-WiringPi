#include "submodules.h"

#include <softPwm.h>
#include <softServo.h>
#include <softTone.h>
#include <pseudoPins.h>

void supplement_softpwm(py::module_ &subm);
void supplement_softservo(py::module_ &subm);
void supplement_softtone(py::module_ &subm);
void supplement_pseudopins(py::module_ &subm);

void add_submodule_softdriven(py::module_ &m)
{
  auto subm = m.def_submodule(
    "softdriven", 
    R"pbdoc(
      softPwm: Provide many channels of software driven PWM.
      softServo: Provide N channels of software driven PWM suitable for RC servo motors.
      softTone: For that authentic retro sound... Er... A little experiment to produce tones out of a Pi using one (or 2) GPIO pins and a piezeo "speaker" element. (Or a high impedance speaker, but don'y blame me if you blow-up the GPIO pins!)
      pseudoPins: Extend wiringPi with a number of pseudo pins which can be digitally or analog written/read. Note: Just one set of pseudo pins can exist per Raspberry Pi. These pins are shared between all programs running on that Raspberry Pi. The values are also persistant as they live in shared RAM. This gives you a means for temporary variable storing/sharing between programs, or for other cunning things I've not thought of yet..

      Copyright (c) 2012-2016 Gordon Henderson
    )pbdoc"
  );
  
  supplement_softpwm(subm);
  supplement_softservo(subm);
  supplement_softtone(subm);
  supplement_pseudopins(subm);


}

void supplement_softpwm(py::module_ &subm)
{
  // softPwm.h
  subm.def(
    "softPwmCreate", &softPwmCreate, 
    py::arg("pin"),
    py::arg("value"),
    py::arg("range"),
    R"pbdoc(
        Create a new softPWM thread.
    )pbdoc"
  );

  subm.def(
    "softPwmWrite", &softPwmWrite, 
    py::arg("pin"),
    py::arg("value"),
    R"pbdoc(
        Write a PWM value to the given pin.
    )pbdoc"
  );
  
  subm.def(
    "softPwmStop", &softPwmStop, 
    py::arg("pin"),
    R"pbdoc(
        Stop an existing softPWM thread.
    )pbdoc"
  );
}

void supplement_softservo(py::module_ &subm)
{
  // softServo.h
  subm.def(
    "softServoWrite", &softServoWrite, 
    py::arg("pin"),
    py::arg("value"),
    R"pbdoc(
        Write a Servo value to the given pin.
    )pbdoc"
  );
  
  subm.def(
    "softServoSetup", &softServoSetup, 
    py::arg("p0"), py::arg("p1"),
    py::arg("p2"), py::arg("p3"),
    py::arg("p4"), py::arg("p5"),
    py::arg("p6"), py::arg("p7"),
    R"pbdoc(
        Setup the software servo system.
    )pbdoc"
  );
}

void supplement_softtone(py::module_ &subm)
{
  // softTone.h
  subm.def(
    "softToneCreate", &softToneCreate, 
    py::arg("pin"),
    R"pbdoc(
        Create a new tone thread.
    )pbdoc"
  );
  
  subm.def(
    "softToneStop", &softToneStop, 
    py::arg("pin"),
    R"pbdoc(
        Stop an existing softTone thread.
    )pbdoc"
  );

  subm.def(
    "softToneWrite", &softToneWrite, 
    py::arg("pin"),
    py::arg("freq"),
    R"pbdoc(
        Write a frequency value to the given pin.
    )pbdoc"
  );
}

void supplement_pseudopins(py::module_ &subm)
{
  subm.def(
    "pseudoPinsSetup", 
    [](int pinBase) -> bool
    {
      return pseudoPinsSetup(pinBase) > 0;
    }, 
    py::arg("pinBase"),
    R"pbdoc(
        Write a frequency value to the given pin.
    )pbdoc"
  );
}