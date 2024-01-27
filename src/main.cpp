#include "submodules.h"
#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;

PYBIND11_MODULE(WiringPi, m) {
    m.doc() = R"pbdoc(
        wiringPi is free software: you can redistribute it and/or modify
        -----------------------

        it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
    )pbdoc";

    add_submodule_adc(m);
    add_submodule_sensors(m);
    add_submodule_drc(m);
    add_submodule_rtc(m);
    add_submodule_lcd(m);
    add_submodule_expansion(m);
    add_submodule_gpio(m);
    add_submodule_softdriven(m);
    add_submodule_led(m);
    add_submodule_wiring(m);
    

#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}
