#include "submodules.h"

#include <gertboard.h>
#include <scrollPhat.h>
#include <piFace.h>
#include <piGlow.h>
#include <piNes.h>

void supplement_gertboard(py::module_ &subm);
void supplement_scrollphat(py::module_ &subm);
void supplement_piface(py::module_ &subm);
void supplement_piglow(py::module_ &subm);
void supplement_pines(py::module_ &subm);

void add_submodule_expansion(py::module_ &m)
{
  auto subm = m.def_submodule(
    "expansion", 
    R"pbdoc(
        Gertboard: Access routines for the SPI devices. It has:
          An MCP3002 dual-channel A to D convertor connected to the SPI bus, selected by chip-select A.
          An MCP4802 dual-channel D to A convertor connected to the SPI bus, selected via chip-select B.

        scrollPhat: Simple driver for the Pimoroni Scroll Phat device
        piFace: the PiFace peripheral device which has an MCP23S17 GPIO device connected via the SPI bus.
        piGlow: Easy access to the Pimoroni PiGlow board.
        piNes: Driver for the NES Joystick controller on the Raspberry Pi
    )pbdoc"
  );

  supplement_gertboard(subm);
  supplement_scrollphat(subm);
  supplement_piface(subm);
  supplement_piglow(subm);
  supplement_pines(subm);

}

void supplement_gertboard(py::module_ &subm)
{
  subm.def(
    "gertboardAnalogWrite", 
    &gertboardAnalogWrite, 
    py::arg("chan"), py::arg("value"),
    R"pbdoc(
        [old routines] Write an 8-bit data value to the MCP4802 Analog to digital convertor on the Gertboard.
    )pbdoc"
  );

  subm.def(
    "gertboardAnalogRead", 
    &gertboardAnalogRead, 
    py::arg("chan"), 
    R"pbdoc(
        [old routines] Return the analog value of the given channel (0/1). The A/D is a 10-bit device
    )pbdoc"
  );

  subm.def(
    "gertboardSPISetup", 
    &gertboardSPISetup, 
    R"pbdoc(
        [old routines] Initialise the SPI bus, etc.
    )pbdoc"
  );

  subm.def(
    "gertboardAnalogSetup", 
    &gertboardAnalogSetup, 
    py::arg("pinBase"), 
    R"pbdoc(
        [new] New wiringPi node extension methods.
        Create a new wiringPi device node for the analog devices on the Gertboard. We create one node with 2 pins - each pin being read and write - although the operations actually go to different hardware devices.
    )pbdoc"
  );
}


void supplement_scrollphat(py::module_ &subm)
{
  subm.def(
    "scrollPhatPoint", &scrollPhatPoint, 
    py::arg("x"), py::arg("y"),
    py::arg("colour"),
    R"pbdoc(
        Plot a pixel. Crude clipping - speed is not the essence here.
    )pbdoc"
  );

  subm.def(
    "scrollPhatLine", &scrollPhatLine, 
    py::arg("x0"), py::arg("y0"),
    py::arg("x1"), py::arg("y1"),
    py::arg("colour"),
    R"pbdoc(
        Classic Bressenham Line code - rely on the point function to do the clipping for us here.
    )pbdoc"
  );

  subm.def(
    "scrollPhatLineTo", &scrollPhatLineTo, 
    py::arg("x"), py::arg("y"),
    py::arg("colour"),
    "scrollPhatLineTo"
  );

  subm.def(
    "scrollPhatRectangle", &scrollPhatRectangle, 
    py::arg("x1"), py::arg("y1"),
    py::arg("x2"), py::arg("y2"),
    py::arg("colour"),
    py::arg("filled"),
    R"pbdoc(
        A rectangle is a spoilt days fishing
    )pbdoc"
  );

  subm.def(
    "scrollPhatUpdate", &scrollPhatUpdate, 
    R"pbdoc(
        Copy our software version to the real display
    )pbdoc"
  );

  subm.def(
    "scrollPhatClear", &scrollPhatClear, 
    R"pbdoc(
        Clear the display
    )pbdoc"
  );

  subm.def(
    "scrollPhatPutchar", &scrollPhatPutchar, 
    py::arg("c"),
    R"pbdoc(
        Print a single character to the screen then advance the pointer by an appropriate ammount (variable width font).
        We rely on the clipping done by the pixel plot function to keep us out of trouble.
        Return the width + space
    )pbdoc"
  );

  subm.def(
    "scrollPhatPuts", &scrollPhatPuts, 
    py::arg("message"),
    R"pbdoc(
        Send a string to the display - and scroll it across.
        This is somewhat of a hack in that we print the entire string to the display and let the point clipping take care of what's off-screen...
    )pbdoc"
  );

  subm.def(
    "scrollPhatPrintSpeed", &scrollPhatPrintSpeed, 
    py::arg("cps10"),
    R"pbdoc(
        Change the print speed - mS per shift by 1 pixel
    )pbdoc"
  );

  subm.def(
    "scrollPhatIntensity", &scrollPhatIntensity, 
    py::arg("percent"),
    R"pbdoc(
        Set the display brightness - percentage
    )pbdoc"
  );

  subm.def(
    "scrollPhatSetup", &scrollPhatSetup, 
    R"pbdoc(
        Initialise the Scroll Phat display
    )pbdoc"
  );
}

void supplement_piface(py::module_ &subm)
{
  subm.def(
    "piFaceSetup", 
    &piFaceSetup, 
    py::arg("pinBase"), 
    R"pbdoc(
        We're going to create an instance of the mcp23s17 here, then provide our own read/write routines on-top of it...
        The supplied PiFace code (in Pithon) treats it as an 8-bit device where you write the output ports and read the input port using the same pin numbers, however I have had a request to be able to read the output port, so reading 8..15 will read the output latch.
    )pbdoc"
  );
}

void supplement_piglow(py::module_ &subm)
{
  // piGlow
  subm.attr("PIGLOW_RED") = PIGLOW_RED;
  subm.attr("PIGLOW_ORANGE") = PIGLOW_ORANGE;
  subm.attr("PIGLOW_YELLOW") = PIGLOW_YELLOW;
  subm.attr("PIGLOW_GREEN") = PIGLOW_GREEN;
  subm.attr("PIGLOW_BLUE") = PIGLOW_BLUE;
  subm.attr("PIGLOW_WHITE") = PIGLOW_WHITE;

  subm.def(
    "piGlow1", 
    &piGlow1, 
    py::arg("leg"), 
    py::arg("ring"), 
    py::arg("intensity"), 
    R"pbdoc(
        Light up an individual LED
    )pbdoc"
  );

  subm.def(
    "piGlowLeg", 
    &piGlowLeg, 
    py::arg("leg"), 
    py::arg("intensity"), 
    R"pbdoc(
        Light up all 6 LEDs on a leg
    )pbdoc"
  );

  subm.def(
    "piGlowRing", 
    &piGlowRing, 
    py::arg("ring"), 
    py::arg("intensity"), 
    R"pbdoc(
        Light up 3 LEDs in a ring. Ring 0 is the outermost, 5 the innermost
    )pbdoc"
  );

  subm.def(
    "piGlowSetup", 
    &piGlowSetup, 
    py::arg("clear"), 
    R"pbdoc(
        Initialise the board & remember the pins we're using
    )pbdoc"
  );

}

void supplement_pines(py::module_ &subm)
{
  // piNes
  subm.attr("MAX_NES_JOYSTICKS") = MAX_NES_JOYSTICKS;
  subm.attr("NES_RIGHT") = NES_RIGHT;
  subm.attr("NES_LEFT") = NES_LEFT;
  subm.attr("NES_DOWN") = NES_DOWN;
  subm.attr("NES_UP") = NES_UP;
  subm.attr("NES_START") = NES_START;
  subm.attr("NES_SELECT") = NES_SELECT;
  subm.attr("NES_B") = NES_B;
  subm.attr("NES_A") = NES_A;

  subm.def(
    "setupNesJoystick", 
    &setupNesJoystick, 
    py::arg("dPin"), 
    py::arg("cPin"), 
    py::arg("lPin"), 
    R"pbdoc(
        Create a new NES joystick interface, program the pins, etc.
    )pbdoc"
  );

  subm.def(
    "readNesJoystick", 
    &readNesJoystick, 
    py::arg("joystick"), 
    R"pbdoc(
        Do a single scan of the NES Joystick.
    )pbdoc"
  );
}