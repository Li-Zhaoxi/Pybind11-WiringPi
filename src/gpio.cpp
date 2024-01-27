#include "submodules.h"
#include "conflict/supplements.h"


#include <mcp23s08.h>
#include <mcp23008.h>
#include <mcp23017.h>
#include <mcp23s17.h>
#include <pcf8574.h>
#include <pcf8591.h>
#include <sr595.h>

void supplement_mcp23s08(py::module_ &subm);
void supplement_mcp23008(py::module_ &subm);
void supplement_mcp23017(py::module_ &subm);
void supplement_mcp23s17(py::module_ &subm);
void supplement_pcf8574(py::module_ &subm);
void supplement_pcf8591(py::module_ &subm);
void supplement_sr595(py::module_ &subm);

void add_submodule_gpio(py::module_ &m)
{
  auto subm = m.def_submodule(
    "gpio", 
    R"pbdoc(
        Extend wiringPi with the 
          MCP 23s08 SPI GPIO expander chip.
          MCP 23016 I2C GPIO expander chip.
          MCP 23008 I2C GPIO expander chip.
          MCP 23017 I2C GPIO expander chip
          MCP 23s17 SPI GPIO expander chip.
          PCF8574 I2C GPIO expander chip
          PCF8591 I2C GPIO Analog expander chip. The chip has 1 8-bit DAC and 4 x 8-bit ADCs
          74x595 shift register as a GPIO expander chip.
        Copyright (c) 2013 Gordon Henderson
    )pbdoc"
  );
  
  supplement_mcp23s08(subm);
  supplement_mcp23x0817(subm);
  supplement_mcp23016(subm);
  supplement_mcp23008(subm);
  supplement_mcp23017(subm);
  supplement_mcp23s17(subm);
  supplement_mcp23x17reg(subm);
  supplement_pcf8574(subm);
  supplement_pcf8591(subm);

  
}


void supplement_mcp23s08(py::module_ &subm)
{
  subm.def(
    "mcp23s08Setup", 
    [](int pinBase, int spiPort, int devId) -> bool
    {
      return mcp23s08Setup(pinBase, spiPort, devId) > 0;
    }, 
    py::arg("pinBase"), py::arg("spiPort"), 
    py::arg("devId"), 
    R"pbdoc(
        Create a new instance of an MCP23s08 SPI GPIO interface. We know it has 8 pins, so all we need to know here is the SPI address and the user-defined pin base.
    )pbdoc"
  );
}

void supplement_mcp23008(py::module_ &subm)
{
  subm.def(
    "mcp23008Setup", 
    [](int pinBase, int i2cAddress) -> bool
    {
      return mcp23008Setup(pinBase, i2cAddress) > 0;
    }, 
    py::arg("pinBase"), py::arg("i2cAddress"), 
    R"pbdoc(
        Create a new instance of an MCP23008 I2C GPIO interface. We know it has 8 pins, so all we need to know here is the I2C address and the user-defined pin base.
    )pbdoc"
  );
}

void supplement_mcp23017(py::module_ &subm)
{
  subm.def(
    "mcp23017Setup", 
    [](int pinBase, int i2cAddress) -> bool
    {
      return mcp23017Setup(pinBase, i2cAddress) > 0;
    }, 
    py::arg("pinBase"), py::arg("i2cAddress"), 
    R"pbdoc(
        Create a new instance of an MCP23017 I2C GPIO interface. We know it has 16 pins, so all we need to know here is the I2C address and the user-defined pin base.
    )pbdoc"
  );
}

void supplement_mcp23s17(py::module_ &subm)
{
  subm.def(
    "mcp23s17Setup", 
    [](int pinBase, int spiPort, int devId) -> bool
    {
      return mcp23s17Setup(pinBase, spiPort, devId) > 0;
    }, 
    py::arg("pinBase"), py::arg("spiPort"), 
    py::arg("devId"), 
    R"pbdoc(
        Create a new instance of an MCP23s17 SPI GPIO interface. We know it has 16 pins, so all we need to know here is the SPI address and the user-defined pin base.
    )pbdoc"
  );
}


void supplement_pcf8574(py::module_ &subm)
{
  subm.def(
    "pcf8574Setup", 
    [](int pinBase, int i2cAddress) -> bool
    {
      return pcf8574Setup(pinBase, i2cAddress) > 0;
    }, 
    py::arg("pinBase"), py::arg("i2cAddress"),
    R"pbdoc(
        Create a new instance of a PCF8574 I2C GPIO interface. We know it has 8 pins, so all we need to know here is the I2C address and the user-defined pin base.
    )pbdoc"
  );
}

void supplement_pcf8591(py::module_ &subm)
{
  subm.def(
    "pcf8591Setup", 
    [](int pinBase, int i2cAddress) -> bool
    {
      return pcf8591Setup(pinBase, i2cAddress) > 0;
    }, 
    py::arg("pinBase"), py::arg("i2cAddress"),
    R"pbdoc(
        Create a new instance of a PCF8591 I2C GPIO interface. We know it has 4 pins, (4 analog inputs and 1 analog output which we'll shadow input 0) so all we need to know here is the I2C address and the user-defined pin base.
    )pbdoc"
  );
}

void supplement_sr595(py::module_ &subm)
{
  subm.def(
    "sr595Setup", 
    [](int pinBase, int numPins, int dataPin, int clockPin, int latchPin) -> void
    {
      sr595Setup(pinBase, numPins, dataPin, clockPin, latchPin);
    }, 
    py::arg("pinBase"), 
    py::arg("numPins"),
    py::arg("dataPin"),
    py::arg("clockPin"),
    py::arg("latchPin"),
    R"pbdoc(
        Create a new instance of a 74x595 shift register GPIO expander.
    )pbdoc"
  );
}