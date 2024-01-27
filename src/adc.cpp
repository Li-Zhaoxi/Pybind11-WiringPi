#include "submodules.h"

#include <ads1115.h>
#include <max5322.h>
#include <max31855.h>
#include <mcp3002.h>
#include <mcp3004.h>
#include <mcp3422.h>
#include <mcp4802.h>

void supplement_ads1115(py::module_ &subm);
void supplement_max5322(py::module_ &subm);
void supplement_max31855(py::module_ &subm);
void supplement_mcp3002(py::module_ &subm);
void supplement_mcp3004(py::module_ &subm);
void supplement_mcp3422(py::module_ &subm);
void supplement_mcp4802(py::module_ &subm);


void add_submodule_adc(py::module_ &m)
{
  auto subm = m.def_submodule(
    "adc", 
    R"pbdoc(
        Extend wiringPi with the
          ADS1115 I2C 16-bit ADC
          MAX5322 SPI Digital to Analog convertor
          max31855 SPI Analog to Digital convertor
          MCP3002 SPI Analog to Digital convertor
          MCP3004 SPI Analog to Digital convertor
          MCP3422/3/4 I2C ADC chip
          MCP4802 SPI Digital to Analog convertor

        Copyright (c) 2012-2016 Gordon Henderson
    )pbdoc"
  );

  supplement_ads1115(subm);
  supplement_max5322(subm);
  supplement_max31855(subm);
  supplement_mcp3002(subm);
  supplement_mcp3004(subm);
  supplement_mcp3422(subm);
  supplement_mcp4802(subm);
  
}

void supplement_ads1115(py::module_ &subm)
{
  subm.def(
    "ads1115Setup", 
    [](int pinBase, int i2cAddr) -> bool
    {
      return ads1115Setup(pinBase, i2cAddr) > 0;
    }, 
    py::arg("pinBase"), py::arg("i2cAddress"), 
    R"pbdoc(
        Create a new wiringPi device node for an ads1115 on the Pi's I2C interface.
    )pbdoc"
  );

  subm.attr("ADS1115_GAIN_6") = ADS1115_GAIN_6;
  subm.attr("ADS1115_GAIN_4") = ADS1115_GAIN_4;
  subm.attr("ADS1115_GAIN_2") = ADS1115_GAIN_2;
  subm.attr("ADS1115_GAIN_1") = ADS1115_GAIN_1;
  subm.attr("ADS1115_GAIN_HALF") = ADS1115_GAIN_HALF;
  subm.attr("ADS1115_GAIN_QUARTER") = ADS1115_GAIN_QUARTER;

  subm.attr("ADS1115_DR_8") = ADS1115_DR_8;
  subm.attr("ADS1115_DR_16") = ADS1115_DR_16;
  subm.attr("ADS1115_DR_32") = ADS1115_DR_32;
  subm.attr("ADS1115_DR_64") = ADS1115_DR_64;
  subm.attr("ADS1115_DR_128") = ADS1115_DR_128;
  subm.attr("ADS1115_DR_250") = ADS1115_DR_250;
  subm.attr("ADS1115_DR_475") = ADS1115_DR_475;
  subm.attr("ADS1115_DR_860") = ADS1115_DR_860;
}

void supplement_max5322(py::module_ &subm)
{
  subm.def(
    "max5322Setup", 
    [](int pinBase, int spiChannel) -> bool
    {
      return max5322Setup(pinBase, spiChannel) > 0;
    }, 
    py::arg("pinBase"),
    py::arg("spiChannel"),
    R"pbdoc(
        Create a new wiringPi device node for an max5322 on the Pi's SPI interface.
    )pbdoc"
  );
}

void supplement_max31855(py::module_ &subm)
{
  subm.def(
    "max31855Setup", 
    [](int pinBase, int spiChannel) -> bool
    {
      return max31855Setup(pinBase, spiChannel) > 0;
    },
    py::arg("pinBase"),
    py::arg("spiChannel"),
    R"pbdoc(
        Create a new wiringPi device node for an max31855 on the Pi's SPI interface.
    )pbdoc"
  );
}

void supplement_mcp3002(py::module_ &subm)
{
  subm.def(
    "mcp3002Setup", 
    [](int pinBase, int spiChannel) -> bool
    {
      return mcp3002Setup(pinBase, spiChannel) > 0;
    }, 
    py::arg("pinBase"), py::arg("spiChannel"), 
    R"pbdoc(
        Create a new wiringPi device node for an mcp3002 on the Pi's SPI interface.
    )pbdoc"
  );
}

void supplement_mcp3004(py::module_ &subm)
{
  subm.def(
    "mcp3004Setup", 
    [](int pinBase, int spiChannel) -> bool
    {
      return mcp3004Setup(pinBase, spiChannel) > 0;
    }, 
    py::arg("pinBase"), py::arg("spiChannel"), 
    R"pbdoc(
        Create a new wiringPi device node for an mcp3004 on the Pi's SPI interface.
    )pbdoc"
  );
}

void supplement_mcp3422(py::module_ &subm)
{
  subm.attr("MCP3422_SR_240") = MCP3422_SR_240;
  subm.attr("MCP3422_SR_60") = MCP3422_SR_60;
  subm.attr("MCP3422_SR_15") = MCP3422_SR_15;
  subm.attr("MCP3422_SR_3_75") = MCP3422_SR_3_75;

  subm.attr("MCP3422_GAIN_1") = MCP3422_GAIN_1;
  subm.attr("MCP3422_GAIN_2") = MCP3422_GAIN_2;
  subm.attr("MCP3422_GAIN_4") = MCP3422_GAIN_4;
  subm.attr("MCP3422_GAIN_8") = MCP3422_GAIN_8;

  subm.def(
    "mcp3422Setup", 
    [](int pinBase, int i2cAddress, int sampleRate, int gain) -> bool
    {
      return mcp3422Setup(pinBase, i2cAddress, sampleRate, gain) > 0;
    }, 
    py::arg("pinBase"), py::arg("i2cAddress"), 
    py::arg("sampleRate"), py::arg("gain"), 
    R"pbdoc(
        Create a new wiringPi device node for the mcp3422
    )pbdoc"
  );

}

void supplement_mcp4802(py::module_ &subm)
{
  subm.def(
    "mcp4802Setup", 
    [](int pinBase, int spiChannel) -> bool
    {
      return mcp4802Setup(pinBase, spiChannel) > 0;
    }, 
    py::arg("pinBase"), py::arg("spiChannel"), 
    R"pbdoc(
        Create a new wiringPi device node for an mcp4802 on the Pi's SPI interface.
    )pbdoc"
  );
}